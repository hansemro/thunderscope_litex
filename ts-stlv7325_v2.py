#!/usr/bin/env python3

#
# This file is part of LitePCIe.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2023 Hansem Ro <hansemro@outlook.com>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *

from litex.gen import *

from litex_boards.platforms import sitlinv_stlv7325_v2
from litex.build.generic_platform import *

from litex.soc.cores.clock import S7PLL
from litex.soc.cores.led import LedChaser
from litex.soc.cores.xadc import XADC
from litex.soc.cores.dna  import DNA
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.gpio import GPIOOut
from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.pwm import PWM
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.core import LitePCIeEndpoint, LitePCIeMSI
from litepcie.frontend.dma import LitePCIeDMA
from litepcie.frontend.wishbone import LitePCIeWishboneBridge
from litepcie.software import generate_litepcie_software

from litescope import LiteScopeAnalyzer
from peripherals.had1511_adc import HAD1511ADC
from peripherals.trigger import Trigger

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys = ClockDomain()

        # # #

        # PLL
        self.pll = pll = S7PLL(speedgrade=-2)
        pll.register_clkin(platform.request("clk200"), 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_period_constraint(self.cd_sys.clk, 1e9/sys_clk_freq)

# GPIO

_i2c_io = [
    # I2C.
    ("test_i2c", 0,
        Subsignal("scl", Pins("J18")),
        Subsignal("sda", Pins("L17")),
        IOStandard("LVCMOS33")
    )
]

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    configs = {
        # Gen2  data_width, sys_clk_freq
        "gen2:x1": (64,   int(125e6)),
        "gen2:x4": (128,  int(200e6)),
    }
    def __init__(self, platform, speed="gen2", nlanes=4,
        with_led_chaser = True,
        with_frontend = True,
        with_adc = True,
        with_jtagbone = True,
        with_analyzer = True,
        **kwargs):
        data_width, sys_clk_freq = self.configs[speed + f":x{nlanes}"]

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident=f"LitePCIe example design on Sitlinv STLV7325 V2 ({speed}:x{nlanes})")

        # JTAGBone ---------------------------------------------------------------------------------
        if with_jtagbone:
            self.add_jtagbone()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # XADC -------------------------------------------------------------------------------------
        self.xadc = XADC()

        # DNA --------------------------------------------------------------------------------------
        self.dna = DNA()
        self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led_n"),
                sys_clk_freq = sys_clk_freq)

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # PCIe -------------------------------------------------------------------------------------
        # PHY
        self.pcie_phy = S7PCIEPHY(platform, platform.request(f"pcie_x{nlanes}"),
            data_width = data_width,
            bar0_size  = 0x20000,
        )
        self.pcie_phy.add_ltssm_tracer()
        platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/sys_clk_freq)

        # Endpoint
        self.pcie_endpoint = LitePCIeEndpoint(self.pcie_phy,
            endianness           = "big",
            max_pending_requests = 8
        )

        # Wishbone bridge
        self.pcie_bridge = LitePCIeWishboneBridge(self.pcie_endpoint,
            base_address = self.mem_map["csr"])
        self.bus.add_master(master=self.pcie_bridge.wishbone)

        # DMA0
        self.pcie_dma0 = LitePCIeDMA(self.pcie_phy, self.pcie_endpoint,
            with_buffering = True, buffering_depth=1024,
            with_loopback  = True)

        # DMA1
        self.pcie_dma1 = LitePCIeDMA(self.pcie_phy, self.pcie_endpoint,
            with_buffering = True, buffering_depth=1024,
            with_loopback  = True)

        self.add_constant("DMA_CHANNELS", 2)
        self.add_constant("DMA_ADDR_WIDTH", 32)

        # MSI
        self.pcie_msi = LitePCIeMSI()
        self.comb += self.pcie_msi.source.connect(self.pcie_phy.msi)
        self.interrupts = {
            "PCIE_DMA0_WRITER":    self.pcie_dma0.writer.irq,
            "PCIE_DMA0_READER":    self.pcie_dma0.reader.irq,
            "PCIE_DMA1_WRITER":    self.pcie_dma1.writer.irq,
            "PCIE_DMA1_READER":    self.pcie_dma1.reader.irq,
        }
        for i, (k, v) in enumerate(sorted(self.interrupts.items())):
            self.comb += self.pcie_msi.irqs[i].eq(v)
            self.add_constant(k + "_INTERRUPT", i)

        # I2C Bus:
        # - Trim DAC (MCP4728 @ 0x61).
        # - PLL      (LMK61E2 @ 0x58).
        platform.add_extension(_i2c_io)
        self.submodules.i2c = I2CMaster(platform.request("test_i2c"))

        # Frontend.
        if with_frontend:

            class Frontend(Module, AutoCSR):
                def __init__(self, control_pads, pga_spi_pads, sys_clk_freq, pga_spi_clk_freq=1e6):
                    # Control/Status.
                    self._control = CSRStorage(fields=[
                        CSRField("fe_en", offset=0, size=1, description="Frontend LDO-Enable.", values=[
                            ("``0b0``", "LDO disabled."),
                            ("``0b1``", "LDO enabled."),
                        ]),
                        CSRField("coupling",  offset=8, size=4, description="Frontend AC/DC Coupling.", values=[
                            ("``0b0``", "AC-Coupling (one bit per channel)."),
                            ("``0b1``", "DC-Coupling (one bit per channel)."),
                        ]),
                        CSRField("attenuation",  offset=16, size=4, description="Frontend Attenuation.", values=[
                            ("``0b0``", " 1X-Attenuation (one bit per channel)."),
                            ("``0b1``", "10X-Attenuation (one bit per channel)."),
                        ]),
                    ])
                    # # #

                    if control_pads is not None:
                        # Power.
                        self.comb += control_pads.fe_en.eq(self._control.fields.fe_en)

                        # Coupling.
                        self.comb += control_pads.coupling.eq(self._control.fields.coupling)

                        # Attenuation.
                        self.comb += control_pads.attenuation.eq(self._control.fields.attenuation)

                    # Programmable Gain Amplifier (LMH6518/SPI).
                    if pga_spi_pads is not None:
                        pga_spi_pads.miso = Signal()
                    self.submodules.spi = SPIMaster(
                        pads         = pga_spi_pads,
                        data_width   = 24,
                        sys_clk_freq = sys_clk_freq,
                        spi_clk_freq = pga_spi_clk_freq
                    )

            self.submodules.frontend = Frontend(
                control_pads     = None,
                pga_spi_pads     = None,
                sys_clk_freq     = sys_clk_freq,
                pga_spi_clk_freq = 1e6,
            )

        # ADC.
        if with_adc:

            class ADC(Module, AutoCSR):
                def __init__(self, control_pads, status_pads, spi_pads, data_pads, sys_clk_freq,
                    data_width   = 128,
                    spi_clk_freq = 1e6
                ):

                    # Control/Status.
                    self._control = CSRStorage(fields=[
                        CSRField("acq_en", offset=0, size=1, description="ADC LDO-Enable.", values=[
                            ("``0b0``", "LDO disabled."),
                            ("``0b1``", "LDO enabled."),
                        ]),
                        CSRField("osc_en", offset=1, size=1, description="ADC-PLL Output-Enable.", values=[
                            ("``0b0``", "PLL output disabled."),
                            ("``0b1``", "PLL output enabled."),
                        ]),
                        CSRField("rst", offset=2, size=1, description="ADC Reset.", values=[
                            ("``0b0``", "ADC in operational mode."),
                            ("``0b1``", "ADC in reset mode."),
                        ]),
                        CSRField("pwr_down", offset=3, size=1, description="ADC Power-Down.", values=[
                            ("``0b0``", "ADC in operational mode."),
                            ("``0b1``", "ADC in power-down mode."),
                        ]),
                    ])


                    # Data Source.
                    self.source = stream.Endpoint([("data", data_width)])

                    # # #

                    # Control-Path -----------------------------------------------------------------
                    if control_pads is not None:
                        # Control.
                        self.comb += [
                            control_pads.acq_en.eq(self._control.fields.acq_en),
                            control_pads.osc_oe.eq(self._control.fields.osc_en),
                        ]

                    # # SPI.
                    # spi_pads.miso = Signal()
                    # self.submodules.spi = SPIMaster(
                    #     pads         = spi_pads,
                    #     data_width   = 24,
                    #     sys_clk_freq = sys_clk_freq,
                    #     spi_clk_freq = spi_clk_freq
                    # )

                    # Data-Path --------------------------------------------------------------------

                    # Trigger.
                    self.submodules.trigger = Trigger()

                    # HAD1511.
                    self.submodules.had1511 = HAD1511ADC(data_pads, sys_clk_freq, lanes_polarity=[1, 1, 0, 1, 1, 1, 1, 1])

                    # Gate/Data-Width Converter.
                    #self.submodules.gate = stream.Gate([("data", 64)], sink_ready_when_disabled=True)
                    self.submodules.conv = stream.Converter(64, data_width)
                    #self.comb += self.gate.enable.eq(self.trigger.enable)

                    # Pipeline.
                    self.submodules += stream.Pipeline(
                        self.had1511,
                        #self.gate,
                        self.conv,
                        self.source
                    )

            self.submodules.adc = ADC(
                control_pads = None,
                status_pads  = None, #platform.request("adc_status"),
                spi_pads     = None,
                data_pads    = None,
                sys_clk_freq = sys_clk_freq,
                spi_clk_freq = 1e6,
            )

            # ADC -> PCIe.
            self.comb += self.adc.source.connect(self.pcie_dma0.sink)

            # Analyzer -----------------------------------------------------------------------------

            if with_analyzer:
                analyzer_signals = [
                    self.adc.source
                ]
                self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
                    depth        = 1024,
                    clock_domain = "sys",
                    samplerate   = sys_clk_freq,
                    csr_csv      = "test/analyzer.csv"
                )

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LitePCIe SoC on Sitlinv STLV7325 V2")
    parser.add_argument("--build",  action="store_true", help="Build bitstream")
    parser.add_argument("--driver", action="store_true", help="Generate LitePCIe driver")
    parser.add_argument("--nlanes", default=1,           help="PCIe lanes: 1 (default), 4")
    args = parser.parse_args()

    platform = sitlinv_stlv7325_v2.Platform()
    soc      = BaseSoC(platform, nlanes=int(args.nlanes))
    builder  = Builder(soc, output_dir="build/sitlinv_stlv7325_v2", csr_csv="csr.csv")
    builder.build(build_name="stlv7325_v2", run=args.build)

    if args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

if __name__ == "__main__":
    main()
