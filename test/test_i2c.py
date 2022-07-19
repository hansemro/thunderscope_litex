#!/usr/bin/env python3

#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import time
import argparse

from litex import RemoteClient

# I2C Constants ------------------------------------------------------------------------------------

I2C_SCL    = 0x01
I2C_SDAOE  = 0x02
I2C_SDAOUT = 0x04
I2C_SDAIN  = 0x01

I2C_DELAY = 1
I2C_WRITE = 0
I2C_READ  = 1

# BitBangI2C ---------------------------------------------------------------------------------------

class BitBangI2C:
    def __init__(self, regs):
        self.regs = regs

        self.started = 0

        self.regs.i2c_w.write(I2C_SCL)
        # Check that the I2C bus is ready.
        while(not (self.regs.i2c_r.read() & I2C_SDAIN)):
            time.sleep(1e-3)

    # I2C bit-banging functions from http://en.wikipedia.org/wiki/I2c.
    def read_bit(self):
        # Let the Slave drive data.
        self.regs.i2c_w.write(0)
        self.regs.i2c_w.write(I2C_SCL)
        bit = (self.regs.i2c_r.read() & I2C_SDAIN)
        self.regs.i2c_w.write(0)
        return bit

    def write_bit(self, bit):
        if bit:
            self.regs.i2c_w.write(I2C_SDAOE| I2C_SDAOUT)
        else:
            self.regs.i2c_w.write(I2C_SDAOE)
        # Clock stretching.
        self.regs.i2c_w.write(self.regs.i2c_w.read() | I2C_SCL);
        self.regs.i2c_w.write(self.regs.i2c_w.read() & ~I2C_SCL);

    def start_cond(self):
        if self.started:
            # Set SDA to 1.
            self.regs.i2c_w.write(I2C_SDAOE| I2C_SDAOUT);
            self.regs.i2c_w.write(self.regs.i2c_w.read() | I2C_SCL);
        # SCL is high, set SDA from 1 to 0.
        self.regs.i2c_w.write(I2C_SDAOE| I2C_SCL)
        self.regs.i2c_w.write(I2C_SDAOE)
        self.started = 1

    def stop_cond(self):
        # Set SDA to 0.
        self.regs.i2c_w.write(I2C_SDAOE)
        # Clock stretching.
        self.regs.i2c_w.write(I2C_SDAOE| I2C_SCL)
        # SCL is high, set SDA from 0 to 1.
        self.regs.i2c_w.write(I2C_SCL)
        self.started = 0

    def write(self, byte):
        for bit in range(8):
            self.write_bit(byte & 0x80)
            byte <<= 1
        ack = not self.read_bit()
        return ack

    def read(self, ack):
        byte = 0
        for bit in range(8):
            byte <<= 1
            byte |= self.read_bit()
        self.write(not ack)
        return byte

    def poll(self, addr):
        self.start_cond()
        ack  = self.write(addr << 1 | 0)
        ack |= self.write(addr << 1 | 1)
        self.stop_cond()
        return ack

# I2C Scan Test ------------------------------------------------------------------------------------

def i2c_scan_test():
    bus = RemoteClient()
    bus.open()

    i2c = BitBangI2C(bus.regs)

    print("       0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f", end="");
    sys.stdout.flush()
    for addr in range(0, 0x80):
        if (addr % 0x10) == 0:
            print(f"\n0x{addr:02x}", end="")
        if i2c.poll(addr):
            print(f" {addr:02x}", end="")
        else:
            print(f" --", end="")
        sys.stdout.flush()
    print("")

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="I2C test utility")
    parser.add_argument("--scan", action="store_true", help="Scan I2C Bus.")
    args = parser.parse_args()

    if args.scan:
        i2c_scan_test()

if __name__ == "__main__":
    main()