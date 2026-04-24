#!/usr/bin/env python3
"""Scan I2C bus for responding slave addresses."""

from rtd_i2c import RealtekI2C

i2c = RealtekI2C()

print("Scanning I2C bus (8-bit write addresses)...")
found = []
for addr in range(0x00, 0xFF, 2):
    try:
        i2c.read(addr, 0x00, 1)[0]
        found.append(addr)
        print(f"  0x{addr:02X} (7-bit 0x{addr>>1:02X}) ACK")
    except IOError:
        pass

i2c.close()
print(f"\n{len(found)} devices found.")
