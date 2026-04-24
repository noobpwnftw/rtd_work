#!/usr/bin/env python3
"""
Read EDID from a monitor via Realtek USB-I2C adapter (FX2LP-based).

The adapter must have OpenRTD firmware loaded and be connected
to the monitor's DDC pins (HDMI pin 15=SCL, pin 16=SDA).
"""

from rtd_i2c import RealtekI2C

EDID_ADDR = 0xA0  # EDID EEPROM write address (0x50 << 1)


def parse_edid(data):
    """Parse and display basic EDID 1.x fields."""
    if data[:8] != bytes([0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00]):
        print("  WARNING: invalid EDID header")
        return

    mfr_id = (data[8] << 8) | data[9]
    c1 = ((mfr_id >> 10) & 0x1F) + ord('A') - 1
    c2 = ((mfr_id >> 5) & 0x1F) + ord('A') - 1
    c3 = (mfr_id & 0x1F) + ord('A') - 1
    print(f"  Manufacturer: {chr(c1)}{chr(c2)}{chr(c3)}")
    print(f"  Product code: 0x{data[10] | (data[11] << 8):04X}")
    print(f"  Serial: {data[12] | (data[13] << 8) | (data[14] << 16) | (data[15] << 24)}")
    print(f"  Year: {data[17] + 1990}, Week: {data[16]}")
    print(f"  EDID version: {data[18]}.{data[19]}")

    for i in range(4):
        block = data[54 + i * 18 : 54 + (i + 1) * 18]
        if block[0:3] == bytes([0, 0, 0]) and block[3] == 0xFC:
            name = block[5:].split(b'\x0a')[0].decode('ascii', errors='replace').strip()
            print(f"  Name: {name}")

    if data[126] > 0:
        print(f"  Extension blocks: {data[126]}")


def find_hf_eeodb(ext):
    """Return HF-EEODB override count from a CTA extension block, or None."""
    if len(ext) < 128 or ext[0] != 0x02:
        return None
    dtd_offset = ext[2]
    if dtd_offset <= 4 or dtd_offset > 127:
        return None
    i = 4
    while i < dtd_offset:
        tag = (ext[i] >> 5) & 0x07
        length = ext[i] & 0x1F
        if length == 0 or i + 1 + length > dtd_offset:
            break
        if tag == 0x07 and length >= 2 and ext[i + 1] == 0x78:
            return ext[i + 2]
        i += 1 + length
    return None


def main():
    dev = RealtekI2C()
    try:
        print("Reading EDID block 0...")
        edid = dev.read(EDID_ADDR, 0x00, 128)
        print(f"  {edid.hex()}")
        print()
        parse_edid(edid)

        blocks = [edid]
        num_ext = edid[126]
        blk = 1
        while blk <= num_ext:
            print(f"\nReading extension block {blk}...")
            offset = (blk * 128) % 256
            segment = (blk * 128) // 256
            if segment == 0:
                ext = dev.read(EDID_ADDR, offset, 128)
            else:
                ext = dev.eddc_read(segment, EDID_ADDR, offset, 128)
            print(f"  {ext.hex()}")
            blocks.append(ext)

            if blk == 1:
                override = find_hf_eeodb(ext)
                if override is not None and override != num_ext:
                    print(f"  HF-EEODB override: extension block count = {override} (was {num_ext})")
                    num_ext = override
            blk += 1

    except IOError as e:
        print(f"Error: {e}")
    finally:
        dev.close()


if __name__ == "__main__":
    main()
