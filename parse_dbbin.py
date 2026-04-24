#!/usr/bin/env python3
"""
RTDCustomerTool DB.bin Parser

Parses the register database from Realtek's RTDCustomerTool into
machine-readable (.json) register maps.

DB.bin file format (from PlugIn_NewCtrlReg.dll reverse engineering):
  Header:  magic(4B) + version(4B) + sect1_size(4B)
  Section 1 (offset 0x0C): 256 x 4B LE page offset table
  [4B LE sect2_size]
  Section 2: binary register records
  [4B LE unknown]
  Section 3: null-terminated ASCII string table

  Page offsets in Section 1 are relative to the start of Section 2.
  Multiple pages may share the same offset (aliased register blocks).

  Record: 0x24-byte header + field_count * 0x24-byte sub-field entries

  Header layout:
    +0x00: LE dword  — flags/properties
    +0x04: LE dword  — register name string offset (into Section 3)
    +0x08: LE word   — scaler address (page_hi:reg_lo)
    +0x0A: LE word   — (copy of scaler address)
    +0x20: byte      — field_count (number of sub-field entries)

  Sub-field layout (0x24 bytes each):
    +0x00: LE dword  — field name string offset
    +0x04: LE sdword — combined high bit index (-1 if not split)
    +0x08: LE dword  — combined low bit index
    +0x0C: byte      — bit_start (MSB of range within this byte)
    +0x0D: byte      — bit_end   (LSB of range within this byte)
    +0x10: LE dword  — access (1=R, 3=RW)
    +0x18: LE dword  — description string offset
    +0x1C: LE dword  — signal/reset string offset

  Port-indexed registers: when multiple records share the same scaler
  address, the first is the physical port data register; subsequent
  records are the virtual sub-registers accessed through that port.
  Their sequential order matches the port index.

Usage:
    python3 parse_dbbin.py <chip_dir>
    python3 parse_dbbin.py /path/to/<chip_dir>/
    python3 parse_dbbin.py <chip_dir> --json out.json
"""
import struct
import json
import argparse
import sys
import os
from collections import defaultdict


def parse_dbbin(db_path):
    """Parse DB.bin register database. Returns (full_map, total_fields)."""
    db = open(db_path, 'rb').read()

    # Header
    magic = struct.unpack_from('<I', db, 0)[0]
    version = struct.unpack_from('<I', db, 4)[0]
    sect1_size = struct.unpack_from('<I', db, 8)[0]

    # Section boundaries
    sect2_size = struct.unpack_from('<I', db, 0x0C + sect1_size)[0]
    sect2 = 0x0C + sect1_size + 4
    sect3 = sect2 + sect2_size + 4

    print(f"DB.bin: magic=0x{magic:08X} version=0x{version:08X}", file=sys.stderr)
    print(f"  Section 1 (page offsets): 0x{0x0C:X}, {sect1_size} bytes", file=sys.stderr)
    print(f"  Section 2 (records):      0x{sect2:X}, {sect2_size} bytes", file=sys.stderr)
    print(f"  Section 3 (strings):      0x{sect3:X}, {len(db) - sect3} bytes", file=sys.stderr)

    def read_str(off):
        a = sect3 + off
        if a >= len(db) or off < 0:
            return ""
        e = db.find(b'\x00', a)
        if e <= a or e - a > 500:
            return ""
        try:
            return db[a:e].decode('ascii')
        except Exception:
            return ""

    # Record layout depends on version:
    #   v1: header 0x24, sub-field 0x24, field_count at +0x20
    #   v3: header 0x2C, sub-field 0x28, field_count at +0x28
    # Both versions store records flat/linearly in section 2.
    if version >= 3:
        hdr_size, sf_size, fc_off = 0x2C, 0x28, 0x28
        sf_desc, sf_signal = 0x1C, 0x20
    else:
        hdr_size, sf_size, fc_off = 0x24, 0x24, 0x20
        sf_desc, sf_signal = 0x18, 0x1C

    # Walk all records linearly
    full_map = {}
    addr_count = defaultdict(int)
    total_fields = 0
    pos = sect2
    end = sect2 + sect2_size

    while pos + hdr_size <= end:
        fc = db[pos + fc_off]
        rec_size = hdr_size + fc * sf_size
        if pos + rec_size > end:
            break

        reg_name = read_str(struct.unpack_from('<I', db, pos + 4)[0])
        sca_addr = struct.unpack_from('<H', db, pos + 8)[0]
        sca_page = (sca_addr >> 8) & 0xFF
        sca_reg = sca_addr & 0xFF

        fields = []
        for fi in range(fc):
            sf_pos = pos + hdr_size + fi * sf_size
            field_name = read_str(struct.unpack_from('<I', db, sf_pos)[0])
            bit_hi = struct.unpack_from('<i', db, sf_pos + 4)[0]
            bit_lo = struct.unpack_from('<I', db, sf_pos + 8)[0]
            bit_start = db[sf_pos + 0x0C]
            bit_end = db[sf_pos + 0x0D]
            access = struct.unpack_from('<I', db, sf_pos + 0x10)[0]
            desc = read_str(struct.unpack_from('<I', db, sf_pos + sf_desc)[0])
            signal = read_str(struct.unpack_from('<I', db, sf_pos + sf_signal)[0])

            bit_str = f"[{bit_start}]" if bit_start == bit_end else f"[{bit_start}:{bit_end}]"
            combined = f"{field_name}[{bit_hi}:{bit_lo}]" if bit_hi >= 0 else field_name
            access_str = "RW" if access == 3 else "R" if access == 1 else f"?{access}"

            fields.append({
                'name': field_name, 'combined': combined,
                'bit_start': bit_start, 'bit_end': bit_end, 'bit_str': bit_str,
                'access': access_str, 'description': desc, 'signal': signal,
            })

        if reg_name:
            pi = addr_count[(sca_page, sca_reg)]
            addr_count[(sca_page, sca_reg)] += 1
            full_map[(sca_page, sca_reg, reg_name)] = {
                'name': reg_name, 'page': sca_page, 'reg': sca_reg,
                'sca_addr': sca_addr, 'port_index': pi, 'fields': fields,
            }
            total_fields += len(fields)

        pos += rec_size

    print(f"  Parsed: {len(full_map)} registers, {total_fields} bitfields, "
          f"{len(set(p for p, _, _ in full_map))} pages", file=sys.stderr)
    return full_map, total_fields


def write_json(full_map, outpath):
    """Write machine-readable register map keyed by page:reg:name."""
    json_map = {}
    for key in sorted(full_map.keys()):
        reg = full_map[key]
        pg = reg['page']
        ra = reg['reg']
        jkey = f"P{pg:02X}:0x{ra:02X}:{reg['name']}"
        json_map[jkey] = {
            'page': pg,
            'reg': ra,
            'sca_addr': reg['sca_addr'],
            'port_index': reg.get('port_index', -1),
            'name': reg['name'],
            'fields': [{
                'name': f['name'],
                'combined': f['combined'],
                'bits': f['bit_str'],
                'bit_start': f['bit_start'],
                'bit_end': f['bit_end'],
                'access': f['access'],
                'description': f.get('description', ''),
                'signal': f.get('signal', ''),
            } for f in reg['fields']]
        }

    with open(outpath, 'w') as f:
        json.dump(json_map, f, indent=2)

    print(f"Wrote {outpath}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description="Parse RTDCustomerTool DB.bin register database",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)

    parser.add_argument('chip_dir',
                        help='Path to RTDCustomerTool chip directory')
    parser.add_argument('--json', '-j', type=str, default=None,
                        help='Output .json file')

    args = parser.parse_args()

    chip_dir = args.chip_dir.rstrip('/')
    db_path = os.path.join(chip_dir, 'DB.bin')

    if not os.path.exists(db_path):
        print(f"Error: {db_path} not found", file=sys.stderr)
        sys.exit(1)

    if not args.json:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        chip_name = os.path.basename(chip_dir).lower().replace('-', '_')
        args.json = os.path.join(script_dir, f'{chip_name}_regmap.json')

    full_map, total_fields = parse_dbbin(db_path)
    write_json(full_map, args.json)


if __name__ == '__main__':
    main()
