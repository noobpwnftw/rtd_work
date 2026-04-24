#!/usr/bin/env python3
"""Simple 8051 disassembler for FX2 firmware analysis."""
import sys

# 8051 instruction table: (mnemonic, size, operand_format)
# operand_format: None, 'addr11', 'addr16', 'rel', 'direct', 'imm', 'bit', etc.
OPCODES = {
    0x00: ("NOP", 1),
    0x01: ("AJMP", 2, 'addr11'), 0x02: ("LJMP", 3, 'addr16'), 0x03: ("RR", 1), 0x04: ("INC", 1, None, "A"),
    0x05: ("INC", 2, 'direct'), 0x06: ("INC", 1, None, "@R0"), 0x07: ("INC", 1, None, "@R1"),
    0x08: ("INC", 1, None, "R0"), 0x09: ("INC", 1, None, "R1"), 0x0A: ("INC", 1, None, "R2"),
    0x0B: ("INC", 1, None, "R3"), 0x0C: ("INC", 1, None, "R4"), 0x0D: ("INC", 1, None, "R5"),
    0x0E: ("INC", 1, None, "R6"), 0x0F: ("INC", 1, None, "R7"),
    0x10: ("JBC", 3, 'bit_rel'), 0x11: ("ACALL", 2, 'addr11'), 0x12: ("LCALL", 3, 'addr16'),
    0x13: ("RRC", 1), 0x14: ("DEC", 1, None, "A"),
    0x15: ("DEC", 2, 'direct'), 0x16: ("DEC", 1, None, "@R0"), 0x17: ("DEC", 1, None, "@R1"),
    0x18: ("DEC", 1, None, "R0"), 0x19: ("DEC", 1, None, "R1"), 0x1A: ("DEC", 1, None, "R2"),
    0x1B: ("DEC", 1, None, "R3"), 0x1C: ("DEC", 1, None, "R4"), 0x1D: ("DEC", 1, None, "R5"),
    0x1E: ("DEC", 1, None, "R6"), 0x1F: ("DEC", 1, None, "R7"),
    0x20: ("JB", 3, 'bit_rel'), 0x21: ("AJMP", 2, 'addr11'), 0x22: ("RET", 1), 0x23: ("RL", 1),
    0x24: ("ADD", 2, 'imm', "A,"), 0x25: ("ADD", 2, 'direct', "A,"),
    0x26: ("ADD", 1, None, "A,@R0"), 0x27: ("ADD", 1, None, "A,@R1"),
    0x28: ("ADD", 1, None, "A,R0"), 0x29: ("ADD", 1, None, "A,R1"), 0x2A: ("ADD", 1, None, "A,R2"),
    0x2B: ("ADD", 1, None, "A,R3"), 0x2C: ("ADD", 1, None, "A,R4"), 0x2D: ("ADD", 1, None, "A,R5"),
    0x2E: ("ADD", 1, None, "A,R6"), 0x2F: ("ADD", 1, None, "A,R7"),
    0x30: ("JNB", 3, 'bit_rel'), 0x31: ("ACALL", 2, 'addr11'), 0x32: ("RETI", 1), 0x33: ("RLC", 1),
    0x34: ("ADDC", 2, 'imm', "A,"), 0x35: ("ADDC", 2, 'direct', "A,"),
    0x36: ("ADDC", 1, None, "A,@R0"), 0x37: ("ADDC", 1, None, "A,@R1"),
    0x38: ("ADDC", 1, None, "A,R0"), 0x39: ("ADDC", 1, None, "A,R1"), 0x3A: ("ADDC", 1, None, "A,R2"),
    0x3B: ("ADDC", 1, None, "A,R3"), 0x3C: ("ADDC", 1, None, "A,R4"), 0x3D: ("ADDC", 1, None, "A,R5"),
    0x3E: ("ADDC", 1, None, "A,R6"), 0x3F: ("ADDC", 1, None, "A,R7"),
    0x40: ("JC", 2, 'rel'), 0x41: ("AJMP", 2, 'addr11'), 0x42: ("ORL", 2, 'direct', None, ",A"),
    0x43: ("ORL", 3, 'direct_imm'), 0x44: ("ORL", 2, 'imm', "A,"),
    0x45: ("ORL", 2, 'direct', "A,"),
    0x46: ("ORL", 1, None, "A,@R0"), 0x47: ("ORL", 1, None, "A,@R1"),
    0x48: ("ORL", 1, None, "A,R0"), 0x49: ("ORL", 1, None, "A,R1"), 0x4A: ("ORL", 1, None, "A,R2"),
    0x4B: ("ORL", 1, None, "A,R3"), 0x4C: ("ORL", 1, None, "A,R4"), 0x4D: ("ORL", 1, None, "A,R5"),
    0x4E: ("ORL", 1, None, "A,R6"), 0x4F: ("ORL", 1, None, "A,R7"),
    0x50: ("JNC", 2, 'rel'), 0x51: ("ACALL", 2, 'addr11'), 0x52: ("ANL", 2, 'direct', None, ",A"),
    0x53: ("ANL", 3, 'direct_imm'), 0x54: ("ANL", 2, 'imm', "A,"),
    0x55: ("ANL", 2, 'direct', "A,"),
    0x56: ("ANL", 1, None, "A,@R0"), 0x57: ("ANL", 1, None, "A,@R1"),
    0x58: ("ANL", 1, None, "A,R0"), 0x59: ("ANL", 1, None, "A,R1"), 0x5A: ("ANL", 1, None, "A,R2"),
    0x5B: ("ANL", 1, None, "A,R3"), 0x5C: ("ANL", 1, None, "A,R4"), 0x5D: ("ANL", 1, None, "A,R5"),
    0x5E: ("ANL", 1, None, "A,R6"), 0x5F: ("ANL", 1, None, "A,R7"),
    0x60: ("JZ", 2, 'rel'), 0x61: ("AJMP", 2, 'addr11'), 0x62: ("XRL", 2, 'direct', None, ",A"),
    0x63: ("XRL", 3, 'direct_imm'), 0x64: ("XRL", 2, 'imm', "A,"),
    0x65: ("XRL", 2, 'direct', "A,"),
    0x66: ("XRL", 1, None, "A,@R0"), 0x67: ("XRL", 1, None, "A,@R1"),
    0x68: ("XRL", 1, None, "A,R0"), 0x69: ("XRL", 1, None, "A,R1"), 0x6A: ("XRL", 1, None, "A,R2"),
    0x6B: ("XRL", 1, None, "A,R3"), 0x6C: ("XRL", 1, None, "A,R4"), 0x6D: ("XRL", 1, None, "A,R5"),
    0x6E: ("XRL", 1, None, "A,R6"), 0x6F: ("XRL", 1, None, "A,R7"),
    0x70: ("JNZ", 2, 'rel'), 0x71: ("ACALL", 2, 'addr11'), 0x72: ("ORL", 2, 'bit', "C,"),
    0x73: ("JMP", 1, None, "@A+DPTR"),
    0x74: ("MOV", 2, 'imm', "A,"), 0x75: ("MOV", 3, 'direct_imm'),
    0x76: ("MOV", 2, 'imm', "@R0,"), 0x77: ("MOV", 2, 'imm', "@R1,"),
    0x78: ("MOV", 2, 'imm', "R0,"), 0x79: ("MOV", 2, 'imm', "R1,"), 0x7A: ("MOV", 2, 'imm', "R2,"),
    0x7B: ("MOV", 2, 'imm', "R3,"), 0x7C: ("MOV", 2, 'imm', "R4,"), 0x7D: ("MOV", 2, 'imm', "R5,"),
    0x7E: ("MOV", 2, 'imm', "R6,"), 0x7F: ("MOV", 2, 'imm', "R7,"),
    0x80: ("SJMP", 2, 'rel'), 0x81: ("AJMP", 2, 'addr11'), 0x82: ("ANL", 2, 'bit', "C,"),
    0x83: ("MOVC", 1, None, "A,@A+PC"),
    0x84: ("DIV", 1, None, "AB"), 0x85: ("MOV", 3, 'direct_direct'),
    0x86: ("MOV", 2, 'direct', None, ",@R0"), 0x87: ("MOV", 2, 'direct', None, ",@R1"),
    0x88: ("MOV", 2, 'direct', None, ",R0"), 0x89: ("MOV", 2, 'direct', None, ",R1"),
    0x8A: ("MOV", 2, 'direct', None, ",R2"), 0x8B: ("MOV", 2, 'direct', None, ",R3"),
    0x8C: ("MOV", 2, 'direct', None, ",R4"), 0x8D: ("MOV", 2, 'direct', None, ",R5"),
    0x8E: ("MOV", 2, 'direct', None, ",R6"), 0x8F: ("MOV", 2, 'direct', None, ",R7"),
    0x90: ("MOV", 3, 'addr16', "DPTR,"), 0x91: ("ACALL", 2, 'addr11'),
    0x92: ("MOV", 2, 'bit', None, ",C"), 0x93: ("MOVC", 1, None, "A,@A+DPTR"),
    0x94: ("SUBB", 2, 'imm', "A,"), 0x95: ("SUBB", 2, 'direct', "A,"),
    0x96: ("SUBB", 1, None, "A,@R0"), 0x97: ("SUBB", 1, None, "A,@R1"),
    0x98: ("SUBB", 1, None, "A,R0"), 0x99: ("SUBB", 1, None, "A,R1"), 0x9A: ("SUBB", 1, None, "A,R2"),
    0x9B: ("SUBB", 1, None, "A,R3"), 0x9C: ("SUBB", 1, None, "A,R4"), 0x9D: ("SUBB", 1, None, "A,R5"),
    0x9E: ("SUBB", 1, None, "A,R6"), 0x9F: ("SUBB", 1, None, "A,R7"),
    0xA0: ("ORL", 2, 'bit', "C,/"), 0xA1: ("AJMP", 2, 'addr11'), 0xA2: ("MOV", 2, 'bit', "C,"),
    0xA3: ("INC", 1, None, "DPTR"), 0xA4: ("MUL", 1, None, "AB"),
    0xA5: ("DB", 1), # reserved
    0xA6: ("MOV", 2, 'direct', "@R0,"), 0xA7: ("MOV", 2, 'direct', "@R1,"),
    0xA8: ("MOV", 2, 'direct', "R0,"), 0xA9: ("MOV", 2, 'direct', "R1,"), 0xAA: ("MOV", 2, 'direct', "R2,"),
    0xAB: ("MOV", 2, 'direct', "R3,"), 0xAC: ("MOV", 2, 'direct', "R4,"), 0xAD: ("MOV", 2, 'direct', "R5,"),
    0xAE: ("MOV", 2, 'direct', "R6,"), 0xAF: ("MOV", 2, 'direct', "R7,"),
    0xB0: ("ANL", 2, 'bit', "C,/"), 0xB1: ("ACALL", 2, 'addr11'),
    0xB2: ("CPL", 2, 'bit'), 0xB3: ("CPL", 1, None, "C"),
    0xB4: ("CJNE", 3, 'imm_rel', "A,"), 0xB5: ("CJNE", 3, 'direct_rel', "A,"),
    0xB6: ("CJNE", 3, 'imm_rel', "@R0,"), 0xB7: ("CJNE", 3, 'imm_rel', "@R1,"),
    0xB8: ("CJNE", 3, 'imm_rel', "R0,"), 0xB9: ("CJNE", 3, 'imm_rel', "R1,"),
    0xBA: ("CJNE", 3, 'imm_rel', "R2,"), 0xBB: ("CJNE", 3, 'imm_rel', "R3,"),
    0xBC: ("CJNE", 3, 'imm_rel', "R4,"), 0xBD: ("CJNE", 3, 'imm_rel', "R5,"),
    0xBE: ("CJNE", 3, 'imm_rel', "R6,"), 0xBF: ("CJNE", 3, 'imm_rel', "R7,"),
    0xC0: ("PUSH", 2, 'direct'), 0xC1: ("AJMP", 2, 'addr11'), 0xC2: ("CLR", 2, 'bit'),
    0xC3: ("CLR", 1, None, "C"), 0xC4: ("SWAP", 1, None, "A"),
    0xC5: ("XCH", 2, 'direct', "A,"), 0xC6: ("XCH", 1, None, "A,@R0"), 0xC7: ("XCH", 1, None, "A,@R1"),
    0xC8: ("XCH", 1, None, "A,R0"), 0xC9: ("XCH", 1, None, "A,R1"), 0xCA: ("XCH", 1, None, "A,R2"),
    0xCB: ("XCH", 1, None, "A,R3"), 0xCC: ("XCH", 1, None, "A,R4"), 0xCD: ("XCH", 1, None, "A,R5"),
    0xCE: ("XCH", 1, None, "A,R6"), 0xCF: ("XCH", 1, None, "A,R7"),
    0xD0: ("POP", 2, 'direct'), 0xD1: ("ACALL", 2, 'addr11'), 0xD2: ("SETB", 2, 'bit'),
    0xD3: ("SETB", 1, None, "C"), 0xD4: ("DA", 1, None, "A"),
    0xD5: ("DJNZ", 3, 'direct_rel'), 0xD6: ("XCHD", 1, None, "A,@R0"), 0xD7: ("XCHD", 1, None, "A,@R1"),
    0xD8: ("DJNZ", 2, 'rel', "R0,"), 0xD9: ("DJNZ", 2, 'rel', "R1,"), 0xDA: ("DJNZ", 2, 'rel', "R2,"),
    0xDB: ("DJNZ", 2, 'rel', "R3,"), 0xDC: ("DJNZ", 2, 'rel', "R4,"), 0xDD: ("DJNZ", 2, 'rel', "R5,"),
    0xDE: ("DJNZ", 2, 'rel', "R6,"), 0xDF: ("DJNZ", 2, 'rel', "R7,"),
    0xE0: ("MOVX", 1, None, "A,@DPTR"), 0xE1: ("AJMP", 2, 'addr11'),
    0xE2: ("MOVX", 1, None, "A,@R0"), 0xE3: ("MOVX", 1, None, "A,@R1"),
    0xE4: ("CLR", 1, None, "A"), 0xE5: ("MOV", 2, 'direct', "A,"),
    0xE6: ("MOV", 1, None, "A,@R0"), 0xE7: ("MOV", 1, None, "A,@R1"),
    0xE8: ("MOV", 1, None, "A,R0"), 0xE9: ("MOV", 1, None, "A,R1"), 0xEA: ("MOV", 1, None, "A,R2"),
    0xEB: ("MOV", 1, None, "A,R3"), 0xEC: ("MOV", 1, None, "A,R4"), 0xED: ("MOV", 1, None, "A,R5"),
    0xEE: ("MOV", 1, None, "A,R6"), 0xEF: ("MOV", 1, None, "A,R7"),
    0xF0: ("MOVX", 1, None, "@DPTR,A"), 0xF1: ("ACALL", 2, 'addr11'),
    0xF2: ("MOVX", 1, None, "@R0,A"), 0xF3: ("MOVX", 1, None, "@R1,A"),
    0xF4: ("CPL", 1, None, "A"), 0xF5: ("MOV", 2, 'direct', None, ",A"),
    0xF6: ("MOV", 1, None, "@R0,A"), 0xF7: ("MOV", 1, None, "@R1,A"),
    0xF8: ("MOV", 1, None, "R0,A"), 0xF9: ("MOV", 1, None, "R1,A"), 0xFA: ("MOV", 1, None, "R2,A"),
    0xFB: ("MOV", 1, None, "R3,A"), 0xFC: ("MOV", 1, None, "R4,A"), 0xFD: ("MOV", 1, None, "R5,A"),
    0xFE: ("MOV", 1, None, "R6,A"), 0xFF: ("MOV", 1, None, "R7,A"),
}

# FX2 SFR names
SFR = {
    0x80: "P0", 0x81: "SP", 0x82: "DPL", 0x83: "DPH", 0x84: "DPL1", 0x85: "DPH1",
    0x86: "DPS", 0x87: "PCON", 0x88: "TCON", 0x89: "TMOD", 0x8A: "TL0", 0x8B: "TL1",
    0x8C: "TH0", 0x8D: "TH1", 0x8E: "CKCON", 0x90: "P1", 0x91: "EXIF", 0x92: "MPAGE",
    0x98: "SCON0", 0x99: "SBUF0", 0xA0: "P2", 0xA8: "IE", 0xB0: "P3", 0xB8: "IP",
    0xC0: "SCON1", 0xC1: "SBUF1", 0xD0: "PSW", 0xD8: "EICON", 0xE0: "ACC", 0xE8: "EIE",
    0xF0: "B", 0xF8: "EIP",
    # FX2 specific
    0x8F: "SPC_FNC", 0x9A: "AUTOPTRH1", 0x9B: "AUTOPTRL1",
    0x9D: "AUTOPTRH2", 0x9E: "AUTOPTRL2",
}

def sfr_name(addr):
    if addr in SFR:
        return SFR[addr]
    return "0x%02X" % addr

def bit_name(addr):
    if addr >= 0x80:
        base = addr & 0xF8
        bit = addr & 0x07
        if base in SFR:
            return "%s.%d" % (SFR[base], bit)
    return "0x%02X" % addr

def disassemble(data, base_addr=0):
    pc = 0
    lines = []
    while pc < len(data):
        addr = base_addr + pc
        opcode = data[pc]
        entry = OPCODES.get(opcode)
        if entry is None:
            lines.append("%04X:  %02X           DB 0x%02X" % (addr, opcode, opcode))
            pc += 1
            continue

        mnem = entry[0]
        size = entry[1]
        fmt = entry[2] if len(entry) > 2 else None
        prefix = entry[3] if len(entry) > 3 and entry[3] is not None else ""
        suffix = entry[4] if len(entry) > 4 and entry[4] is not None else ""

        raw = data[pc:pc+size]
        if len(raw) < size:
            lines.append("%04X:  %s  DB %s" % (addr, ' '.join('%02X' % b for b in raw),
                         ','.join('0x%02X' % b for b in raw)))
            break

        hex_str = ' '.join('%02X' % b for b in raw)

        if fmt is None:
            operand = prefix if prefix else ""
        elif fmt == 'addr11':
            a = ((opcode & 0xE0) << 3) | raw[1]
            page = (addr + size) & 0xF800
            target = page | a
            operand = "0x%04X" % target
        elif fmt == 'addr16':
            target = (raw[1] << 8) | raw[2]
            operand = "%s#0x%04X" % (prefix, target)
        elif fmt == 'rel':
            offset = raw[1] if raw[1] < 128 else raw[1] - 256
            target = addr + size + offset
            operand = "%s0x%04X" % (prefix, target)
        elif fmt == 'direct':
            d = raw[1]
            operand = "%s%s%s" % (prefix, sfr_name(d), suffix)
        elif fmt == 'imm':
            operand = "%s#0x%02X" % (prefix, raw[1])
        elif fmt == 'bit':
            operand = "%s%s%s" % (prefix, bit_name(raw[1]), suffix)
        elif fmt == 'bit_rel':
            offset = raw[2] if raw[2] < 128 else raw[2] - 256
            target = addr + size + offset
            operand = "%s,%s,0x%04X" % (prefix if prefix else "", bit_name(raw[1]), target)
        elif fmt == 'direct_imm':
            operand = "%s,#0x%02X" % (sfr_name(raw[1]), raw[2])
        elif fmt == 'direct_direct':
            operand = "%s,%s" % (sfr_name(raw[2]), sfr_name(raw[1]))
        elif fmt == 'imm_rel':
            offset = raw[2] if raw[2] < 128 else raw[2] - 256
            target = addr + size + offset
            operand = "%s#0x%02X,0x%04X" % (prefix, raw[1], target)
        elif fmt == 'direct_rel':
            offset = raw[2] if raw[2] < 128 else raw[2] - 256
            target = addr + size + offset
            operand = "%s%s,0x%04X" % (prefix, sfr_name(raw[1]), target)
        else:
            operand = ""

        lines.append("%04X:  %-12s %s %s" % (addr, hex_str, mnem, operand))
        pc += size

    return '\n'.join(lines)


if __name__ == '__main__':
    fname = sys.argv[1] if len(sys.argv) > 1 else 'realsil.bin'

    with open(fname, 'rb') as f:
        data = f.read()

    print("; Disassembly of %s (%d bytes)" % (fname, len(data)))
    print("; " + "=" * 60)
    print(disassemble(data))
