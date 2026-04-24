#!/usr/bin/env python3
"""
RTD 8051 MCU Firmware Disassembler

Disassembler for RTD 8051 MCU banked firmware (up to 16 x 64K banks).
Produces reassemblable sdas8051 output — byte-identical when assembled.

State is stored in a persistent JSON database (<firmware>.rtddb.json).
Analysis and emission are strictly separate — emit only reads from db.

== Quick Start ==

  python3 rtd_dis.py fw.bin --analyze           # detect noreturn, analyze, solve dispatch
  python3 rtd_dis.py fw.bin --asm --all         # emit all banks to fw_asm/
  python3 rtd_dis.py fw.bin --asm --addr 0:0    # single bank to stdout

  # Reassemble (per bank):
  sdas8051 -o bank_00.asm
  sdld -n -b CABS=0x0000 -i bank_00.ihx bank_00.rel

== Database Commands ==

  --analyze           Recursive descent from interrupt vectors + cross-bank stubs
  --solve-dispatch    Parse noreturn dispatch tables, mark targets as code
  --mark-code B:A     Mark address as code entry (e.g. 3:0x5000), rollback on conflict
  --noreturn ADDR     Mark function as noreturn (e.g. 0x1aa3)
  --label B:A NAME    Add/update label (e.g. 0:0x1234 my_func)
  --search VALUE      Search code + data for value (decimal or 0x hex)
  --trace B:A         Trace how an address got marked as code
  --db-info           Show database statistics

== Listing Modes (human-readable, not reassemblable) ==

  --addr [B:]A      Disassemble at address (auto-detects function end, or use --end/--length)
  --map             Memory map overview
  --xrefs           Cross-references (from db if available)
  --calls           Inter-bank call map
  --strings         Extract strings
  --all             All banks
"""
import sys
import os
import json
import hashlib
import argparse
from collections import defaultdict

# =============================================================================
# RTD 8051 MCU SFR Map
# Direct-addressable 0x80-0xFF
# =============================================================================
SFR = {
    # Standard DW8051 SFRs
    0x80: "P0",    0x81: "SP",    0x82: "DPL0",  0x83: "DPH0",
    0x84: "DPL1",  0x85: "DPH1",  0x86: "DPS",   0x87: "PCON",
    0x88: "TCON",  0x89: "TMOD",  0x8A: "TL0",   0x8B: "TL1",
    0x8C: "TH0",   0x8D: "TH1",   0x8E: "CKCON",
    0x8F: "SPC_FNC",
    0x90: "P1",    0x91: "EXIF",  0x92: "MPAGE",
    0x98: "SCON0", 0x99: "SBUF0",
    0xA0: "P2",    0xA8: "IE",    0xAA: "P4",
    0xB0: "P3",    0xB8: "IP",
    0xC0: "SCON1", 0xC1: "SBUF1",
    0xC8: "T2CON", 0xCA: "RCAP2L", 0xCB: "RCAP2H",
    0xCC: "TL2",   0xCD: "TH2",
    0xD0: "PSW",   0xD8: "EICON",
    0xE0: "ACC",   0xE8: "EIE",
    0xF0: "B",     0xF8: "EIP",
}

# =============================================================================
# 8051 Bit-addressable SFR map
# =============================================================================
BIT_SFR = {
    0x80: "P0",   0x88: "TCON",  0x90: "P1",   0x98: "SCON0",
    0xA0: "P2",   0xA8: "IE",    0xB0: "P3",   0xB8: "IP",
    0xC0: "SCON1",0xC8: "T2CON", 0xD0: "PSW",  0xD8: "EICON",
    0xE0: "ACC",  0xE8: "EIE",   0xF0: "B",    0xF8: "EIP",
    0xAA: "P4",
}

# TCON bits
TCON_BITS = {0x88: "IT0", 0x89: "IE0", 0x8A: "IT1", 0x8B: "IE1",
             0x8C: "TR0", 0x8D: "TF0", 0x8E: "TR1", 0x8F: "TF1"}
# IE bits
IE_BITS = {0xA8: "EX0", 0xA9: "ET0", 0xAA: "EX1", 0xAB: "ET1",
           0xAC: "ES0", 0xAD: "ET2", 0xAE: "ES1", 0xAF: "EA"}
# PSW bits
PSW_BITS = {0xD0: "P", 0xD1: "F1", 0xD2: "OV", 0xD3: "RS0",
            0xD4: "RS1", 0xD5: "F0", 0xD6: "AC", 0xD7: "CY"}

NAMED_BITS = {}
NAMED_BITS.update(TCON_BITS)
NAMED_BITS.update(IE_BITS)
NAMED_BITS.update(PSW_BITS)

# =============================================================================
# RTD Register Map (from DB.bin parser)
# Loaded at runtime from rtd2795t_cg_regmap.json
# =============================================================================
REGMAP = {}  # {xdata_addr: [{"name": str, "sca_page": int, "sca_reg": int, ...}, ...]}

def load_regmap(path=None):
    """Load the parsed register map JSON into REGMAP global.
    With scalar_addr_remapping=1, XDATA addr = sca_page * 256 + sca_reg."""
    global REGMAP
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rtd2795t_cg_regmap.json')
    if not os.path.exists(path):
        print(f"; Warning: register map not found at {path}", file=sys.stderr)
        return 0
    with open(path) as f:
        raw = json.load(f)
    for key, info in raw.items():
        if info.get('port_index', -1) > 0:
            continue  # skip port-indexed sub-registers
        xdata = info['page'] * 256 + info['reg']
        if xdata not in REGMAP:
            REGMAP[xdata] = info
    return len(REGMAP)

# =============================================================================
# 8051 Instruction Table
# =============================================================================
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
    0xA5: ("DB", 1),
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

# =============================================================================
# Banking Constants
# =============================================================================
BANK_SIZE = 0x10000         # 64K per bank
NUM_BANKS = 16
TRAMPOLINE_LONG_SIG = bytes([0x54, 0x0F, 0xC4, 0xC0, 0xE0])  # ANL A,#0x0F; SWAP A; PUSH ACC
TRAMPOLINE_LONG_SIZE = 18   # Long-form trampoline size
TRAMPOLINE_SHORT_SIZE = 10  # Short-form trampoline size (some fw use for bank 0)

# 8051 interrupt vectors
INT_VECTORS = {
    0x0000: "RESET",
    0x0003: "INT0_ISR",
    0x000B: "TIMER0_ISR",
    0x0013: "INT1_ISR",
    0x001B: "TIMER1_ISR",
    0x0023: "SERIAL_ISR",
    0x002B: "TIMER2_ISR",
}


class RTDFirmware:
    """Represents the RTD 8051 MCU firmware image with banking support."""

    def __init__(self, filename):
        with open(filename, 'rb') as f:
            self.data = f.read()
        self.size = len(self.data)
        self.num_banks = self.size // BANK_SIZE
        self.filename = filename

        # Analysis results
        self.comments = {}          # addr -> comment
        self.functions = {}         # addr -> {bank, end, calls, callers, name}
        self.bank_calls = []        # (stub_addr, target_bank, target_func)
        self.jump_targets = set()   # addresses that are jump/call targets

        # Persistent database state (code/data classification + labels)
        self.code_addrs = defaultdict(dict)  # bank -> {addr: instr_size}
        self.asm_labels = {}                 # (bank, addr) -> label_name
        self.noreturn = set()                # addrs of functions that never return
        self.code_origin = defaultdict(dict) # bank -> {addr: [from_bank, from_addr]}
        self.callers = defaultdict(set)      # target_addr -> set of (bank, caller_addr)
        self._fw_sha256 = hashlib.sha256(self.data).hexdigest()

        # Auto-detect trampoline layout
        self._detect_trampolines()
        self._init_labels()

    def _detect_trampolines(self):
        """Auto-detect trampoline layout from firmware bytes.

        Finds all long-form trampolines (E5 xx 54 0F C4 ... 02 HH LL),
        derives handler base and bank mapping from their LJMP targets.
        Also detects optional short-form bank 0 trampoline.
        """
        d = self.data
        self._tramp_to_bank = {}

        # Find all long-form trampolines in the 0x0300-0x0600 region
        long_tramps = []  # (addr, handler_target)
        pos = 0x0300
        while pos < 0x0600:
            pos = d.find(TRAMPOLINE_LONG_SIG, pos, 0x0600)
            if pos < 0:
                break
            tramp_addr = pos - 2
            if d[tramp_addr] == 0xE5 and d[tramp_addr + 15] == 0x02:
                handler_target = (d[tramp_addr + 16] << 8) | d[tramp_addr + 17]
                long_tramps.append((tramp_addr, handler_target))
            pos += 1

        if not long_tramps:
            raise ValueError("Cannot find any trampoline signatures in firmware")

        # All handler targets share a base; bank = (target - base) / 0x10
        # The lowest handler target is bank 0's handler
        self._handler_base = min(t for _, t in long_tramps)

        for tramp_addr, handler_target in long_tramps:
            bank = (handler_target - self._handler_base) // 0x10
            if 0 <= bank < self.num_banks:
                self._tramp_to_bank[tramp_addr] = bank

        # Check for short-form bank 0 trampoline between long bank 0 and bank 1
        # Short form: 24 xx E0 C0 82 C0 83 02 HH LL (10 bytes)
        # It also targets the bank 0 handler
        bank0_long = [a for a, b in self._tramp_to_bank.items() if b == 0]
        bank1_long = [a for a, b in self._tramp_to_bank.items() if b == 1]
        if bank0_long and bank1_long:
            gap_start = bank0_long[0] + TRAMPOLINE_LONG_SIZE
            gap_end = bank1_long[0]
            if gap_end - gap_start == TRAMPOLINE_SHORT_SIZE:
                candidate = gap_start
                if (d[candidate] == 0x24 and d[candidate + 2] == 0xE0 and
                        d[candidate + 3] == 0xC0 and d[candidate + 4] == 0x82):
                    self._tramp_to_bank[candidate] = 0  # second bank 0 entry

        # Stub scan range: from after last trampoline to handler base
        last_tramp_addr = max(self._tramp_to_bank.keys())
        self._stub_start = last_tramp_addr + TRAMPOLINE_LONG_SIZE

    def _init_labels(self):
        """Set up initial labels from known addresses.
        Common area code may differ per bank, so we scan each bank's
        data independently and label only what matches."""
        for b in range(self.num_banks):
            bdata = self.get_bank_data(b)

            # Interrupt vectors
            for addr, name in INT_VECTORS.items():
                if addr < len(bdata):
                    op = bdata[addr]
                    if op == 0x02 and addr + 2 < len(bdata):
                        target = (bdata[addr+1] << 8) | bdata[addr+2]
                        self.asm_labels[(b, target)] = name
                        self.jump_targets.add(target)

            # Bank switch handlers
            for bank in range(self.num_banks):
                handler_addr = self._handler_base + bank * 0x10
                self.asm_labels[(b, handler_addr)] = f"__bank_switch_{bank}"

            # Bank call trampolines
            for addr, tramp_bank in self._tramp_to_bank.items():
                self.asm_labels[(b, addr)] = f"__trampoline_bank{tramp_bank}"

            # Cross-bank call stubs
            for i in range(self._stub_start, self._handler_base):
                if (bdata[i] == 0x90 and i + 5 < self._handler_base and
                        bdata[i+3] == 0x02):
                    target_addr = (bdata[i+1] << 8) | bdata[i+2]
                    tramp_addr = (bdata[i+4] << 8) | bdata[i+5]
                    bank = self._trampoline_to_bank(tramp_addr)
                    if bank >= 0:
                        if b == 0:
                            self.bank_calls.append((i, bank, target_addr))
                            # Register the resolved target as a function; the
                            # xcall stub at (0, i) is its cross-bank caller.
                            if target_addr not in self.functions:
                                self.functions[target_addr] = {
                                    'bank': bank,
                                    'callers': set(),
                                    'name': self.asm_labels.get(
                                        (bank, target_addr),
                                        f"sub_{target_addr:04X}"),
                                }
                            self.functions[target_addr]['callers'].add((0, i))
                        self.asm_labels[(b, i)] = f"__xcall_b{bank}_{target_addr:04X}"
                        self.jump_targets.add(i)

    def _trampoline_to_bank(self, tramp_addr):
        """Map a trampoline address to its bank number, or -1 if not a trampoline."""
        return self._tramp_to_bank.get(tramp_addr, -1)


    @property
    def db_path(self):
        """Path to the persistent database file."""
        base = os.path.splitext(self.filename)[0]
        return base + '.rtddb.json'

    def load_db(self):
        """Load persistent code/label database from JSON file."""
        if not os.path.exists(self.db_path):
            return False
        with open(self.db_path) as f:
            db = json.load(f)
        if db.get('firmware_sha256') != self._fw_sha256:
            print(f"; WARNING: db sha256 mismatch — firmware changed since last analyze", file=sys.stderr)
        # Load code addresses: stored as {bank_str: {addr_str: size}}
        for bank_str, addrs in db.get('code', {}).items():
            bank = int(bank_str)
            for addr_str, size in addrs.items():
                self.code_addrs[bank][int(addr_str)] = size
        # Load labels: stored as {bank_str: {addr_str: name}}
        for bank_str, addrs in db.get('labels', {}).items():
            bank = int(bank_str)
            if isinstance(addrs, dict):
                for addr_str, name in addrs.items():
                    self.asm_labels[(bank, int(addr_str))] = name
            else:
                # Legacy format: {addr_str: name} — treat as bank 0
                self.asm_labels[(0, int(bank_str))] = addrs
        # Load noreturn functions
        for addr in db.get('noreturn', []):
            self.noreturn.add(int(addr))
        # Load code origin chain
        for bank_str, origins in db.get('origin', {}).items():
            bank = int(bank_str)
            for addr_str, orig in origins.items():
                self.code_origin[bank][int(addr_str)] = orig
        # Load callers
        for target_str, refs in db.get('callers', {}).items():
            target = int(target_str)
            for ref in refs:
                self.callers[target].add(tuple(ref))
        n_code = sum(len(v) for v in self.code_addrs.values())
        print(f"; Loaded db: {n_code} instructions, {len(self.asm_labels)} labels, {len(self.noreturn)} noreturn", file=sys.stderr)
        return True

    def save_db(self):
        """Save persistent code/label database to JSON file."""
        db = {
            'firmware_sha256': self._fw_sha256,
            'code': {},
            'labels': {},
            'noreturn': sorted(self.noreturn),
            'origin': {},
        }
        for bank, addrs in self.code_addrs.items():
            db['code'][str(bank)] = {str(a): s for a, s in sorted(addrs.items())}
        labels_by_bank = defaultdict(dict)
        for (bank, addr), name in sorted(self.asm_labels.items()):
            labels_by_bank[bank][str(addr)] = name
        for bank, addrs in labels_by_bank.items():
            db['labels'][str(bank)] = addrs
        for bank, origins in self.code_origin.items():
            db['origin'][str(bank)] = {str(a): o for a, o in sorted(origins.items())}
        db['callers'] = {str(t): sorted(list(refs)) for t, refs in sorted(self.callers.items())}
        with open(self.db_path, 'w') as f:
            json.dump(db, f, separators=(',', ':'))
        n_code = sum(len(v) for v in self.code_addrs.values())
        print(f"; Saved db: {n_code} instructions, {len(self.asm_labels)} labels -> {self.db_path}", file=sys.stderr)

    def get_byte(self, bank, addr):
        """Read a byte from a specific bank at a 16-bit address."""
        offset = bank * BANK_SIZE + addr
        if 0 <= offset < self.size:
            return self.data[offset]
        return 0xFF

    def get_bank_data(self, bank):
        """Get the full 64K for a bank from flash."""
        base = bank * BANK_SIZE
        if base + BANK_SIZE <= self.size:
            return self.data[base:base + BANK_SIZE]
        return b'\xFF' * BANK_SIZE

    def flash_offset(self, bank, addr):
        """Convert bank:addr to flash file offset."""
        return bank * BANK_SIZE + addr

    def mark_code(self):
        """Recursive descent code marking — distinguishes code from data."""
        self.code_addrs = defaultdict(dict)  # bank -> {addr: instr_size}
        self.code_origin = defaultdict(dict) # bank -> {addr: [from_bank, from_addr]}
        # worklist: (bank, addr, from_bank, from_addr)
        # from_addr=-1 means seed entry point
        worklist = set()

        # Seed: interrupt vectors
        for vec_addr in INT_VECTORS:
            worklist.add((0, vec_addr, -1, vec_addr))
            if vec_addr + 2 < self.size:
                op = self.data[vec_addr]
                if op == 0x02:
                    target = (self.data[vec_addr + 1] << 8) | self.data[vec_addr + 2]
                    worklist.add((0, target, 0, vec_addr))

        # Seed: cross-bank call stubs and their targets
        for stub_addr, target_bank, target_func in self.bank_calls:
            worklist.add((0, stub_addr, -1, stub_addr))
            worklist.add((target_bank, target_func, 0, stub_addr))

        while worklist:
            bank, start, from_bank, from_addr = worklist.pop()
            bank_data = self.get_bank_data(bank)
            pc = start
            prev_pc = from_addr  # tracks the "reason" for each instruction

            while 0 <= pc < len(bank_data):
                if pc in self.code_addrs[bank]:
                    break

                opcode = bank_data[pc]
                entry = OPCODES.get(opcode)
                if entry is None:
                    break

                mnem = entry[0]
                size = entry[1]
                fmt = entry[2] if len(entry) > 2 else None

                raw = bank_data[pc:pc + size]
                if len(raw) < size:
                    break

                # Check overlap before committing
                overlap = False
                for j in range(1, 4):
                    prev = pc - j
                    if prev in self.code_addrs.get(bank, {}) and prev + self.code_addrs[bank][prev] > pc:
                        overlap = True
                        break
                for j in range(1, size):
                    if (pc + j) in self.code_addrs.get(bank, {}):
                        overlap = True
                        break
                if overlap:
                    break

                # Compute branch/call target before committing
                target = None
                if fmt == 'addr16' and mnem in ('LJMP', 'LCALL'):
                    target = (raw[1] << 8) | raw[2]
                elif fmt == 'addr11':
                    a = ((opcode & 0xE0) << 3) | raw[1]
                    page = (pc + size) & 0xF800
                    target = page | a
                elif fmt == 'rel':
                    offset = raw[1] if raw[1] < 128 else raw[1] - 256
                    target = pc + size + offset
                elif fmt in ('bit_rel', 'imm_rel', 'direct_rel'):
                    offset = raw[-1] if raw[-1] < 128 else raw[-1] - 256
                    target = pc + size + offset

                # Check target doesn't land inside an existing instruction
                if target is not None and 0 <= target < BANK_SIZE:
                    for j in range(1, 4):
                        prev = target - j
                        if prev in self.code_addrs.get(bank, {}) and prev + self.code_addrs[bank][prev] > target:
                            overlap = True
                            break
                if overlap:
                    break

                self.code_addrs[bank][pc] = size
                if pc == start:
                    self.code_origin[bank][pc] = [from_bank, from_addr]
                else:
                    self.code_origin[bank][pc] = [bank, prev_pc]

                if target is not None:
                    self.callers[target].add((bank, pc))

                prev_pc = pc

                if mnem in ('RET', 'RETI'):
                    break
                elif opcode == 0x73:  # JMP @A+DPTR
                    break
                elif mnem in ('LJMP', 'SJMP', 'AJMP'):
                    if target is not None:
                        tb = bank
                        worklist.add((tb, target, bank, pc))
                    break
                elif mnem in ('LCALL', 'ACALL'):
                    if target is not None:
                        tb = bank
                        worklist.add((tb, target, bank, pc))
                        if target in self.noreturn:
                            break
                    pc += size
                elif target is not None:
                    tb = bank
                    worklist.add((tb, target, bank, pc))
                    pc += size
                else:
                    pc += size

    def cleanup_code(self):
        """Remove committed instructions whose branch targets aren't valid code starts.
        Repeats until clean."""
        while True:
            to_remove = []
            for bank, code in self.code_addrs.items():
                bdata = self.get_bank_data(bank)
                for addr, size in list(code.items()):
                    raw = bdata[addr:addr + size]
                    if len(raw) < size:
                        continue
                    opcode = raw[0]
                    entry = OPCODES.get(opcode)
                    if entry is None:
                        continue
                    mnem = entry[0]
                    fmt = entry[2] if len(entry) > 2 else None
                    target = None
                    if fmt == 'addr16' and mnem in ('LJMP', 'LCALL'):
                        target = (raw[1] << 8) | raw[2]
                    elif fmt == 'addr11':
                        a = ((opcode & 0xE0) << 3) | raw[1]
                        page = (addr + size) & 0xF800
                        target = page | a
                    elif fmt == 'rel':
                        offset = raw[1] if raw[1] < 128 else raw[1] - 256
                        target = addr + size + offset
                    elif fmt in ('bit_rel', 'imm_rel', 'direct_rel'):
                        offset = raw[-1] if raw[-1] < 128 else raw[-1] - 256
                        target = addr + size + offset
                    if target is None:
                        continue
                    if 0 <= target < BANK_SIZE and target not in code:
                        to_remove.append((bank, addr))
            if not to_remove:
                break
            for bank, addr in to_remove:
                del self.code_addrs[bank][addr]
                self.code_origin[bank].pop(addr, None)
            print(f"; Cleanup: removed {len(to_remove)} instructions with invalid branch targets", file=sys.stderr)

    def build_asm_labels(self):
        """Build label table for assembler output. Labels are per-bank."""
        # Auto-detected labels already in asm_labels from _init_labels
        # Create labels for all addresses that are in code_addrs and have callers
        for addr, refs in self.callers.items():
            for bank in self.code_addrs:
                if addr in self.code_addrs[bank] and (bank, addr) not in self.asm_labels:
                    self.asm_labels[(bank, addr)] = f"loc_{addr:04x}"

    def try_mark_code(self, bank, addr, origin_tag=-2):
        """Try to mark addr as a code entry point. Traces recursively.
        Rejects on conflict only (overlap with existing code).
        Returns True if accepted, False if rolled back."""
        added_code = []
        added_labels = []
        added_origin = []
        added_callers = []  # (target, bank, pc)
        conflict = False

        worklist = {(bank, addr, origin_tag, addr)}

        while worklist and not conflict:
            wbank, wstart, from_bank, from_addr = worklist.pop()
            wdata = self.get_bank_data(wbank)
            pc = wstart
            prev_pc = from_addr

            while 0 <= pc < len(wdata) and not conflict:
                if pc in self.code_addrs.get(wbank, {}):
                    break

                opcode = wdata[pc]
                entry = OPCODES.get(opcode)
                if entry is None:
                    conflict = True
                    break

                mnem = entry[0]
                size = entry[1]
                fmt = entry[2] if len(entry) > 2 else None
                raw = wdata[pc:pc + size]
                if len(raw) < size:
                    conflict = True
                    break

                # Overlap: existing instruction spans into pc?
                for j in range(1, 4):
                    prev = pc - j
                    if prev in self.code_addrs.get(wbank, {}) and prev + self.code_addrs[wbank][prev] > pc:
                        conflict = True
                        break
                if conflict:
                    break
                # Overlap: our instruction spans into existing code?
                for j in range(1, size):
                    if (pc + j) in self.code_addrs.get(wbank, {}):
                        conflict = True
                        break
                if conflict:
                    break

                if wbank not in self.code_addrs:
                    self.code_addrs[wbank] = {}
                self.code_addrs[wbank][pc] = size
                added_code.append((wbank, pc))
                if pc == wstart:
                    self.code_origin[wbank][pc] = [from_bank, from_addr]
                else:
                    self.code_origin[wbank][pc] = [wbank, prev_pc]
                added_origin.append((wbank, pc))

                target = None
                if fmt == 'addr16' and mnem in ('LJMP', 'LCALL'):
                    target = (raw[1] << 8) | raw[2]
                elif fmt == 'addr11':
                    a = ((opcode & 0xE0) << 3) | raw[1]
                    page = (pc + size) & 0xF800
                    target = page | a
                elif fmt == 'rel':
                    offset = raw[1] if raw[1] < 128 else raw[1] - 256
                    target = pc + size + offset
                elif fmt in ('bit_rel', 'imm_rel', 'direct_rel'):
                    offset = raw[-1] if raw[-1] < 128 else raw[-1] - 256
                    target = pc + size + offset

                # Reject if target outside bank or in padding
                if target is not None:
                    if target < 0 or target >= BANK_SIZE:
                        conflict = True
                        break
                    tdata = self.get_bank_data(wbank)
                    if tdata[target] == 0xFF:
                        conflict = True
                        break

                if target is not None:
                    tkey = (wbank, target)
                    if tkey not in self.asm_labels:
                        if mnem in ('LCALL', 'ACALL'):
                            self.asm_labels[tkey] = f"sub_{target:04x}"
                        else:
                            self.asm_labels[tkey] = f"loc_{target:04x}"
                        added_labels.append(tkey)
                    self.callers[target].add((wbank, pc))
                    added_callers.append((target, wbank, pc))

                prev_pc = pc

                if mnem in ('RET', 'RETI'):
                    break
                elif opcode == 0x73:  # JMP @A+DPTR
                    break
                elif mnem in ('LJMP', 'SJMP', 'AJMP'):
                    if target is not None:
                        tb = wbank
                        worklist.add((tb, target, wbank, pc))
                    break
                elif mnem in ('LCALL', 'ACALL'):
                    if target is not None:
                        tb = wbank
                        worklist.add((tb, target, wbank, pc))
                        if target in self.noreturn:
                            break
                    pc += size
                elif target is not None:
                    tb = wbank
                    worklist.add((tb, target, wbank, pc))
                    pc += size
                else:
                    pc += size

        if conflict:
            for rb, ra in added_code:
                del self.code_addrs[rb][ra]
            for lkey in added_labels:
                self.asm_labels.pop(lkey, None)
            for rb, ra in added_origin:
                self.code_origin[rb].pop(ra, None)
            for t, rb, ra in added_callers:
                self.callers[t].discard((rb, ra))
            return False
        else:
            if (bank, addr) not in self.asm_labels:
                self.asm_labels[(bank, addr)] = f"loc_{addr:04x}"
            return True

    def solve_dispatch(self):
        """Parse dispatch tables for known noreturn trampolines, mark targets as code.
        sub_19b5 table format: [target_hi, target_lo, key] x N, 00 00 default_hi default_lo"""
        total = 0
        for bank in sorted(self.code_addrs.keys()):
            bdata = self.get_bank_data(bank)
            for addr, size in list(self.code_addrs[bank].items()):
                if size != 3 or bdata[addr] != 0x12:
                    continue
                call_target = (bdata[addr + 1] << 8) | bdata[addr + 2]
                if call_target not in self.noreturn:
                    continue
                # Parse table starting after the LCALL
                table = addr + 3
                targets = []
                pc = table
                for _ in range(30):  # max entries
                    if pc + 2 >= BANK_SIZE:
                        break
                    hi, lo = bdata[pc], bdata[pc + 1]
                    if hi == 0 and lo == 0:
                        # Default entry: next 2 bytes
                        if pc + 3 < BANK_SIZE:
                            dhi, dlo = bdata[pc + 2], bdata[pc + 3]
                            targets.append((dhi << 8) | dlo)
                        break
                    targets.append((hi << 8) | lo)
                    pc += 3
                # Try to mark each target
                for t in targets:
                    if t in self.code_addrs.get(bank, {}) or t in self.code_addrs.get(0, {}):
                        continue  # already known
                    tb = bank
                    if self.try_mark_code(tb, t):
                        total += 1
                        label = self.asm_labels.get((tb, t), f"0x{t:04x}")
                        print(f";   dispatch {bank}:0x{addr:04x} -> {label}", file=sys.stderr)
        return total



def sfr_name(addr):
    """Get SFR name for a direct address >= 0x80."""
    if addr in SFR:
        return SFR[addr]
    if addr >= 0x80:
        return f"SFR_{addr:02X}h"
    return f"0x{addr:02X}"


def direct_name(addr):
    """Get name for any direct address (RAM 0x00-0x7F or SFR 0x80-0xFF)."""
    if addr >= 0x80:
        return sfr_name(addr)
    return f"0x{addr:02X}"


def bit_name(addr):
    """Get bit-addressable name."""
    if addr in NAMED_BITS:
        return NAMED_BITS[addr]
    if addr >= 0x80:
        base = addr & 0xF8
        bit = addr & 0x07
        if base in BIT_SFR:
            return f"{BIT_SFR[base]}.{bit}"
        return f"SFR_{base:02X}h.{bit}"
    # Bit-addressable RAM (0x20-0x2F -> bits 0x00-0x7F)
    byte_addr = 0x20 + (addr >> 3)
    bit_num = addr & 0x07
    return f"BIT_{byte_addr:02X}h.{bit_num}"


def xdata_name(addr):
    """Get register/variable name if known."""
    if addr in REGMAP:
        return REGMAP[addr]['name']
    return None



def regmap_field_for_mask(addr, mask):
    """Given an XDATA addr and a bitmask, return which fields the mask selects."""
    info = REGMAP.get(addr)
    if not info:
        return None
    fields = info.get('fields', [])
    matched = []
    for f in fields:
        if f['name'] == 'Reserved':
            continue
        # Build field mask from bit_start..bit_end
        fmask = 0
        for b in range(f['bit_start'], f['bit_end'] + 1):
            fmask |= (1 << b)
        if mask & fmask:
            matched.append(f['name'])
    return matched if matched else None


# =============================================================================
# Disassembler Core
# =============================================================================

class Disassembler:
    def __init__(self, fw):
        self.fw = fw
        self.output_lines = []
        self.current_dptr = None  # Track last MOV DPTR,#imm for annotation
        self.current_acc_imm = None  # Track last MOV A,#imm for page tracking
        self.last_read_addr = None   # Track last MOVX A,@DPTR target for mask annotation
        # Build xcall stub lookup: stub_addr -> (target_bank, target_addr)
        self._xcall_map = {s: (b, t) for s, b, t in fw.bank_calls}

    def disassemble_instruction(self, data, pc, base_addr=0, bank=None):
        """Disassemble a single instruction. Returns (text, size, info_dict)."""
        addr = base_addr + pc
        opcode = data[pc]
        entry = OPCODES.get(opcode)

        if entry is None:
            return (f"DB      0x{opcode:02X}", 1,
                    {'type': 'data', 'raw': [opcode]})

        mnem = entry[0]
        size = entry[1]
        fmt = entry[2] if len(entry) > 2 else None
        prefix = entry[3] if len(entry) > 3 and entry[3] is not None else ""
        suffix = entry[4] if len(entry) > 4 and entry[4] is not None else ""

        raw = data[pc:pc+size]
        if len(raw) < size:
            return (f"DB      {','.join(f'0x{b:02X}' for b in raw)}", len(raw),
                    {'type': 'data', 'raw': list(raw)})

        info = {'type': 'code', 'mnem': mnem, 'raw': list(raw)}
        comment = ""
        target = None

        if fmt is None:
            operand = prefix if prefix else ""
        elif fmt == 'addr11':
            a = ((opcode & 0xE0) << 3) | raw[1]
            page = (addr + size) & 0xF800
            target = page | a
            operand = f"0x{target:04X}"
            info['target'] = target
        elif fmt == 'addr16':
            target = (raw[1] << 8) | raw[2]
            if mnem == "MOV" and prefix == "DPTR,":
                # MOV DPTR,#addr16 - annotate with XDATA name
                # But not if DPTR is used for code access (MOVC/JMP @A+DPTR) nearby
                code_use = False
                for i in range(pc + size, min(pc + size + 8, len(data))):
                    if data[i] in (0x93, 0x73):  # MOVC A,@A+DPTR / JMP @A+DPTR
                        code_use = True
                        break
                    if data[i] == 0x90:  # another MOV DPTR — stop looking
                        break
                xname = xdata_name(target) if not code_use else None
                if xname:
                    operand = f"{prefix}#{xname}"
                    comment = f"0x{target:04X}"
                else:
                    operand = f"{prefix}#0x{target:04X}"
                self.current_dptr = target
            else:
                operand = f"{prefix}0x{target:04X}"
                info['target'] = target
                if mnem in ('LCALL', 'ACALL'):
                    xcall = self._xcall_map.get(target)
                    if xcall:
                        tb, ta = xcall
                        label = self.fw.asm_labels.get((tb, ta))
                        if label and not label.startswith("loc_"):
                            comment = label
                    elif bank is not None:
                        label = self.fw.asm_labels.get((bank, target))
                        if label and not label.startswith("loc_"):
                            comment = label
        elif fmt == 'rel':
            offset = raw[1] if raw[1] < 128 else raw[1] - 256
            target = addr + size + offset
            operand = f"{prefix}0x{target:04X}"
            info['target'] = target
        elif fmt == 'direct':
            d = raw[1]
            operand = f"{prefix}{direct_name(d)}{suffix}"
            info['direct'] = d
        elif fmt == 'imm':
            operand = f"{prefix}#0x{raw[1]:02X}"
            info['imm'] = raw[1]
        elif fmt == 'bit':
            operand = f"{prefix}{bit_name(raw[1])}{suffix}"
            info['bit'] = raw[1]
        elif fmt == 'bit_rel':
            offset = raw[2] if raw[2] < 128 else raw[2] - 256
            target = addr + size + offset
            operand = f"{bit_name(raw[1])},0x{target:04X}"
            info['target'] = target
            info['bit'] = raw[1]
        elif fmt == 'direct_imm':
            operand = f"{direct_name(raw[1])},#0x{raw[2]:02X}"
            info['direct'] = raw[1]
            info['imm'] = raw[2]
        elif fmt == 'direct_direct':
            operand = f"{direct_name(raw[2])},{direct_name(raw[1])}"
        elif fmt == 'imm_rel':
            offset = raw[2] if raw[2] < 128 else raw[2] - 256
            target = addr + size + offset
            operand = f"{prefix}#0x{raw[1]:02X},0x{target:04X}"
            info['target'] = target
            info['imm'] = raw[1]
        elif fmt == 'direct_rel':
            offset = raw[2] if raw[2] < 128 else raw[2] - 256
            target = addr + size + offset
            operand = f"{prefix}{direct_name(raw[1])},0x{target:04X}"
            info['target'] = target
            info['direct'] = raw[1]
        else:
            operand = ""

        # Track MOV A,#imm for page-select and mask detection
        if mnem == "MOV" and fmt == 'imm' and prefix == "A,":
            self.current_acc_imm = raw[1]

        # Track INC DPTR
        if mnem == "INC" and operand == "DPTR" and self.current_dptr is not None:
            self.current_dptr += 1

        # Annotate MOVX with tracked DPTR
        if mnem == "MOVX" and self.current_dptr is not None:
            ea = self.current_dptr
            xn = xdata_name(ea)
            if "A,@DPTR" in (prefix or operand or ""):
                if xn:
                    comment = f"read {xn}"
                self.last_read_addr = ea
            elif "@DPTR,A" in (prefix or operand or ""):
                if xn:
                    comment = f"write {xn}"

        # Annotate ANL/ORL A,#imm after a MOVX read — show which bitfields the mask hits
        if (mnem in ("ANL", "ORL") and fmt == 'imm' and prefix == "A,"
                and self.last_read_addr is not None):
            mask = raw[1]
            fields = regmap_field_for_mask(self.last_read_addr, mask)
            if fields:
                comment = f"mask 0x{mask:02X} -> {','.join(fields[:3])}"

        # Reset DPTR tracking on non-sequential operations
        if mnem in ("LJMP", "SJMP", "AJMP", "LCALL", "ACALL", "RET", "RETI", "JMP"):
            self.current_dptr = None
            self.current_acc_imm = None
            self.last_read_addr = None

        text = f"{mnem:<7s} {operand}"
        info['comment'] = comment
        return (text, size, info)

    def _resolve_label(self, bank, target):
        label = self.fw.asm_labels.get((bank, target))
        if label:
            return label
        print(f"; ERROR: missing label for {bank}:0x{target:04x} — "
              f"analysis incomplete, use --trace to investigate", file=sys.stderr)
        return f"0x{target:04x}"

    def format_asm_instruction(self, data, pc, bank=0):
        """Format one instruction as sdas8051 assembly. Returns (asm_line, size, comment)."""
        opcode = data[pc]
        entry = OPCODES.get(opcode)

        if entry is None:
            return (f"\t.db\t0x{opcode:02x}", 1, "")

        mnem = entry[0].lower()
        size = entry[1]
        fmt = entry[2] if len(entry) > 2 else None
        prefix = (entry[3] if len(entry) > 3 and entry[3] is not None else "").lower()
        suffix = (entry[4] if len(entry) > 4 and entry[4] is not None else "").lower()

        raw = data[pc:pc + size]
        if len(raw) < size:
            return (f"\t.db\t{','.join(f'0x{b:02x}' for b in raw)}", len(raw), "")

        # Handle reserved opcode 0xA5 (encoded as "DB" in OPCODES)
        if mnem == "db":
            return (f"\t.db\t0x{opcode:02x}", 1, "")

        # Instructions that need explicit 'a' operand for assembler
        if mnem in ('rr', 'rrc', 'rl', 'rlc') and not prefix:
            prefix = "a"

        comment = ""

        if fmt is None:
            operand = prefix if prefix else ""
        elif fmt == 'addr11':
            a = ((opcode & 0xE0) << 3) | raw[1]
            page = (pc + size) & 0xF800
            target = page | a
            operand = self._resolve_label(bank, target)
        elif fmt == 'addr16':
            target = (raw[1] << 8) | raw[2]
            if mnem == "mov" and "dptr" in prefix:
                operand = f"dptr,#0x{target:04x}"
                self.current_dptr = target
                code_use = False
                for i in range(pc + size, min(pc + size + 8, len(data))):
                    if data[i] in (0x93, 0x73):
                        code_use = True
                        break
                    if data[i] == 0x90:
                        break
                xname = xdata_name(target) if not code_use else None
                if xname:
                    comment = xname
            else:
                operand = self._resolve_label(bank, target)
                if mnem in ("lcall", "acall"):
                    xcall = self._xcall_map.get(target)
                    if xcall:
                        tb, ta = xcall
                        label = self.fw.asm_labels.get((tb, ta))
                        if label and not label.startswith("loc_"):
                            comment = label
                    elif bank is not None:
                        label = self.fw.asm_labels.get((bank, target))
                        if label and not label.startswith("loc_"):
                            comment = label
        elif fmt == 'rel':
            offset = raw[1] if raw[1] < 128 else raw[1] - 256
            target = pc + size + offset
            label = self._resolve_label(bank, target)
            operand = f"{prefix}{label}" if prefix else label
        elif fmt == 'direct':
            operand = f"{prefix}0x{raw[1]:02x}{suffix}"
        elif fmt == 'imm':
            operand = f"{prefix}#0x{raw[1]:02x}"
        elif fmt == 'bit':
            operand = f"{prefix}0x{raw[1]:02x}{suffix}"
        elif fmt == 'bit_rel':
            offset = raw[2] if raw[2] < 128 else raw[2] - 256
            target = pc + size + offset
            label = self._resolve_label(bank, target)
            operand = f"0x{raw[1]:02x},{label}"
        elif fmt == 'direct_imm':
            operand = f"0x{raw[1]:02x},#0x{raw[2]:02x}"
        elif fmt == 'direct_direct':
            operand = f"0x{raw[2]:02x},0x{raw[1]:02x}"
        elif fmt == 'imm_rel':
            offset = raw[2] if raw[2] < 128 else raw[2] - 256
            target = pc + size + offset
            label = self._resolve_label(bank, target)
            operand = f"{prefix}#0x{raw[1]:02x},{label}"
        elif fmt == 'direct_rel':
            offset = raw[2] if raw[2] < 128 else raw[2] - 256
            target = pc + size + offset
            label = self._resolve_label(bank, target)
            operand = f"{prefix}0x{raw[1]:02x},{label}"
        else:
            operand = ""

        # DPTR tracking for MOVX annotation
        if mnem == "mov" and fmt == 'imm' and prefix == "a,":
            self.current_acc_imm = raw[1]
        if mnem == "inc" and operand == "dptr" and self.current_dptr is not None:
            self.current_dptr += 1
        if mnem == "movx" and self.current_dptr is not None:
            ea = self.current_dptr
            xn = xdata_name(ea)
            if "a,@dptr" in operand:
                if xn:
                    comment = f"read {xn}"
                self.last_read_addr = ea
            elif "@dptr,a" in operand:
                if xn:
                    comment = f"write {xn}"
        if mnem in ("anl", "orl") and fmt == 'imm' and prefix == "a," and self.last_read_addr is not None:
            fields = regmap_field_for_mask(self.last_read_addr, raw[1])
            if fields:
                comment = f"mask 0x{raw[1]:02x} -> {','.join(fields[:3])}"
        if mnem in ('ljmp', 'sjmp', 'ajmp', 'lcall', 'acall', 'ret', 'reti', 'jmp'):
            self.current_dptr = None
            self.current_acc_imm = None
            self.last_read_addr = None

        asm_line = f"\t{mnem}\t{operand}" if operand else f"\t{mnem}"
        return (asm_line, size, comment)

    def disassemble_range(self, data, start, end, base_addr=0, bank=None):
        """Disassemble a range of code, yielding formatted lines."""
        pc = start
        lines = []
        while pc < end and pc < len(data):
            addr = base_addr + pc

            # Check for labels
            label = self.fw.asm_labels.get((bank, addr)) if bank is not None else None
            if label and not label.startswith("loc_"):
                lines.append("")
                lines.append(f"; -------- {label} --------")

            # Check for known jump targets (add anonymous label)
            elif addr in self.fw.jump_targets:
                lines.append(f"loc_{addr:04X}:")

            text, size, info = self.disassemble_instruction(data, pc, base_addr, bank=bank)
            hex_str = ' '.join(f'{data[pc+i]:02X}' for i in range(size))

            comment = info.get('comment', '')
            comment_str = f"  ; {comment}" if comment else ""

            # Bank indicator for banked addresses
            bank_str = ""
            if bank is not None:
                bank_str = f"{bank}:"

            line = f"  {bank_str}0x{addr:04X}:  {hex_str:<14s} {text}{comment_str}"
            lines.append(line)

            # Track targets for xref
            target = info.get('target')
            if target is not None:
                self.fw.jump_targets.add(target)

            pc += size

        return lines

    def disassemble_bank(self, bank_num, start=None, end=None):
        """Disassemble a full bank or range within it."""
        bank_data = self.fw.get_bank_data(bank_num)
        if start is None:
            start = 0
        if end is None:
            # Find last non-FF byte
            end = len(bank_data) - 1
            while end > start and bank_data[end] == 0xFF:
                end -= 1
            end += 1

        header = [
            f"; {'='*70}",
            f"; Bank {bank_num} disassembly (0x{start:04X} - 0x{end-1:04X})",
            f"; Flash offset: 0x{bank_num * BANK_SIZE + start:06X}",
            f"; {'='*70}",
            "",
        ]

        code_lines = self.disassemble_range(bank_data, start, end,
                                             base_addr=0, bank=bank_num)
        return header + code_lines

    def disassemble_function(self, addr, bank=0):
        """Disassemble a single function starting at addr.
        Tracks max forward branch target so RET/LJMP doesn't end the function early
        if there's still reachable code after (via conditional jumps over a RET)."""
        bank_data = self.fw.get_bank_data(bank)
        pc = addr
        lines = [f"; Function at 0x{addr:04X} (bank {bank})"]

        # Conditional branches whose forward target extends function extent.
        BRANCH_MNEMS = {'JC', 'JNC', 'JZ', 'JNZ', 'JB', 'JNB', 'JBC',
                        'CJNE', 'DJNZ', 'SJMP'}

        max_target = addr  # furthest forward-branch target seen
        max_scan = min(addr + 0x2000, len(bank_data))
        while pc < max_scan:
            text, size, info = self.disassemble_instruction(bank_data, pc, base_addr=0, bank=bank)
            hex_str = ' '.join(f'{bank_data[pc+i]:02X}' for i in range(size))

            label = self.fw.asm_labels.get((bank, pc))
            if label and not label.startswith("loc_"):
                lines.append(f"\n; -------- {label} --------")
            elif pc in self.fw.jump_targets and pc != addr:
                lines.append(f"loc_{pc:04X}:")

            comment = info.get('comment', '')
            comment_str = f"  ; {comment}" if comment else ""
            bstr = f"{bank}:"
            lines.append(f"  {bstr}0x{pc:04X}:  {hex_str:<14s} {text}{comment_str}")

            pc += size
            mnem = info.get('mnem', '')

            # Track furthest forward branch target (for in-function reachability)
            target = info.get('target')
            if target is not None and mnem in BRANCH_MNEMS and target > pc:
                max_target = max(max_target, target)

            # End function on unconditional terminator, but only if we've reached
            # the furthest known forward branch target (no more reachable code)
            terminator = mnem in ('RET', 'RETI') or (
                mnem in ('LJMP', 'AJMP')
                and (target is None or target <= pc)
            )
            if terminator and pc > max_target:
                break

        return lines

    def emit_bank_asm(self, bank_num):
        """Emit complete sdas8051 assembly source for one bank.
        Pure reader of the db — emits labels inline, code as instructions, rest as .db."""
        lines = []

        lines.append(f"; RTD 8051 MCU Bank {bank_num} — disassembled by rtd_dis.py")
        lines.append(f"\t.module\tbank{bank_num}")
        lines.append(f"\t.area\tCABS\t(ABS,CODE)")
        lines.append(f"\t.org\t0x0000")
        lines.append("")

        bank_data = self.fw.get_bank_data(bank_num)
        code_map = self.fw.code_addrs.get(bank_num, {})

        # Find end of meaningful content — include code_addrs extent
        end = BANK_SIZE
        while end > 0 and bank_data[end - 1] == 0xFF:
            end -= 1
        if code_map:
            code_end = max(a + code_map[a] for a in code_map)
            end = max(end, code_end)
        if end == 0:
            end = 1

        # Reset DPTR tracking
        self.current_dptr = None
        self.current_acc_imm = None
        self.last_read_addr = None

        pc = 0
        while pc < end:
            is_code = pc in code_map

            # Emit label from db if present for this bank
            label = self.fw.asm_labels.get((bank_num, pc))
            if label:
                lines.append(f"{label}:")

            if is_code:
                size = code_map[pc]
                for j in range(1, size):
                    if (pc + j) in code_map:
                        print(f"; ERROR: code overlap in bank {bank_num}: "
                              f"instruction at 0x{pc:04x} (size={size}) "
                              f"overlaps code at 0x{pc+j:04x}", file=sys.stderr)
                asm_line, size, comment = self.format_asm_instruction(bank_data, pc, bank_num)
                if comment:
                    lines.append(f"{asm_line}\t; {comment}")
                else:
                    lines.append(asm_line)
                pc += size
            else:
                # Collect non-code bytes as .db, break at labels
                data_bytes = []
                while pc < end and pc not in code_map:
                    if data_bytes and (bank_num, pc) in self.fw.asm_labels:
                        break
                    data_bytes.append(bank_data[pc])
                    pc += 1
                for i in range(0, len(data_bytes), 16):
                    chunk = data_bytes[i:i + 16]
                    hex_vals = ','.join(f'0x{b:02x}' for b in chunk)
                    lines.append(f"\t.db\t{hex_vals}")

        # Pad to 64K
        if end < BANK_SIZE:
            lines.append(f"\t.org\t0x{BANK_SIZE - 1:04x}")
            lines.append(f"\t.db\t0x{bank_data[BANK_SIZE - 1]:02x}")

        return lines


# =============================================================================
# Analysis Passes
# =============================================================================

def analyze_firmware(fw):
    """Run analysis passes to identify functions, xrefs, etc."""
    dis = Disassembler(fw)

    # Pass 1: Scan all code to find jump/call targets
    for bank in range(min(fw.num_banks, 8)):
        bank_data = fw.get_bank_data(bank)
        start = 0

        # Find end of code
        end = len(bank_data) - 1
        while end > start and bank_data[end] == 0xFF:
            end -= 1
        end += 1

        pc = start
        while pc < end:
            opcode = bank_data[pc]
            entry = OPCODES.get(opcode)
            if entry is None:
                pc += 1
                continue

            size = entry[1]
            fmt = entry[2] if len(entry) > 2 else None
            mnem = entry[0]
            raw = bank_data[pc:pc+size]
            if len(raw) < size:
                break

            target = None
            if fmt == 'addr16' and mnem in ('LJMP', 'LCALL'):
                target = (raw[1] << 8) | raw[2]
            elif fmt == 'addr11':
                a = ((opcode & 0xE0) << 3) | raw[1]
                page = (pc + size) & 0xF800
                target = page | a
            elif fmt == 'rel':
                offset = raw[1] if raw[1] < 128 else raw[1] - 256
                target = pc + size + offset
            elif fmt in ('bit_rel', 'imm_rel', 'direct_rel'):
                offset = raw[-1] if raw[-1] < 128 else raw[-1] - 256
                target = pc + size + offset

            if target is not None:
                fw.jump_targets.add(target)

                if mnem in ('LCALL', 'ACALL'):
                    if target not in fw.functions:
                        fw.functions[target] = {
                            'bank': bank,
                            'callers': set(),
                            'name': fw.asm_labels.get((bank, target), f"sub_{target:04X}"),
                        }
                    fw.functions[target]['callers'].add((bank, pc))

            pc += size

    return dis


# =============================================================================
# Output Modes
# =============================================================================

def print_memory_map(fw):
    """Print firmware memory map overview."""
    print(f"; RTD 8051 MCU Firmware Memory Map")
    print(f"; File: {fw.filename} ({fw.size} bytes)")
    print(f"; Architecture: 8051 with {fw.num_banks}x64K code banking")
    print(f"; Bank register: SFR 0x4D (MCU_BANK) + XDATA 0xFFFF")
    print(f"; Bank 0 low addresses contain shared code (software convention)")
    print()

    # Interrupt vectors
    print("; 8051 Interrupt Vectors:")
    for addr, name in sorted(INT_VECTORS.items()):
        if addr + 2 < fw.size:
            op = fw.data[addr]
            if op == 0x02:
                target = (fw.data[addr+1] << 8) | fw.data[addr+2]
                print(f";   0x{addr:04X} {name:<16s} -> 0x{target:04X}")
    print()

    # Bank usage
    print("; Code Banks:")
    print(f";  {'Bank':>4} {'Flash':>16} {'Code Range':>18} {'Size':>8} {'Used%':>6} {'LCALLs':>7}")
    print(f";  {'-'*4} {'-'*16} {'-'*18} {'-'*8} {'-'*6} {'-'*7}")
    for bank in range(fw.num_banks):
        base = bank * BANK_SIZE
        bank_data = fw.data[base:base+BANK_SIZE]
        non_ff = sum(1 for b in bank_data if b != 0xFF)
        pct = non_ff / BANK_SIZE * 100
        lcalls = sum(1 for i in range(len(bank_data))
                     if bank_data[i] == 0x12)

        # Find code extent
        last = 0
        for i in range(BANK_SIZE - 1, -1, -1):
            if bank_data[i] != 0xFF:
                last = i
                break

        if non_ff == 0:
            status = "EMPTY"
        else:
            status = f"0x0000-0x{last:04X}"

        print(f";  {bank:4d} 0x{base:06X}-0x{base+0xFFFF:06X} {status:>18s} "
              f"{non_ff:6d}B {pct:5.1f}% {lcalls:6d}")

    # Cross-bank calls summary
    print()
    print(f"; Cross-bank call stubs: {len(fw.bank_calls)}")
    bank_counts = defaultdict(int)
    for _, target_bank, _ in fw.bank_calls:
        bank_counts[target_bank] += 1
    for bank in sorted(bank_counts):
        print(f";   -> Bank {bank}: {bank_counts[bank]} stubs")


def print_bank_calls(fw):
    """Print inter-bank call map."""
    print("; Inter-bank call stubs")
    print()
    # Group by target bank
    by_bank = defaultdict(list)
    for stub_addr, target_bank, target_func in fw.bank_calls:
        by_bank[target_bank].append((stub_addr, target_func))

    for bank in sorted(by_bank):
        stubs = by_bank[bank]
        print(f"; Bank {bank} ({len(stubs)} entry points):")
        for stub_addr, target_func in sorted(stubs, key=lambda x: x[1]):
            print(f";   stub 0x{stub_addr:04X} -> bank {bank}:0x{target_func:04X}")
        print()


def print_strings(fw):
    """Extract and print OSD strings with the RTD character encoding."""
    import re
    data = fw.data

    # Try to find OSD string tables
    # RTD OSD strings are typically stored as byte sequences with
    # a custom font mapping. Common pattern: sequential string entries
    # with length prefixes or null terminators.

    # Also extract plaintext strings
    print("; Plaintext strings (>= 6 chars)")
    print()
    seen = set()
    for m in re.finditer(rb'[\x20-\x7e]{6,}', data):
        s = m.group().decode('ascii')
        if s not in seen:
            seen.add(s)
            addr = m.start()
            bank = addr // BANK_SIZE
            offset = addr % BANK_SIZE
            print(f"  0x{addr:06X} (bank {bank}:0x{offset:04X}): \"{s}\"")

    # OSD-encoded strings (heuristic: check if +0x2C decoding produces
    # readable text for strings in the bank 0 data area)
    print()
    print("; OSD-encoded strings (bank 0, offset +0x2C decode)")
    print()
    for m in re.finditer(rb'[\x20-\x5F]{4,}', data[:BANK_SIZE]):
        raw = m.group()
        # Try decode: first char +0x27, rest +0x2C
        decoded = chr(raw[0] + 0x27)
        for b in raw[1:]:
            c = b + 0x2C
            if 0x20 <= c <= 0x7E:
                decoded += chr(c)
            else:
                decoded = None
                break
        if decoded and len(decoded) >= 4 and decoded.isascii():
            # Check if it looks like real text (has vowels, reasonable chars)
            lower = decoded.lower()
            if any(v in lower for v in 'aeiou'):
                addr = m.start()
                print(f"  0x{addr:06X}: \"{decoded}\" (raw: {raw[:20]})")


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="RTD 8051 MCU Firmware Disassembler",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)

    parser.add_argument('firmware', help='Firmware binary file')
    parser.add_argument('--addr', '-a', type=str, default=None,
                        help='Start disassembly at [bank:]address (e.g. 6:0xFB79 or 0x1234)')
    parser.add_argument('--end', type=lambda x: int(x, 0), default=None,
                        help='End address for disassembly')
    parser.add_argument('--xrefs', '-x', action='store_true',
                        help='Show cross-references')
    parser.add_argument('--calls', action='store_true',
                        help='Show inter-bank call map')
    parser.add_argument('--strings', '-s', action='store_true',
                        help='Extract strings')
    parser.add_argument('--map', '-m', action='store_true',
                        help='Show memory map overview')
    parser.add_argument('--all', action='store_true',
                        help='Full disassembly of all banks')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output file (default: stdout)')
    parser.add_argument('--length', '-l', type=lambda x: int(x, 0),
                        default=None, help='Number of bytes to disassemble')
    parser.add_argument('--regmap', type=str, default=None,
                        help='Path to regmap JSON (default: rtd2795t_cg_regmap.json next to script)')

    # Assembler output mode
    parser.add_argument('--asm', action='store_true',
                        help='Output sdas8051 assembly (use with --addr BANK:0 or --all)')
    parser.add_argument('--asm-dir', type=str, default=None,
                        help='Output directory for --asm --all (default: <firmware>_asm/)')

    # Database commands
    parser.add_argument('--analyze', action='store_true',
                        help='Run recursive descent analysis, save code/label db')
    parser.add_argument('--mark-code', type=str, default=None, metavar='BANK:ADDR',
                        help='Mark address as code entry point (e.g. 3:0x5000)')
    parser.add_argument('--label', nargs=2, metavar=('BANK:ADDR', 'NAME'),
                        help='Add/update label (e.g. 0:0x1234 my_func)')
    parser.add_argument('--noreturn', type=lambda x: int(x, 0), default=None, metavar='ADDR',
                        help='Mark function as noreturn (e.g. 0x19b5)')
    parser.add_argument('--solve-dispatch', action='store_true',
                        help='Parse dispatch tables (noreturn trampolines), mark targets as code')
    parser.add_argument('--search', type=lambda x: int(x, 0), default=None, metavar='VALUE',
                        help='Search for value in code (immediates, addresses, DPTR targets)')
    parser.add_argument('--search-label', type=str, default=None, metavar='PATTERN',
                        help='Search labels by name (case-insensitive substring match)')
    parser.add_argument('--trace', type=str, default=None, metavar='BANK:ADDR',
                        help='Trace how an address got marked as code (e.g. 3:0x3220)')
    parser.add_argument('--db-info', action='store_true',
                        help='Show database statistics')

    args = parser.parse_args()

    def parse_bank_addr(s):
        """Parse 'bank:addr' or 'addr' string, return (bank, addr)."""
        if ':' in s:
            b, a = s.split(':', 1)
            return int(b), int(a, 0)
        return None, int(s, 0)

    # Load register map
    nregs = load_regmap(args.regmap)
    if nregs:
        print(f"; Loaded {nregs} registers from regmap", file=sys.stderr)

    fw = RTDFirmware(args.firmware)

    # Handle database commands before full analysis
    if args.analyze:
        fw.load_db()

        # Auto-detect noreturn trampolines: POP DPH (D0 83) / POP DPL (D0 82)
        bank0 = fw.get_bank_data(0)
        nr_count = 0
        for i in range(0, min(len(bank0), 0x3000) - 4):
            if bank0[i:i+4] == b'\xd0\x83\xd0\x82':
                fw.noreturn.add(i)
                nr_count += 1
        if nr_count:
            print(f"; Auto-detected {nr_count} noreturn trampolines", file=sys.stderr)

        # Recursive descent analysis
        dis = analyze_firmware(fw)
        fw.mark_code()
        fw.solve_dispatch()
        fw.cleanup_code()
        fw.build_asm_labels()
        fw.save_db()

        # DB info
        total = sum(len(v) for v in fw.code_addrs.values())
        labels = len(fw.asm_labels)
        nr = len(fw.noreturn)
        print(f"; Total: {total} instructions, {labels} labels, {nr} noreturn", file=sys.stderr)
        return

    if args.solve_dispatch:
        fw.load_db()
        if not any(fw.code_addrs.values()):
            print("; Run --analyze first", file=sys.stderr)
            return
        old = sum(len(v) for v in fw.code_addrs.values())
        found = fw.solve_dispatch()
        fw.cleanup_code()
        fw.build_asm_labels()
        fw.save_db()
        new = sum(len(v) for v in fw.code_addrs.values())
        print(f"; Dispatch tables: {found} new targets, {new - old} new instructions", file=sys.stderr)
        return

    if args.search is not None:
        fw.load_db()
        val = args.search
        results = []
        for bank in sorted(fw.code_addrs.keys()):
            bdata = fw.get_bank_data(bank)
            for addr, size in sorted(fw.code_addrs[bank].items()):
                raw = bdata[addr:addr + size]
                if len(raw) < size:
                    continue
                opcode = raw[0]
                entry = OPCODES.get(opcode)
                if entry is None:
                    continue
                mnem = entry[0]
                fmt = entry[2] if len(entry) > 2 else None

                # Extract values from instruction
                found = None
                if fmt == 'addr16':
                    target = (raw[1] << 8) | raw[2]
                    if target == val:
                        if mnem in ('LJMP', 'LCALL'):
                            found = f"{mnem} 0x{val:04x} [code]"
                        elif mnem == 'MOV':
                            found = f"MOV DPTR,#0x{val:04x} [xdata]"
                elif fmt == 'addr11':
                    a = ((opcode & 0xE0) << 3) | raw[1]
                    page = (addr + size) & 0xF800
                    target = page | a
                    if target == val:
                        found = f"{mnem} 0x{val:04x} [code]"
                elif fmt == 'rel':
                    offset = raw[1] if raw[1] < 128 else raw[1] - 256
                    target = addr + size + offset
                    if target == val:
                        found = f"{mnem} 0x{val:04x} [code]"
                elif fmt in ('bit_rel', 'imm_rel', 'direct_rel'):
                    offset = raw[-1] if raw[-1] < 128 else raw[-1] - 256
                    target = addr + size + offset
                    if target == val:
                        found = f"{mnem} ...,0x{val:04x} [code]"
                elif fmt == 'imm' and raw[1] == (val & 0xFF) and val <= 0xFF:
                    found = f"{mnem} #0x{val:02x} [imm]"
                elif fmt == 'direct' and raw[1] == (val & 0xFF) and val <= 0xFF:
                    found = f"{mnem} 0x{val:02x} [sfr/iram]"
                elif fmt == 'direct_imm':
                    if raw[1] == (val & 0xFF) and val <= 0xFF:
                        found = f"{mnem} 0x{raw[1]:02x},#... [sfr/iram]"
                    elif raw[2] == (val & 0xFF) and val <= 0xFF:
                        found = f"{mnem} ...,#0x{val:02x} [imm]"
                elif fmt == 'bit' and raw[1] == (val & 0xFF) and val <= 0xFF:
                    found = f"{mnem} bit 0x{val:02x} [bit]"

                if found:
                    # Find containing function — nearest label at or before this addr
                    func_name = ""
                    for a in range(addr, max(addr - 0x2000, -1), -1):
                        lbl = fw.asm_labels.get((bank, a), "")
                        if lbl and not lbl.startswith("loc_"):
                            func_name = lbl
                            break
                    func_str = f" in {func_name}" if func_name else ""
                    results.append(f"  {bank}:0x{addr:04x}{func_str}: {found}")

        # Show what this value maps to in memory
        mappings = []
        xn = xdata_name(val)
        if xn:
            mappings.append(f"XDATA: {xn}")
        if val <= 0x7F:
            mappings.append(f"IRAM 0x{val:02x}")
        elif val <= 0xFF:
            mappings.append(f"SFR: {sfr_name(val)}")
        if val < BANK_SIZE:
            mappings.append("code space")
        map_str = f" = {', '.join(mappings)}" if mappings else ""
        # Show label if the searched value has one
        label_hits = []
        for bank in range(fw.num_banks):
            lbl = fw.asm_labels.get((bank, val), "")
            if lbl:
                label_hits.append(f"{bank}:{lbl}")
        label_str = f" ({', '.join(label_hits)})" if label_hits else ""
        print(f"; Search 0x{val:04x}{label_str}{map_str}: {len(results)} hits in code")
        for r in results:
            print(r)

        # Search in data regions — look for the value as big-endian 16-bit or 8-bit byte
        data_results = []
        needle_8 = bytes([val & 0xFF]) if val <= 0xFF else None
        needle_16 = bytes([(val >> 8) & 0xFF, val & 0xFF]) if val > 0xFF else None
        for bank in range(fw.num_banks):
            raw = fw.get_bank_data(bank)
            code_map = fw.code_addrs.get(bank, {})
            start = 0
            # Build set of all code bytes for fast lookup
            code_bytes = set()
            for ca, cs in code_map.items():
                for j in range(cs):
                    code_bytes.add(ca + j)
            # Scan for needle in non-code bytes
            needle = needle_16 if needle_16 else needle_8
            if needle is None:
                continue
            pos = start
            while pos < BANK_SIZE - len(needle) + 1:
                pos = raw.find(needle, pos)
                if pos == -1:
                    break
                # Check all bytes of match are NOT in code
                if not any((pos + j) in code_bytes for j in range(len(needle))):
                    flash_off = bank * BANK_SIZE + pos
                    data_results.append(f"  {bank}:0x{pos:04x}  (flash 0x{flash_off:06x})")
                pos += 1
        if data_results:
            print(f"; Search 0x{val:04x}: {len(data_results)} hits in data")
            for r in data_results:
                print(r)
        return

    if args.search_label is not None:
        fw.load_db()
        pattern = args.search_label.lower()
        results = []
        for (bank, addr), name in sorted(fw.asm_labels.items()):
            if pattern in name.lower():
                flash_off = bank * BANK_SIZE + addr
                results.append(f"  {bank}:0x{addr:04X}  (flash 0x{flash_off:06X})  {name}")
        print(f"; Label search '{args.search_label}': {len(results)} matches")
        for r in results:
            print(r)
        return

    if args.mark_code:
        parts = args.mark_code.split(':')
        mbank = int(parts[0])
        maddr = int(parts[1], 0)
        fw.load_db()
        old_count = sum(len(v) for v in fw.code_addrs.values())
        ok = fw.try_mark_code(mbank, maddr)
        if ok:
            fw.cleanup_code()
            fw.build_asm_labels()
            new_count = sum(len(v) for v in fw.code_addrs.values())
            print(f"; Marked {new_count - old_count} new instructions from {mbank}:0x{maddr:04x}", file=sys.stderr)
            fw.save_db()
        else:
            print(f"; REJECTED {mbank}:0x{maddr:04x} — conflict with existing code", file=sys.stderr)
        return

    if args.label:
        parts = args.label[0].split(':')
        lbank = int(parts[0])
        laddr = int(parts[1], 0)
        name = args.label[1]
        fw.load_db()
        fw.asm_labels[(lbank, laddr)] = name
        fw.save_db()
        print(f"; Label {lbank}:0x{laddr:04x} = {name}", file=sys.stderr)
        return

    if args.noreturn is not None:
        fw.load_db()
        fw.noreturn.add(args.noreturn)
        label = f"0x{args.noreturn:04x}"
        fw.save_db()
        print(f"; Marked {label} as noreturn", file=sys.stderr)
        return

    if args.trace:
        parts = args.trace.split(':')
        tbank = int(parts[0])
        taddr = int(parts[1], 0)
        if not fw.load_db():
            print("; No database found", file=sys.stderr)
            return
        bank_data = fw.get_bank_data(tbank)
        # Walk the origin chain
        cur_bank, cur_addr = tbank, taddr
        for step in range(50):  # max depth
            if cur_addr not in fw.code_addrs.get(cur_bank, {}):
                print(f"  {cur_bank}:0x{cur_addr:04x} — NOT in code_addrs")
                break
            size = fw.code_addrs[cur_bank][cur_addr]
            label = fw.asm_labels.get((cur_bank, cur_addr), '')
            label_str = f" ({label})" if label else ""
            # Decode instruction
            bdata = fw.get_bank_data(cur_bank)
            raw = bdata[cur_addr:cur_addr + size]
            hex_str = ' '.join(f'{b:02x}' for b in raw)
            origin = fw.code_origin.get(cur_bank, {}).get(cur_addr)
            if origin is None:
                print(f"  {cur_bank}:0x{cur_addr:04x} [{hex_str}]{label_str} — no origin recorded")
                break
            from_bank, from_addr = origin
            if from_bank == -1:
                # Seed: from_addr is the seed location (vector or xcall stub addr)
                seed_label = fw.asm_labels.get((0, from_addr), '')
                seed_str = f" {seed_label}" if seed_label else ""
                print(f"  {cur_bank}:0x{cur_addr:04x} [{hex_str}]{label_str} <-- SEED at 0:0x{from_addr:04x}{seed_str}")
                break
            print(f"  {cur_bank}:0x{cur_addr:04x} [{hex_str}]{label_str} <-- {from_bank}:0x{from_addr:04x}")
            cur_bank, cur_addr = from_bank, from_addr
        return

    if args.db_info:
        if fw.load_db():
            n_code = sum(len(v) for v in fw.code_addrs.values())
            n_bytes = sum(sum(v.values()) for v in fw.code_addrs.values())
            print(f"; Database: {fw.db_path}")
            print(f"; Firmware: {fw.filename} ({fw.size} bytes, sha256={fw._fw_sha256[:16]}...)")
            print(f"; Code: {n_code} instructions ({n_bytes} bytes)")
            print(f"; Labels: {len(fw.asm_labels)}")
            for bank in sorted(fw.code_addrs.keys()):
                n = len(fw.code_addrs[bank])
                b = sum(fw.code_addrs[bank].values())
                print(f";   Bank {bank}: {n} instrs ({b} bytes)")
        else:
            print(f"; No database found at {fw.db_path}", file=sys.stderr)
        return

    # Normal disassembly modes
    dis = analyze_firmware(fw)

    # For --asm mode, load the database
    if args.asm:
        if not fw.load_db():
            print("; No database found. Run --analyze first, or output will be all .db", file=sys.stderr)

    lines = []

    if args.asm and args.all:
        # Emit all banks to separate .asm files
        asm_dir = args.asm_dir or os.path.splitext(fw.filename)[0] + '_asm'
        os.makedirs(asm_dir, exist_ok=True)
        for bank in range(fw.num_banks):
            bank_lines = dis.emit_bank_asm(bank)
            outpath = os.path.join(asm_dir, f'bank_{bank:02d}.asm')
            with open(outpath, 'w') as f:
                f.write('\n'.join(bank_lines) + '\n')
            n_code = len(fw.code_addrs.get(bank, {}))
            print(f"; Bank {bank:2d}: {outpath} ({n_code} instructions)", file=sys.stderr)
        print(f"; All banks written to {asm_dir}/", file=sys.stderr)
        return

    elif args.asm and args.addr is not None:
        asm_bank, _ = parse_bank_addr(args.addr)
        lines = dis.emit_bank_asm(asm_bank if asm_bank is not None else 0)

    elif args.map:
        import io
        old_stdout = sys.stdout
        sys.stdout = buf = io.StringIO()
        print_memory_map(fw)
        sys.stdout = old_stdout
        lines = buf.getvalue().splitlines()

    elif args.xrefs:
        fw.load_db()
        lines = ["; Cross-references from db (targets with multiple callers)", ""]
        for addr in sorted(fw.callers.keys()):
            refs = fw.callers[addr]
            if len(refs) < 2:
                continue
            for rbank, raddr in sorted(refs):
                label = fw.asm_labels.get((rbank, addr), f"0x{addr:04x}")
                break  # just use first ref's bank for the label
            lines.append(f"  0x{addr:04x} ({label}): {len(refs)} refs")
            for rbank, raddr in sorted(refs)[:10]:
                lines.append(f"    <- {rbank}:0x{raddr:04x}")
            if len(refs) > 10:
                lines.append(f"    ... and {len(refs)-10} more")

    elif args.calls:
        import io
        old_stdout = sys.stdout
        sys.stdout = buf = io.StringIO()
        print_bank_calls(fw)
        sys.stdout = old_stdout
        lines = buf.getvalue().splitlines()

    elif args.strings:
        import io
        old_stdout = sys.stdout
        sys.stdout = buf = io.StringIO()
        print_strings(fw)
        sys.stdout = old_stdout
        lines = buf.getvalue().splitlines()

    elif args.addr is not None:
        fw.load_db()
        addr_bank, addr_val = parse_bank_addr(args.addr)
        bank = addr_bank if addr_bank is not None else 0
        end = args.end if args.end else None
        if args.length:
            end = addr_val + args.length
        if end is not None:
            lines = dis.disassemble_bank(bank, start=addr_val, end=end)
        else:
            lines = dis.disassemble_function(addr_val, bank=bank)

    elif args.all:
        for bank in range(fw.num_banks):
            bank_data = fw.get_bank_data(bank)
            non_ff = sum(1 for b in bank_data if b != 0xFF)
            if non_ff > 0:
                if lines:
                    lines.append("")
                lines.extend(dis.disassemble_bank(bank))

    else:
        parser.print_help()
        return

    output = '\n'.join(lines)
    if args.output:
        with open(args.output, 'w') as f:
            f.write(output + '\n')
        print(f"Output written to {args.output}", file=sys.stderr)
    else:
        print(output)


if __name__ == '__main__':
    main()
