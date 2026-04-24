#!/usr/bin/env python3
"""RX3081 MIPS16 LE assembler for lexra_dis.py.

Accepts either plain `mnemonic operands` lines or the listing format emitted by
lexra_dis.py:

    85000000:  f222 6814   li32     s0,0x12340000

It emits raw little-endian bytes and understands the explicit `...32` mnemonics
used by the disassembler to preserve width intent.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Iterable, Optional

from lexra_dis import CREG, GPR, CP0_NAMES


CREG_INDEX = {name: idx for idx, name in enumerate(CREG)}
GPR_INDEX = {name: idx for idx, name in enumerate(GPR)}
CP0_INDEX = {name: idx for idx, name in CP0_NAMES.items()}

LISTING_RE = re.compile(
    r'^\s*(?:[0-9a-fA-F]+:\s+(?:[0-9a-fA-F]{4}(?:\s+[0-9a-fA-F]{4})?)\s+)?'
    r'([.\w$-]+)(?:\s+(.*?))?\s*$'
)

BASE32 = {
    'addiu32': 'addiu',
    'li32': 'li',
    'la32': 'la',
    'b32': 'b',
    'beqz32': 'beqz',
    'bnez32': 'bnez',
    'bteqz32': 'bteqz',
    'btnez32': 'btnez',
    'sll32': 'sll',
    'srl32': 'srl',
    'sra32': 'sra',
    'slti32': 'slti',
    'sltiu32': 'sltiu',
    'cmpi32': 'cmpi',
    'lb32': 'lb',
    'lbu32': 'lbu',
    'lh32': 'lh',
    'lhu32': 'lhu',
    'lw32': 'lw',
    'lwpc32': 'lwpc',
    'sb32': 'sb',
    'sh32': 'sh',
    'sw32': 'sw',
    'swra32': 'swra',
    'adjsp32': 'adjsp',
}

SHIFT_FUNCS = {'sll': 0, 'srl': 2, 'sra': 3}
RRR_FUNCS = {'addu': 1, 'subu': 3}
LEXRA_FUNCS = {
    'madh': 0x00, 'madl': 0x02, 'mazh': 0x04, 'mazl': 0x06,
    'msbh': 0x10, 'msbl': 0x12, 'mszh': 0x14, 'mszl': 0x16,
}
RR_FUNCS = {
    'slt': 0x02,
    'sltu': 0x03,
    'sllv': 0x04,
    'break': 0x05,
    'srlv': 0x06,
    'srav': 0x07,
    'entry': 0x09,
    'exit': 0x09,
    'cmp': 0x0A,
    'neg': 0x0B,
    'and': 0x0C,
    'or': 0x0D,
    'xor': 0x0E,
    'not': 0x0F,
    'mult': 0x18,
    'multu': 0x19,
    'div': 0x1A,
    'divu': 0x1B,
    'sdbbp': 0x01,
}
ENTRY_ARGS = {'': 0, 'a0': 1, 'a0-a1': 2, 'a0-a2': 3, 'a0-a3': 4}
EXIT_FP = {'': 7, '$f0': 5, '$f0-$f1': 6}
SAVE_LIST = {'': 0, 'ra': 1, 's0': 2, 's0,ra': 3, 's0-s1': 4, 's0-s1,ra': 5,
             '??': 6, '??,ra': 7}


def parse_int(text: str) -> int:
    text = text.strip()
    neg = text.startswith('-')
    if neg:
        text = text[1:]
    base = 16 if text.lower().startswith('0x') else 10
    value = int(text, base)
    return -value if neg else value


def parse_addr(text: str) -> int:
    return parse_int(text) & 0xFFFFFFFF


def parse_creg(text: str) -> int:
    return CREG_INDEX[text]


def parse_gpr(text: str) -> int:
    return GPR_INDEX[text]


def parse_cp0(text: str) -> int:
    if text in CP0_INDEX:
        return CP0_INDEX[text]
    if text.startswith('$'):
        return int(text[1:], 10)
    raise ValueError(f'unknown cp0 register: {text}')


def split_args(args: str) -> list[str]:
    if not args:
        return []
    return [part.strip() for part in args.split(',') if part.strip()]


def hw_bytes(hw: int) -> bytes:
    return bytes((hw & 0xFF, (hw >> 8) & 0xFF))


def pack_ext16_u(value: int) -> tuple[int, int]:
    value &= 0xFFFF
    ext = ((value >> 11) & 0x1F) | (((value >> 5) & 0x3F) << 5)
    low5 = value & 0x1F
    return ext, low5


def pack_ext16_s(value: int) -> tuple[int, int]:
    if not -(1 << 15) <= value < (1 << 16):
        raise ValueError(f'extended immediate out of range: {value}')
    return pack_ext16_u(value & 0xFFFF)


def pack_ext15(value: int) -> tuple[int, int]:
    if not -(1 << 14) <= value < (1 << 14):
        raise ValueError(f'extended 15-bit immediate out of range: {value}')
    raw = value & 0x7FFF
    ext = ((raw >> 11) & 0x0F) | (((raw >> 4) & 1) << 4) | (((raw >> 5) & 0x3F) << 5)
    low4 = raw & 0x0F
    return ext, low4


def emit_ext(base_hw: int, ext: int) -> bytes:
    return hw_bytes(0xF000 | (ext & 0x7FF)) + hw_bytes(base_hw)


def emit_base_or_ext(base_hw: int, short_ok: bool, ext: Optional[int]) -> bytes:
    if ext is not None:
        return emit_ext(base_hw, ext)
    if not short_ok:
        raise ValueError('instruction requires extended form')
    return hw_bytes(base_hw)


def parse_mem(text: str) -> tuple[str, int]:
    m = re.fullmatch(r'(.+)\(([^()]+)\)', text.strip())
    if not m:
        raise ValueError(f'bad memory operand: {text}')
    return m.group(2).strip(), parse_int(m.group(1).strip())


def choose_short_range(value: int, lo: int, hi: int, step: int = 1) -> bool:
    return lo <= value <= hi and value % step == 0


def encode_line(mnemonic: str, args: str, pc: int) -> bytes:
    raw_mnem = mnemonic
    if raw_mnem.startswith('0x') and not args:
        return hw_bytes(int(raw_mnem, 16))

    force_ext = raw_mnem in BASE32
    mnem = BASE32.get(raw_mnem, raw_mnem)
    parts = split_args(args)

    if mnem == 'extend':
        return hw_bytes(0xF000 | (parse_int(parts[0]) & 0x7FF))

    if mnem == 'nop':
        return hw_bytes(0x6500)

    if mnem in ('jal', 'jalx'):
        target = parse_addr(parts[0])
        imm26 = (target >> 2) & 0x03FFFFFF
        hw1 = (0x03 << 11) | ((1 if mnem == 'jalx' else 0) << 10)
        hw1 |= ((imm26 >> 16) & 0x1F) << 5
        hw1 |= (imm26 >> 21) & 0x1F
        hw2 = imm26 & 0xFFFF
        return hw_bytes(hw1) + hw_bytes(hw2)

    if mnem == 'addiu':
        if len(parts) == 3 and parts[1] == 'sp' and parts[0] in CREG_INDEX:
            rx = parse_creg(parts[0])
            immv = parse_int(parts[2])
            if force_ext or not choose_short_range(immv, 0, 0x3FC, 4):
                ext, low5 = pack_ext16_s(immv)
                return emit_ext((0x00 << 11) | (rx << 8) | low5, ext)
            return hw_bytes((0x00 << 11) | (rx << 8) | ((immv // 4) & 0xFF))
        if len(parts) == 3 and parts[0] in CREG_INDEX and parts[1] in CREG_INDEX:
            ry = parse_creg(parts[0])
            rx = parse_creg(parts[1])
            immv = parse_int(parts[2])
            if force_ext or not -(1 << 3) <= immv < (1 << 3):
                ext, low4 = pack_ext15(immv)
                return emit_ext((0x08 << 11) | (rx << 8) | (ry << 5) | low4, ext)
            return hw_bytes((0x08 << 11) | (rx << 8) | (ry << 5) | (immv & 0xF))
        if len(parts) == 2 and parts[0] == 'sp':
            immv = parse_int(parts[1])
            if force_ext:
                ext, low5 = pack_ext16_s(immv)
                return emit_ext((0x0C << 11) | (3 << 8) | low5, ext)
            if choose_short_range(immv, -1024, 1016, 8):
                return hw_bytes((0x0C << 11) | (3 << 8) | ((immv // 8) & 0xFF))
            ext, low5 = pack_ext16_s(immv)
            return emit_ext((0x0C << 11) | (3 << 8) | low5, ext)
        if len(parts) == 2 and parts[0] in CREG_INDEX:
            rx = parse_creg(parts[0])
            immv = parse_int(parts[1])
            if force_ext or not -(1 << 7) <= immv < (1 << 7):
                ext, low5 = pack_ext16_s(immv)
                return emit_ext((0x09 << 11) | (rx << 8) | low5, ext)
            return hw_bytes((0x09 << 11) | (rx << 8) | (immv & 0xFF))
        raise ValueError(f'unsupported addiu form: {args}')

    if mnem == 'adjsp':
        if len(parts) == 2 and parts[0] == 'sp':
            immv = parse_int(parts[1])
        elif len(parts) == 1:
            immv = parse_int(parts[0])
        else:
            raise ValueError(f'bad adjsp form: {args}')
        ext, low5 = pack_ext16_s(immv)
        return emit_ext((0x0C << 11) | (3 << 8) | low5, ext)

    if mnem == 'li':
        rx = parse_creg(parts[0])
        immv = parse_int(parts[1])
        if force_ext or not 0 <= immv <= 0xFF:
            low16 = immv & 0xFFFF
            high16 = (immv >> 16) & 0xFFFF
            # LUI-style upper-half load iff imm is 0xHHHH0000; otherwise plain
            # 16-bit LI. Zero goes in the LI bucket.
            if low16 == 0 and high16 != 0:
                upper = high16
                bm = 1
            elif high16 == 0:
                upper = low16
                bm = 0
            else:
                raise ValueError(
                    f'li32 imm must be 0xHHHH0000 (LUI) or 0xLLLL (LI), got {parts[1]}')
            ext, low5 = pack_ext16_u(upper)
            return emit_ext((0x0D << 11) | (rx << 8) | (bm << 5) | low5, ext)
        return hw_bytes((0x0D << 11) | (rx << 8) | (immv & 0xFF))

    if mnem in ('slti', 'sltiu', 'cmpi'):
        rx = parse_creg(parts[0])
        immv = parse_int(parts[1])
        op = {'slti': 0x0A, 'sltiu': 0x0B, 'cmpi': 0x0E}[mnem]
        if force_ext or not 0 <= immv <= 0xFF:
            ext, low5 = pack_ext16_s(immv)
            return emit_ext((op << 11) | (rx << 8) | low5, ext)
        return hw_bytes((op << 11) | (rx << 8) | (immv & 0xFF))

    if mnem in ('la', 'lwpc'):
        rx = parse_creg(parts[0])
        target = parse_addr(parts[1])
        base_pc = pc & ~3
        off = (target - base_pc) & 0xFFFFFFFF
        signed_off = off if off < 0x80000000 else off - 0x100000000
        if mnem == 'la':
            op = 0x01
        else:
            op = 0x16
        if not force_ext and choose_short_range(signed_off, 0, 0x3FC, 4):
            return hw_bytes((op << 11) | (rx << 8) | ((signed_off // 4) & 0xFF))
        ext, low5 = pack_ext16_s(signed_off)
        return emit_ext((op << 11) | (rx << 8) | low5, ext)

    if mnem in ('b', 'beqz', 'bnez', 'bteqz', 'btnez'):
        target = parse_addr(parts[-1])
        off = ((target - (pc + (4 if force_ext else 2))) // 2)
        if mnem == 'b':
            if not force_ext and choose_short_range(off, -(1 << 10), (1 << 10) - 1):
                return hw_bytes((0x02 << 11) | (off & 0x7FF))
            ext, low5 = pack_ext16_s(off)
            return emit_ext((0x02 << 11) | low5, ext)
        if mnem in ('beqz', 'bnez'):
            rx = parse_creg(parts[0])
            op = 0x04 if mnem == 'beqz' else 0x05
            if not force_ext and choose_short_range(off, -(1 << 7), (1 << 7) - 1):
                return hw_bytes((op << 11) | (rx << 8) | (off & 0xFF))
            ext, low5 = pack_ext16_s(off)
            return emit_ext((op << 11) | (rx << 8) | low5, ext)
        func = 0 if mnem == 'bteqz' else 1
        if not force_ext and choose_short_range(off, -(1 << 7), (1 << 7) - 1):
            return hw_bytes((0x0C << 11) | (func << 8) | (off & 0xFF))
        ext, low5 = pack_ext16_s(off)
        return emit_ext((0x0C << 11) | (func << 8) | low5, ext)

    if mnem in SHIFT_FUNCS:
        sa = parse_int(parts[-1])
        regs = parts[:-1]
        if len(regs) == 1:
            rx = ry = parse_creg(regs[0])
        else:
            rx = parse_creg(regs[0])
            ry = parse_creg(regs[1])
        func = SHIFT_FUNCS[mnem]
        if not force_ext and 1 <= sa <= 8:
            enc = 0 if sa == 8 else sa
            return hw_bytes((0x06 << 11) | (rx << 8) | (ry << 5) | ((enc & 7) << 2) | func)
        if not 0 <= sa <= 31:
            raise ValueError(f'shift amount out of range: {sa}')
        ext = (sa & 0x1F) << 6
        return emit_ext((0x06 << 11) | (rx << 8) | (ry << 5) | func, ext)

    if mnem in ('lb', 'lbu', 'lh', 'lhu', 'lw', 'sb', 'sh', 'sw', 'swra'):
        if mnem == 'swra' or (mnem == 'sw' and parts[0] == 'ra'):
            base_name, off = parse_mem(parts[1])
            if base_name != 'sp':
                raise ValueError(f'sw ra expects (sp) base: {args}')
            if force_ext or mnem == 'swra' or not choose_short_range(off, 0, 0x3FC, 4):
                ext, low5 = pack_ext16_s(off)
                return emit_ext((0x0C << 11) | (2 << 8) | low5, ext)
            return hw_bytes((0x0C << 11) | (2 << 8) | ((off // 4) & 0xFF))

        reg = parts[0]
        base_name, off = parse_mem(parts[1])
        is_load = mnem in ('lb', 'lbu', 'lh', 'lhu', 'lw')
        op_map = {'lb': 0x10, 'lh': 0x11, 'lw': 0x13, 'lbu': 0x14, 'lhu': 0x15,
                  'sb': 0x18, 'sh': 0x19, 'sw': 0x1B}
        scale_map = {'lb': 1, 'lbu': 1, 'sb': 1, 'lh': 2, 'lhu': 2, 'sh': 2, 'lw': 4, 'sw': 4}
        op = op_map[mnem]
        scale = scale_map[mnem]

        if base_name == 'sp':
            rx = parse_creg(reg)
            sp_op = 0x12 if is_load else 0x1A
            if force_ext or not choose_short_range(off, 0, 0x3FC, 4):
                ext, low5 = pack_ext16_s(off)
                return emit_ext((sp_op << 11) | (rx << 8) | low5, ext)
            return hw_bytes((sp_op << 11) | (rx << 8) | ((off // 4) & 0xFF))

        base = parse_creg(base_name)
        data = parse_creg(reg)
        if force_ext or not choose_short_range(off, 0, 31 * scale, scale):
            ext, low5 = pack_ext16_s(off)
            return emit_ext((op << 11) | (base << 8) | (data << 5) | low5, ext)
        return hw_bytes((op << 11) | (base << 8) | (data << 5) | ((off // scale) & 0x1F))

    if mnem in ('move', 'movr32', 'mov32r'):
        dst, src = parts
        # `move` is unambiguous iff exactly one side lies outside the CREG
        # subset. `movr32`/`mov32r` override the implicit choice (used by the
        # disassembler when both sides are CREG-compatible and the func bit
        # would otherwise be lost).
        if mnem == 'movr32' or (mnem == 'move' and dst in GPR_INDEX and src in CREG_INDEX and dst not in CREG_INDEX):
            r32 = parse_gpr(dst)
            creg = parse_creg(src)
            hw = (0x0C << 11) | (5 << 8) | ((r32 & 0x07) << 5) | (r32 & 0x18) | creg
            return hw_bytes(hw)
        if mnem == 'mov32r' or (mnem == 'move' and dst in CREG_INDEX and src in GPR_INDEX):
            rx = parse_creg(dst)
            r32 = parse_gpr(src)
            hw = (0x0C << 11) | (7 << 8) | (rx << 5) | (r32 & 0x1F)
            return hw_bytes(hw)
        raise ValueError(f'unsupported move form: {args}')

    if mnem in ('mfc0', 'mtc0', 'eret', 'sleep', 'di', 'ei', 'deret', 'cache'):
        if mnem == 'mfc0':
            ry = parse_creg(parts[0])
            cp0 = parse_cp0(parts[1])
            return hw_bytes((0x17 << 11) | (0 << 8) | (ry << 5) | cp0)
        if mnem == 'mtc0':
            src = parse_creg(parts[0])
            cp0 = parse_cp0(parts[1])
            return hw_bytes((0x17 << 11) | (1 << 8) | ((cp0 & 7) << 5) | (cp0 & 0x18) | src)
        if mnem == 'cache':
            opv = parse_int(parts[0])
            base_name, off = parse_mem(parts[1])
            if off != 0:
                raise ValueError('cache only supports 0(base) syntax here')
            ry = parse_creg(base_name)
            return hw_bytes((0x17 << 11) | (3 << 8) | (ry << 5) | (opv & 0x1F))
        sel = {'eret': 0, 'sleep': 1, 'di': 2, 'ei': 3, 'deret': 7}[mnem]
        operand = parse_int(parts[0]) if parts else 0
        if mnem in ('di', 'ei', 'deret') and operand != 0:
            raise ValueError(f'{mnem} takes no operand')
        if operand <= 0x1F and not force_ext:
            return hw_bytes((0x17 << 11) | (2 << 8) | (sel << 5) | operand)
        ext, low5 = pack_ext16_u(operand)
        return emit_ext((0x17 << 11) | (2 << 8) | (sel << 5) | low5, ext)

    if mnem in RRR_FUNCS:
        regs = [parse_creg(p) for p in parts]
        func = RRR_FUNCS[mnem]
        if len(regs) == 2:
            rz, rx, ry = regs[0], regs[0], regs[1]
        elif len(regs) == 3:
            rz, rx, ry = regs
        else:
            raise ValueError(f'bad {mnem} operands: {args}')
        return hw_bytes((0x1C << 11) | (rx << 8) | (ry << 5) | (rz << 2) | func)

    if mnem == 'jr':
        if parts[0] == 'ra':
            return hw_bytes(0xE820)
        rx = parse_creg(parts[0])
        return hw_bytes(0xE800 | (rx << 8))
    if mnem == 'jalr':
        rx = parse_creg(parts[0])
        return hw_bytes(0xE840 | (rx << 8))
    if mnem in ('mfhi', 'mflo', 'mthi', 'mtlo'):
        rx = parse_creg(parts[0])
        base = {'mfhi': 0xE810, 'mflo': 0xE812, 'mthi': 0xE830, 'mtlo': 0xE832}[mnem]
        return hw_bytes(base | (rx << 8))

    if mnem in ('slt', 'sltu', 'cmp', 'and', 'or', 'xor', 'mult', 'multu'):
        rx = parse_creg(parts[0])
        ry = parse_creg(parts[1])
        return hw_bytes((0x1D << 11) | (rx << 8) | (ry << 5) | RR_FUNCS[mnem])

    if mnem in ('sllv', 'srlv', 'srav'):
        ry = parse_creg(parts[0])
        rx = parse_creg(parts[1])
        return hw_bytes((0x1D << 11) | (rx << 8) | (ry << 5) | RR_FUNCS[mnem])

    if mnem in ('neg', 'not'):
        if len(parts) == 1:
            rx = ry = parse_creg(parts[0])
        else:
            rx = parse_creg(parts[0])
            ry = parse_creg(parts[1])
        return hw_bytes((0x1D << 11) | (rx << 8) | (ry << 5) | RR_FUNCS[mnem])

    if mnem in ('div', 'divu'):
        if parts[0] != 'zero':
            raise ValueError(f'{mnem} expects zero as first operand')
        rx = parse_creg(parts[1])
        ry = parse_creg(parts[2])
        return hw_bytes((0x1D << 11) | (rx << 8) | (ry << 5) | RR_FUNCS[mnem])

    if mnem == 'sdbbp':
        code = parse_int(parts[0]) if parts else 0
        return hw_bytes((0x1D << 11) | ((code & 0x3F) << 5) | RR_FUNCS[mnem])

    if mnem == 'break':
        code = parse_int(parts[0]) if parts else 0
        return hw_bytes((0x1D << 11) | ((code & 0x3F) << 5) | RR_FUNCS[mnem])

    if mnem in ('entry', 'exit'):
        tokens = split_args(args)
        if mnem == 'entry':
            arg_token = ''
            if tokens and tokens[0] in ENTRY_ARGS and tokens[0]:
                arg_token = tokens.pop(0)
            save_token = ','.join(tokens)
            cnt = ENTRY_ARGS[arg_token]
            sl = SAVE_LIST[save_token]
            return hw_bytes((0x1D << 11) | (cnt << 8) | (sl << 5) | RR_FUNCS[mnem])
        fp_token = ''
        if tokens and tokens[-1] in ('$f0', '$f0-$f1'):
            fp_token = tokens.pop()
        save_token = ','.join(tokens)
        cnt = EXIT_FP[fp_token]
        sl = SAVE_LIST[save_token]
        return hw_bytes((0x1D << 11) | (cnt << 8) | (sl << 5) | RR_FUNCS[mnem])

    if mnem in LEXRA_FUNCS:
        rx = parse_creg(parts[0])
        ry = parse_creg(parts[1])
        return hw_bytes((0x1F << 11) | (rx << 8) | (ry << 5) | LEXRA_FUNCS[mnem])

    raise ValueError(f'unsupported mnemonic: {raw_mnem}')


def parse_listing_line(line: str) -> Optional[tuple[str, str]]:
    line = line.split('#', 1)[0].strip()
    if not line:
        return None
    m = LISTING_RE.match(line)
    if not m:
        raise ValueError(f'cannot parse line: {line!r}')
    mnemonic = m.group(1)
    args = m.group(2) or ''
    return mnemonic, args


def assemble_lines(lines: Iterable[str], vma: int) -> bytes:
    out = bytearray()
    for lineno, line in enumerate(lines, 1):
        parsed = parse_listing_line(line)
        if parsed is None:
            continue
        mnemonic, args = parsed
        pc = (vma + len(out)) & 0xFFFFFFFF
        try:
            out.extend(encode_line(mnemonic, args, pc))
        except Exception as exc:  # pragma: no cover - CLI path
            raise ValueError(f'line {lineno}: {exc}\n  {line.rstrip()}') from exc
    return bytes(out)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Assemble lexra_dis.py-style RX3081 MIPS16 listings into raw bytes.'
    )
    parser.add_argument('input', type=Path, help='listing or assembly text file')
    parser.add_argument('-o', '--output', type=Path, required=True, help='raw binary output')
    parser.add_argument('--vma', type=lambda s: int(s, 16), default=0x85000000,
                        help='starting virtual address in hex (default: 0x85000000)')
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    data = assemble_lines(args.input.read_text().splitlines(), args.vma)
    args.output.write_bytes(data)


if __name__ == '__main__':
    main()
