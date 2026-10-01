#!/usr/bin/env python3
"""Assembler for the motor-model engine (src/motor_model.v).

    python3 tools/masm.py src/model.masm src/model_prog.hex src/model_data.hex

Instruction word: [31:27] op  [26:24] cond  [23:16] a  [15:8] b  [7:0] d
Operands are data-RAM names (see DATA below) or ports (A operand only). The pipeline has no
interlocks: a value stored (STO) or a port updated by instruction i can be read by instruction
i + 3 or later (W32ADD's w32: i + 6). The assembler inserts NOPs where the program is tighter than that and reports
them, so the program text can stay in logical order.
"""
import sys, re

OPS = {'NOP': 0, 'LD': 1, 'LDN': 2, 'ADDA': 3, 'SUBA': 4, 'MUL': 5, 'MAC': 6, 'SHR': 7,
       'MAX': 9, 'MIN': 10, 'MAX0': 11, 'CLAMP': 12, 'STO': 13, 'DTC': 14, 'W32LD': 15, 'W32ADD': 16,
       'CMADD': 17, 'CMZ': 18, 'DIV': 19, 'WAIT': 20, 'THADV': 21, 'END': 22, 'ACCW': 24,
       'CMPS': 25, 'CMPN': 26, 'SELA': 27, 'SELNA': 28, 'SELNOT': 29, 'RCP': 30, 'DTS': 23}
# compares are two instructions in hardware (flag, then select): MAX/MIN/MAXN expand to those pairs
MACROS = {'MAX': ('CMPS', 'SELA'), 'MIN': ('CMPS', 'SELNOT'), 'MAXN': ('CMPN', 'SELNA')}
CONDS = {None: 0, 'mode1': 1, 'mode2': 2, 'cm': 3, 'nocm': 4, 'dt': 5}

# data RAM: parameters at SPI register - 0x20 (written by the link), then constants and temporaries
DATA = {'W_SET': 0x01, 'KE': 0x03, 'KT': 0x04, 'LOAD': 0x05, 'DAMP': 0x06, 'INVJ': 0x07,
        'CM_KP': 0x09, 'CM_KI': 0x0A, 'MARG2': 0x0B, 'DTF2': 0x0C,
        'OFS_A': 0x1A, 'OFS_B': 0x1B, 'OFS_C': 0x1C, 'OFS_BUS': 0x1D, 'RSUB': 0x1F,
        'K21845': 0x20}
TEMPS = ['IA', 'IB', 'IC', 'VB', 'NA', 'NB', 'NC', 'T0', 'HVB', 'E', 'I0', 'DTV', 'EA', 'EB', 'EC',
         'IQ', 'TQ', 'NET', 'CMP', 'VCM', 'UA', 'UB', 'UC', 'MX', 'OFF', 'VA', 'VBB', 'VCC',
         'MG', 'LIM', 'SPR', 'SH1', 'SH2', 'TOFF', 'DMP']
for i, n in enumerate(TEMPS):
    DATA[n] = 0x40 + i
CONSTS = {0x20: 21845}
PORTS = {'SINC_A': 0xE0, 'SINC_B': 0xE1, 'SINC_C': 0xE2, 'SINC_BUS': 0xE3, 'S_A': 0xE4, 'S_B': 0xE5,
         'S_C': 0xE6, 'W16': 0xE7, 'CM_HI': 0xE8, 'ZERO': 0xE9,
         'R_A': 0xEA, 'R_B': 0xEB, 'R_C': 0xEC}      # S_x: sines at the advanced angle, R_x: at the angle
# which ops read which fields, and what they write (for hazard spacing)
READS_A = {'LD', 'LDN', 'ADDA', 'SUBA', 'MUL', 'MAC', 'CMPS', 'CMPN', 'SELA', 'SELNA', 'SELNOT', 'DTC', 'DTS',
           'W32LD', 'DIV', 'RCP'}
READS_B = {'MUL', 'MAC', 'DTC', 'DIV'}
GAP = 3
# results that land later than GAP: W32ADD's shift takes three more clk (DSPs) before w32 changes
LATENCY = {'W32ADD': 6}
# STO / SHR shift amounts the engine supports (a fixed select, not a barrel shifter): value -> code
SHIFTS = {0: 0, 1: 1, 2: 2, 8: 3, 14: 4, 15: 5}


def shcode(v):
    if int(v) not in SHIFTS:
        raise SystemExit(f'shift {v} not supported (use {sorted(SHIFTS)})')
    return SHIFTS[int(v)]


def operand(tok):
    if tok in PORTS:
        return PORTS[tok], 'P:' + tok
    if tok in DATA:
        return DATA[tok], 'D:' + tok
    raise SystemExit(f'unknown operand {tok}')


def assemble(src):
    prog = []            # (word, text, reads, writes)
    for ln, line in enumerate(open(src), 1):
        line = line.split(';')[0].strip()
        if not line:
            continue
        cond = None
        m = re.search(r'\[(\w+)\]\s*$', line)
        if m:
            cond = m.group(1); line = line[:m.start()].strip()
        parts = [p.strip() for p in re.split(r'[\s,]+', line) if p.strip()]
        op, args = parts[0].upper(), parts[1:]
        if op in MACROS:
            for sub in MACROS[op]:
                a_, ra = operand(args[0])
                word = (OPS[sub] << 27) | (CONDS[cond] << 24) | (a_ << 16)
                prog.append((word, f'{sub} {args[0]}  ({op})' + (f' [{cond}]' if cond else ''), {ra}, set(), GAP))
            continue
        if op == 'DTC':
            # DTC i, v: DTS latches the sign of i (against the band) a clk ahead, so the adder's
            # operand can be inverted before the Y stage
            a_, ra = operand(args[0])
            prog.append(((OPS['DTS'] << 27) | (CONDS[cond] << 24) | (a_ << 16),
                         f'DTS {args[0]}  (DTC)' + (f' [{cond}]' if cond else ''), {ra}, set(), GAP))
        if op not in OPS or cond not in CONDS:
            raise SystemExit(f'{src}:{ln}: bad instruction {line!r} [{cond}]')
        a = b = d = 0
        reads, writes = set(), set()
        if op == 'STO' and int(args[1]) not in (0, 15):
            # the engine's STO shifts by 0 or 15: other amounts go through SHR first (same result)
            prog.append(((OPS['SHR'] << 27) | (CONDS[cond] << 24) | (shcode(args[1]) << 16),
                         f'SHR {args[1]}  (STO {args[0]}, {args[1]})', set(), set(), GAP))
            args = [args[0], '0']
        if op == 'STO':
            d, _ = operand(args[0]); a = 1 if int(args[1]) == 15 else 0; writes.add('D:' + args[0])
        elif op == 'SHR':
            a = shcode(args[0])
        elif op == 'WAIT':
            d = int(args[0])
        elif op == 'DIV':                        # DIV v, leg: duty = 0x8000 + v / vbus (needs RCP first)
            a, ra = operand(args[0]); d = int(args[1]); reads |= {ra}
        elif op in READS_A:
            a, ra = operand(args[0]); reads.add(ra)
            if op in READS_B:
                b, rb = operand(args[1]); reads.add(rb)
        if op in ('W32LD', 'W32ADD'):
            writes.add('P:W16')
        if op == 'ACCW':
            reads.add('P:W16')                  # reads w32, which W32ADD commits a clk late
        if op in ('CMADD', 'CMZ'):
            writes.add('P:CM_HI')
        word = (OPS[op] << 27) | (CONDS[cond] << 24) | (a << 16) | (b << 8) | d
        prog.append((word, f'{line} [{cond}]' if cond else line, reads, writes, LATENCY.get(op, GAP)))
    # insert NOPs so every read is >= GAP instructions after the write it depends on
    out, last_write, nops = [], {}, 0
    for word, text, reads, writes, lat in prog:
        need = max([last_write[r] - len(out) for r in reads if r in last_write] + [0])
        for _ in range(need):
            out.append((0, 'NOP (inserted)')); nops += 1
        for w in writes:
            last_write[w] = len(out) + lat
        out.append((word, text))
    if len(out) > 256:
        raise SystemExit('program too long')
    return out, nops


def main():
    src, prog_hex, data_hex = sys.argv[1:4]
    out, nops = assemble(src)
    with open(prog_hex, 'w') as f:
        for i in range(256):
            f.write('%08x\n' % (out[i][0] if i < len(out) else 0))
    with open(data_hex, 'w') as f:
        for i in range(256):
            f.write('%04x\n' % CONSTS.get(i, 0))
    print(f'{src}: {len(out)} instructions ({nops} NOPs inserted)')
    if '-l' in sys.argv:
        for i, (w, t) in enumerate(out):
            print(f'{i:3d}  {w:08x}  {t}')


if __name__ == '__main__':
    main()
