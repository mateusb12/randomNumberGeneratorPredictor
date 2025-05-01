#!/usr/bin/python3
import math
import struct
import sys
import numpy
import z3

from decimal import Decimal
from z3 import Or

""" 
Solving for seed states in XorShift128+ used in V8...
"""

# ── CONFIGURATION ───────────────────────────────────────────────────────────────
USE_MULTIPLIER = True  # True: sequence is floats in [0,1).  False: sequence is ints [0,multi).
multi = 5000  # only used if USE_MULTIPLIER==True
# ────────────────────────────────────────────────────────────────────────────────

# Array.from({ length: 15 }, () => Math.floor(Math.random() * 5000))
sequence = [
    0.9292888224345438,
    0.5421541414147769,
    0.7106233624583104,
    0.865568862624066,
    0.7306638748605095,
    0.86019465079271,
    0.4530332185107775,
    0.41316017789495985,
    0.505377722608071,
    0.20096562609021962,
    0.1360223967350339,
    0.8439080462166392,
    0.0887174515197855,
    0.28261488997191087,
    0.7176860465355501
]

# Build the lookup table of leading‐bit “categories” for each bin
boundaries = []
categories = []

for i in range(multi):
    number = i / multi + 1
    if Decimal(number) * Decimal(multi) < Decimal(i):
        number = numpy.nextafter(number, 1)
    if Decimal(number) * Decimal(multi) < Decimal(i):
        raise Exception("can not be")
    float_64 = struct.pack("d", number)
    u_long_long_64 = struct.unpack("<Q", float_64)[0]
    mantissa = u_long_long_64 & ((1 << 52) - 1)
    boundaries.append(mantissa)

highest_52_bit_number_int = (1 << 52) - 1
boundaries.append(highest_52_bit_number_int)


def count_leading_bits(n1, n2):
    bin1 = bin(n1)[2:].zfill(52)
    bin2 = bin(n2)[2:].zfill(52)
    prefix = []
    for b1, b2 in zip(bin1, bin2):
        if b1 == b2:
            prefix += b1
        else:
            break
    return prefix


for i in range(multi):
    lower = boundaries[i]
    upper = boundaries[i + 1] - 1
    categories.append(count_leading_bits(lower, upper))

# reverse the sequence (V8 pops from a LIFO pool)
sequence = sequence[::-1]

solver = z3.Solver()

# initial state variables
se_state0, se_state1, se_state2 = z3.BitVecs("se_state0 se_state1 se_state2", 64)

for output_random in sequence:
    # update XORShift128+ state
    s1, s0 = se_state0, se_state1
    se_state0 = s0
    s1 ^= s1 << 23
    s1 ^= z3.LShR(s1, 17)
    s1 ^= s0
    s1 ^= z3.LShR(s0, 26)
    se_state1 = s1

    # pick the right “category” based on float or int input
    if USE_MULTIPLIER:
        idx = int(Decimal(output_random) * Decimal(multi))
    else:
        idx = int(output_random)

    if not (0 <= idx < len(categories)):
        raise ValueError(f"Index {idx} out of range; check USE_MULTIPLIER and your sequence values.")

    cat_bits = categories[idx]
    cat_number = int(''.join(cat_bits), 2)

    # constrain the high bits of se_state0 to match
    shift = 12 + (52 - len(cat_bits))
    solver.add(cat_number == z3.LShR(se_state0, shift))

result = solver.check()

if result == z3.sat:
    model = solver.model()
    states = {d.name(): model[d] for d in model.decls()}
    print(states)

    # exclude this solution and see if there's another
    solver.add(Or([model[d] != d() for d in model.decls()]))
    if solver.check() == z3.sat:
        print("There is more than one solution.")
    else:
        print("There is a unique solution.")

    # recover the next output
    state0 = states["se_state0"].as_long()
    u_long_long_64 = (state0 >> 12) | 0x3FF0000000000000
    float_64 = struct.pack("<Q", u_long_long_64)
    next_sequence = struct.unpack("d", float_64)[0] - 1
    print(next_sequence)

    if USE_MULTIPLIER:
        # you want the raw float
        print("next random float:", next_sequence)
    else:
        # convert back into your integer bin index
        next_idx = int(Decimal(next_sequence) * Decimal(multi))
        print("next integer index:", next_idx)

elif result == z3.unsat:
    print("❌ Unsatisfiable: the given sequence cannot come from any XorShift128+ state.")
else:
    print("⚠️ Solver returned unknown. Try simplifying constraints or a different solver config.")
