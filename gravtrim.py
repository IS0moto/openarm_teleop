#!/usr/bin/env python3
# Edit one element of leader.yaml GravityScale / GravityOffset.
# Usage:
#   python3 gravtrim.py                      # show current arrays
#   python3 gravtrim.py offset 7 0.03        # set J7 (1-based) GravityOffset = 0.03
#   python3 gravtrim.py scale  7 0.9         # set J7 GravityScale  = 0.9
import re, sys
P = "config/leader.yaml"

def load():
    return open(P).read()

def get(x, key):
    m = re.search(r'(%s:\s*\[)([^\]]*)(\])' % key, x)
    vals = [v.strip() for v in m.group(2).split(',')]
    return m, vals

def show(x):
    for key in ("GravityScale", "GravityOffset"):
        _, vals = get(x, key)
        print(f"{key:14s} = [{', '.join(vals)}]")

x = load()
if len(sys.argv) == 1:
    show(x); sys.exit(0)

which = {"scale": "GravityScale", "offset": "GravityOffset"}[sys.argv[1]]
joint = int(sys.argv[2])            # 1-based joint index (7 = J7)
value = float(sys.argv[3])
m, vals = get(x, which)
vals[joint - 1] = f"{value}"
x = x[:m.start(2)] + ", ".join(vals) + x[m.end(2):]
open(P, "w").write(x)
print(f"set {which}[J{joint}] = {value}")
show(x)
