#!/usr/bin/env python3
"""Read step_response.csv -> print step-response metrics, write an SVG plot.

Dependency-free (no matplotlib needed here). Usage:
    python3 tools/plot_step.py [step_response.csv]
"""
import csv, sys

path = sys.argv[1] if len(sys.argv) > 1 else "step_response.csv"
rows = list(csv.DictReader(open(path)))
if not rows:
    sys.exit("empty csv: " + path)

t  = [float(r["t_ms"]) / 1000.0 for r in rows]
T  = [float(r["temp"])     for r in rows]
SP = [float(r["setpoint"]) for r in rows]

sp, t0 = SP[-1], T[0]
step = sp - t0

def first_at(frac):
    for ti, yi in zip(t, T):
        if step and (yi - t0) / step >= frac:
            return ti
    return float("nan")

t10, t90 = first_at(0.10), first_at(0.90)
tr = t90 - t10
peak = max(T); tp = t[T.index(peak)]
overshoot = (peak - sp) / step * 100.0 if step else 0.0
band = 0.02 * abs(step)
t_settle = 0.0
for ti, yi in zip(t, T):
    if abs(yi - sp) > band:
        t_settle = ti
sse = sp - T[-1]

print(f"file             : {path}")
print(f"samples          : {len(rows)}  ({t[-1]:.1f} s, 100 Hz control / 20 Hz ADC)")
print(f"setpoint         : {sp:.1f} C   (step from {t0:.1f} C)")
print(f"rise time 10-90% : {tr:.2f} s")
print(f"peak             : {peak:.1f} C @ {tp:.2f} s  ->  overshoot {overshoot:.1f} %")
print(f"settling (+-2%)  : {t_settle:.2f} s")
print(f"steady-state err : {sse:+.2f} C")

# The harness samples the tip on every 5th frame, so show the duty of both frame
# kinds next to the cycle average the plant actually sees.
if "duty" in rows[0]:
    duty = [float(r["duty"]) for r in rows]
    tail = [i for i in range(max(0, len(rows) - 100), len(rows) - 1)]
    ctrl = [duty[i] for i in tail if i % 5]
    samp = [duty[i] for i in tail if i % 5 == 0]
    if ctrl and samp:
        cycle = (sum(ctrl) + sum(samp)) / (len(ctrl) + len(samp))
        print(f"duty (control)   : {ctrl[-1]:.5f}")
        print(f"duty (sample)    : {samp[-1]:.5f}")
        print(f"duty (cycle)     : {cycle:.5f}")

# ---- ASCII plot: '*' temp, '.' setpoint ----
W, H = 64, 20
ymin = min(min(T), t0); ymax = max(max(T), sp)
pad = (ymax - ymin) * 0.05 or 1.0
ymin -= pad; ymax += pad
grid = [[" "] * W for _ in range(H)]
for i in range(W):
    xi = int(i * (len(t) - 1) / (W - 1))
    yi = int((ymax - T[xi]) / (ymax - ymin) * (H - 1))
    if 0 <= yi < H: grid[yi][i] = "*"
    ys = int((ymax - SP[xi]) / (ymax - ymin) * (H - 1))
    if 0 <= ys < H and grid[ys][i] == " ": grid[ys][i] = "."
print("\n  ASCII  ( * = temperature, . = setpoint )")
for r in range(H):
    ylab = ymax - (ymax - ymin) * r / (H - 1)
    print(f"{ylab:6.0f} |" + "".join(grid[r]))
print("       +" + "-" * W + f"\n        0{'':{W//2 - 1}}t ->{t[-1]:.0f}s")

# ---- SVG ----
Wp, Hp, ml, mr, mt, mb = 900, 420, 60, 20, 24, 40
sx = lambda v: ml + (v / t[-1]) * (Wp - ml - mr)
sy = lambda v: mt + (ymax - v) / (ymax - ymin) * (Hp - mt - mb)
def poly(series, color, width=2.0):
    pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(t, series))
    return f'<polyline fill="none" stroke="{color}" stroke-width="{width}" points="{pts}"/>'
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{Wp}" height="{Hp}" font-family="monospace">',
       '<rect width="100%" height="100%" fill="white"/>',
       f'<line x1="{ml}" y1="{mt}" x2="{ml}" y2="{Hp-mb}" stroke="#ccc"/>',
       f'<line x1="{ml}" y1="{Hp-mb}" x2="{Wp-mr}" y2="{Hp-mb}" stroke="#ccc"/>',
       poly(SP, "#999", 1.5), poly(T, "#c00", 2),
       f'<text x="{ml}" y="16" fill="#000">T12 PID step response  |  rise(10-90%) {tr:.2f}s  overshoot {overshoot:.1f}%  settle(+-2%) {t_settle:.2f}s  SSE {sse:+.2f}C</text>',
       f'<text x="{ml}" y="{Hp-12}" fill="#555">0s</text>',
       f'<text x="{Wp-mr-40}" y="{Hp-12}" fill="#555">{t[-1]:.0f}s</text>',
       '</svg>']
open("step_response.svg", "w").write("\n".join(svg))
print("\nwrote step_response.svg")
