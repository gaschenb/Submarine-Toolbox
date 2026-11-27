"""Headless runs: `submarine-run --depth 100 --heading 90 --speed 8 --time 600`."""

import argparse
import csv
import math
import sys

from submarine_toolbox import Simulation
from submarine_toolbox.subsystems import build_default_boat

KN = 0.5144
COLS = ("t", "north", "east", "depth", "heading_deg", "pitch_deg", "speed_kn",
        "rudder_deg", "planes_deg", "rpm", "soc")

def main(argv=None):
    ap = argparse.ArgumentParser(prog="submarine-run", description=__doc__)
    ap.add_argument("--depth", type=float, default=100.0, help="ordered depth [m]")
    ap.add_argument("--heading", type=float, default=0.0, help="ordered heading [deg]")
    ap.add_argument("--speed", type=float, default=6.0, help="ordered speed [kn]")
    ap.add_argument("--start-depth", type=float, default=15.0)
    ap.add_argument("--time", type=float, default=600.0, help="sim duration [s]")
    ap.add_argument("--every", type=float, default=1.0, help="log interval [s]")
    ap.add_argument("--csv", help="write the log here (default: summary only)")
    a = ap.parse_args(argv)

    sim = build_default_boat(Simulation())
    b = sim.bus
    sim.eta[2] = a.start_depth
    b["order.depth"] = a.depth
    b["order.heading"] = math.radians(a.heading % 360.0)
    b["order.speed"] = a.speed * KN
    n = max(int(round(a.every / sim.dt)), 1)
    rows = []
    for _ in range(int(a.time / (n * sim.dt))):
        sim.step(n)
        e = sim.eta
        rows.append((sim.time, e[0], e[1], e[2], math.degrees(e[5]) % 360,
                     math.degrees(e[4]), b["state.u"] / KN,
                     math.degrees(b["act.rudder"]), math.degrees(b["act.planes"]),
                     b.get("propulsion.rpm", 0.0), b.get("power.soc", 0.0)))
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(COLS)
            w.writerows([f"{x:.4g}" for x in r] for r in rows)
    r = rows[-1]
    print(f"t={r[0]:.0f} s  depth={r[3]:.1f} m  heading={r[4]:.1f}°  "
          f"speed={r[6]:.1f} kn  pitch={r[5]:+.1f}°  soc={100 * r[10]:.0f}%")
    return 0

if __name__ == "__main__":
    sys.exit(main())
