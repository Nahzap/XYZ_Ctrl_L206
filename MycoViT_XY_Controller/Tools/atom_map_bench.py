#!/usr/bin/env python3
"""Mapa átomo A/B: dispara P,axis,sign,idx y mide ΔADC / Δµm.

Uso (CTRL_ENV, puerto libre — cerrar GUI):
  python Tools/atom_map_bench.py COM5
  python Tools/atom_map_bench.py COM5 --idxs 0,1,2,3 --reps 3

Protocolo: P,<A|B>,<+/-1>,<idx>  @ 1 Mbps, telemetría 6 campos.
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(1)

# Slopes provisionales (µm/ADC) — override con --sx/--sy si hay calibración fresca
DEFAULT_SX = 2.946375957572186
DEFAULT_SY = 2.8735632183908044


def read_frame(ser: serial.Serial, timeout_s: float = 0.5):
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout_s:
        line = ser.readline().decode("ascii", "ignore").strip()
        if not line or line[0] not in "-0123456789":
            continue
        parts = line.split(",")
        if len(parts) < 6:
            continue
        try:
            return {
                "pot_a": int(parts[0]),
                "pot_b": int(parts[1]),
                "s1": int(parts[2]),  # Y
                "s2": int(parts[3]),  # X
                "state": parts[4].strip(),
                "settled": parts[5].strip(),
                "raw": line,
            }
        except ValueError:
            continue
    return None


def wait_not_pulse(ser: serial.Serial, max_s: float = 0.5) -> None:
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < max_s:
        fr = read_frame(ser, 0.05)
        if fr and fr["state"] != "PULSE":
            return


def fire_and_measure(ser, axis: str, sign: int, idx: int, settle_s: float):
    """Lab/GUI: A=MOTOR_A=X=s2, B=MOTOR_B=Y=s1 (alineado con A,pwm_a,pwm_b)."""
    pre = read_frame(ser)
    if pre is None:
        return None
    key = "s2" if axis == "A" else "s1"
    adc0 = pre[key]
    cmd = f"P,{axis},{sign},{idx}\n"
    ser.write(cmd.encode("ascii"))
    ser.flush()
    # Esperar ventana ON+OFF (~t_on+2 ms + margen) + settle
    time.sleep(0.05 + settle_s)
    wait_not_pulse(ser, 0.4)
    post = read_frame(ser)
    if post is None:
        return None
    adc1 = post[key]
    return {
        "axis": axis,
        "sign": sign,
        "idx": idx,
        "adc0": adc0,
        "adc1": adc1,
        "d_adc": adc1 - adc0,
        "state0": pre["state"],
        "state1": post["state"],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Atom LUT bench (P command)")
    ap.add_argument("port", nargs="?", default="COM5")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--idxs", default="0,1,2,3,4,5,6,7")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--settle", type=float, default=0.15)
    ap.add_argument("--sx", type=float, default=DEFAULT_SX)
    ap.add_argument("--sy", type=float, default=DEFAULT_SY)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    idxs = [int(x) for x in args.idxs.split(",") if x.strip() != ""]

    out = Path(args.out) if args.out else Path(__file__).resolve().parent / (
        f"atom_map_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    )

    print(f"Open {args.port} @ {args.baud}")
    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"A,0,0\n")
    ser.flush()
    time.sleep(0.1)
    wait_not_pulse(ser)

    rows = []
    for axis in ("A", "B"):
        slope = args.sx if axis == "A" else args.sy  # A=X=s2, B=Y=s1
        for idx in idxs:
            for sign in (+1, -1):
                dadcs = []
                for r in range(args.reps):
                    m = fire_and_measure(ser, axis, sign, idx, args.settle)
                    if m is None:
                        print(f"FAIL {axis} sign={sign} idx={idx} rep={r}")
                        continue
                    dadcs.append(m["d_adc"])
                    dum = m["d_adc"] * slope
                    print(
                        f"{axis} sign={sign:+d} idx={idx} rep={r} "
                        f"d_adc={m['d_adc']:+d} d_um={dum:+.1f} "
                        f"adc {m['adc0']}->{m['adc1']}"
                    )
                    rows.append({**m, "d_um": round(dum, 3), "rep": r})
                    time.sleep(0.08)
                if dadcs:
                    print(
                        f"  >> {axis} sign={sign:+d} idx={idx} "
                        f"median_d_adc={statistics.median(dadcs):+.1f} "
                        f"n={len(dadcs)}"
                    )

    ser.write(b"B\n")
    ser.close()

    if rows:
        with out.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"CSV -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
