#!/usr/bin/env python3
"""Resumen rápido de motor_control_*.log para validar Fase 4/5 FOV.

Uso:
  python Tools/check_fov_log.py path/to/motor_control_20260714.log
"""
from __future__ import annotations

import collections
import re
import sys
from pathlib import Path

UM = r"(?:µm|um|μs|�m)"  # log Windows a veces corrompe µm
RE_ATOM = re.compile(
    rf"FOV_ATOM.*?P,([AB]),([+-]?\d+),(\d+).*?err=([+-]?\d+(?:\.\d+)?){UM}"
)
RE_PULSE = re.compile(
    rf"FOV_PULSE.*?idx=(\d+).*?moved_adc=([+-]?\d+).*?err=([+-]?\d+(?:\.\d+)?)(?:→|->)([+-]?\d+(?:\.\d+)?){UM}.*?idx_next=(\d+)"
)
RE_METRICS = re.compile(
    rf"POINT_METRICS idx=(\d+) residual=\(([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?)\){UM} "
    rf"tol=(\d+(?:\.\d+)?) best=(\d+(?:\.\d+)?){UM} pulses=(\d+).*?accepted_valid=(\d+)"
)
RE_MAX = re.compile(r"FOV max_pulses|FOV sin_mejora|FOV gate_stable|FOV timeout")


def main() -> int:
    path = Path(sys.argv[1] if len(sys.argv) > 1 else "")
    if not path.is_file():
        print("Uso: check_fov_log.py <motor_control_YYYYMMDD.log>", file=sys.stderr)
        return 2
    text = path.read_text(encoding="utf-8", errors="ignore")
    atoms = RE_ATOM.findall(text)
    pulses = RE_PULSE.findall(text)
    metrics = RE_METRICS.findall(text)
    accepts = RE_MAX.findall(text)

    idx_hist = collections.Counter(int(a[2]) for a in atoms)
    print(f"Archivo: {path}")
    print(f"FOV_ATOM: {len(atoms)}  idx_hist={dict(sorted(idx_hist.items()))}")
    print(f"FOV_PULSE (adapt idx_next): {len(pulses)}")
    if pulses:
        nexts = collections.Counter(int(p[4]) for p in pulses)
        print(f"  idx_next hist={dict(sorted(nexts.items()))}")
    print(f"POINT_METRICS: {len(metrics)}")
    valid = 0
    for m in metrics:
        idx, rx, ry, tol, best, pulses_n, av = m
        ok = abs(float(rx)) <= float(tol) and abs(float(ry)) <= float(tol)
        valid += int(av) or int(ok)
        print(
            f"  P{idx}: residual=({rx},{ry}) best={best} pulses={pulses_n} "
            f"accepted_valid={av} in_tol={ok}"
        )
    print(f"Accept warnings: {len(accepts)}")
    print(f"in_tol or accepted_valid: {valid}/{len(metrics)}")
    # Heurística verde: idx no todo en un solo valor + algún punto en tol
    stuck = len(idx_hist) <= 1 and len(atoms) > 10
    print(f"idx_stuck_single={stuck}")
    return 0 if not stuck else 1


if __name__ == "__main__":
    raise SystemExit(main())
