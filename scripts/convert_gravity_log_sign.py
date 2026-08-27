#!/usr/bin/env python3
"""Convert a gravity residual log into the model's sign convention.

Logs written before the capture applied `gravity_feedforward_sign` compared the
raw motor torque against the raw model, so the residual carried twice the
gravity torque and a fit of them returned a scale near -1. This rewrites such a
log in place of a new file: `tau_measured_*` divided by the sign, `tau_error_*`
recomputed, and the untouched reading kept as `tau_raw_*`.

    scripts/convert_gravity_log_sign.py logs/left_arm_gravity_freedrive.csv
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

JOINTS = 7


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    parser.add_argument("--sign", type=float, default=-1.0)
    parser.add_argument("--output", default="", help="Default: <name>_model_convention.csv")
    args = parser.parse_args()
    if args.sign == 0.0:
        raise SystemExit("--sign must not be zero")

    source = Path(args.csv_path).expanduser().resolve()
    rows = list(csv.DictReader(source.open(encoding="utf-8")))
    if not rows:
        raise SystemExit(f"{source} holds no rows")
    if f"tau_raw_1" in rows[0]:
        raise SystemExit(f"{source} already carries tau_raw_*; it was written with the sign applied")

    target = Path(args.output) if args.output else source.with_name(
        f"{source.stem}_model_convention.csv"
    )
    fields = list(rows[0]) + [f"tau_raw_{i}" for i in range(1, JOINTS + 1)]
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            row = dict(row)
            for j in range(1, JOINTS + 1):
                raw = float(row[f"tau_measured_{j}"])
                measured = raw / args.sign
                row[f"tau_raw_{j}"] = raw
                row[f"tau_measured_{j}"] = measured
                row[f"tau_error_{j}"] = measured - float(row[f"tau_g_urdf_{j}"])
            writer.writerow(row)
    print(f"wrote {target} ({len(rows)} rows, sign {args.sign})")


if __name__ == "__main__":
    main()
