from __future__ import annotations

import argparse
import csv
from pathlib import Path

from .feedforward_model import CalibrationModel, save_calibration_model


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fit a simple scale+bias calibration model from gravity compare CSV")
    parser.add_argument("csv_path")
    parser.add_argument("--output", default="config/nero_gravity_calibration.json")
    return parser.parse_args()


def _fit_scale_and_bias(x: list[float], y: list[float]) -> tuple[float, float]:
    count = len(x)
    if count == 0:
        return 1.0, 0.0
    x_mean = sum(x) / count
    y_mean = sum(y) / count
    denom = sum((value - x_mean) ** 2 for value in x)
    if abs(denom) < 1e-9:
        return 1.0, y_mean - x_mean
    scale = sum((x[i] - x_mean) * (y[i] - y_mean) for i in range(count)) / denom
    bias = y_mean - scale * x_mean
    return scale, bias


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv_path).expanduser().resolve()
    rows = list(csv.DictReader(csv_path.open("r", encoding="utf-8")))
    joint_names = [f"joint{i}" for i in range(1, 8)]
    scale = []
    bias = []
    for joint_index in range(1, 8):
        tau_model = [float(row[f"tau_g_urdf_{joint_index}"]) for row in rows]
        tau_measured = [float(row[f"tau_measured_{joint_index}"]) for row in rows]
        s, b = _fit_scale_and_bias(tau_model, tau_measured)
        scale.append(s)
        bias.append(b)
    model = CalibrationModel(
        joint_names=joint_names,
        scale=scale,
        bias=bias,
        source_log=str(csv_path),
        note="Least-squares scale and bias fit against measured motor torque feedback.",
    )
    output_path = save_calibration_model(model, args.output)
    print(f"Saved calibration model to {output_path}")
    print(f"scale={scale}")
    print(f"bias={bias}")