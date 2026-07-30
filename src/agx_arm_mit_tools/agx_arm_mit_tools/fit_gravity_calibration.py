from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from agx_arm_mit_controller.feedforward_model import CalibrationModel, save_calibration_model


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Fit a simple scale+bias calibration model from gravity compare CSV")
	parser.add_argument("csv_paths", nargs="+")
	parser.add_argument("--output", default="config/nero_gravity_calibration.json")
	parser.add_argument("--min-samples", type=int, default=20)
	parser.add_argument("--min-joint-span", type=float, default=0.05)
	parser.add_argument("--min-model-span", type=float, default=0.1)
	parser.add_argument("--max-abs-scale", type=float, default=10.0)
	parser.add_argument("--max-abs-bias", type=float, default=16.0)
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


def _span(values: list[float]) -> float:
	if not values:
		return 0.0
	return max(values) - min(values)


def _provenance_path(path: Path) -> str:
	"""Render an input path for the stored source_log provenance field.

	Kept relative to the invocation directory when possible, matching the
	cwd-relative --output default. Absolute paths would otherwise bake this
	machine's home directory into a source-managed config and make the
	regenerated file differ per host.
	"""
	try:
		return str(path.resolve().relative_to(Path.cwd().resolve()))
	except ValueError:
		return str(path)


def _fit_joint_calibration(
	joint_positions: list[float],
	tau_model: list[float],
	tau_measured: list[float],
	*,
	min_joint_span: float,
	min_model_span: float,
	max_abs_scale: float,
	max_abs_bias: float,
) -> tuple[float, float, bool, str]:
	joint_span = _span(joint_positions)
	model_span = _span(tau_model)

	if joint_span < min_joint_span:
		return 1.0, 0.0, False, (
			f"joint span {joint_span:.6f} rad is below the required {min_joint_span:.6f} rad"
		)
	if model_span < min_model_span:
		return 1.0, 0.0, False, (
			f"gravity model span {model_span:.6f} N*m is below the required {min_model_span:.6f} N*m"
		)

	scale, bias = _fit_scale_and_bias(tau_model, tau_measured)
	if not math.isfinite(scale) or not math.isfinite(bias):
		return 1.0, 0.0, False, "fit produced a non-finite scale or bias"
	if abs(scale) > max_abs_scale:
		return 1.0, 0.0, False, (
			f"fitted scale {scale:.6f} exceeds the configured safety bound {max_abs_scale:.6f}"
		)
	if abs(bias) > max_abs_bias:
		return 1.0, 0.0, False, (
			f"fitted bias {bias:.6f} N*m exceeds the configured safety bound {max_abs_bias:.6f} N*m"
		)
	return scale, bias, True, ""


def _load_csv_rows(csv_paths: list[str]) -> tuple[list[dict[str, str]], list[Path]]:
	rows: list[dict[str, str]] = []
	resolved_paths: list[Path] = []
	for csv_path in csv_paths:
		path = Path(csv_path).expanduser().resolve()
		resolved_paths.append(path)
		with path.open("r", encoding="utf-8") as handle:
			rows.extend(csv.DictReader(handle))
	return rows, resolved_paths


def main() -> None:
	args = parse_args()
	rows, resolved_paths = _load_csv_rows(args.csv_paths)
	if len(rows) < args.min_samples:
		raise SystemExit(
			f"Need at least {args.min_samples} samples to fit calibration, got {len(rows)}"
		)

	joint_names = [f"joint{i}" for i in range(1, 8)]
	scale = []
	bias = []
	fitted_joint_names: list[str] = []
	skipped_messages: list[str] = []
	for joint_index in range(1, 8):
		joint_positions = [float(row[f"q{joint_index}"]) for row in rows]
		tau_model = [float(row[f"tau_g_urdf_{joint_index}"]) for row in rows]
		tau_measured = [float(row[f"tau_measured_{joint_index}"]) for row in rows]
		s, b, fitted, reason = _fit_joint_calibration(
			joint_positions,
			tau_model,
			tau_measured,
			min_joint_span=args.min_joint_span,
			min_model_span=args.min_model_span,
			max_abs_scale=args.max_abs_scale,
			max_abs_bias=args.max_abs_bias,
		)
		scale.append(s)
		bias.append(b)
		joint_name = joint_names[joint_index - 1]
		if fitted:
			fitted_joint_names.append(joint_name)
		else:
			skipped_messages.append(f"{joint_name}: {reason}")

	if not fitted_joint_names:
		details = "\n".join(skipped_messages)
		raise SystemExit(
			"Refusing to save calibration: no joints had sufficient excitation for a safe fit.\n"
			"Capture a log across multiple distinct static poses before fitting again.\n"
			f"{details}"
		)

	note = "Least-squares scale and bias fit against measured motor torque feedback."
	if skipped_messages:
		note += " Some joints were left uncalibrated because the input log did not excite them sufficiently."

	model = CalibrationModel(
		joint_names=joint_names,
		scale=scale,
		bias=bias,
		source_log="\n".join(_provenance_path(path) for path in resolved_paths),
		note=note,
	)
	output_path = save_calibration_model(model, args.output)
	print(f"Saved calibration model to {output_path}")
	if len(resolved_paths) > 1:
		print(f"Loaded {len(rows)} samples from {len(resolved_paths)} CSV files")
	if skipped_messages:
		print("Skipped joints:")
		for message in skipped_messages:
			print(f"  - {message}")
	print(f"scale={scale}")
	print(f"bias={bias}")


__all__ = ["main"]