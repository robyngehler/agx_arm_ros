from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from agx_arm_mit_controller.gravity_model import GravityModelError, create_gravity_model


def wait_until(condition_fn, timeout_s: float, sleep_s: float = 0.1, description: str = "condition"):
	start = time.monotonic()
	while time.monotonic() - start < timeout_s:
		result = condition_fn()
		if result:
			return result
		time.sleep(sleep_s)
	raise TimeoutError(f"Timed out waiting for {description} after {timeout_s:.1f}s")


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Compare measured motor torques against URDF gravity torques")
	parser.add_argument("--interface", default="socketcan")
	parser.add_argument("--can-port", default="can0")
	parser.add_argument("--duration", type=float, default=10.0)
	parser.add_argument("--rate", type=float, default=10.0)
	parser.add_argument("--backend", default="pinocchio")
	parser.add_argument("--urdf-path", default="")
	parser.add_argument("--csv-path", default="logs/nero_urdf_gravity_compare.csv")
	parser.add_argument("--append", action="store_true", help="Append samples to an existing CSV instead of overwriting it")
	parser.add_argument("--enable-timeout", type=float, default=5.0)
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	try:
		gravity_model = create_gravity_model(args.backend, args.urdf_path or None)
	except GravityModelError as exc:
		raise SystemExit(str(exc))

	cfg = create_agx_arm_config(
		robot=ArmModel.NERO,
		firmeware_version=NeroFW.DEFAULT,
		interface=args.interface,
		channel=args.can_port,
	)
	robot = AgxArmFactory.create_arm(cfg)
	csv_path = Path(args.csv_path).expanduser().resolve()
	csv_path.parent.mkdir(parents=True, exist_ok=True)
	csv_field_names = [
		"time",
		*[f"q{i}" for i in range(1, 8)],
		*[f"tau_measured_{i}" for i in range(1, 8)],
		*[f"tau_g_urdf_{i}" for i in range(1, 8)],
		*[f"tau_error_{i}" for i in range(1, 8)],
	]
	file_mode = "a" if args.append else "w"
	write_header = not args.append or not csv_path.exists() or csv_path.stat().st_size == 0

	try:
		robot.connect()
		wait_until(robot.enable, timeout_s=args.enable_timeout, description="robot enable")
		robot.set_normal_mode()
		wait_until(
			lambda: robot.get_firmware(timeout=1.0, min_interval=0.2),
			timeout_s=args.enable_timeout,
			sleep_s=0.2,
			description="firmware feedback",
		)

		start = time.monotonic()
		period = 1.0 / args.rate
		with csv_path.open(file_mode, newline="", encoding="utf-8") as handle:
			writer = csv.writer(handle)
			if write_header:
				writer.writerow(csv_field_names)
			while time.monotonic() - start < args.duration:
				loop_start = time.monotonic()
				joint_angles = robot.get_joint_angles()
				if joint_angles is None:
					time.sleep(period)
					continue
				q = [float(value) for value in joint_angles.msg]
				tau_measured = []
				for joint_index in range(1, 8):
					motor_state = robot.get_motor_states(joint_index)
					tau_measured.append(0.0 if motor_state is None else float(motor_state.msg.torque))
				tau_model = gravity_model.compute_gravity(q)
				tau_error = [tau_measured[i] - tau_model[i] for i in range(7)]
				writer.writerow([time.monotonic() - start, *q, *tau_measured, *tau_model, *tau_error])
				print(f"q={q}")
				print(f"tau_measured={tau_measured}")
				print(f"tau_g_urdf={tau_model}")
				print(f"tau_error={tau_error}")
				time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
	finally:
		try:
			robot.disconnect()
		except Exception:
			pass


__all__ = ["main"]