"""Drive the arm through a pose set and log gravity residuals at each hold.

`compare_gravity` samples wherever the arm happens to be, so a dataset with
enough span per joint has to be hand-guided. This commands the poses instead,
settles at each one, and writes the same CSV `fit_gravity_calibration` reads:

    agx_arm_gravity_pose_sweep --arm-side right --custom-model <duo_system.urdf.xacro> \\
        --csv-path logs/right_hand_gravity.csv
    agx_arm_fit_gravity_calibration logs/right_hand_gravity.csv \\
        --output config/nero_right_hand_calibration.json

The gravity URDF is resolved through the same `resolve_gravity_urdf_path` the MIT
controller uses at bring-up, so the model being fitted is the model that will run
— body mount baked in, hand subtree frozen.

This owns the arm's SDK session, so the MIT stack must be down while it runs: one
owner of a device session at a time. It never calls `disable()` and never calls
the vendor emergency stop; an interrupt re-asserts the firmware hold at the pose
the arm is at (`docs/sprint_refactor/reference/emergency_stop_ladder.md`).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

JOINT_COUNT = 7
#: Thresholds `fit_gravity_calibration` rejects a joint below. Reported per joint
#: at the end, so a dataset that cannot be fitted is visible before the fit runs.
MIN_JOINT_SPAN_RAD = 0.05
MIN_MODEL_SPAN_NM = 0.1
#: Per-side CAN interface and protocol tier. The two arms run different,
#: unflashable firmware; anything derived from the protocol is per tier.
SIDE_DEFAULTS = {
	"right": ("can_nero_right", "DEFAULT"),
	"left": ("can_nero_left", "V111"),
}


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description="Drive a pose set and log gravity residuals for fit_gravity_calibration",
	)
	parser.add_argument("--arm-side", choices=sorted(SIDE_DEFAULTS), default="right")
	parser.add_argument("--can-port", default="", help="Overrides the side's default interface")
	parser.add_argument("--interface", default="socketcan")
	parser.add_argument(
		"--firmware", default="", choices=["", "DEFAULT", "V111", "V112"],
		help="Protocol tier override; empty picks the side's known tier",
	)

	parser.add_argument("--backend", default="pinocchio")
	parser.add_argument(
		"--urdf-path", default="",
		help="Gravity URDF to compare against. Wins over --custom-model.",
	)
	parser.add_argument(
		"--custom-model", default="",
		help="Xacro the gravity URDF is derived from (e.g. duo_system.urdf.xacro), "
			 "so the body mount and the hand are in the model being fitted",
	)
	parser.add_argument("--custom-model-xacro-args", default="")
	parser.add_argument("--effector-type", default="omnihand")
	parser.add_argument(
		"--hand-joints", default="",
		help="JSON object of hand joint angles by URDF name, for an articulated "
			 "hand payload. Omit when the hand is at the pose the URDF freezes.",
	)

	parser.add_argument(
		"--poses", default="",
		help="YAML/JSON pose file. Either a list of 7-value lists, or the "
			 "arm_config 'poses:' mapping of {name: {q: [...]}}.",
	)
	parser.add_argument(
		"--sweep-joints", default="2,3,4,6",
		help="Joints the generated sweep moves, one at a time from the start pose",
	)
	parser.add_argument("--sweep-steps", type=int, default=5)
	parser.add_argument(
		"--sweep-fraction", type=float, default=0.5,
		help="Share of each joint's travel the sweep spans, centred on the start pose",
	)

	parser.add_argument("--csv-path", default="logs/gravity_pose_sweep.csv")
	parser.add_argument("--append", action="store_true")
	parser.add_argument("--dwell", type=float, default=2.0, help="Seconds sampled per pose")
	parser.add_argument("--rate", type=float, default=10.0)
	parser.add_argument("--speed-percent", type=int, default=20)
	parser.add_argument(
		"--settle-tolerance", type=float, default=0.02,
		help="Rad within which a joint counts as arrived",
	)
	parser.add_argument("--settle-timeout", type=float, default=20.0)
	parser.add_argument(
		"--max-step", type=float, default=1.2,
		help="Largest per-joint jump a single move may command; a pose further "
			 "than this is refused rather than swung to",
	)
	parser.add_argument(
		"--skip-unreached", action="store_true",
		help="Log a pose the arm did not reach and continue. Default aborts, "
			 "because a joint short of its target may be pressing on something.",
	)
	parser.add_argument(
		"--feedforward-sign", type=float, default=-1.0,
		help="The MIT controller's gravity_feedforward_sign. The measured torque is "
			 "divided by it, so the logged residual is in the model's own sign "
			 "convention; a mismatch shows up as a fitted scale of -1.",
	)
	parser.add_argument("--enable-timeout", type=float, default=5.0)
	parser.add_argument("--dry-run", action="store_true", help="Print the plan, touch no hardware")
	parser.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
	return parser.parse_args()


# --- poses -------------------------------------------------------------------

def joint_limits(urdf_path: str | Path) -> list[tuple[float, float]]:
	"""(lower, upper) for joint1..joint7, read from the gravity URDF."""
	root = ET.parse(str(Path(urdf_path).expanduser().resolve())).getroot()
	found: dict[str, tuple[float, float]] = {}
	for joint in root.iter("joint"):
		limit = joint.find("limit")
		if limit is None or joint.get("type") not in ("revolute", "prismatic"):
			continue
		name = joint.get("name") or ""
		if name.endswith(tuple(f"joint{i}" for i in range(1, JOINT_COUNT + 1))):
			found[name[-6:]] = (float(limit.get("lower", 0.0)), float(limit.get("upper", 0.0)))
	missing = [f"joint{i}" for i in range(1, JOINT_COUNT + 1) if f"joint{i}" not in found]
	if missing:
		raise SystemExit(f"{urdf_path} has no limits for {missing}")
	return [found[f"joint{i}"] for i in range(1, JOINT_COUNT + 1)]


def load_poses(path: str | Path, arm_side: str = "") -> list[tuple[str, list[float]]]:
	"""Named 7-value poses from a list file or an arm_config 'poses:' mapping.

	``arm_side`` drops entries a stored ``robot_id`` assigns to the other arm, so
	the coordinator's pose library can be pointed at directly; it mixes per-arm
	and 14-value ``both_arms`` entries, and only the matching side is commandable
	here.
	"""
	import yaml

	data = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8")) or {}
	if isinstance(data, dict):
		data = data.get("arm_executor", data)
	if isinstance(data, dict):
		data = data.get("poses", data)
	poses: list[tuple[str, list[float]]] = []
	skipped = 0
	if isinstance(data, dict):
		entries = list(data.items())
	else:
		entries = [(f"pose{index + 1}", entry) for index, entry in enumerate(data)]
	for name, entry in entries:
		robot_id = str(entry.get("robot_id", "")) if isinstance(entry, dict) else ""
		if arm_side and robot_id and robot_id != f"{arm_side}_arm":
			skipped += 1
			continue
		vector = entry.get("q", entry.get("joints")) if isinstance(entry, dict) else entry
		values = [float(v) for v in vector or []]
		if len(values) != JOINT_COUNT:
			# A both_arms pose is 14 values; taking half of it silently would
			# command the wrong arm's angles.
			raise SystemExit(
				f"pose '{name}' has {len(values)} values, expected {JOINT_COUNT}"
			)
		poses.append((str(name), values))
	if skipped:
		print(f"  {skipped} pose(s) belong to another robot_id and were skipped")
	if not poses:
		raise SystemExit(f"{path} holds no pose for the {arm_side or 'requested'} arm")
	return poses


def build_sweep(
	start: list[float],
	limits: list[tuple[float, float]],
	joints: list[int],
	steps: int,
	fraction: float,
) -> list[tuple[str, list[float]]]:
	"""One joint at a time from the start pose, over a share of its travel.

	One joint at a time rather than a grid: the fit is per joint, and combining
	extremes puts the arm in poses nobody chose.

	Each joint is traversed monotonically — up to its top, down through its
	bottom, back to where it started — so no single move is larger than one grid
	interval. Jumping straight to an end instead makes the first move of every
	joint half its span, which is the one move nobody is expecting.
	"""
	poses: list[tuple[str, list[float]]] = [("start", list(start))]
	for joint in joints:
		index = joint - 1
		lower, upper = limits[index]
		half = 0.5 * fraction * (upper - lower)
		low = max(lower, start[index] - half)
		high = min(upper, start[index] + half)
		if high - low < MIN_JOINT_SPAN_RAD:
			print(f"  joint{joint}: only {high - low:.3f} rad of travel here, skipped")
			continue
		count = max(2, steps)
		grid = [low + (high - low) * step / (count - 1) for step in range(count)]
		here = min(range(count), key=lambda step: abs(grid[step] - start[index]))
		order = list(range(here, count)) + list(range(count - 2, -1, -1)) + list(range(1, here + 1))
		emitted = 0
		previous = start[index]
		for step in order:
			if abs(grid[step] - previous) < 1e-9:
				continue
			previous = grid[step]
			emitted += 1
			pose = list(start)
			pose[index] = grid[step]
			poses.append((f"joint{joint}_{emitted}", pose))
		poses.append((f"joint{joint}_back", list(start)))
	return poses


def check_reachable(
	poses: list[tuple[str, list[float]]],
	limits: list[tuple[float, float]],
	start: list[float],
	max_step: float,
) -> None:
	"""Refuse a pose outside the limits or further than one move should swing."""
	problems: list[str] = []
	previous = start
	for name, pose in poses:
		for index, value in enumerate(pose):
			lower, upper = limits[index]
			if not math.isfinite(value) or value < lower or value > upper:
				problems.append(
					f"{name}: joint{index + 1}={value:.4f} outside [{lower:.4f}, {upper:.4f}]"
				)
			step = abs(value - previous[index])
			if step > max_step:
				problems.append(
					f"{name}: joint{index + 1} jumps {step:.3f} rad from the previous pose "
					f"(--max-step {max_step:.3f})"
				)
		previous = pose
	if problems:
		raise SystemExit("refusing to run:\n  " + "\n  ".join(problems))


# --- hardware ----------------------------------------------------------------

def read_joints(robot) -> list[float] | None:
	angles = robot.get_joint_angles()
	return None if angles is None else [float(value) for value in angles.msg]


def read_torques(robot) -> list[float]:
	torques = []
	for joint_index in range(1, JOINT_COUNT + 1):
		state = robot.get_motor_states(joint_index)
		torques.append(0.0 if state is None else float(state.msg.torque))
	return torques


def move_and_settle(robot, target: list[float], tolerance: float, timeout: float) -> list[float]:
	"""Command the pose and wait until the arm stops moving. Returns where it landed.

	Both conditions matter: inside tolerance while still travelling is a sample
	taken mid-motion, where the measured torque carries acceleration too.
	"""
	robot.move_j(list(target))
	deadline = time.monotonic() + timeout
	previous = read_joints(robot)
	still_since = None
	while time.monotonic() < deadline:
		time.sleep(0.1)
		current = read_joints(robot)
		if current is None or previous is None:
			previous = current
			continue
		moved = max(abs(current[i] - previous[i]) for i in range(JOINT_COUNT))
		previous = current
		if moved > 0.002:
			still_since = None
			continue
		still_since = still_since or time.monotonic()
		if time.monotonic() - still_since >= 0.5:
			return current
	return previous or list(target)


def hold_here(robot) -> None:
	"""Re-assert the firmware position hold at the current pose.

	The stop ladder ends here: the vendor emergency stop applies damping without
	stiffness, and a de-energized Nero has no brakes.
	"""
	try:
		current = read_joints(robot)
		if current is not None:
			robot.move_j(current)
			print(f"holding at {[round(v, 4) for v in current]}")
		else:
			print("no trustworthy pose to hold — nothing commanded")
	except Exception as exc:  # noqa: BLE001 - a failed hold must not mask the exit
		print(f"could not re-assert the hold: {exc}")


# --- reporting ---------------------------------------------------------------

def report_span(rows: list[dict]) -> None:
	"""Per-joint span against what the fitter needs, before the fit is attempted."""
	print(f"\n{len(rows)} samples. Span per joint:")
	print(f"  {'joint':7s} {'q span (rad)':>13s} {'model span (Nm)':>16s}   fit")
	for joint_index in range(1, JOINT_COUNT + 1):
		q = [float(row[f"q{joint_index}"]) for row in rows]
		model = [float(row[f"tau_g_urdf_{joint_index}"]) for row in rows]
		q_span = max(q) - min(q)
		model_span = max(model) - min(model)
		ok = q_span >= MIN_JOINT_SPAN_RAD and model_span >= MIN_MODEL_SPAN_NM
		reason = "" if ok else "  <-- fitter will fall back to scale 1 / bias 0"
		print(f"  joint{joint_index}  {q_span:13.4f} {model_span:16.4f}   "
			  f"{'yes' if ok else 'no '}{reason}")


def main() -> None:
	# Imported here, not at module scope: the vendor SDK and the controller
	# package are needed to move an arm, not to build or check a pose set.
	from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

	from agx_arm_mit_controller.gravity_launch_utils import resolve_gravity_urdf_path
	from agx_arm_mit_controller.gravity_model import GravityModelError, create_gravity_model
	from agx_arm_mit_tools.compare_gravity import wait_until

	args = parse_args()
	can_port, side_firmware = SIDE_DEFAULTS[args.arm_side]
	can_port = args.can_port or can_port
	firmware = getattr(NeroFW, args.firmware or side_firmware)

	urdf_path = args.urdf_path or resolve_gravity_urdf_path(
		custom_model=args.custom_model,
		custom_model_xacro_args=args.custom_model_xacro_args,
		effector_type=args.effector_type,
		duo_side=args.arm_side,
	)
	if not urdf_path:
		raise SystemExit(
			"no gravity URDF: pass --urdf-path, or --custom-model to derive one "
			"(without it the model would be the upright hand-less Nero, which is "
			"what the stale calibration was already fitted on)"
		)
	try:
		gravity_model = create_gravity_model(args.backend, urdf_path)
	except GravityModelError as exc:
		raise SystemExit(str(exc))
	feedforward_sign = float(args.feedforward_sign)
	if feedforward_sign == 0.0:
		raise SystemExit("--feedforward-sign must not be zero")
	extras = json.loads(args.hand_joints) if args.hand_joints.strip() else None
	limits = joint_limits(urdf_path)

	print(f"arm      : {args.arm_side} on {can_port}, tier {args.firmware or side_firmware}")
	print(f"gravity  : {urdf_path}")
	if extras:
		print(f"hand     : {len(extras)} articulated joint(s) supplied")

	if args.dry_run and args.poses:
		poses = load_poses(args.poses, args.arm_side)
		start = poses[0][1]
	elif args.dry_run:
		start = [0.0] * JOINT_COUNT
		print("dry run without --poses: sweeping from the all-zero pose")
		poses = build_sweep(
			start, limits, [int(j) for j in args.sweep_joints.split(",") if j.strip()],
			args.sweep_steps, args.sweep_fraction,
		)
	else:
		poses = []
		start = []

	if args.dry_run:
		check_reachable(poses, limits, start, args.max_step)
		print(f"\n{len(poses)} pose(s), ~{len(poses) * (args.dwell + 3.0):.0f}s:")
		for name, pose in poses:
			print(f"  {name:16s} {[round(v, 4) for v in pose]}")
		print("\ndry run — nothing was sent to the arm")
		return

	cfg = create_agx_arm_config(
		robot=ArmModel.NERO,
		firmeware_version=firmware,
		interface=args.interface,
		channel=can_port,
	)
	robot = AgxArmFactory.create_arm(cfg)
	csv_path = Path(args.csv_path).expanduser().resolve()
	csv_path.parent.mkdir(parents=True, exist_ok=True)
	field_names = [
		"time", "pose",
		*[f"q{i}" for i in range(1, JOINT_COUNT + 1)],
		*[f"tau_measured_{i}" for i in range(1, JOINT_COUNT + 1)],
		*[f"tau_g_urdf_{i}" for i in range(1, JOINT_COUNT + 1)],
		*[f"tau_error_{i}" for i in range(1, JOINT_COUNT + 1)],
		*[f"tau_raw_{i}" for i in range(1, JOINT_COUNT + 1)],
	]
	write_header = not args.append or not csv_path.exists() or csv_path.stat().st_size == 0
	rows: list[dict] = []

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
		robot.set_speed_percent(int(args.speed_percent))

		start = wait_until(
			lambda: read_joints(robot), timeout_s=args.enable_timeout,
			description="joint feedback",
		)
		print(f"start    : {[round(v, 4) for v in start]}")

		poses = (
			load_poses(args.poses, args.arm_side) if args.poses
			else build_sweep(
				start, limits,
				[int(j) for j in args.sweep_joints.split(",") if j.strip()],
				args.sweep_steps, args.sweep_fraction,
			)
		)
		check_reachable(poses, limits, start, args.max_step)
		print(f"\n{len(poses)} pose(s) at {args.speed_percent}% speed, "
			  f"~{len(poses) * (args.dwell + 3.0) / 60.0:.1f} min")
		if not args.yes:
			if input("the arm will move. type 'go' to start: ").strip().lower() != "go":
				print("nothing sent")
				return

		period = 1.0 / args.rate
		clock = time.monotonic()
		with csv_path.open("a" if args.append else "w", newline="", encoding="utf-8") as handle:
			writer = csv.DictWriter(handle, fieldnames=field_names)
			if write_header:
				writer.writeheader()
			for index, (name, pose) in enumerate(poses, start=1):
				print(f"[{index}/{len(poses)}] {name}")
				landed = move_and_settle(
					robot, pose, args.settle_tolerance, args.settle_timeout
				)
				error = max(abs(landed[i] - pose[i]) for i in range(JOINT_COUNT))
				if error > args.settle_tolerance:
					message = (
						f"  {name}: stopped {error:.4f} rad short of the target "
						f"(tolerance {args.settle_tolerance:.4f})"
					)
					if not args.skip_unreached:
						raise SystemExit(
							message + "\n  aborting: a joint short of its target may be "
							"pressing on something. --skip-unreached logs and continues."
						)
					print(message + " — logged anyway")

				dwell_end = time.monotonic() + args.dwell
				while time.monotonic() < dwell_end:
					loop_start = time.monotonic()
					q = read_joints(robot)
					if q is None:
						time.sleep(period)
						continue
					tau_raw = read_torques(robot)
					# Into the model's sign convention: the controller commands
					# `sign * scale * model`, so the motor reports the negative of
					# the model at the default sign of -1.
					tau_measured = [value / feedforward_sign for value in tau_raw]
					tau_model = gravity_model.compute_gravity(q, extras)
					row = {"time": time.monotonic() - clock, "pose": name}
					for i in range(JOINT_COUNT):
						row[f"q{i + 1}"] = q[i]
						row[f"tau_measured_{i + 1}"] = tau_measured[i]
						row[f"tau_g_urdf_{i + 1}"] = tau_model[i]
						row[f"tau_error_{i + 1}"] = tau_measured[i] - tau_model[i]
						row[f"tau_raw_{i + 1}"] = tau_raw[i]
					writer.writerow(row)
					rows.append(row)
					time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
				handle.flush()
		print(f"\nwrote {csv_path}")
	except KeyboardInterrupt:
		print("\ninterrupted")
		hold_here(robot)
	finally:
		if rows:
			report_span(rows)
			print(
				"\nfit with:\n"
				f"  ros2 run agx_arm_mit_tools agx_arm_fit_gravity_calibration -- {csv_path} "
				"--output config/<name>_calibration.json"
			)
		# The arm is left enabled and holding: a de-energized Nero has no brakes.
		try:
			robot.disconnect()
		except Exception:
			pass


__all__ = ["main", "build_sweep", "joint_limits", "load_poses", "check_reachable"]
