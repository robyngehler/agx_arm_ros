from __future__ import annotations

import argparse

from agx_arm_mit_controller.gravity_model import GravityModelError, create_gravity_model


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Validate URDF FK against pyAgxArm MDH FK for Nero")
	parser.add_argument(
		"--samples",
		nargs="*",
		default=[
			"0,0,0,0,0,0,0",
			"0.1,-0.2,0.3,-0.4,0.2,-0.1,0.05",
			"-0.3,0.4,-0.2,0.5,-0.2,0.1,-0.05",
		],
		help="Comma-separated joint vectors in SDK order",
	)
	parser.add_argument("--backend", default="pinocchio")
	parser.add_argument("--urdf-path", default="")
	return parser.parse_args()


def _parse_vector(text: str) -> list[float]:
	return [float(value.strip()) for value in text.split(",") if value.strip()]


def main() -> None:
	args = parse_args()
	try:
		gravity_model = create_gravity_model(args.backend, args.urdf_path or None)
	except GravityModelError as exc:
		raise SystemExit(str(exc))

	from pyAgxArm.utiles.mdh_kinematics import fk_from_mdh, get_mdh

	mdh = list(get_mdh("nero"))
	for index, sample_text in enumerate(args.samples, start=1):
		q = _parse_vector(sample_text)
		mdh_pose = [float(value) for value in fk_from_mdh(mdh, q)]
		urdf_pose = gravity_model.compute_flange_pose(q)
		delta = [urdf_pose[i] - mdh_pose[i] for i in range(len(mdh_pose))]
		max_abs_delta = max(abs(value) for value in delta)
		print(f"sample {index}: q={q}")
		print(f"  mdh_pose : {mdh_pose}")
		print(f"  urdf_pose: {urdf_pose}")
		print(f"  delta    : {delta}")
		print(f"  max_abs_delta={max_abs_delta:.6f}")


__all__ = ["main"]