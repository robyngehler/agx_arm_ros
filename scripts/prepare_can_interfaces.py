#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "can_interface_roles.json"
)
ROLE_NAMES = ("nero", "effector", "omnihand")


@dataclass(frozen=True)
class InterfaceInfo:
    name: str
    bus_info: str
    details: str
    bitrate: int | None
    dbitrate: int | None
    is_up: bool


@dataclass(frozen=True)
class RoleConfig:
    name: str
    description: str
    target_name: str
    mode: str
    bitrate: int
    sample_point: float | None
    dbitrate: int | None
    dsample_point: float | None
    restart_ms: int | None
    tx_queue_len: int | None
    usb_bus_info: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Prepare SocketCAN interfaces for repo-defined roles such as the Nero arm "
            "and future effectors."
        )
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="Path to the JSON config with CAN role definitions.",
    )
    parser.add_argument(
        "--roles",
        default="nero",
        help="Comma-separated role list to prepare. Default: nero.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be changed without modifying interfaces.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="Print detected CAN interfaces and exit.",
    )

    for role in ROLE_NAMES:
        parser.add_argument(
            f"--{role}-target-name",
            help=f"Override the configured target interface name for role '{role}'.",
        )
        parser.add_argument(
            f"--{role}-bitrate",
            type=int,
            help=f"Override the arbitration bitrate for role '{role}'.",
        )
        parser.add_argument(
            f"--{role}-sample-point",
            type=float,
            help=f"Override the arbitration sample point for role '{role}'.",
        )
        parser.add_argument(
            f"--{role}-dbitrate",
            type=int,
            help=f"Override the CAN FD data bitrate for role '{role}'.",
        )
        parser.add_argument(
            f"--{role}-dsample-point",
            type=float,
            help=f"Override the CAN FD data sample point for role '{role}'.",
        )
        parser.add_argument(
            f"--{role}-can-interface",
            help=(
                f"Explicitly select the current Linux CAN interface or USB bus-info for role '{role}'. "
                "If omitted, the script tries configured bus-info, current target names, then leftover interfaces."
            ),
        )

    return parser.parse_args()


def require_command(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(f"Required command '{name}' is not installed")


def run_command(
    command: list[str],
    *,
    sudo: bool = False,
    dry_run: bool = False,
    capture_output: bool = True,
) -> str:
    full_command = (["sudo"] if sudo else []) + command
    print("+", " ".join(full_command))
    if dry_run:
        return ""

    completed = subprocess.run(
        full_command,
        check=True,
        text=True,
        capture_output=capture_output,
    )
    if capture_output:
        return completed.stdout
    return ""


def load_roles(config_path: Path) -> dict[str, RoleConfig]:
    payload = json.loads(config_path.read_text(encoding="utf-8"))
    roles_section = payload.get("roles")
    if not isinstance(roles_section, dict):
        raise RuntimeError(f"Invalid config file {config_path}: missing object key 'roles'")

    roles: dict[str, RoleConfig] = {}
    for role_name, raw in roles_section.items():
        if not isinstance(raw, dict):
            raise RuntimeError(f"Invalid role entry for '{role_name}' in {config_path}")
        roles[role_name] = RoleConfig(
            name=role_name,
            description=str(raw.get("description", role_name)),
            target_name=str(raw["target_name"]),
            mode=str(raw.get("mode", "can")),
            bitrate=int(raw["bitrate"]),
            sample_point=None if raw.get("sample_point") in (None, "") else float(raw["sample_point"]),
            dbitrate=None if raw.get("dbitrate") in (None, "") else int(raw["dbitrate"]),
            dsample_point=None if raw.get("dsample_point") in (None, "") else float(raw["dsample_point"]),
            restart_ms=None if raw.get("restart_ms") in (None, "") else int(raw["restart_ms"]),
            tx_queue_len=None if raw.get("tx_queue_len") in (None, "") else int(raw["tx_queue_len"]),
            usb_bus_info=str(raw.get("usb_bus_info", "")).strip(),
        )
    return roles


def _extract_details_field(details: str, pattern: str) -> int | None:
    match = re.search(pattern, details)
    if not match:
        return None
    return int(match.group(1))


def _extract_details_float(details: str, pattern: str) -> float | None:
    match = re.search(pattern, details, re.MULTILINE)
    if not match:
        return None
    return float(match.group(1))


def _format_float(value: float) -> str:
    return f"{value:.6f}".rstrip("0").rstrip(".")


def _float_matches(actual: float | None, expected: float, *, tolerance: float = 0.001) -> bool:
    return actual is not None and abs(actual - expected) <= tolerance


def detect_interfaces() -> list[InterfaceInfo]:
    names_output = run_command(["ip", "-br", "link", "show", "type", "can"])
    interface_names = []
    for line in names_output.splitlines():
        fields = line.split()
        if fields:
            interface_names.append(fields[0])

    interfaces: list[InterfaceInfo] = []
    for name in interface_names:
        driver_output = run_command(["ethtool", "-i", name])
        bus_info = ""
        for line in driver_output.splitlines():
            if line.startswith("bus-info:"):
                bus_info = line.split(":", 1)[1].strip()
                break

        details = run_command(["ip", "-details", "link", "show", name])
        interfaces.append(
            InterfaceInfo(
                name=name,
                bus_info=bus_info,
                details=details,
                bitrate=_extract_details_field(details, r"\bbitrate\s+(\d+)"),
                dbitrate=_extract_details_field(details, r"\bdbitrate\s+(\d+)"),
                is_up=" UP " in f" {details} ",
            )
        )

    return interfaces


def print_interfaces(interfaces: list[InterfaceInfo]) -> None:
    if not interfaces:
        print("No CAN interfaces were detected.")
        return

    print("Detected CAN interfaces:")
    for interface in interfaces:
        print(
            f"- {interface.name}: bus-info={interface.bus_info or 'n/a'}, "
            f"bitrate={interface.bitrate or 'n/a'}, dbitrate={interface.dbitrate or 'n/a'}, up={interface.is_up}"
        )


def resolve_requested_roles(value: str, known_roles: dict[str, RoleConfig]) -> list[str]:
    requested = [role.strip() for role in value.split(",") if role.strip()]
    if not requested:
        raise RuntimeError("No roles were requested")
    unknown = [role for role in requested if role not in known_roles]
    if unknown:
        raise RuntimeError(f"Unknown CAN roles requested: {', '.join(unknown)}")
    return requested


def override_role_config(args: argparse.Namespace, role: RoleConfig) -> RoleConfig:
    target_name_override = getattr(args, f"{role.name}_target_name")
    bitrate_override = getattr(args, f"{role.name}_bitrate")
    sample_point_override = getattr(args, f"{role.name}_sample_point")
    dbitrate_override = getattr(args, f"{role.name}_dbitrate")
    dsample_point_override = getattr(args, f"{role.name}_dsample_point")
    return RoleConfig(
        name=role.name,
        description=role.description,
        target_name=target_name_override or role.target_name,
        mode=role.mode,
        bitrate=bitrate_override if bitrate_override is not None else role.bitrate,
        sample_point=sample_point_override if sample_point_override is not None else role.sample_point,
        dbitrate=dbitrate_override if dbitrate_override is not None else role.dbitrate,
        dsample_point=dsample_point_override if dsample_point_override is not None else role.dsample_point,
        restart_ms=role.restart_ms,
        tx_queue_len=role.tx_queue_len,
        usb_bus_info=role.usb_bus_info,
    )


def _match_selector(selector: str, interfaces: list[InterfaceInfo], claimed: set[str]) -> InterfaceInfo:
    for interface in interfaces:
        if interface.name == selector and interface.name not in claimed:
            return interface
    matches = [
        interface
        for interface in interfaces
        if interface.bus_info == selector and interface.name not in claimed
    ]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise RuntimeError(f"Selector '{selector}' did not match any unclaimed CAN interface")
    raise RuntimeError(f"Selector '{selector}' matched multiple CAN interfaces")


def resolve_interface_for_role(
    args: argparse.Namespace,
    role: RoleConfig,
    interfaces: list[InterfaceInfo],
    claimed: set[str],
) -> InterfaceInfo:
    selector = getattr(args, f"{role.name}_can_interface")
    if selector:
        return _match_selector(selector, interfaces, claimed)

    if role.usb_bus_info:
        return _match_selector(role.usb_bus_info, interfaces, claimed)

    for interface in interfaces:
        if interface.name == role.target_name and interface.name not in claimed:
            return interface

    unclaimed = [interface for interface in interfaces if interface.name not in claimed]
    if len(unclaimed) == 1:
        return unclaimed[0]

    raise RuntimeError(
        f"Could not auto-resolve role '{role.name}'. Use --{role.name}-can-interface or set usb_bus_info in the config."
    )


def ensure_target_name_available(
    selected: InterfaceInfo, role: RoleConfig, interfaces: list[InterfaceInfo]
) -> None:
    if selected.name == role.target_name:
        return
    for interface in interfaces:
        if interface.name == role.target_name and interface.name != selected.name:
            raise RuntimeError(
                f"Cannot rename {selected.name} to {role.target_name}: that target name is already in use"
            )


def verify_configured_interface(name: str, role: RoleConfig) -> None:
    details = run_command(["ip", "-details", "link", "show", name])
    bitrate = _extract_details_field(details, r"\bbitrate\s+(\d+)")
    if bitrate != role.bitrate:
        raise RuntimeError(
            f"Interface '{name}' reports bitrate {bitrate}, expected {role.bitrate} for role '{role.name}'"
        )
    if role.sample_point is not None:
        sample_point = _extract_details_float(details, r"^\s+bitrate\s+\d+\s+sample-point\s+([0-9.]+)")
        if not _float_matches(sample_point, role.sample_point):
            raise RuntimeError(
                f"Interface '{name}' reports sample-point {sample_point}, expected {role.sample_point} for role '{role.name}'"
            )
    if role.mode == "canfd":
        dbitrate = _extract_details_field(details, r"\bdbitrate\s+(\d+)")
        if dbitrate != role.dbitrate:
            raise RuntimeError(
                f"Interface '{name}' reports dbitrate {dbitrate}, expected {role.dbitrate} for role '{role.name}'"
            )
        if role.dsample_point is not None:
            dsample_point = _extract_details_float(details, r"^\s+dbitrate\s+\d+\s+dsample-point\s+([0-9.]+)")
            if not _float_matches(dsample_point, role.dsample_point):
                raise RuntimeError(
                    f"Interface '{name}' reports dsample-point {dsample_point}, expected {role.dsample_point} for role '{role.name}'"
                )
        if "mtu 72" not in details:
            raise RuntimeError(f"Interface '{name}' did not come up with CAN FD MTU 72")


def configure_role(role: RoleConfig, selected: InterfaceInfo, *, dry_run: bool) -> str:
    current_name = selected.name
    print(
        f"Preparing role '{role.name}' ({role.description}) from interface '{current_name}' "
        f"to target '{role.target_name}'"
    )

    run_command(["ip", "link", "set", current_name, "down"], sudo=True, dry_run=dry_run)

    if role.mode == "canfd":
        type_command = [
            "ip",
            "link",
            "set",
            current_name,
            "type",
            "can",
            "bitrate",
            str(role.bitrate),
        ]
        if role.sample_point is not None:
            type_command.extend(["sample-point", _format_float(role.sample_point)])
        type_command.extend(["dbitrate", str(role.dbitrate)])
        if role.dsample_point is not None:
            type_command.extend(["dsample-point", _format_float(role.dsample_point)])
        type_command.extend(["fd", "on"])
    elif role.mode == "can":
        type_command = [
            "ip",
            "link",
            "set",
            current_name,
            "type",
            "can",
            "bitrate",
            str(role.bitrate),
        ]
        if role.sample_point is not None:
            type_command.extend(["sample-point", _format_float(role.sample_point)])
    else:
        raise RuntimeError(f"Unsupported CAN role mode '{role.mode}' for role '{role.name}'")

    if role.restart_ms is not None:
        type_command.extend(["restart-ms", str(role.restart_ms)])

    run_command(type_command, sudo=True, dry_run=dry_run)

    final_name = current_name
    if current_name != role.target_name:
        run_command(
            ["ip", "link", "set", current_name, "name", role.target_name],
            sudo=True,
            dry_run=dry_run,
        )
        final_name = role.target_name

    if role.tx_queue_len is not None:
        run_command(
            ["ip", "link", "set", final_name, "txqueuelen", str(role.tx_queue_len)],
            sudo=True,
            dry_run=dry_run,
        )

    run_command(["ip", "link", "set", final_name, "up"], sudo=True, dry_run=dry_run)

    if not dry_run:
        verify_configured_interface(final_name, role)
    return final_name


def main() -> int:
    args = parse_args()

    try:
        require_command("ip")
        require_command("ethtool")
        roles = load_roles(Path(args.config).expanduser().resolve())
        interfaces = detect_interfaces()
        if args.list:
            print_interfaces(interfaces)
            return 0
        if not interfaces:
            raise RuntimeError("No Linux CAN interfaces are present")

        requested_role_names = resolve_requested_roles(args.roles, roles)
        claimed: set[str] = set()
        prepared: list[tuple[str, str]] = []

        print_interfaces(interfaces)
        for role_name in requested_role_names:
            role = override_role_config(args, roles[role_name])
            selected = resolve_interface_for_role(args, role, interfaces, claimed)
            ensure_target_name_available(selected, role, interfaces)
            final_name = configure_role(role, selected, dry_run=args.dry_run)
            claimed.add(selected.name)
            prepared.append((role.name, final_name))

        print("Prepared roles:")
        for role_name, final_name in prepared:
            print(f"- {role_name}: {final_name}")
        return 0
    except subprocess.CalledProcessError as exc:
        print(exc.stderr or exc.stdout or str(exc), file=sys.stderr)
        return exc.returncode or 1
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())