#!/usr/bin/env python3.10
"""Verify the OmniHand Pro (agibot_hand) SDK imports before touching hardware.

Run with the SAME Python that ROS uses (python3.10). If agibot_hand is not on
PYTHONPATH, this locates the repo's built package automatically.

    python3.10 scripts/omnihand_pro/pro_import_check.py
"""

from __future__ import annotations

from pathlib import Path
import sys

_VENDOR_PKG_REL = Path("vendor") / "OmniHand-Pro-2025" / "build" / "agibot_hand_pkg"


def _ensure_on_path() -> None:
    try:
        import agibot_hand  # noqa: F401
        return
    except ImportError:
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / _VENDOR_PKG_REL
        if (candidate / "agibot_hand" / "__init__.py").exists():
            sys.path.insert(0, str(candidate))
            return


def main() -> int:
    _ensure_on_path()
    try:
        from agibot_hand import AgibotHandO12, EFinger, EHandType, EControlMode
    except ImportError as exc:
        print(f"import FAILED: {exc}")
        print("Build vendor/OmniHand-Pro-2025 (SocketCAN, Python 3.10) first.")
        return 1
    print("import ok")
    print("  AgibotHandO12:", AgibotHandO12)
    print("  EFinger:", list(EFinger))
    print("  EHandType:", list(EHandType))
    print("  EControlMode:", list(EControlMode))
    print(f"  python: {sys.version.split()[0]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
