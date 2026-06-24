"""Model-aware OmniHand definitions shared by the bridge, exerciser, and tools.

This subpackage isolates everything that differs between OmniHand hardware
generations (joint set, limits, tactile layout, vendor SDK class) so the ROS
surface and the bridge node can stay model-agnostic. See models.py for the
registry of supported hands (``o10`` and ``o12_pro``).
"""

from agx_arm_ctrl.omnihand.models import (
    HAND_MODELS,
    DEFAULT_HAND_MODEL,
    HandModel,
    get_hand_model,
)

__all__ = [
    "HAND_MODELS",
    "DEFAULT_HAND_MODEL",
    "HandModel",
    "get_hand_model",
]
