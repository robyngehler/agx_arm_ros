# agx_arm_mit_tools

ROS2 debug, bridge, validation, and calibration entry points for the Nero MIT controller stack.

This package owns the non-production helper surface around the controller, including:

- the RViz soft-target bridge
- MIT hold validation
- gravity comparison and calibration helpers
- URDF versus MDH validation

The runtime MIT controller remains in `agx_arm_mit_controller`. This package depends on its shared libraries, but owns these debug/helper implementations and entry points directly so package discovery and source ownership stay aligned.