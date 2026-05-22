# agx_arm_mit_demos

ROS2 demo and workflow entry points for the Nero MIT controller stack.

This package owns interactive demo flows such as:

- leader-mode recording
- saved-trajectory playback
- wakeword-triggered teach-and-playback workflows

The runtime MIT controller stays in `agx_arm_mit_controller`. This package depends on its shared libraries, but owns these demo implementations and entry points directly so package discovery and source ownership stay aligned.