## Sprint Goal
Fix the remaining issues with handshakes between the external CAN watchdog on a BUS failure / MIT command stall.

## Current Workflow:
If triggert, the watchdog cuts the Jetson-Arm connection and replaces it with a termination. The Jetson now sees a stall feedback until the watchdog releases the BUS again. In the meantime the watchdog commands a moveJ at the current pose. 
If released, the Jetson now gets feedback again and could continue/recover it's MIT hold.
Any other command would work after the entities' safety epochs are synched with the unit's by the coordinator and a new command is issued.

## Problem:
The flow works up until the point the BUS is released and the jetson recovers MIT hold.. but new issued MIT commands don't come through, maybe because the external watchdog triggered an un-authorized moveJ hold.
A possible solution would be to check the control mode and try to switch towards mit again.