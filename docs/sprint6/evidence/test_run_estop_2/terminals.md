#### l3 script:
```bash
user@ubuntu:~/workspace/agx_arm_ros$ python3 scripts/l3_estop_pcap_run.py --trigger-action-no 110 --trigger-delay 5
run:      tea_pour_left_v1, stop during node 110 +5.0s
capture:  can_nero_left, can_nero_right
output:   /home/user/workspace/agx_arm_ros/logs/estop_20260820_165925

starting capture ...
sending the activity goal ...
waiting for node 110 to start ...
  [  10] left_hand_rest_fist -> running
  [  10] left_hand_rest_fist -> completed
  [  20] left_arm_to_teapot_grip_idle -> running
  [  20] left_arm_to_teapot_grip_idle -> completed
  [  30] left_hand_pre_grip_handle -> running
  [  30] left_hand_pre_grip_handle -> completed
  [  40] left_arm_to_teapot_pre_grip -> running
  [  40] left_arm_to_teapot_pre_grip -> completed
  [  50] left_arm_teapot_handle_entry -> running
  [  50] left_arm_teapot_handle_entry -> completed
  [  60] left_arm_to_teapot_grip -> running
  [  60] left_arm_to_teapot_grip -> completed
  [  70] left_hand_grip_handle -> running
  [  70] left_hand_grip_handle -> completed
  [  80] left_arm_to_teapot_post_grip -> running
  [  80] left_arm_to_teapot_post_grip -> completed
  [  90] left_arm_to_pour_init -> running
  [  90] left_arm_to_pour_init -> completed
  [ 100] left_arm_to_pour_idle -> running
  [ 100] left_arm_to_pour_idle -> completed
  [ 110] left_arm_pour_tea -> running
node 110 is running; stopping in 5.0s
>>> EMERGENCY STOP
  left: success=True nero stop=verified — confirmed stopped (peak 0.021 rad/s (dt=20ms)); this device is latched and refuses motion until clear_fault_lockout
  right: success=True nero stop=verified — confirmed stopped (peak 0.000 rad/s (dt=20ms)); this device is latched and refuses motion until clear_fault_lockout
holding the capture open for 8.0s ...
capture stopped:
  can_nero_left: /home/user/workspace/agx_arm_ros/logs/estop_20260820_165925/can_nero_left.pcap (6.6 MB)
  can_nero_right: /home/user/workspace/agx_arm_ros/logs/estop_20260820_165925/can_nero_right.pcap (5.4 MB)

/usr/bin/python3 /home/user/workspace/agx_arm_ros/scripts/analyze_can_pcap.py --stop-at 1787238011.329455 /home/user/workspace/agx_arm_ros/logs/estop_20260820_165925/can_nero_left.pcap /home/user/workspace/agx_arm_ros/logs/estop_20260820_165925/can_nero_right.pcap

=== /home/user/workspace/agx_arm_ros/logs/estop_20260820_165925/can_nero_left.pcap ===
  137484 frames over 51.57s = 2666 f/s
  RX 2154 f/s, TX 512 f/s, 35 distinct IDs
  MIT command frames: 26376 = 511 f/s (73 Hz per joint)
  joint feedback — reported velocity vs velocity derived from the same frames:
    joint 1: reported     peak 1   derived peak=  2408.7 mean=  106.1   position span=2469
    joint 2: reported     peak 1   derived peak=  1744.7 mean=   80.1   position span=1864
    joint 3: reported  FLAT ZERO   derived peak=  1600.0 mean=   57.1   position span=1345
    joint 4: reported     peak 1   derived peak=  1605.1 mean=   26.0   position span=605
    joint 5: reported     peak 1   derived peak=  1845.5 mean=   65.3   position span=1378
    joint 6: reported     peak 1   derived peak=  1332.2 mean=   36.6   position span=540
    joint 7: reported  FLAT ZERO   derived peak=  1918.5 mean=   44.1   position span=745
  VERDICT: joints moved while the velocity field stayed at 0 (+/-1).
           The firmware does not report usable velocity — deriving it
           from positions is the only available source, and removing
           the vendor's zeroing would expose nothing but zeros.
=== stop signature: /home/user/workspace/agx_arm_ros/logs/estop_20260820_165925/can_nero_left.pcap ===
  MIT command frames: 3500 in the 5s before the stop, 7 after it
  joint position control frames after the stop: 4
  MOVE modes commanded after the stop: MOVE-J x1
  PASS: no electronic emergency stop frame anywhere in the capture
  FAIL: 7 MIT command frames after the stop — the control stream did not end
  VERDICT: NOT clean
=== /home/user/workspace/agx_arm_ros/logs/estop_20260820_165925/can_nero_right.pcap ===
  111816 frames over 51.54s = 2170 f/s
  RX 2169 f/s, TX 0 f/s, 27 distinct IDs
  joint feedback — reported velocity vs velocity derived from the same frames:
    joint 1: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
    joint 2: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
    joint 3: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
    joint 4: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
    joint 5: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
    joint 6: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
    joint 7: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
=== stop signature: /home/user/workspace/agx_arm_ros/logs/estop_20260820_165925/can_nero_right.pcap ===
  MIT command frames: 0 in the 5s before the stop, 0 after it
  joint position control frames after the stop: 4
  MOVE modes commanded after the stop: MOVE-J x1
  PASS: no electronic emergency stop frame anywhere in the capture
  VERDICT: clean

The stop latched. Before running anything else:
  ros2 service call /left_arm/clear_fault_lockout std_srvs/srv/Trigger {}
  ros2 service call /right_arm/clear_fault_lockout std_srvs/srv/Trigger {}
  ros2 service call /unit_safety/rearm std_srvs/srv/Trigger {}
```