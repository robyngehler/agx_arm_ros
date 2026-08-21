#### components.launch:
```bash
[move_group-10] [INFO] [1787233821.598791040] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233821.599361600] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233821.610171808] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233821.610297376] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233821.610337216] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233821.610371040] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233821.618572160] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233821.872978784] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233821.873055456] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233821.873176544] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233821.889748992] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233821.889915776] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233821.890008032] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233821.890424512] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233821.899122048] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233821.899215168] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_ctrl_single-2] [INFO] [1787233821.905039744] [left_arm.agx_arm_ctrl_single_node]: arm_left claimed by 'left_arm/mit_controller' at device generation 2
[agx_arm_mit_controller-3] [INFO] [1787233821.912074656] [left_arm.mit_controller]: arm_left claimed by 'left_arm/mit_controller' at device generation 2
[agx_arm_mit_controller-3] [INFO] [1787233821.912531328] [left_arm.mit_controller]: MIT controller enabled
[agx_arm_mit_controller-3] [WARN] [1787233821.916322432] [left_arm.mit_controller]: 'left_arm/mit_controller' does not hold this device (held by nobody); not commanding
[agx_arm_mit_controller-3] [INFO] [1787233821.916666848] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 41 points and 3.959s duration
[agx_arm_mit_controller-3] [INFO] [1787233821.917737024] [left_arm.mit_controller]: Took command of 'arm_left' at device generation 2
[move_group-10] [INFO] [1787233825.890351168] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233825.927171424] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233825.930102944] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[move_group-10] [INFO] [1787233828.595388768] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233828.595687392] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233828.615571584] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233828.615652928] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233828.615666720] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233828.615674240] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233828.622568064] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233828.831366144] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233828.831448704] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233828.831564736] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233828.843557216] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233828.843656128] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233828.843723744] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233828.844031712] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233828.849844096] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233828.849908480] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233828.868221504] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 18 points and 1.641s duration
[move_group-10] [INFO] [1787233830.502109536] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233830.535464960] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233830.543793952] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[move_group-10] [INFO] [1787233831.059783744] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233831.060094112] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233831.073937120] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233831.074048000] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233831.074081696] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233831.074098240] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233831.077693088] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233831.167082368] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233831.167280480] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233831.167413728] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233831.174725152] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233831.174824128] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233831.174885408] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233831.175166624] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233831.180552096] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233831.180616512] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233831.199100064] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 5 points and 0.326s duration
[move_group-10] [INFO] [1787233831.522749728] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233831.570663744] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233831.574929376] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[move_group-10] [INFO] [1787233831.585930848] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-10] [INFO] [1787233831.586228352] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-10] [INFO] [1787233831.586325152] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233831.586427584] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233831.586691520] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233831.595696064] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233831.595830624] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233831.595891840] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233831.596179648] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233831.601834336] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233831.601890560] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233831.611143776] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 18 points and 4.556s duration
[move_group-10] [INFO] [1787233836.178853824] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233836.223911488] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233836.224087136] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: SUCCEEDED
[move_group-10] [INFO] [1787233836.736384832] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233836.736677216] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233836.744373376] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233836.744476960] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233836.744506880] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233836.744521824] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233836.748017664] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233836.906248896] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233836.906343360] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233836.906404992] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233836.914287424] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233836.914479584] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233836.914558752] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233836.914921888] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233836.918922336] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233836.918971040] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233836.928323904] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 20 points and 1.882s duration
[move_group-10] [INFO] [1787233838.812127808] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233838.873916608] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233838.874516288] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[agx_arm_mit_controller-3] [INFO] [1787233840.321102720] [left_arm.mit_controller]: payload attached; gravity model /tmp/duo_system.urdf_gravity_l72119gq_payload_i58dm7f5.urdf
[move_group-10] [INFO] [1787233840.869918624] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233840.870200640] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233840.884281472] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233840.884369344] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233840.884395648] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233840.884415200] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233840.890135904] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233841.080531936] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233841.080620672] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233841.080738144] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233841.090039264] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233841.090161568] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233841.090253440] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233841.090602592] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233841.094951104] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233841.095006720] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233841.108425248] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 23 points and 2.163s duration
[move_group-10] [INFO] [1787233843.268585408] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233843.324098080] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233843.330269376] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[move_group-10] [INFO] [1787233843.847508544] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233843.847806080] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233843.860828160] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233843.860916096] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233843.860945632] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233843.860963936] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233843.864671872] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233844.058044896] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233844.058130880] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233844.058240256] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233844.070139584] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233844.070250304] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233844.070320736] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233844.070644096] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233844.076993280] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233844.077062720] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233844.099220864] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 36 points and 3.471s duration
[move_group-10] [INFO] [1787233847.572451520] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233847.610516640] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233847.620414976] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[move_group-10] [INFO] [1787233848.137099424] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233848.137433600] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233848.144540448] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233848.144614912] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233848.144636992] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233848.144654176] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233848.147509344] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233848.422471520] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233848.422629248] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233848.422776064] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233848.433862752] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233848.433988480] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233848.434081984] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233848.434447616] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233848.439588992] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233848.439655936] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233848.450382944] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 43 points and 4.185s duration
[move_group-10] [INFO] [1787233852.642807680] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233852.703915872] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233852.714186944] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[move_group-10] [INFO] [1787233853.233204928] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233853.233517184] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233853.244137056] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233853.244225184] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233853.244257216] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233853.244275136] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233853.251626048] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233853.424270592] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233853.424360640] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233853.424480672] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233853.437833088] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233853.437952992] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233853.438017824] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233853.438323008] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233853.443166432] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233853.443213344] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233853.462580320] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 12 points and 1.015s duration
[move_group-10] [INFO] [1787233854.467583552] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233854.515612544] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233854.518052704] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[move_group-10] [INFO] [1787233854.534796992] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-10] [INFO] [1787233854.535059648] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-10] [INFO] [1787233854.535149760] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233854.535231264] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233854.535434144] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233854.555159648] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233854.555276800] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233854.555347264] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233854.555620032] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233854.562534528] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233854.562589440] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233854.578910336] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 73 points and 19.364s duration
[move_group-10] [INFO] [1787233873.935459584] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233873.975662784] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233873.975902304] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: SUCCEEDED
[move_group-10] [INFO] [1787233874.488780448] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233874.489107232] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233874.505873504] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233874.505953408] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233874.505978784] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233874.505991552] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233874.509214880] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233874.714168672] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233874.714253504] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233874.714312448] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233874.726529376] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233874.726615808] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233874.726662080] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233874.726922592] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233874.730907488] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233874.730977344] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233874.739774272] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 42 points and 4.084s duration
[move_group-10] [INFO] [1787233878.821000352] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233878.875599392] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233878.876819424] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[move_group-10] [INFO] [1787233879.394574976] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233879.394846496] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233879.407062432] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233879.407141600] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233879.407165920] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233879.407183456] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233879.410165888] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233879.649398400] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233879.649477408] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233879.649585184] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233879.665706912] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233879.665838016] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233879.665916960] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233879.666260672] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233879.672655808] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233879.672725856] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233879.691174112] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 36 points and 3.475s duration
[move_group-10] [INFO] [1787233883.167156928] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233883.225562784] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233883.225962240] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[move_group-10] [INFO] [1787233883.743218496] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233883.743547168] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233883.763066560] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233883.763162784] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233883.763189952] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233883.763206560] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233883.766079136] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233883.993729184] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233883.993810496] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233883.993910304] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233884.006764576] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233884.006896864] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233884.006961568] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233884.007345920] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233884.012709728] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233884.012760448] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233884.021954560] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 11 points and 0.956s duration
[move_group-10] [INFO] [1787233884.990531296] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233885.026958400] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233885.036954912] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[agx_arm_mit_controller-3] [INFO] [1787233886.437084224] [left_arm.mit_controller]: payload detached; gravity model /tmp/duo_system.urdf_gravity_l72119gq.urdf
[move_group-10] [INFO] [1787233886.987228288] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-10] [INFO] [1787233886.987566592] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-10] [INFO] [1787233886.995826336] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-10] [WARN] [1787233886.995899424] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as start state in the motion planning request
[move_group-10] [WARN] [1787233886.995924192] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-10] [INFO] [1787233886.995941408] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-10] [INFO] [1787233887.003343776] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'left_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-10] [INFO] [1787233887.089562816] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233887.089818272] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233887.090003552] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233887.105058304] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233887.105168896] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233887.105230208] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233887.105545632] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233887.108884160] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233887.108929568] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233887.112476832] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 5 points and 0.359s duration
[move_group-10] [INFO] [1787233887.481078656] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' successfully finished
[move_group-10] [INFO] [1787233887.532186880] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-10] [INFO] [1787233887.535209344] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[move_group-10] [INFO] [1787233887.549852672] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-10] [INFO] [1787233887.550100384] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-10] [INFO] [1787233887.550183264] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233887.550266752] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233887.550456672] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.05
[move_group-10] [INFO] [1787233887.565040832] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-10] [INFO] [1787233887.565220992] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233887.565288256] [moveit.plugins.moveit_simple_controller_manager]: Returned 4 controllers in list
[move_group-10] [INFO] [1787233887.565576704] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to left_arm/arm_controller
[move_group-10] [INFO] [1787233887.570225952] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: left_arm/arm_controller started execution
[move_group-10] [INFO] [1787233887.570289664] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[agx_arm_mit_controller-3] [INFO] [1787233887.578388960] [left_arm.mit_controller]: Accepted FollowJointTrajectory goal with 50 points and 13.177s duration
[unit_safety-1] [ERROR] [1787233892.006452224] [unit_safety]: UNIT STOP generation 1 allocated on request from 'arm_left': emergency stop requested
[unit_safety-1] [WARN] [1787233892.009220832] [unit_safety]: 'arm_right' requested a unit stop (emergency stop requested); already stopped at generation 1
[agx_arm_mit_controller-5] [WARN] [1787233892.016385376] [right_arm.mit_controller]: Device authority changed (state=4, device_epoch=2, unit_safety_epoch=0, motion_ready=False): emergency stop requested
[agx_arm_mit_controller-3] [WARN] [1787233892.023039168] [left_arm.mit_controller]: Device authority changed (state=4, device_epoch=3, unit_safety_epoch=0, motion_ready=False): emergency stop requested; aborted the active trajectory
[move_group-10] [WARN] [1787233892.026808960] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'left_arm/arm_controller' failed with error INVALID_GOAL: device authority changed while executing (state=4, device_epoch=3, unit_safety_epoch=0, motion_ready=False): emergency stop requested
[move_group-10] [WARN] [1787233892.026932640] [moveit_ros.trajectory_execution_manager]: Controller handle left_arm/arm_controller reports status ABORTED
[move_group-10] [INFO] [1787233892.026994400] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status ABORTED ...
[move_group-10] [INFO] [1787233892.027152928] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: ABORTED
[agx_arm_ctrl_single-4] [INFO] [1787233892.069566592] [right_arm.agx_arm_ctrl_single_node]: Emergency stop hold commanded on nero (attempt 1/3)
[agx_arm_ctrl_single-2] [INFO] [1787233892.074110848] [left_arm.agx_arm_ctrl_single_node]: Emergency stop hold commanded on nero (attempt 1/3)
[agx_arm_ctrl_single-4] [INFO] [1787233892.092308320] [right_arm.agx_arm_ctrl_single_node]: Emergency stop verified: nero joints settled (peak 0.048 rad/s (dt=18ms))
[agx_arm_mit_controller-5] [WARN] [1787233892.100555328] [right_arm.mit_controller]: Device authority changed (state=5, device_epoch=2, unit_safety_epoch=1, motion_ready=False): unit stop: arm_left: emergency stop requested
[agx_arm_ctrl_single-4] [WARN] [1787233892.100907808] [right_arm.agx_arm_ctrl_single_node]: unit safety generation 1 from 'unit_safety': stopped=True (arm_left: emergency stop requested)
[agx_arm_ctrl_single-2] [INFO] [1787233892.160504704] [left_arm.agx_arm_ctrl_single_node]: Emergency stop verified: nero joints settled (peak 0.034 rad/s (dt=19ms))
[agx_arm_mit_controller-3] [WARN] [1787233892.168157152] [left_arm.mit_controller]: Device authority changed (state=5, device_epoch=3, unit_safety_epoch=1, motion_ready=False): unit stop: arm_left: emergency stop requested
[agx_arm_ctrl_single-2] [WARN] [1787233892.171131744] [left_arm.agx_arm_ctrl_single_node]: unit safety generation 1 from 'unit_safety': stopped=True (arm_left: emergency stop requested)
[agx_arm_ctrl_single-2] [ERROR] [1787233892.175362880] [left_arm.agx_arm_ctrl_single_node]: move_mit rejected (not_ready): arm_left is stopped: unit stop: arm_left: emergency stop requested [1 rejected on this path for this reason so far]
```

#### launch tea demo
```bash
user@ubuntu:~$ ros2 launch agx_arm_coordination start_tea_demo.launch.py
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2026-08-20-15-48-40-682501-ubuntu-5304
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [omnihand_bridge-1]: process started with pid [5305]
[INFO] [omnihand_skill_controller-2]: process started with pid [5307]
[INFO] [omnihand_bridge-3]: process started with pid [5309]
[INFO] [omnihand_skill_controller-4]: process started with pid [5311]
[INFO] [coordinator-5]: process started with pid [5313]
[omnihand_bridge-1] [INFO] [1787233721.540401568] [left_hand.omnihand_bridge_node]: OmniHand bridge started with hand_side=left, hand_model=o12_pro (12 joints), backend_type=mock_backend, joint_states_command_topic=control/joint_states
[omnihand_bridge-1] [WARN] [1787233721.542620512] [left_hand.omnihand_bridge_node]: unit safety generation 0: stopped=False (init)
[omnihand_skill_controller-4] [INFO] [1787233721.614773344] [right_hand.omnihand_skill_controller]: OmniHand skill controller up: side=right, model=o12_pro (12 joints), action=perform, skills=['grasp_bottle_until_contact', 'grasp_glass_until_contact', 'grip_handle', 'hand_rest_fist', 'open_hand', 'pre_grip_handle', 'release_bottle', 'release_glass', 'release_handle', 'stop_hand']
[omnihand_bridge-3] [INFO] [1787233721.632303808] [right_hand.omnihand_bridge_node]: OmniHand bridge started with hand_side=right, hand_model=o12_pro (12 joints), backend_type=mock_backend, joint_states_command_topic=control/joint_states
[omnihand_bridge-3] [WARN] [1787233721.634316192] [right_hand.omnihand_bridge_node]: unit safety generation 0: stopped=False (init)
[omnihand_skill_controller-2] [INFO] [1787233721.796371008] [left_hand.omnihand_skill_controller]: OmniHand skill controller up: side=left, model=o12_pro (12 joints), action=perform, skills=['grasp_bottle_until_contact', 'grasp_glass_until_contact', 'grip_handle', 'hand_rest_fist', 'open_hand', 'pre_grip_handle', 'release_bottle', 'release_glass', 'release_handle', 'stop_hand']
[coordinator-5] [INFO] [1787233722.171507008] [agx_arm_coordinator]: Coordinator up: config_dir=/home/user/workspace/agx_arm_ros/install/agx_arm_coordination/share/agx_arm_coordination/config, activities=['both_arms_lift_pour_return_v1', 'both_arms_pregrasp_grasp_retract_v1', 'hands_open_close_release_v1', 'hands_open_release_v1', 'hefeweizen_pour_v1', 'tea_pour_left_v1'], arm_groups=['both_arms', 'left_arm', 'right_arm'], arm_dry_run=False, bus_topology=dedicated_per_device (same-side arm and hand may overlap, arm handoff off)
[coordinator-5] [INFO] [1787233819.093224800] [agx_arm_coordinator]: running activity 'tea_pour_left_v1' (17 nodes)
[coordinator-5] [INFO] [1787233819.096531072] [agx_arm_coordinator]: -> dispatch left_hand_rest_fist ([10])
[omnihand_skill_controller-2] [INFO] [1787233819.103603776] [left_hand.omnihand_skill_controller]: [left] perform left_hand_rest_fist (skill=hand_rest_fist, motion=pose)
[omnihand_bridge-1] [INFO] [1787233819.107550112] [left_hand.omnihand_bridge_node]: hand_left transport claimed by 'reactive:omnihand_skill_controller'
[omnihand_bridge-1] [INFO] [1787233821.067675296] [left_hand.omnihand_bridge_node]: hand_left transport released by 'reactive:omnihand_skill_controller'
[coordinator-5] [INFO] [1787233821.598886496] [agx_arm_coordinator]: -> dispatch left_arm_to_teapot_grip_idle ([20])
[coordinator-5] [INFO] [1787233826.446286816] [agx_arm_coordinator]: -> dispatch left_hand_pre_grip_handle ([30])
[omnihand_skill_controller-2] [INFO] [1787233826.455346144] [left_hand.omnihand_skill_controller]: [left] perform left_hand_pre_grip_handle (skill=pre_grip_handle, motion=pose)
[omnihand_bridge-1] [INFO] [1787233826.459063168] [left_hand.omnihand_bridge_node]: hand_left transport claimed by 'reactive:omnihand_skill_controller'
[omnihand_bridge-1] [INFO] [1787233828.066037984] [left_hand.omnihand_bridge_node]: hand_left transport released by 'reactive:omnihand_skill_controller'
[coordinator-5] [INFO] [1787233828.602226208] [agx_arm_coordinator]: -> dispatch left_arm_to_teapot_pre_grip ([40])
[coordinator-5] [INFO] [1787233831.067437344] [agx_arm_coordinator]: -> dispatch left_arm_teapot_handle_entry ([50])
[coordinator-5] [INFO] [1787233836.744556864] [agx_arm_coordinator]: -> dispatch left_arm_to_teapot_grip ([60])
[coordinator-5] [INFO] [1787233839.389269568] [agx_arm_coordinator]: -> dispatch left_hand_grip_handle ([70])
[omnihand_skill_controller-2] [INFO] [1787233839.394551360] [left_hand.omnihand_skill_controller]: [left] perform left_hand_grip_handle (skill=grip_handle, motion=pose)
[omnihand_bridge-1] [INFO] [1787233839.398598976] [left_hand.omnihand_bridge_node]: hand_left transport claimed by 'reactive:omnihand_skill_controller'
[omnihand_bridge-1] [INFO] [1787233840.288396032] [left_hand.omnihand_bridge_node]: hand_left transport released by 'reactive:omnihand_skill_controller'
[coordinator-5] [INFO] [1787233840.363946976] [agx_arm_coordinator]: payload attach applied on left: payload attached; gravity model /tmp/duo_system.urdf_gravity_l72119gq_payload_i58dm7f5.urdf
[coordinator-5] [INFO] [1787233840.878393248] [agx_arm_coordinator]: -> dispatch left_arm_to_teapot_post_grip ([80])
[coordinator-5] [INFO] [1787233843.858328608] [agx_arm_coordinator]: -> dispatch left_arm_to_pour_init ([90])
[coordinator-5] [INFO] [1787233848.145634016] [agx_arm_coordinator]: -> dispatch left_arm_to_pour_idle ([100])
[coordinator-5] [INFO] [1787233853.240949088] [agx_arm_coordinator]: -> dispatch left_arm_pour_tea ([110])
[coordinator-5] [INFO] [1787233874.497331072] [agx_arm_coordinator]: -> dispatch left_arm_to_pour_init ([120])
[coordinator-5] [INFO] [1787233879.402727424] [agx_arm_coordinator]: -> dispatch left_arm_to_teapot_pre_place ([130])
[coordinator-5] [INFO] [1787233883.750910528] [agx_arm_coordinator]: -> dispatch left_arm_to_teapot_place ([140])
[coordinator-5] [INFO] [1787233885.552244032] [agx_arm_coordinator]: -> dispatch left_hand_release_handle ([150])
[omnihand_skill_controller-2] [INFO] [1787233885.561799712] [left_hand.omnihand_skill_controller]: [left] perform left_hand_release_handle (skill=release_handle, motion=pose)
[omnihand_bridge-1] [INFO] [1787233885.565381792] [left_hand.omnihand_bridge_node]: hand_left transport claimed by 'reactive:omnihand_skill_controller'
[omnihand_bridge-1] [INFO] [1787233886.404020704] [left_hand.omnihand_bridge_node]: hand_left transport released by 'reactive:omnihand_skill_controller'
[coordinator-5] [INFO] [1787233886.480732672] [agx_arm_coordinator]: payload detach applied on left: payload detached; gravity model /tmp/duo_system.urdf_gravity_l72119gq.urdf
[coordinator-5] [INFO] [1787233886.994525696] [agx_arm_coordinator]: -> dispatch left_arm_teapot_handle_release ([160])
[omnihand_bridge-3] [WARN] [1787233892.008403744] [right_hand.omnihand_bridge_node]: unit safety generation 1: stopped=True (arm_left: emergency stop requested)
[omnihand_bridge-1] [WARN] [1787233892.008758592] [left_hand.omnihand_bridge_node]: unit safety generation 1: stopped=True (arm_left: emergency stop requested)
[omnihand_bridge-3] [WARN] [1787233892.010551328] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-1] [WARN] [1787233892.010587840] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
[coordinator-5] [ERROR] [1787233892.035648352] [agx_arm_coordinator]: aborting 'tea_pour_left_v1': child failed: recorded replay: MoveIt error_code=-4
[omnihand_bridge-1] [WARN] [1787233893.962278592] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
[omnihand_bridge-3] [WARN] [1787233893.962406496] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-3] [WARN] [1787233895.961941536] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-1] [WARN] [1787233895.962296768] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
[omnihand_bridge-1] [WARN] [1787233897.962281664] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
[omnihand_bridge-3] [WARN] [1787233897.962547104] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-1] [WARN] [1787233899.962244096] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
[omnihand_bridge-3] [WARN] [1787233899.962422144] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-3] [WARN] [1787233901.961281248] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-1] [WARN] [1787233901.962299040] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
[omnihand_bridge-1] [WARN] [1787233903.962353408] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
[omnihand_bridge-3] [WARN] [1787233903.962736768] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-3] [WARN] [1787233905.962087840] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-1] [WARN] [1787233905.963171328] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
[omnihand_bridge-3] [WARN] [1787233907.961902592] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-1] [WARN] [1787233907.962256064] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
[omnihand_bridge-1] [WARN] [1787233909.962207776] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
[omnihand_bridge-3] [WARN] [1787233909.962464352] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-3] [WARN] [1787233911.962245024] [right_hand.omnihand_bridge_node]: unit stop: hand_right holding its measured pose
[omnihand_bridge-1] [WARN] [1787233911.962403872] [left_hand.omnihand_bridge_node]: unit stop: hand_left holding its measured pose
```

#### l3 script:
```bash
user@ubuntu:~/workspace/agx_arm_ros$ python3 scripts/l3_estop_pcap_run.py
run:      tea_pour_left_v1, stop during node 160 +5.0s
capture:  can_nero_left, can_nero_right
output:   /home/user/workspace/agx_arm_ros/logs/estop_20260820_155013

starting capture ...
sending the activity goal ...
waiting for node 160 to start ...
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
  [ 110] left_arm_pour_tea -> completed
  [ 120] left_arm_to_pour_init -> running
  [ 120] left_arm_to_pour_init -> completed
  [ 130] left_arm_to_teapot_pre_place -> running
  [ 130] left_arm_to_teapot_pre_place -> completed
  [ 140] left_arm_to_teapot_place -> running
  [ 140] left_arm_to_teapot_place -> completed
  [ 150] left_hand_release_handle -> running
  [ 150] left_hand_release_handle -> completed
  [ 160] left_arm_teapot_handle_release -> running
node 160 is running; stopping in 5.0s
>>> EMERGENCY STOP
  left: success=True nero stop=verified — confirmed stopped (peak 0.034 rad/s (dt=19ms)); this device is latched and refuses motion until clear_fault_lockout
  right: success=True nero stop=verified — confirmed stopped (peak 0.048 rad/s (dt=18ms)); this device is latched and refuses motion until clear_fault_lockout
holding the capture open for 8.0s ...
capture stopped:
  can_nero_left: /home/user/workspace/agx_arm_ros/logs/estop_20260820_155013/can_nero_left.pcap (11.0 MB)
  can_nero_right: /home/user/workspace/agx_arm_ros/logs/estop_20260820_155013/can_nero_right.pcap (8.6 MB)

/usr/bin/python3 /home/user/workspace/agx_arm_ros/scripts/analyze_can_pcap.py --stop-at 1787233892.001450 /home/user/workspace/agx_arm_ros/logs/estop_20260820_155013/can_nero_left.pcap /home/user/workspace/agx_arm_ros/logs/estop_20260820_155013/can_nero_right.pcap

=== /home/user/workspace/agx_arm_ros/logs/estop_20260820_155013/can_nero_left.pcap ===
  229125 frames over 83.33s = 2750 f/s
  RX 2161 f/s, TX 589 f/s, 35 distinct IDs
  MIT command frames: 49070 = 589 f/s (84 Hz per joint)
  joint feedback — reported velocity vs velocity derived from the same frames:
    joint 1: reported     peak 1   derived peak=  2416.9 mean=  114.8   position span=2966
    joint 2: reported     peak 1   derived peak=  2215.7 mean=   76.1   position span=2029
    joint 3: reported  FLAT ZERO   derived peak=  2391.3 mean=   50.8   position span=1411
    joint 4: reported     peak 1   derived peak=  1556.2 mean=   25.1   position span=603
    joint 5: reported     peak 1   derived peak=  3680.1 mean=   96.8   position span=1832
    joint 6: reported     peak 1   derived peak=  2661.0 mean=   44.3   position span=555
    joint 7: reported  FLAT ZERO   derived peak=  2032.6 mean=   43.2   position span=748
  VERDICT: joints moved while the velocity field stayed at 0 (+/-1).
           The firmware does not report usable velocity — deriving it
           from positions is the only available source, and removing
           the vendor's zeroing would expose nothing but zeros.
=== stop signature: /home/user/workspace/agx_arm_ros/logs/estop_20260820_155013/can_nero_left.pcap ===
  MIT command frames: 3501 in the 5s before the stop, 11 after it
  joint position control frames after the stop: 4
  MOVE modes commanded after the stop: MIT x2, MOVE-J x1
  PASS: no electronic emergency stop frame anywhere in the capture
  FAIL: 11 MIT command frames after the stop — the control stream did not end
  FAIL: a MIT move mode was commanded after the stop
  VERDICT: NOT clean
=== /home/user/workspace/agx_arm_ros/logs/estop_20260820_155013/can_nero_right.pcap ===
  179966 frames over 83.14s = 2165 f/s
  RX 2165 f/s, TX 0 f/s, 27 distinct IDs
  joint feedback — reported velocity vs velocity derived from the same frames:
    joint 1: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
    joint 2: reported  FLAT ZERO   derived peak=   598.1 mean=    1.0   position span=78
    joint 3: reported  FLAT ZERO   derived peak=   166.2 mean=    0.0   position span=1
    joint 4: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
    joint 5: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
    joint 6: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
    joint 7: reported  FLAT ZERO   derived peak=     0.0 mean=    0.0   position span=0
  VERDICT: joints moved while the velocity field stayed at 0 (+/-1).
           The firmware does not report usable velocity — deriving it
           from positions is the only available source, and removing
           the vendor's zeroing would expose nothing but zeros.
=== stop signature: /home/user/workspace/agx_arm_ros/logs/estop_20260820_155013/can_nero_right.pcap ===
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