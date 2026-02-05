#!/bin/bash
# ros2 topic pub -r 33 servo_node2/delta_joint_cmds control_msgs/msg/JointJog "{header: {stamp: now, frame_id: 'robot2_base_link'}, joint_names: ['robot2_j1'], velocities: [-1.5]}" 
# ros2 topic pub /servo_node2/delta_joint_cmds control_msgs/msg/JointJog "{header: {stamp: now, frame_id: 'robot2_base_link'}, joint_names: ['robot2_j1'], velocities: [-1.0]}" --once
# ros2 topic pub --once /wall_target geometry_msgs/msg/PoseStamped "
# header:
#   stamp:
#     sec: 0
#     nanosec: 0
#   frame_id: 'robot1_base_link'
# pose:
#   position:
#     x: 0.921
#     y: -0.222
#     z: -0.232
#   orientation:
#     x: -0.09369560843459067
#     y: 0.7202041313939372
#     z: 0.10378768814790662
#     w: 0.6795257595353461

# "

ros2 param set /pose_forwarder_node target_id $1