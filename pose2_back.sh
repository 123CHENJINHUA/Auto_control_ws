#!/bin/bash
# ros2 topic pub -r 33 servo_node2/delta_joint_cmds control_msgs/msg/JointJog "{header: {stamp: now, frame_id: 'robot2_base_link'}, joint_names: ['robot2_j1'], velocities: [-1.5]}" 
# ros2 topic pub /servo_node2/delta_joint_cmds control_msgs/msg/JointJog "{header: {stamp: now, frame_id: 'robot2_base_link'}, joint_names: ['robot2_j1'], velocities: [-1.0]}" --once
ros2 topic pub --once /goal_pose2 geometry_msgs/msg/PoseStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'robot2_base_link'
pose:
  position:
    x: 0.23450540533869954
    y: -0.44661269688627264
    z: 0.17997527055519272
  orientation:
    x: -0.2659374246821895
    y: 0.6554323947420785
    z: 0.2497795437449664
    w: 0.6612834805154485

"
