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
    x: 0.18165257196706897
    y: -0.5610071873755366
    z: 0.22368551298388026
  orientation:
    x: -0.2651204942363172
    y: 0.6499259598615332
    z: 0.2477150048458313
    w: 0.6677908704139234
"
