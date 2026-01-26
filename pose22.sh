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
    x: -0.5484701132038559
    y: 0.07534576057888653
    z: 0.078582953339292
  orientation:
    x: -0.3560159443232004
    y: -0.5875057560221204
    z: -0.3636573475119533
    w: 0.6291605261212017

"
