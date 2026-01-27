#!/bin/bash
# ros2 topic pub -r 33 servo_node2/delta_joint_cmds control_msgs/msg/JointJog "{header: {stamp: now, frame_id: 'robot2_base_link'}, joint_names: ['robot2_j1'], velocities: [-1.5]}" 
# ros2 topic pub /servo_node2/delta_joint_cmds control_msgs/msg/JointJog "{header: {stamp: now, frame_id: 'robot2_base_link'}, joint_names: ['robot2_j1'], velocities: [-1.0]}" --once
# ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: 'fairino16_controller'}"
# ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: 'fairino5_controller'}"
ros2 control set_controller_state fairino5_controller_servo inactive
ros2 control set_controller_state fairino5_controller active