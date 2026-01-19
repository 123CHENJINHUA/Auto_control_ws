from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_ip = LaunchConfiguration('robot_ip')
    robot2_ip = LaunchConfiguration('robot2_ip')
    mapping = LaunchConfiguration('mapping')
    cmd_period = LaunchConfiguration('cmd_period')
    joint6_step = LaunchConfiguration('joint6_step')
    start_bridge = LaunchConfiguration('start_bridge')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('robot_ip', default_value='192.168.57.3', description='Right robot IP'),
        DeclareLaunchArgument('robot2_ip', default_value='192.168.57.2', description='Left robot IP'),
        DeclareLaunchArgument('mapping', default_value='delta_base', description='Mapping: absolute | delta_base | delta_tool'),
        DeclareLaunchArgument('cmd_period', default_value='0.008', description='Command period in seconds'),
        DeclareLaunchArgument('joint6_step', default_value='-5.0', description='Joint-6 delta degrees on A-press'),
        DeclareLaunchArgument('start_bridge', default_value='true', description='Start button_command_bridge node'),

        # VR actions publisher node
        Node(
            package='vr_oculus2_pkg',
            executable='VR',
            output='screen',
        ),

        # Halfauto dual-robot controller node listening to /vr/actions
        Node(
            package='halfauto_pkg',
            executable='halfauto_node',
            output='screen',
            arguments=[
                '--robot-ip', robot_ip,
                '--robot2-ip', robot2_ip,
                '--mapping', mapping,
                '--cmd-period', cmd_period,
                '--use-gripper',
                '--joint6-step', joint6_step,
            ],
        ),

        # Optional: button_command_bridge to forward buttons to an external tool
        Node(
            package='button_command_bridge',
            executable='button_command_bridge_node',
            output='screen',
            # Uses its own defaults; override via ros2 params if needed
            condition=IfCondition(start_bridge),
        ),
    ])
