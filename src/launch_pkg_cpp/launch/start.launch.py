import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
import sys

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config',default='xbox')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev',default='0')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist',default='false')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath',default=[
        launch.substitutions.TextSubstitution(text=os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'config', '')),
        joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')])

    launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel')
    launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox')
    launch.actions.DeclareLaunchArgument('joy_dev', default_value='0')
    launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='false')
    launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
        launch.substitutions.TextSubstitution(text=os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'config', '')),
        joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')])

    action_node_joy_node = launch_ros.actions.Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{
            'device_id': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }])
    action_node_teleop_node = launch_ros.actions.Node(
        package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
        remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel',default='cmd_vel'))},
        )

    action_node_joy_trans_node = launch_ros.actions.Node(
        package='joy_pkg', executable='joy_trans', output='screen'
    )


    action_node_joy_sub_node = launch_ros.actions.Node(
        package='joy_pkg', executable='joy_sub', output='screen'
    )

    action_node_joy2robot_node = launch_ros.actions.Node(
        package='joy_pkg', executable='joy2robot', output='screen'
    )

   

    return launch.LaunchDescription([
        action_node_joy_node,
        action_node_teleop_node,
        action_node_joy_trans_node,
        # action_node_joy_sub_node,
        action_node_joy2robot_node,
    ])