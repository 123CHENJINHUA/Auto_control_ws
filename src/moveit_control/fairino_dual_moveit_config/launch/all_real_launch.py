import os
from launch import LaunchDescription
import launch
import launch_ros
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    declared_arguments = []

    default_rviz_path = os.path.join(
        get_package_share_directory('fairino_dual_moveit_config'),
        'config',
        'moveit_node.rviz'
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_path,
            description="RViz configuration file",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):

    controller_file_name = 'ros2_controllers_fake.yaml'

    default_controller_path = os.path.join(
        get_package_share_directory('fairino_dual_moveit_config'),
        'config',
        controller_file_name
    )

    robot_description_file = 'fairino_dual_robot_real.xacro'

    default_description_path = os.path.join(
        get_package_share_directory('fairino_dual_moveit_config'),
        'config',
        robot_description_file
    )

    default_semantic_path = os.path.join(
        get_package_share_directory('fairino_dual_moveit_config'),
        'config',
        'fairino_dual_robot.srdf'
    )

    default_kinemetics_path = os.path.join(
        get_package_share_directory('fairino_dual_moveit_config'),
        'config',
        'kinematics.yaml'
    )

    moveit_config = (
        MoveItConfigsBuilder("fairino_dual")
        .robot_description(file_path=default_description_path)
        .robot_description_kinematics(file_path=default_kinemetics_path)
        .robot_description_semantic(file_path=default_semantic_path)
        .to_moveit_configs()
    )

    servo_yaml1 = load_yaml("fairino_dual_moveit_config", "config/left_servo.yaml")
    servo_params1 = {"moveit_servo": servo_yaml1}
    servo_yaml2 = load_yaml("fairino_dual_moveit_config", "config/right_servo.yaml")
    servo_params2 = {"moveit_servo": servo_yaml2}


    servo_node1 = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node1",
        parameters=[
            servo_params1,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics
        ],
        output="screen",
    )

    servo_node2 = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node2",
        parameters=[
            servo_params2,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics
        ],
        output="screen",
    )

    call_service1 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/servo_node1/start_servo',
            'std_srvs/srv/Trigger',
            '{}'
        ],
        output='screen'
    )

    call_service2 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/servo_node2/start_servo',
            'std_srvs/srv/Trigger',
            '{}'
        ],
        output='screen'
    )

    config_pkg = DeclareLaunchArgument(
        name='config_package',
        default_value='fairino_dual',
        description='MoveIt config pkg',
    )
    
    planner_name = DeclareLaunchArgument(
        name='planner_name',
        default_value='RRTConnect',
        description='OMPL planner (RRTConnect, RRT, PRM, etc.)',
    )
    
    planning_time = DeclareLaunchArgument(
        name='planning_time',
        default_value='5.0',
        description='planning time',
    )
    
    arm_planner = Node(
        package='fairino_dual_moveit_config',
        executable='arm_planner.py',
        name='arm_planner',
        parameters=[{
            'config_package': LaunchConfiguration('config_package'),
            'planner_name': LaunchConfiguration('planner_name'),
            'planning_time': LaunchConfiguration('planning_time'),
            'planning_attempts': 5,
            'use_sim_time': False
        }],
        output='screen',
    )    
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,{'use_sim_time': False}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, default_controller_path,{'use_sim_time': False}],
        output="both",
    )

    # load the real controller
    action_load_controller1 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fairino16_controller_servo', '--controller-manager', 'controller_manager'],
        output='screen',
    )

    action_load_controller2 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fairino5_controller_servo', '--controller-manager', 'controller_manager'],
        output='screen',
    )

    action_load_controller3 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fairino16_controller', '--controller-manager', 'controller_manager','--load-only'],
        output='screen',
    )

    action_load_controller4 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fairino5_controller', '--controller-manager', 'controller_manager', '--load-only'],
        output='screen',
    )

    configure_controller16 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/controller_manager/configure_controller',
            'controller_manager_msgs/srv/ConfigureController',
            "{name: fairino16_controller}"
        ],
        output='screen',
    )

    configure_controller5 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/controller_manager/configure_controller',
            'controller_manager_msgs/srv/ConfigureController',
            "{name: fairino5_controller}"
        ],
        output='screen',
    )    

    configure_after_load5 = RegisterEventHandler(
        OnProcessExit(
            target_action=action_load_controller4,  # 假设这是加载fairino5_controller的节点
            on_exit=[configure_controller5],
        )
    )

    configure_after_load16 = RegisterEventHandler(
        OnProcessExit(
            target_action=action_load_controller3,  # 假设这是加载fairino5_controller的节点
            on_exit=[configure_controller16],
        )
    )

    action_load_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
        output='screen',
    )

    VR = Node(
        package='fairino_dual_moveit_config',
        executable='vr_node.py',
        output='screen',
    )

    Transform = Node(
        package='fairino_dual_moveit_config',
        executable='transform_robot.py',
        output='screen',
    )

    nodes_to_start = [
        config_pkg,
        planner_name,
        planning_time,
        arm_planner,
        robot_state_publisher,
        ros2_control_node,
        action_load_controller1,
        action_load_controller2,
        action_load_controller3,
        action_load_controller4,
        action_load_broadcaster,
        servo_node1,
        servo_node2,
        call_service1,
        call_service2,
        configure_after_load5,
        configure_after_load16,
        VR,
        Transform
    ]

########################################################################################################

#######################################################################################################

    return nodes_to_start

