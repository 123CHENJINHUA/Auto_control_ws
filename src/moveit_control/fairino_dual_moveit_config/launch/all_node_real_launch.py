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

    moveit_config = (
        MoveItConfigsBuilder("fairino_dual")
        .robot_description(file_path=default_description_path)
        .to_moveit_configs()
    )


    rviz_config = LaunchConfiguration("rviz_config")

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{'use_sim_time': False}],
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
        arguments=['fairino16_controller', '--controller-manager', 'controller_manager'],
        output='screen',
    )

    action_load_controller2 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fairino5_controller', '--controller-manager', 'controller_manager'],
        output='screen',
    )

    action_load_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
        output='screen',
    )

    nodes_to_start = [
        config_pkg,
        planner_name,
        planning_time,
        arm_planner,
        rviz_node,
        robot_state_publisher,
        ros2_control_node,
        action_load_controller1,
        action_load_controller2,
        action_load_broadcaster
    ]

########################################################################################################

    # package_name = 'fairino_dual_moveit_config'  # 替换为你的包名
    # urdf_file_name = 'fairino_dual_robot_gazebo.xacro'  # 替换为你的URDF文件名

    # goal_list = []


    # default_urdf_file_path = os.path.join(
    #     get_package_share_directory(package_name),
    #     'config',
    #     urdf_file_name
    # )

    # action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
    #     name='model', default_value=str(default_urdf_file_path),description='URDF file'
    # )

    # substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    # robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result)

    # # 启动机器人状态发布节点
    # action_robot_state_publisher = launch_ros.actions.Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     namespace='gazebo',
    #     parameters=[{'robot_description': robot_description_value},{'use_sim_time': True},{"publish_frequency":50.0}],
    # )


    # # 启动机器人模型加载到Gazebo
    # # this will activate the controller_manager and set the parameters for the manager
    # # but will not load the controllers specified in the yaml
    # action_spawn_entity = launch_ros.actions.Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     name='spawn_entity',
    #     namespace='gazebo',
    #     arguments=['-topic', 'robot_description', '-entity', 'dual_robot'],
    #     output='screen'
    # )
    
    # # load the real controller
    # action_load_controller1 = launch_ros.actions.Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     namespace='gazebo',
    #     arguments=['fairino5_controller', '--controller-manager', 'controller_manager'],
    #     output='screen',
    # )

    # action_load_controller2 = launch_ros.actions.Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     namespace='gazebo',
    #     arguments=['fairino16_controller', '--controller-manager', 'controller_manager'],
    #     output='screen',
    # )

    # action_load_broadcaster = launch_ros.actions.Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     namespace='gazebo',
    #     arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
    #     output='screen',
    # )

    # goal_list.append(action_declare_arg_model_path)
    # goal_list.append(action_robot_state_publisher)
    # goal_list.append(action_spawn_entity)
    # goal_list.append(action_load_controller1)
    # goal_list.append(action_load_controller2)
    # goal_list.append(action_load_broadcaster)

    # default_gazebo_file_path = os.path.join(
    #     get_package_share_directory(package_name),
    #     'worlds',
    #     'my_env.world'
    # )

    #  # 启动Gazebo节点
    # gazebo_launch_file = os.path.join(
    #     get_package_share_directory('gazebo_ros'),
    #     'launch',
    #     'gazebo.launch.py'
    # )

    # action_gazebo = launch.actions.IncludeLaunchDescription(
    #     launch.launch_description_sources.PythonLaunchDescriptionSource(gazebo_launch_file),
    #     launch_arguments=[('world', default_gazebo_file_path),('verbose', 'true')]
    # )

    # goal_list.append(action_gazebo)


#######################################################################################################

    # # 转发结点，将moveit的结果转发至两个systems
    # broadcaster_robot1_node = Node(
    #     package='fairino_dual_moveit_config',
    #     executable='broadcaster_robot1.py', 
    #     name='broadcaster_robot1',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # broadcaster_robot2_node = Node(
    #     package='fairino_dual_moveit_config',
    #     executable='broadcaster_robot2.py',
    #     name='broadcaster_robot2',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )



    # return nodes_to_start + goal_list + [broadcaster_robot1_node,broadcaster_robot2_node]
    return nodes_to_start

