import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.parameter_descriptions
from launch.actions import LogInfo, TimerAction, SetEnvironmentVariable
import subprocess

def generate_launch_description():
    # 获取URDF文件路径
    package_name = 'fairino_dual_moveit_config'  # 替换为你的包名
    urdf_file_name = 'fairino_dual_robot_gazebo.xacro'  # 替换为你的URDF文件名

    goal_list = []


    default_urdf_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        urdf_file_name
    )

    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_urdf_file_path),description='URDF file'
    )

    substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result)

    # 启动机器人状态发布节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace="gazebo",
        parameters=[{'robot_description': robot_description_value},{'use_sim_time': True},{"publish_frequency":50.0}],
    )


    # 启动机器人模型加载到Gazebo
    # this will activate the controller_manager and set the parameters for the manager
    # but will not load the controllers specified in the yaml
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        namespace="gazebo",
        arguments=['-topic', 'robot_description', '-entity', 'dual_robot'],
        output='screen'
    )
    
    # load the real controller
    action_load_controller1 = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        namespace="gazebo",
        arguments=['fairino5_controller', '--controller-manager', 'controller_manager'],
        output='screen',
    )

    action_load_controller2 = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        namespace="gazebo",
        arguments=['fairino16_controller', '--controller-manager', 'controller_manager'],
        output='screen',
    )

    action_load_broadcaster = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        namespace="gazebo",
        arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
        output='screen',
    )

    goal_list.append(action_declare_arg_model_path)
    goal_list.append(action_robot_state_publisher)
    goal_list.append(action_spawn_entity)
    goal_list.append(action_load_controller1)
    goal_list.append(action_load_controller2)
    goal_list.append(action_load_broadcaster)



    default_gazebo_file_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'my_env.world'
    )

     # 启动Gazebo节点
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    action_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments=[('world', default_gazebo_file_path),('verbose', 'true')]
    )

    goal_list.append(action_gazebo)


    return LaunchDescription(goal_list)