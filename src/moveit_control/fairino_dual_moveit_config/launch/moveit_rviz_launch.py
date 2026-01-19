import os
from launch import LaunchDescription
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
        'moveit.rviz'
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

    robot_description_file = 'fairino_dual_robot_fake.xacro'

    default_description_path = os.path.join(
        get_package_share_directory('fairino_dual_moveit_config'),
        'config',
        robot_description_file
    )

    moveit_config = (
        MoveItConfigsBuilder("fairino_dual")
        .robot_description(file_path=default_description_path)
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )


    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("fairino_dual_moveit_config"), 'config', 'moveit.rviz']
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )


    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
        # parameters=[{'robot_description': moveit_config.robot_description},{'use_sim_time': True},{"publish_frequency":50.0}]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, default_controller_path],
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
        rviz_node,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        action_load_controller1,
        action_load_controller2,
        action_load_broadcaster
    ]


    return nodes_to_start



# def launch_setup(context, *args, **kwargs):

#     controller_file_name = 'ros2_controllers.yaml'

#     robots = ['fairino16','fairino5']
#     name_space = ['robot1','robot2']

#     all_nodes = []

#     for i in range(2):

#         default_controller_path = os.path.join(
#             get_package_share_directory(robots[i]+'_v6_moveit_config'),
#             'config',
#             controller_file_name
#         )

#         robot_description_file = robots[i]+'_v6_robot.urdf.xacro'

#         default_description_path = os.path.join(
#             get_package_share_directory(robots[i]+'_v6_moveit_config'),
#             'config',
#             robot_description_file
#         )

#         moveit_config = (
#             MoveItConfigsBuilder(robots[i]+"_v6")
#             .robot_description(file_path=default_description_path)
#             .planning_scene_monitor(
#                 publish_robot_description=True, publish_robot_description_semantic=True
#             )
#             .planning_pipelines(
#                 pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
#             )
#             .to_moveit_configs()
#         )

#         if i == 0:
#             rviz_base = LaunchConfiguration("rviz_config")
#             rviz_config = PathJoinSubstitution(
#                 [FindPackageShare("moveit2_tutorials"), "launch", rviz_base]
#             )

#             # RViz
#             rviz_node = Node(
#                 package="rviz2",
#                 executable="rviz2",
#                 name="rviz2",
#                 output="log",
#                 arguments=["-d", rviz_config],
#                 parameters=[
#                     moveit_config.robot_description,
#                     moveit_config.robot_description_semantic,
#                     moveit_config.robot_description_kinematics,
#                     moveit_config.planning_pipelines,
#                     moveit_config.joint_limits,
#                 ],
#             )


#         # Start the actual move_group node/action server
#         run_move_group_node = Node(
#             package="moveit_ros_move_group",
#             executable="move_group",
#             output="screen",
#             namespace=name_space[i],
#             parameters=[moveit_config.to_dict()],
#         )

#         robot_state_publisher = Node(
#             package="robot_state_publisher",
#             executable="robot_state_publisher",
#             name="robot_state_publisher",
#             output="both",
#             namespace=name_space[i],
#             parameters=[moveit_config.robot_description],
#             # parameters=[{'robot_description': moveit_config.robot_description},{'use_sim_time': True},{"publish_frequency":50.0}]
#         )

#         ros2_control_node = Node(
#             package="controller_manager",
#             executable="ros2_control_node",
#             namespace=name_space[i],
#             parameters=[moveit_config.robot_description, default_controller_path],
#             output="both",
#         )

#         # load the real controller
#         action_load_controller = Node(
#             package='controller_manager',
#             executable='spawner',
#             namespace=name_space[i],
#             arguments=[robots[i]+'_controller', '--controller-manager', 'controller_manager'],
#             output='screen',
#         )

#         action_load_broadcaster = Node(
#             package='controller_manager',
#             executable='spawner',
#             namespace=name_space[i],
#             arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
#             output='screen',
#         )

#         nodes_to_start = [
#             # rviz_node,
#             robot_state_publisher,
#             run_move_group_node,
#             ros2_control_node,
#             action_load_controller,
#             action_load_broadcaster
#         ]

#         if i == 0:
#             nodes_to_start.append(rviz_node)

#         all_nodes = all_nodes + nodes_to_start

#     return all_nodes
