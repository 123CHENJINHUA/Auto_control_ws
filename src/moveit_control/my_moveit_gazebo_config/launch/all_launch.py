import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import subprocess


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


def load_robot_description_from_xacro(xacro_file_path):
    """
    一个可靠的函数，用于在Launch上下文的环境中执行xacro命令。
    它会捕获并打印完整的错误信息，便于调试。
    """
    try:
        # 关键：使用当前进程的环境（它包含了通过source设置的所有ROS变量）
        env = os.environ.copy()
        
        # 运行xacro命令，捕获输出和错误
        result = subprocess.run(
            ['xacro', xacro_file_path],
            capture_output=True,  # 捕获stdout和stderr
            text=True,            # 以文本形式返回
            check=True,           # 如果返回码非零则抛出CalledProcessError异常
            env=env,              # 传递当前环境
            cwd=os.path.dirname(xacro_file_path)  # 将工作目录设为xacro文件所在目录
        )
        # 成功则返回解析后的URDF字符串
        return result.stdout
        
    except subprocess.CalledProcessError as e:
        # 如果失败，打印出完整的、之前被吞掉的错误信息！
        print("\n========== XACRO 命令执行失败 ==========")
        print("命令:", e.cmd)
        print("返回码:", e.returncode)
        print("\n--- 标准输出 (stdout) ---")
        print(e.stdout)
        print("\n--- 标准错误 (stderr) ---")
        print(e.stderr)  # 这就是我们一直想看到的关键错误！
        print("=====================================\n")
        # 重新抛出异常，让Launch过程终止
        raise

def launch_setup(context, *args, **kwargs):



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

    # substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    # robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result)

    robot_description_xml_string = load_robot_description_from_xacro(default_urdf_file_path)
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        robot_description_xml_string,
        value_type=str
    )






    controller_file_name = 'ros2_controllers.yaml'


    default_controller_path = os.path.join(
        get_package_share_directory('fairino_dual_moveit_config'),
        'config',
        controller_file_name
    )

    robot_description_file = 'fairino_dual_robot.urdf.xacro'

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
        [FindPackageShare("moveit2_tutorials"), "launch", rviz_base]
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
    
    # 启动机器人状态发布节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_value},{'use_sim_time': True},{"publish_frequency":50.0}]
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-topic', 'robot_description', '-entity', 'dual_robot'],
        output='screen'
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
