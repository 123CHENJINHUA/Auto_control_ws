import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.parameter_descriptions

from moveit_configs_utils import MoveItConfigsBuilder


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_rsp_launch(ld,moveit_config):
    """Launch file for robot state publisher (rsp)"""

    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
                'use_sim_time': True,
            },
        ],
    )
    ld.add_action(rsp_node)

    return ld


def generate_moveit_rviz_launch(ld,moveit_config):
    """Launch file for rviz"""

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": True},
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return ld


def generate_setup_assistant_launch(ld,moveit_config):
    """Launch file for MoveIt Setup Assistant"""

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    add_debuggable_node(
        ld,
        package="moveit_setup_assistant",
        executable="moveit_setup_assistant",
        arguments=[["--config_pkg=", str(moveit_config.package_path)]],
    )

    return ld


def generate_static_virtual_joint_tfs_launch(ld,moveit_config):

    name_counter = 0

    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            ld.add_action(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link,
                    ],
                )
            )
            name_counter += 1
    return ld


def generate_spawn_controllers_launch(ld,moveit_config):
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )
    return ld


def generate_warehouse_db_launch(ld,moveit_config):
    """Launch file for warehouse database"""
    ld.add_action(
        DeclareLaunchArgument(
            "moveit_warehouse_database_path",
            default_value=str(
                moveit_config.package_path / "default_warehouse_mongo_db"
            ),
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("reset", default_value=False))

    # The default DB port for moveit (not default MongoDB port to avoid potential conflicts)
    ld.add_action(DeclareLaunchArgument("moveit_warehouse_port", default_value="33829"))

    # The default DB host for moveit
    ld.add_action(
        DeclareLaunchArgument("moveit_warehouse_host", default_value="localhost")
    )

    # Load warehouse parameters
    db_parameters = [
        {
            "overwrite": False,
            "database_path": LaunchConfiguration("moveit_warehouse_database_path"),
            "warehouse_port": LaunchConfiguration("moveit_warehouse_port"),
            "warehouse_host": LaunchConfiguration("moveit_warehouse_host"),
            "warehouse_exec": "mongod",
            "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection",
        },
    ]
    # Run the DB server
    db_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        # TODO(dlu): Figure out if this needs to be run in a specific directory
        # (ROS 1 version set cwd="ROS_HOME")
        parameters=db_parameters,
    )
    ld.add_action(db_node)

    # If we want to reset the database, run this node
    reset_node = Node(
        package="moveit_ros_warehouse",
        executable="moveit_init_demo_warehouse",
        output="screen",
        condition=IfCondition(LaunchConfiguration("reset")),
    )
    ld.add_action(reset_node)

    return ld


def generate_move_group_launch(ld,moveit_config):

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {"use_sim_time": True},
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    return ld


def generate_launch_description():
    # 获取URDF文件路径
    package_name = 'my_moveit_gazebo_config'  # 替换为你的包名
 
    moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="my_moveit_gazebo_config").to_moveit_configs()

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

    # 启动机器人模型加载到Gazebo
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-topic', '/robot_description', '-entity', 'fairino5_v6_robot'],
        output='screen'
    )


    urdf_file_name = 'fairino5_v6_robot.urdf.xacro'  

    default_urdf_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        urdf_file_name
    )
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_urdf_file_path),description='URDF file'
    )
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 启动机器人状态发布节点
    action_robot_state_publisher= launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_value},{'use_sim_time': True},{"publish_frequency":15.0}]
    )

    ld = LaunchDescription()
    
    

    ld.add_action(action_declare_arg_model_path)
    ld.add_action(action_robot_state_publisher)
    ld.add_action(action_gazebo)
    ld.add_action(action_spawn_entity)
    
    # ld = generate_rsp_launch(ld,moveit_config)
    ld = generate_static_virtual_joint_tfs_launch(ld,moveit_config)
    ld = generate_spawn_controllers_launch(ld,moveit_config)
    ld = generate_moveit_rviz_launch(ld,moveit_config)
    ld = generate_move_group_launch(ld,moveit_config)

    return ld
