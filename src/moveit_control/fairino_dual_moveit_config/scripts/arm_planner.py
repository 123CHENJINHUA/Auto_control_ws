#!/usr/bin/env python3
"""
机械臂规划节点 - 使用OMPL
"""

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit.planning import PlanRequestParameters
import os
import time

class ArmPlanner(Node):
    def __init__(self):
        super().__init__('arm_planner')
        
        # 获取配置包路径
        self.declare_parameter('config_package', '')
        self.config_package = self.get_parameter('config_package').value
        
        # OMPL规划器参数
        self.declare_parameter('planner_name', 'RRTConnect')
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('planning_attempts', 5)

        # 初始化Action客户端
        self.action_client1 = ActionClient(
            self,
            FollowJointTrajectory,
            # 'broadcaster_robot1/follow_joint_trajectory'
            'fairino5_controller/follow_joint_trajectory'
        )

        # 初始化Action客户端
        self.action_client2 = ActionClient(
            self,
            FollowJointTrajectory,
            # 'broadcaster_robot2/follow_joint_trajectory'
            'fairino16_controller/follow_joint_trajectory'
        )

        # 订阅位姿目标
        self.create_subscription(
            PoseStamped,
            'goal_pose1',
            self.pose_callback1,
            10
        )

        self.create_subscription(
            PoseStamped,
            'goal_pose2',
            self.pose_callback2,
            10
        )

        self.init_moveit()
    
    def init_moveit(self):
        """使用MoveItConfigsBuilder初始化MoveIt和OMPL"""

        config_path = get_package_share_directory("fairino_dual_moveit_config")
        
        # 使用MoveItConfigsBuilder加载配置
        moveit_config = (
            MoveItConfigsBuilder(self.config_package)
            # .robot_description(file_path=os.path.join(config_path, "config", "fairino_dual_robot_fake.xacro"))
            .robot_description(file_path=os.path.join(config_path, "config", "fairino_dual_robot_real.xacro"))
            # .trajectory_execution(file_path=os.path.join(config_path, "config", "moveit_controllers_all.yaml"))
            # .planning_scene_monitor(
            #     publish_robot_description=True, publish_robot_description_semantic=True
            # )
            .joint_limits(file_path=os.path.join(config_path, "config", "joint_limits_slow.yaml"))
            .moveit_cpp(
                file_path=os.path.join(
                        get_package_share_directory("fairino_dual_moveit_config"),
                        "config",
                        "moveit_cpp.yaml",
                        )
            )
            .to_moveit_configs()
        )


        self.moveit = MoveItPy(node_name="moveit_py", config_dict=moveit_config.to_dict())
        self.arm1 = self.moveit.get_planning_component('fairino5_v6_group')
        self.arm2 = self.moveit.get_planning_component('fairino16_v6_group')
        
        # self.get_logger().info(self.arm1.get_start_state())
        # self.get_logger().info(self.arm2.get_start_state())


    #     # 设置OMPL规划器参数
    #     self.set_ompl_planner()

    
    # def set_ompl_planner(self):
    #     """设置OMPL规划器参数"""
    #     self.planner_name = self.get_parameter('planner_name').value
    #     planning_time = self.get_parameter('planning_time').value
    #     planning_attempts = self.get_parameter('planning_attempts').value
        
    #     # 设置规划器ID（OMPL规划器格式为 "ompl_interface/OMPLPlanner[规划器名]"）
    #     planner_id = f"ompl_interface/OMPLPlanner[{self.planner_name}]"
    #     self.arm.set_planner_id(planner_id)
        
    #     # 设置规划时间和尝试次数
    #     self.arm.set_planning_time(planning_time)
    #     self.arm.set_num_planning_attempts(planning_attempts)
    
    def pose_callback1(self, msg):
        """位姿规划回调 - 使用OMPL规划"""
        
        self.arm1.set_goal_state(pose_stamped_msg=msg, pose_link="tcp1")
        result = self.arm1.plan() # 使用OMPL规划
        self.execute_trajectory1(result.trajectory)

    
    def execute_trajectory1(self, trajectory):
        """执行轨迹"""
        # 准备action目标
        goal_msg = FollowJointTrajectory.Goal()
        if trajectory is None:
            self.get_logger().info("sorry, we cannot reach the goal")
        else:
            goal_msg.trajectory = trajectory.get_robot_trajectory_msg().joint_trajectory
            goal_msg.trajectory.joint_names = self.moveit.get_robot_model().\
                get_joint_model_group('fairino5_v6_group').joint_model_names
            
            # 发送目标
            self.get_logger().info("planning!!!!!!!!!!!!!!!!!!!!")
            self.action_client1.wait_for_server()
            future = self.action_client1.send_goal_async(goal_msg)
            # rclpy.spin_until_future_complete(self, future)
            # gh = future.result()
            # result_future = gh.get_result_async()
            # rclpy.spin_until_future_complete(self, result_future)
            # self.get_logger().info("complete!!!!")


    def pose_callback2(self, msg):
        """位姿规划回调 - 使用OMPL规划"""

        self.arm2.set_goal_state(pose_stamped_msg=msg, pose_link="tcp2")
        result = self.arm2.plan() # 使用OMPL规划
        self.execute_trajectory2(result.trajectory)

    
    def execute_trajectory2(self, trajectory):
        """执行轨迹"""
        # 准备action目标
        goal_msg = FollowJointTrajectory.Goal()
        if trajectory is None:
            self.get_logger().info("sorry, we cannot reach the goal")
        else:
            goal_msg.trajectory = trajectory.get_robot_trajectory_msg().joint_trajectory
            goal_msg.trajectory.joint_names = self.moveit.get_robot_model().\
                get_joint_model_group('fairino16_v6_group').joint_model_names
            
            # 发送目标
            self.get_logger().info("planning!!!!!!!!!!!!!!!!!!!!")
            self.action_client2.wait_for_server()
            future = self.action_client2.send_goal_async(goal_msg)
            # rclpy.spin_until_future_complete(self, future)
            # gh = future.result()
            # result_future = gh.get_result_async()robot8
            # rclpy.spin_until_future_complete(self, result_future)
            # self.get_logger().info("complete!!!!")


def main(args=None):
    rclpy.init(args=args)
    node = ArmPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()