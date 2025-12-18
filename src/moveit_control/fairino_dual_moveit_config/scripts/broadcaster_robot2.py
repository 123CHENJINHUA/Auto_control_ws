#!/usr/bin/env python3
"""
重构版串行轨迹广播节点 (解决 _executor_event 错误)
核心：使用完全同步的回调模型，避免任何潜在的 async 与 rclpy 的冲突。
"""
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.task import Future
from control_msgs.action import FollowJointTrajectory

class RobustSerialBroadcaster(Node):
    def __init__(self, node_name):
        super().__init__(node_name)


        self.client1 = ActionClient(self, FollowJointTrajectory,'/fairino5_controller/follow_joint_trajectory')
        self.client2 = ActionClient(self, FollowJointTrajectory,'/gazebo/fairino5_controller/follow_joint_trajectory')

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/'+node_name+'/follow_joint_trajectory',
            self.execute_callback  # 这是一个普通的同步方法
        )


    def execute_callback(self, goal_handle):

        trajectory = goal_handle.request.trajectory

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory  # 注意：此处假设关节顺序完全一致
        
        future1 = self.client1.send_goal_async(goal_msg)
        future2 = self.client2.send_goal_async(goal_msg)

        # 2. 等待两个目标都被接受
        rclpy.spin_until_future_complete(self, future1)
        rclpy.spin_until_future_complete(self, future2)
        gh1 = future1.result()
        gh2 = future2.result()

        if not (gh1.accepted and gh2.accepted):
            goal_handle.abort()
            return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED)

        # 3. 等待两个任务都执行出最终结果
        result_future1 = gh1.get_result_async()
        result_future2 = gh2.get_result_async()
        rclpy.spin_until_future_complete(self, result_future1)
        rclpy.spin_until_future_complete(self, result_future2)
        final_result1 = result_future1.result().result
        final_result2 = result_future2.result().result

        
        result = FollowJointTrajectory.Result()
        goal_handle.succeed()
        self.get_logger().info("succeed!!!!!!!!!!!!!!!!!!!!")
        return result

        # if final_result1.error_code == final_result1.SUCCESSFUL and final_result2.error_code == final_result2.SUCCESSFUL:
        #     self.get_logger().info("succeed!!!!!!!!!!!!!!!!!!!!")
        #     goal_handle.succeed()
            
        #     result.error_code = result.SUCCESSFUL
        # else:
        #     self.get_logger().info("fail!!!!!!!!!!!!!!!!!!!!")
        #     goal_handle.abort()
            
        #     result.error_code = result.PATH_TOLERANCE_VIOLATED

        # self.get_logger().info("return")
        # return result

def main_robot1(args=None):

    rclpy.init(args=args)
    
    node_name = 'broadcaster_robot2'

    
    node = RobustSerialBroadcaster(node_name)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main_robot1()