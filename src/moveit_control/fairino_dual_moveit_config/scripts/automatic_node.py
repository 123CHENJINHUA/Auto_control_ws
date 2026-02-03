#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import transforms3d.quaternions as tfq
import numpy as np

class PoseForwarderNode(Node):
    def __init__(self):
        super().__init__('pose_forwarder_node')
        
        # 创建订阅者，订阅 'wall' 话题
        self.subscription = self.create_subscription(
            PoseStamped,
            '/wall_target',  # 源话题
            self.wall_callback,
            10  # QoS深度
        )
        
        # 创建发布者，发布到 '/goal_pose1' 话题
        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose1',  # 目标话题
            10  # QoS深度
        )
        
        # self.get_logger().info('Pose forwarder node 已启动')
        # self.get_logger().info('正在监听 "wall" 话题...')
        # self.get_logger().info('将转发消息到 "/goal_pose1" 话题')

    def wall_callback(self, msg):
        """
        当从 'wall' 话题接收到消息时的回调函数
        """
        # 打印接收到的消息信息（可选，用于调试）
        # self.get_logger().debug(
        #     f'收到消息: position=({msg.pose.position.x:.2f}, '
        #     f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
        # )
        
        # 创建新的PoseStamped消息
        forward_msg = PoseStamped()
        
        # 复制头部信息
        forward_msg.header = msg.header
        forward_msg.header.stamp = self.get_clock().now().to_msg()  # 更新时间戳
        
        quaternion = msg.pose.orientation
        
        q = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
    
    # 将向量转换为numpy数组
        vector = [0, 0, -1]  # 示例向量
        v = np.array(vector)
        v_rotated = 0.45*tfq.rotate_vector(v, q)
        
        # 复制位置信息
        forward_msg.pose.position.x = msg.pose.position.x + v_rotated[0]
        forward_msg.pose.position.y = msg.pose.position.y + v_rotated[1]
        forward_msg.pose.position.z = msg.pose.position.z + v_rotated[2]
        
        # 复制姿态信息
        forward_msg.pose.orientation.x = quaternion.x
        forward_msg.pose.orientation.y = quaternion.y
        forward_msg.pose.orientation.z = quaternion.z
        forward_msg.pose.orientation.w = quaternion.w
        
        # 发布到目标话题
        self.publisher.publish(forward_msg)
        
        # 记录发布信息（可选）
        self.get_logger().debug(f'已转发消息到 "/goal_pose1"')

def main(args=None):
    rclpy.init(args=args)
    
    node = PoseForwarderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('正在关闭节点...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()