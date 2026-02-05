#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
import transforms3d.quaternions as tfq
from rclpy.parameter import Parameter  # 添加这行
import numpy as np

class PoseForwarderNode(Node):
    def __init__(self):
        super().__init__('pose_forwarder_node')
        
        # 创建订阅者，订阅 'wall' 话题
        self.subscription = self.create_subscription(
            PointStamped,
            '/points_position',  # 源话题
            self.wall_callback,
            10  # QoS深度
        )

        self.subscription2 = self.create_subscription(
            PoseStamped,
            '/depth_to_pose/base1',  # 源话题
            self.pose_callback,
            10  # QoS深度
        )

        
        # 创建发布者，发布到 '/goal_pose1' 话题
        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose1',  # 目标话题
            10  # QoS深度
        )

        self.quaternion = None

        self.declare_parameter('target_id', -10086)  # 默认值为 -10086
        
        # self.get_logger().info('Pose forwarder node 已启动')
        # self.get_logger().info('正在监听 "wall" 话题...')
        # self.get_logger().info('将转发消息到 "/goal_pose1" 话题')

    def pose_callback(self, msg):      

        self.quaternion = msg.pose.orientation

    def wall_callback(self, msg):

        self.target_id = self.get_parameter('target_id').value
        message_id = int(msg.header.frame_id)
        
        if message_id != self.target_id or self.quaternion is None:
            return

        self.get_logger().info(f"Send!!!!!!!!!!!!message_id: {message_id}, target_id: {self.target_id}")
        
        # 创建新的PoseStamped消息
        forward_msg = PoseStamped()
        
        # 复制头部信息
        forward_msg.header = msg.header
        forward_msg.header.frame_id = 'robot1_base_link'  # 修改坐标系为 'robot1_base_link'
        forward_msg.header.stamp = self.get_clock().now().to_msg()  # 更新时间戳
        
        quaternion = self.quaternion
        
        q = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]

    
    # 将向量转换为numpy数组
        vector = [0, 0, -1]  # 示例向量
        v = np.array(vector)
        v_rotated = 0.45*tfq.rotate_vector(v, q)
        
        # 复制位置信息
        forward_msg.pose.position.x = msg.point.x + v_rotated[0]
        forward_msg.pose.position.y = msg.point.y + v_rotated[1]
        forward_msg.pose.position.z = msg.point.z + v_rotated[2]
        
        # 复制姿态信息
        forward_msg.pose.orientation.x = quaternion.x
        forward_msg.pose.orientation.y = quaternion.y
        forward_msg.pose.orientation.z = quaternion.z
        forward_msg.pose.orientation.w = quaternion.w
        
        # 发布到目标话题
        self.publisher.publish(forward_msg)
        
        # 记录发布信息（可选）
        self.get_logger().debug(f'已转发消息到 "/goal_pose1"')
        
        param = Parameter('target_id', Parameter.Type.INTEGER, -10086)
        self.set_parameters([param])


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