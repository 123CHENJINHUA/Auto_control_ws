#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import TransformStamped
import sys
from scipy.spatial.transform import Rotation as R
import copy

def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler

class Robot2TFPublisher(Node):
    def __init__(self):
        super().__init__('robot2_tf_publisher')
        
        # 创建TF缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建发布器
        self.publisher1 = self.create_publisher(
            TransformStamped,
            '/robot1_transform',
            10
        )

        # 创建发布器
        self.publisher11 = self.create_publisher(
            TransformStamped,
            '/robot1_transform_rpy',
            10
        )

        self.publisher2 = self.create_publisher(
            TransformStamped,
            '/robot2_transform',
            10
        )

        self.publisher22 = self.create_publisher(
            TransformStamped,
            '/robot2_transform_rpy',
            10
        )
        
        # 创建100Hz的定时器
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info('Robot2 TF Publisher 节点已启动')
    
    def timer_callback(self):
        try:
            # 获取从robot2_base_line到robot2_wrist3_link的变换
            transform1 = self.tf_buffer.lookup_transform(
                'robot1_base_link',     # 源坐标系
                'tcp1',  # 目标坐标系
                
                rclpy.time.Time()       # 获取最新变换
            )

            q = transform1.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]
            euler = quaternion2euler(quaternion)

            transform11 = copy.deepcopy(transform1)
            transform11.transform.rotation.x = euler[0]
            transform11.transform.rotation.y = euler[1]
            transform11.transform.rotation.z = euler[2]
            transform11.transform.rotation.w = 0.0
            
            # 发布变换
            self.publisher1.publish(transform1)
            self.publisher11.publish(transform11)

            transform2 = self.tf_buffer.lookup_transform(
                'robot2_base_link',     # 源坐标系
                'tcp2',  # 目标坐标系
                
                rclpy.time.Time()       # 获取最新变换
            )
            
            # 发布变换
            
            q2 = transform2.transform.rotation
            quaternion2 = [q2.x, q2.y, q2.z, q2.w]
            euler2 = quaternion2euler(quaternion2)

            transform22 = copy.deepcopy(transform2)
            transform22.transform.rotation.x = euler2[0]
            transform22.transform.rotation.y = euler2[1]
            transform22.transform.rotation.z = euler2[2]
            transform22.transform.rotation.w = 0.0

            self.publisher2.publish(transform2)
            self.publisher22.publish(transform22)

        except TransformException as e:
            self.get_logger().debug(f'无法获取TF变换: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Robot2TFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()