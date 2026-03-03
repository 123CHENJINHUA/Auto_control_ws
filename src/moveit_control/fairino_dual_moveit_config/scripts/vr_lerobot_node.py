#!/usr/bin/env python3
"""
VR动作数据转换节点
将/vr/left_action和/vr/right_action的JSON数据转换为TwistStamped消息
发布到/servo_node1/delta_twist_cmds和/servo_node2/delta_twist_cmds
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
import json
import numpy as np

class VRActionToTwistNode(Node):
    """VR动作数据转换节点"""
    
    def __init__(self, node_name, vr_topic, servo_topic, prefix):
        """
        初始化节点
        
        Args:
            node_name: 节点名称
            vr_topic: 订阅的VR话题名称
            servo_topic: 发布的servo话题名称  
            prefix: 数据字段前缀 ('left' 或 'right')
        """
        super().__init__(node_name)
        
        self.prefix = prefix
        self.vr_topic = vr_topic
        self.servo_topic = servo_topic
        self.buffer_size = 15 # 平滑缓冲区大小 
        self.buffer = np.zeros((self.buffer_size, 6))  # 用于存储最近5个Twist数据的缓冲区
        self.index = 0  # 缓冲区索引
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            String,
            self.vr_topic,
            self.vr_callback,
            1  # QoS队列深度
        )
        
        # 创建发布者
        self.publisher = self.create_publisher(
            TwistStamped,
            self.servo_topic,
            1
        )
        
        self.get_logger().info(f'{node_name} 初始化完成')
        self.get_logger().info(f'订阅: {vr_topic}')
        self.get_logger().info(f'发布: {servo_topic}')
        
    def vr_callback(self, msg):
        """VR话题回调函数"""
        try:
            # 解析JSON数据
            data = json.loads(msg.data)
            # if self.prefix == 'right':
            #     self.get_logger().info(f'{self.prefix} delta_y: {-data.get(f"{self.prefix}_delta_y", 0.0)}')
            
            # 创建TwistStamped消息
            twist_msg = TwistStamped()
            
            # 设置时间戳
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            if self.prefix == 'left':
                twist_msg.header.frame_id = 'robot1_base_link'  # 可根据需要修改
                # 根据前缀提取数据并赋值
                # 线性速度 (delta_x, deself.buffer_sizelta_y, delta_z)
                dt1, dt2  = 0.001, 0.001

                tmp = self.index % self.buffer_size
                self.buffer[tmp, 0] = -data.get(f'{self.prefix}_delta_z', 0.0)/dt1
                self.buffer[tmp, 1] = -data.get(f'{self.prefix}_delta_y', 0.0)/dt1
                self.buffer[tmp, 2] = -data.get(f'{self.prefix}_delta_x', 0.0)/dt1
                self.buffer[tmp, 3] = -data.get(f'{self.prefix}_delta_yaw', 0.0)/dt2
                self.buffer[tmp, 4] = -data.get(f'{self.prefix}_delta_pitch', 0.0)/dt2
                self.buffer[tmp, 5] = -data.get(f'{self.prefix}_delta_roll', 0.0)/dt2
                self.index += 1

                twist_msg.twist.linear.x = self.buffer[:,0].mean()
                twist_msg.twist.linear.y = self.buffer[:,1].mean()
                twist_msg.twist.linear.z = self.buffer[:,2].mean()

                twist_msg.twist.angular.x = self.buffer[:,3].mean()
                twist_msg.twist.angular.y = self.buffer[:,4].mean()
                twist_msg.twist.angular.z = self.buffer[:,5].mean()
            else:
                twist_msg.header.frame_id = 'robot2_base_link'  # 可根据需要修改
                # 根据前缀提取数据并赋值
                # 线性速度 (delta_x, delta_y, delta_z)
                dt1, dt2 = 0.001, 0.001
                
                tmp = self.index % self.buffer_size
                self.buffer[tmp, 0] = -data.get(f'{self.prefix}_delta_y', 0.0)/dt1
                self.buffer[tmp, 1] = -data.get(f'{self.prefix}_delta_z', 0.0)/dt1
                self.buffer[tmp, 2] = data.get(f'{self.prefix}_delta_x', 0.0)/dt1
                self.buffer[tmp, 3] = -data.get(f'{self.prefix}_delta_pitch', 0.0)/dt2
                self.buffer[tmp, 4] = -data.get(f'{self.prefix}_delta_yaw', 0.0)/dt2
                self.buffer[tmp, 5] = data.get(f'{self.prefix}_delta_roll', 0.0)/dt2
                self.index += 1

                twist_msg.twist.linear.x = self.buffer[:,0].mean()
                twist_msg.twist.linear.y = self.buffer[:,1].mean()
                twist_msg.twist.linear.z = self.buffer[:,2].mean()
                
                twist_msg.twist.angular.x = self.buffer[:,3].mean()
                twist_msg.twist.angular.y = self.buffer[:,4].mean()
                twist_msg.twist.angular.z = self.buffer[:,5].mean()
            
            # # 发布消息
            # if twist_msg.twist.linear.x == 0.0 and \
            #     twist_msg.twist.linear.y == 0.0 and \
            #     twist_msg.twist.linear.z == 0.0 and \
            #     twist_msg.twist.angular.x == 0.0 and \
            #     twist_msg.twist.angular.y == 0.0 and \
            #     twist_msg.twist.angular.z == 0.0:
            #     return
            self.publisher.publish(twist_msg)
            
            # 可选：打印调试信息
            # self.get_logger().debug(f'发布到 {self.servo_topic}: {twist_msg.twist}')
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析错误: {e}')
        except KeyError as e:
            self.get_logger().error(f'缺少字段: {e}')
        except Exception as e:
            self.get_logger().error(f'处理消息时出错: {e}')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        # 创建左侧节点
        left_node = VRActionToTwistNode(
            node_name='vr_left_to_twist_node',
            vr_topic='/vr/left_action',
            servo_topic='/lerobot1/delta_twist_cmds',
            prefix='left'
        )
        
        # 创建右侧节点  
        right_node = VRActionToTwistNode(
            node_name='vr_right_to_twist_node',
            vr_topic='/vr/right_action',
            servo_topic='/lerobot2/delta_twist_cmds',
            prefix='right'
        )
        
        # 创建执行器
        executor = rclpy.executors.MultiThreadedExecutor()
        
        # 添加节点到执行器
        executor.add_node(left_node)
        executor.add_node(right_node)
        
        # 运行执行器
        left_node.get_logger().info('开始处理VR动作数据...')
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        left_node.destroy_node()
        right_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()