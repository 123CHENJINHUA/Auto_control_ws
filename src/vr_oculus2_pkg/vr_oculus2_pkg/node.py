import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class VRNode(Node):
    def __init__(self, node_name: str = 'VR') -> None:
        super().__init__(node_name)
        # Parameters
        self.declare_parameter('publish_topic', '/vr/actions')
        self.declare_parameter('left_topic', '/vr/left_action')
        self.declare_parameter('right_topic', '/vr/right_action')
        self.declare_parameter('publish_rate_hz', 100.0)

        topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        left_topic = self.get_parameter('left_topic').get_parameter_value().string_value
        right_topic = self.get_parameter('right_topic').get_parameter_value().string_value
        rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.pub = self.create_publisher(String, topic, 10)
        self.pub_left = self.create_publisher(String, left_topic, 10)
        self.pub_right = self.create_publisher(String, right_topic, 10)
        self.gripper_pub = self.create_publisher(Vector3, '/gripper/cmd_percent', 10)

        # Import controller from existing package to avoid duplication
        try:
            from vr_oculus2_pkg.vr_oculus2 import OculusQuest3Controller, VRConfig  # type: ignore
        except Exception as e:
            self.get_logger().error(
                f'Failed to import Oculus controller from vr_oculus2_pkg: {e}'
            )
            raise

        self.ctrl = OculusQuest3Controller(VRConfig())
        self.ctrl.connect()

        period = 1.0 / max(rate_hz, 1e-6)
        self.timer = self.create_timer(period, self._on_timer)

    def _on_timer(self) -> None:
        try:
            action = self.ctrl.get_action()
            
            # Publish combined action
            msg = String()
            msg.data = json.dumps(action, ensure_ascii=False)
            self.pub.publish(msg)

            # Separate and publish left/right actions
            left_action = {k: v for k, v in action.items() if k.startswith('left') or k == 'buttons'}
            right_action = {k: v for k, v in action.items() if k.startswith('right') or k == 'buttons'}

            msg_left = String()
            msg_left.data = json.dumps(left_action, ensure_ascii=False)
            self.pub_left.publish(msg_left)

            msg_right = String()
            msg_right.data = json.dumps(right_action, ensure_ascii=False)
            self.pub_right.publish(msg_right)

            # Publish rightTrig to gripper/cmd_percent
            buttons = action.get('buttons', {})
            if 'rightTrig' in buttons:
                val = buttons['rightTrig']
                if isinstance(val, (tuple, list)):
                    val = val[0]
                try:
                    grip_val = float(1-val)
                except (ValueError, TypeError):
                    grip_val = 1.0

                gripper_msg = Vector3()
                gripper_msg.x = grip_val * 100.0  # position_percent (0~100)
                gripper_msg.y = 50.0              # speed_percent
                gripper_msg.z = 0.0               # wait_for_completion: 0 (no wait)
                self.gripper_pub.publish(gripper_msg)

        except Exception as e:
            self.get_logger().warn(f'Error while reading/publishing VR action: {e}')

    def destroy_node(self) -> bool:
        try:
            if self.ctrl is not None:
                self.ctrl.disconnect()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VRNode('VR')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
