#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 硬件控制节点

- 通过话题订阅控制 motor 与 gripper
- 周期发布状态（motor: position/speed/status；gripper: position/current）

话题约定（可按需改名）：
订阅:
- /motor/enable                 std_msgs/Bool
- /motor/position_cmd           geometry_msgs/Vector3   (x=target_position, y=speed(rpm), z=is_absolute(1/0))
- /motor/jog_cmd                geometry_msgs/Vector3   (x=direction(1 forward / -1 reverse), y=speed(rpm), z=duration(s), <=0表示持续)
- /motor/stop                   std_msgs/Empty
- /motor/home                   std_msgs/Int32          (home_mode)

- /gripper/cmd_percent          geometry_msgs/Vector3   (x=position_percent, y=speed_percent, z=wait_for_completion(1/0))
- /gripper/stop_monitoring      std_msgs/Empty

发布:
- /motor/state                  sensor_msgs/JointState  (position[0]=pulse, velocity[0]=rpm, effort[0]=fault(0/1))
- /motor/status_text            std_msgs/String         (人类可读状态)
- /gripper/state                sensor_msgs/JointState  (position[0]=raw_position, effort[0]=current(A))

说明：
- 为避免自定义msg，本节点使用 std_msgs / geometry_msgs / sensor_msgs 进行快速集成。
- 串口端口参数通过ROS参数配置。
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty, Int32, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

from .motor_ctl import CL57RDriver
# from .gripper_ctl import GripperController
from .gripper_ctl2 import BusServo


class HardwareNode(Node):
    def __init__(self) -> None:
        super().__init__('hardware_node')

        # -------- Parameters --------
        self.declare_parameter('motor.port', '/dev/ttyCH341USB0')
        self.declare_parameter('motor.baudrate', 38400)
        self.declare_parameter('motor.slave_id', 1)
        self.declare_parameter('motor.timeout', 0.1)

        self.declare_parameter('gripper.port', '/dev/ttyCH341USB1')
        self.declare_parameter('gripper.baudrate', 1000000)
        self.declare_parameter('gripper.timeout', 1.0)

        self.declare_parameter('state_pub_period', 0.1)

        motor_port = self.get_parameter('motor.port').get_parameter_value().string_value
        motor_baudrate = int(self.get_parameter('motor.baudrate').get_parameter_value().integer_value)
        motor_slave_id = int(self.get_parameter('motor.slave_id').get_parameter_value().integer_value)
        motor_timeout = float(self.get_parameter('motor.timeout').get_parameter_value().double_value)

        gripper_port = self.get_parameter('gripper.port').get_parameter_value().string_value
        gripper_baudrate = int(self.get_parameter('gripper.baudrate').get_parameter_value().integer_value)
        gripper_timeout = float(self.get_parameter('gripper.timeout').get_parameter_value().double_value)

        self.state_pub_period = float(self.get_parameter('state_pub_period').get_parameter_value().double_value)

        # -------- Hardware drivers --------
        self._lock = threading.Lock()

        self.motor: Optional[CL57RDriver] = None
        self.gripper: Optional[BusServo] = None

        # Motor connect
        try:
            self.motor = CL57RDriver(
                port=motor_port,
                baudrate=motor_baudrate,
                slave_id=motor_slave_id,
                timeout=motor_timeout,
            )
            self.get_logger().info(f"Motor connected: port={motor_port} baudrate={motor_baudrate} id={motor_slave_id}")
        except Exception as e:
            self.get_logger().error(f"Motor connect failed: {e}")

        # Gripper connect
        try:
            self.gripper = BusServo(
                port=gripper_port,
                baudrate=gripper_baudrate,
                timeout=gripper_timeout,
            )
            if not self.gripper.ping(10)==0:
                self.get_logger().error("Gripper connect failed")
                self.gripper = None
            else:
                self.get_logger().info(f"Gripper connected: port={gripper_port} baudrate={gripper_baudrate}")
        except Exception as e:
            self.get_logger().error(f"Gripper connect failed: {e}")
            self.gripper = None

        # -------- Publishers --------
        self.pub_motor_state = self.create_publisher(JointState, 'motor/state', 10)
        self.pub_motor_status_text = self.create_publisher(String, 'motor/status_text', 10)
        self.pub_gripper_state = self.create_publisher(JointState, 'gripper/state', 10)

        # -------- Subscribers (Motor) --------
        self.create_subscription(Bool, 'motor/enable', self._cb_motor_enable, 10)
        self.create_subscription(Vector3, 'motor/position_cmd', self._cb_motor_position_cmd, 10)
        self.create_subscription(Vector3, 'motor/jog_cmd', self._cb_motor_jog_cmd, 10)
        self.create_subscription(Empty, 'motor/stop', self._cb_motor_stop, 10)
        self.create_subscription(Int32, 'motor/home', self._cb_motor_home, 10)

        # -------- Subscribers (Gripper) --------
        self.create_subscription(Vector3, 'gripper/cmd_percent', self._cb_gripper_cmd_percent, 10)
        self.create_subscription(Empty, 'gripper/stop_monitoring', self._cb_gripper_stop_monitoring, 10)

        # -------- Timers --------
        self._timer = self.create_timer(self.state_pub_period, self._publish_state)

    # ---------------- Motor callbacks ----------------
    def _cb_motor_enable(self, msg: Bool) -> None:
        if not self.motor:
            self.get_logger().error('motor not initialized')
            return
        with self._lock:
            ok = self.motor.enable_motor(bool(msg.data))
        if not ok:
            self.get_logger().error('motor enable command failed')

    def _cb_motor_position_cmd(self, msg: Vector3) -> None:
        """x=target_position(pulse), y=speed(rpm), z=is_absolute(1/0)"""
        if not self.motor:
            self.get_logger().error('motor not initialized')
            return

        target_position = int(msg.x)
        speed = int(msg.y)
        is_absolute = bool(int(msg.z) != 0)

        with self._lock:
            ok = self.motor.set_position_mode(
                target_position=target_position,
                speed=speed,
                is_absolute=is_absolute,
            )
        if not ok:
            self.get_logger().error('motor position command failed')

    def _cb_motor_jog_cmd(self, msg: Vector3) -> None:
        """x=direction(1/-1), y=speed(rpm), z=duration(s)"""
        if not self.motor:
            self.get_logger().error('motor not initialized')
            return

        direction = 1 if msg.x >= 0 else -1
        speed = int(abs(msg.y))
        duration = float(msg.z)

        def _run_jog():
            with self._lock:
                ok = self.motor.jog_forward(speed=speed) if direction > 0 else self.motor.jog_reverse(speed=speed)
            if not ok:
                self.get_logger().error('motor jog command failed')
                return
            if duration > 0:
                time.sleep(duration)
                with self._lock:
                    self.motor.stop()

        threading.Thread(target=_run_jog, daemon=True).start()

    def _cb_motor_stop(self, _: Empty) -> None:
        if not self.motor:
            self.get_logger().error('motor not initialized')
            return
        with self._lock:
            ok = self.motor.stop()
        if not ok:
            self.get_logger().error('motor stop failed')

    def _cb_motor_home(self, msg: Int32) -> None:
        if not self.motor:
            self.get_logger().error('motor not initialized')
            return
        home_mode = int(msg.data)
        with self._lock:
            ok = self.motor.home(home_mode=home_mode)
        if not ok:
            self.get_logger().error('motor home failed')

    # ---------------- Gripper callbacks ----------------
    def _cb_gripper_cmd_percent(self, msg: Vector3) -> None:
        """x=position_percent, y=speed_percent, z=wait_for_completion(1/0)"""
        if not self.gripper:
            self.get_logger().error('gripper not initialized')
            return

        position_percent = float(msg.x)
        speed_percent = float(msg.y)
        wait_for_completion = bool(int(msg.z) != 0)

        def _run_gripper():
            with self._lock:
                ok = self.gripper.set_gripper_position(
                    servo_id= 10,
                    gripper_type="100mm",
                    position_mm=position_percent,
                    speed=1000,
                    # wait_for_completion=wait_for_completion,
                )
            # if not ok:
            #     self.get_logger().error('gripper command failed')

        threading.Thread(target=_run_gripper, daemon=True).start()

    def _cb_gripper_stop_monitoring(self, _: Empty) -> None:
        if not self.gripper:
            self.get_logger().error('gripper not initialized')
            return
        with self._lock:
            self.gripper.stop_monitoring()

    # ---------------- State publisher ----------------
    def _publish_state(self) -> None:
        now = self.get_clock().now().to_msg()

        # Motor
        if self.motor:
            with self._lock:
                pos = self.motor.get_position()
                spd = self.motor.get_speed()
                st = self.motor.get_status()

            js = JointState()
            js.header.stamp = now
            js.name = ['motor']
            js.position = [float(pos)]
            js.velocity = [float(spd)]
            js.effort = [1.0 if st.get('故障') else 0.0]
            self.pub_motor_state.publish(js)

            txt = String()
            txt.data = str(st)
            self.pub_motor_status_text.publish(txt)

        # # Gripper
        # if self.gripper:
        #     with self._lock:
        #         gpos = self.gripper.read_position()
        #         gcur = self.gripper.read_current()

        #     js = JointState()
        #     js.header.stamp = now
        #     js.name = ['gripper']
        #     js.position = [float(gpos) if gpos is not None else float('nan')]
        #     js.velocity = []
        #     js.effort = [float(gcur) if gcur is not None else float('nan')]
        #     self.pub_gripper_state.publish(js)

    def destroy_node(self) -> bool:
        # clean-up serial connections
        # try:
        #     if self.gripper:
        #         self.gripper.stop_monitoring()
        #         self.gripper.disconnect()
        # except Exception:
        #     pass

        try:
            if self.motor:
                self.motor.close()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
