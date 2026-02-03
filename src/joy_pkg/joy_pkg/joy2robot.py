import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import Bool, Empty
from my_robot_interfaces.msg import TransJoy
import time

class Joy2Robot(Node):
    def __init__(self):
        super().__init__('joy2robot')
        self.subscription = self.create_subscription(
            TransJoy,
            '/joy_trans',
            self.joy_callback,
            10)
        self.publisher_1 = self.create_publisher(
            TwistStamped,
            'servo_node1/delta_twist_cmds',
            10)
        self.publisher_2 = self.create_publisher(
            TwistStamped,
            'servo_node2/delta_twist_cmds',
            10)
        
        # Motor Publishers
        self.pub_motor_enable = self.create_publisher(Bool, '/motor/enable', 10)
        self.pub_motor_jog = self.create_publisher(Vector3, '/motor/jog_cmd', 10)
        self.pub_motor_stop = self.create_publisher(Empty, '/motor/stop', 10)
        
        # Gripper Publisher
        self.pub_gripper = self.create_publisher(Vector3, '/gripper/cmd_percent', 10)
        
        self.current_robot = 1
        self.last_button_y_state = 0
        self.motor_running = False
        self.motor_speed = 100.0
        self.gripper_val = 100.0
        self.last_button_x_state = 0
        self.last_button_tl_state = 0
        self.last_button_tr_state = 0
        
        # Debounce
        self.last_time_y = 0.0
        self.last_time_x = 0.0
        self.last_time_tl = 0.0
        self.last_time_tr = 0.0
        self.last_time_lx = 0.0
        self.last_time_ab = 0.0
        self.z_angular_val = 0.0
        self.debounce_duration = 0.2

    def joy_callback(self, msg):
        # Extract values from TransJoy message
        left_stick_x = 0.0
        left_stick_y = 0.0
        right_stick_x = 0.0
        right_stick_y = 0.0
        top_stick_l = 0.0
        top_stick_r = 0.0

        button_y_state = 0
        button_x_state = 0
        button_tl_state = 0
        button_tr_state = 0
        button_lx_state = 0
        button_rx_state = 0
        button_a_state = 0
        button_b_state = 0

        for button in msg.buttons:
            if button.name == 'button_y':
                button_y_state = button.data
            elif button.name == 'button_x':
                button_x_state = button.data
            elif button.name == 'button_tl':
                button_tl_state = button.data
            elif button.name == 'button_tr':
                button_tr_state = button.data
            elif button.name == 'button_lx':
                button_lx_state = button.data
            elif button.name == 'button_rx':
                button_rx_state = button.data
            elif button.name == 'button_a':
                button_a_state = button.data
            elif button.name == 'button_b':
                button_b_state = button.data

        # Robot Switch (Button Y)
        current_time = time.time()
        if button_y_state == 1 and self.last_button_y_state == 0 and (current_time - self.last_time_y) > self.debounce_duration:
            if self.current_robot == 1:
                self.current_robot = 2
            else:
                self.current_robot = 1
            self.last_time_y = current_time
        self.last_button_y_state = button_y_state

        # Motor Control (Button X: Start/Stop)
        if button_x_state == 1 and self.last_button_x_state == 0 and (current_time - self.last_time_x) > self.debounce_duration:
            self.motor_running = not self.motor_running
            if self.motor_running:
                # Enable and Start
                self.get_logger().info("Motor STARTED")
                self.pub_motor_enable.publish(Bool(data=True))
                
                jog_msg = Vector3()
                jog_msg.x = 1.0        # Direction
                jog_msg.y = self.motor_speed
                jog_msg.z = 0.0        # Continuous
                self.pub_motor_jog.publish(jog_msg)
            else:
                # Stop and Disable
                self.get_logger().info("Motor STOPPED")
                self.pub_motor_stop.publish(Empty())
                self.pub_motor_enable.publish(Bool(data=False))
                self.motor_speed = 200.0
            self.last_time_x = current_time
        self.last_button_x_state = button_x_state

        # Motor Speed (Button TL/TR)
        speed_changed = False
        if button_tl_state == 1 and self.last_button_tl_state == 0 and (current_time - self.last_time_tl) > self.debounce_duration:
            self.motor_speed += 30.0
            speed_changed = True
            self.last_time_tl = current_time
        elif button_tr_state == 1 and self.last_button_tr_state == 0 and (current_time - self.last_time_tr) > self.debounce_duration:
            self.motor_speed -= 30.0
            if self.motor_speed < 0.0:
                self.motor_speed = 0.0
            speed_changed = True
            self.last_time_tr = current_time
        
        if speed_changed and self.motor_running:
            jog_msg = Vector3()
            jog_msg.x = 1.0
            jog_msg.y = self.motor_speed
            jog_msg.z = 0.0
            self.pub_motor_jog.publish(jog_msg)
            
        self.last_button_tl_state = button_tl_state
        self.last_button_tr_state = button_tr_state

        # Gripper Control
        if (current_time - self.last_time_lx) > self.debounce_duration:
            gripper_changed = False
            if button_lx_state == 1:
                self.gripper_val += 3.0
                gripper_changed = True
            elif button_lx_state == -1:
                self.gripper_val -= 3.0
                gripper_changed = True
            
            if button_rx_state == 1:
                self.gripper_val = 100.0
                gripper_changed = True
            elif button_rx_state == -1:
                self.gripper_val = 65.0
                gripper_changed = True
            
            if gripper_changed:
                if self.gripper_val > 100.0:
                    self.gripper_val = 100.0
                elif self.gripper_val < 0.0:
                    self.gripper_val = 0.0

                gripper_msg = Vector3()
                gripper_msg.x = self.gripper_val
                gripper_msg.y = 100.0  # Speed percent
                gripper_msg.z = 1.0   # Don't wait
                self.pub_gripper.publish(gripper_msg)
                self.get_logger().info(f"Gripper set to {self.gripper_val}%")
                self.last_time_lx = current_time

        for axis in msg.axes:
            if axis.name == 'left_stick_x':
                left_stick_x = axis.data
            elif axis.name == 'left_stick_y':
                left_stick_y = axis.data
            elif axis.name == 'right_stick_x':
                right_stick_x = axis.data
            elif axis.name == 'right_stick_y':
                right_stick_y = axis.data
            elif axis.name == 'top_stick_l':
                top_stick_l = axis.data
            elif axis.name == 'top_stick_r':
                top_stick_r = axis.data

        # Create TwistStamped message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.current_robot == 1:
            twist_msg.header.frame_id = 'robot1_wrist3_link'
        else:
            twist_msg.header.frame_id = 'robot2_wrist3_link'

        # Map to geometry_msg including linear and angular
        # Mapping: 
        # linear.x <- left_stick_x
        # linear.y <- left_stick_y
        # linear.z <- top_stick_l
        # angular.z <- top_stick_r
        
        twist_msg.twist.linear.x = -left_stick_x
        twist_msg.twist.linear.y = -left_stick_y
        twist_msg.twist.linear.z = top_stick_l - top_stick_r
        twist_msg.twist.angular.x = right_stick_y
        twist_msg.twist.angular.y = -right_stick_x
        
        # Debounce for Angler Z (Button A/B)
        if (current_time - self.last_time_ab) > self.debounce_duration:
            if button_a_state == 1:
                self.z_angular_val = 1.0
            elif button_b_state == 1:
                self.z_angular_val = -1.0
            else:
                self.z_angular_val = 0.0
            self.last_time_ab = current_time
            
        twist_msg.twist.angular.z = self.z_angular_val
  

        if self.current_robot == 1:
            self.publisher_1.publish(twist_msg)
        else:
            self.publisher_2.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    joy2robot = Joy2Robot()
    rclpy.spin(joy2robot)
    joy2robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
