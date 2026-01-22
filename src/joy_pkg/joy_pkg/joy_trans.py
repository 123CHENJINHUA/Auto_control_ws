# filepath: /home/hkust/Facade Robot/control_ws/src/control_pkg_py/control_pkg_py/joy_sub.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from my_robot_interfaces.msg import TransJoy, JoyData, ButtonData  # Import the custom messages

class XboxController(Node):
    def __init__(self):
        super().__init__('xbox_controller')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.transjoy_publisher = self.create_publisher(TransJoy, '/joy_trans', 30)
        self.timer = self.create_timer(1.0/30, self.timer_callback)
        self.subscription  # prevent unused variable warning
        self.axis_names = ['left_stick_x', 'left_stick_y', 'right_stick_x', 'right_stick_y', 'dpad_x', 'dpad_y']
        self.button_names = ['button_a', 'button_b', 'button_x', 'button_y', 'button_lb', 'button_rb']
        self.processed_axes = []
        self.processed_buttons = []

    def joy_callback(self, msg):
        # Process the axes data
        self.processed_axes = []
        self.processed_buttons = []
        #wire link
        self.processed_axes.append(JoyData(name='left_stick_x', data=msg.axes[0]))
        self.processed_axes.append(JoyData(name='left_stick_y', data=msg.axes[1]))
        self.processed_axes.append(JoyData(name='right_stick_x', data=msg.axes[3]))
        self.processed_axes.append(JoyData(name='right_stick_y', data=msg.axes[4]))
        self.processed_axes.append(JoyData(name='top_stick_l', data=(msg.axes[2]-1)/-2.0))
        self.processed_axes.append(JoyData(name='top_stick_r', data=(msg.axes[5]-1)/-2.0))

        self.processed_buttons.append(ButtonData(name='button_a', data=msg.buttons[0]))
        self.processed_buttons.append(ButtonData(name='button_b', data=msg.buttons[1]))
        self.processed_buttons.append(ButtonData(name='button_x', data=msg.buttons[2]))
        self.processed_buttons.append(ButtonData(name='button_y', data=msg.buttons[3]))
        self.processed_buttons.append(ButtonData(name='button_lx', data=int(msg.axes[6])))
        self.processed_buttons.append(ButtonData(name='button_rx', data=int(msg.axes[7])))
        self.processed_buttons.append(ButtonData(name='button_tl', data=msg.buttons[4]))
        self.processed_buttons.append(ButtonData(name='button_tr', data=msg.buttons[5]))

        #bluetooth
        # self.processed_axes.append(JoyData(name='left_stick_x', data=msg.axes[0]))
        # self.processed_axes.append(JoyData(name='left_stick_y', data=msg.axes[1]))
        # self.processed_axes.append(JoyData(name='right_stick_x', data=msg.axes[2]))
        # self.processed_axes.append(JoyData(name='right_stick_y', data=msg.axes[3]))
        # self.processed_axes.append(JoyData(name='top_stick_l', data=(msg.axes[5]-1)/-2.0))
        # self.processed_axes.append(JoyData(name='top_stick_r', data=(msg.axes[4]-1)/-2.0))

        # self.processed_buttons.append(ButtonData(name='button_a', data=msg.buttons[0]))
        # self.processed_buttons.append(ButtonData(name='button_b', data=msg.buttons[1]))
        # self.processed_buttons.append(ButtonData(name='button_x', data=msg.buttons[3]))
        # self.processed_buttons.append(ButtonData(name='button_y', data=msg.buttons[4]))
        # self.processed_buttons.append(ButtonData(name='button_lx', data=int(-msg.axes[6])))
        # self.processed_buttons.append(ButtonData(name='button_ly', data=int(-msg.axes[7])))
        # self.processed_buttons.append(ButtonData(name='button_tl', data=msg.buttons[6]))
        # self.processed_buttons.append(ButtonData(name='button_tr', data=msg.buttons[7]))

    def timer_callback(self):
        # Create the custom message
        transjoy_msg = TransJoy()
        transjoy_msg.header.stamp = self.get_clock().now().to_msg()
        transjoy_msg.axes = self.processed_axes
        transjoy_msg.buttons = self.processed_buttons

        # Publish the message
        self.transjoy_publisher.publish(transjoy_msg)

def main(args=None):
    rclpy.init(args=args)
    xbox_controller = XboxController()
    rclpy.spin(xbox_controller)
    xbox_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()