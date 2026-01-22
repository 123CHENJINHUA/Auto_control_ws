import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import TransJoy  # Import the custom messages

class JoySub(Node):
    def __init__(self):
        super().__init__('joy_sub')
        self.subscription = self.create_subscription(TransJoy, '/joy_trans', self.joy_callback, 10)


    def joy_callback(self, msg):
        # Log the axes and buttons
        for axis in msg.axes:
            print(f'{axis.name}: {axis.data}')
        for button in msg.buttons:
            print(f'{button.name}: {button.data}')

def main(args=None):
    rclpy.init(args=args)
    joy_sub = JoySub()
    rclpy.spin(joy_sub)
    joy_sub.destroy_node()
    rclpy.shutdown()