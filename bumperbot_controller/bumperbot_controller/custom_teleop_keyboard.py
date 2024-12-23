#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty

# Key bindings for movement
MOVE_BINDINGS = {
    'w': (1, 0),  # Forward
    's': (-1, 0),  # Backward
    'a': (0, 1),  # Turn left
    'd': (0, -1),  # Turn right
}

class CustomTeleopKeyboard(Node):
    def __init__(self):
        super().__init__('custom_teleop_keyboard')

        # Declare parameters with defaults
        self.declare_parameter('topic_name', '/bumperbot_controller/cmd_vel')
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 1.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('autorepeat_rate', 20.0)
        self.declare_parameter('sticky_keys', False)

        # Get parameter values
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.turn = self.get_parameter('turn').get_parameter_value().double_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.autorepeat_rate = self.get_parameter('autorepeat_rate').get_parameter_value().double_value
        self.sticky_keys = self.get_parameter('sticky_keys').get_parameter_value().bool_value

        # Create publisher using the topic name parameter
        self.publisher = self.create_publisher(TwistStamped, self.topic_name, 10)

        self.get_logger().info('Use WASD keys to control the robot. Press CTRL+C to quit.')

    def get_key(self):
        """Get a single keypress."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin.fileno()))
        return key

    def run(self):
        try:
            last_key = None
            while rclpy.ok():
                key = self.get_key()
                if key in MOVE_BINDINGS:
                    linear, angular = MOVE_BINDINGS[key]
                    twist_stamped = TwistStamped()
                    twist_stamped.header.stamp = self.get_clock().now().to_msg()
                    twist_stamped.header.frame_id = "base_link"
                    twist_stamped.twist.linear.x = linear * self.speed
                    twist_stamped.twist.angular.z = angular * self.turn
                    self.publisher.publish(twist_stamped)
                    last_key = key
                elif key == '\x03':  # CTRL+C
                    break
                elif self.sticky_keys and last_key:
                    # Re-publish last key's command if sticky keys are enabled
                    linear, angular = MOVE_BINDINGS[last_key]
                    twist_stamped = TwistStamped()
                    twist_stamped.header.stamp = self.get_clock().now().to_msg()
                    twist_stamped.header.frame_id = "base_link"
                    twist_stamped.twist.linear.x = linear * self.speed
                    twist_stamped.twist.angular.z = angular * self.turn
                    self.publisher.publish(twist_stamped)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop the robot before exiting
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = "base_link"
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
            self.publisher.publish(twist_stamped)
            self.get_logger().info('Exiting.')

def main(args=None):
    rclpy.init(args=args)
    node = CustomTeleopKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
