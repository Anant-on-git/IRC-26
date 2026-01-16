#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToCmdVel(Node):

    def __init__(self):
        super().__init__('joy_to_cmdvel')

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Parameters (change these, not your soul)
        self.linear_axis = 1     # Left stick vertical
        self.angular_axis = 3    # Right stick horizontal
        self.enable_button = 5   # RB button

        self.linear_scale = 1.0
        self.angular_scale = 1.0

        self.get_logger().info("Joy → cmd_vel node started")

    def joy_callback(self, msg: Joy):
        twist = Twist()

        # Deadman switch
        if msg.buttons[self.enable_button] == 1:
            twist.linear.x = msg.axes[self.linear_axis] * self.linear_scale
            twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
