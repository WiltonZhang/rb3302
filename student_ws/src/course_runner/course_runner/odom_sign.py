#!/usr/bin/env python3
"""Give the Wave Rover's wheel speed its direction back: /odom/raw -> /odom/raw_signed.

    ros2 run course_runner odom_sign
    ros2 launch planning_rpp planning.launch.py raw_odom_topic:=/odom/raw_signed

The driver works out /odom/raw linear.x from the board's wheel distance, which
only counts up. Reversing therefore reads as driving forward, and so does
turning on the spot (one side's wheels run backwards). The planning launch's
EKF integrates that speed along the IMU heading, so every turn on the spot
pushed the odometry round a circle: RViz showed the robot sliding across the
map while it was really turning in place, and AMCL followed it.

This node republishes /odom/raw with
    linear.x = sign of the last /cmd_vel linear.x * measured speed
and 0 when the command has no forward part (stopped, or turning on the spot).
The speed is also capped at max_ratio x the commanded speed, so the outer
wheels of a tight turn do not add forward motion either. Everything else in
the message is passed through unchanged (the EKF only uses linear.x from it).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomSign(Node):

    def __init__(self):
        super().__init__('odom_sign')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('raw_topic', '/odom/raw')
        self.declare_parameter('signed_topic', '/odom/raw_signed')
        # |speed| <= max_ratio * |commanded speed|; 0 turns the cap off
        self.declare_parameter('max_ratio', 1.5)
        # commands below this (m/s) count as "no forward part"
        self.declare_parameter('min_command', 0.005)
        # forget the last command after this long; the driver itself stops the
        # motors 0.8 s after the last command (cmd_vel_timeout)
        self.declare_parameter('command_timeout', 1.0)

        param = lambda name: self.get_parameter(name).value
        self.max_ratio = float(param('max_ratio'))
        self.min_command = float(param('min_command'))
        self.command_timeout = float(param('command_timeout'))
        self.command = 0.0
        self.command_time = None

        self.create_subscription(Twist, str(param('cmd_topic')), self._cmd_callback, 10)
        self.create_subscription(Odometry, str(param('raw_topic')), self._odom_callback, 10)
        self.pub = self.create_publisher(Odometry, str(param('signed_topic')), 10)
        self.get_logger().info(
            'signing %s with %s -> %s (max_ratio %.2f)'
            % (param('raw_topic'), param('cmd_topic'), param('signed_topic'),
               self.max_ratio))

    def _cmd_callback(self, msg):
        self.command = msg.linear.x
        self.command_time = self.get_clock().now()

    def _odom_callback(self, msg):
        command = self.command
        if self.command_time is None or (
                self.get_clock().now() - self.command_time).nanoseconds * 1e-9 \
                > self.command_timeout:
            command = 0.0
        speed = abs(msg.twist.twist.linear.x)
        if abs(command) < self.min_command:
            speed = 0.0
        else:
            if self.max_ratio > 0.0:
                speed = min(speed, self.max_ratio * abs(command))
            speed = math.copysign(speed, command)
        msg.twist.twist.linear.x = speed
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomSign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
