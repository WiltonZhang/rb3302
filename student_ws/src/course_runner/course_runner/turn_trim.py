#!/usr/bin/env python3
"""Make the Wave Rover turn at the rate RPP asks for: cmd_vel_trim_in -> /cmd_vel.

    ros2 run course_runner turn_trim      (started by planning.launch.py)

The skid-steer turns at only about 1/3 to 1/2 of the commanded rate (tyre
scrub), so Regulated Pure Pursuit asks for a curve and gets a wider one, then
corrects, and the robot weaves into squares. Nav2 has no turn-rate feedback
(the velocity smoother's CLOSED_LOOP only limits acceleration), so this node
sits between the velocity smoother and the driver and trims angular.z with
the turn rate the EKF measures from the IMU (/odom twist.angular.z):

    command = requested + clamp(gain * error + integral, max_trim * |requested|)
    error = requested - measured;   integral += gain / integral_time * error * dt

i.e. unity feed-forward plus a bounded PI correction (a friend's working
SUCCESS_RPP run used gain 0.25, integral_time 1 s, max_trim 0.5). linear.x
passes through unchanged. The integral restarts when the requested turn
stops or reverses, or when /odom is older than odom_timeout; then, and with
enabled:=false, the command passes through untouched. The integral does not
grow further while the correction is at its bound (anti-windup).

Only the smoother's output goes through here: behavior_server Spin and
course_runner's fast dead-end exit close the loop on heading themselves and
still publish straight to /cmd_vel.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def clamp(value, limit):
    return max(-limit, min(limit, value))


class RateTrim:
    """Bounded PI trim of a requested turn rate; no ROS, so it can be tested."""

    def __init__(self, gain=0.25, integral_time=1.0, max_trim=0.5, command_limit=3.0):
        self.gain = gain
        self.integral_time = integral_time
        self.max_trim = max_trim
        self.command_limit = command_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.direction = 0.0

    def update(self, requested, measured, dt):
        if requested == 0.0 or measured is None:
            self.reset()
            return requested
        requested = clamp(requested, self.command_limit)
        direction = math.copysign(1.0, requested)
        if direction != self.direction:
            self.integral = 0.0
        self.direction = direction
        error = requested - measured
        limit = self.max_trim * abs(requested)
        self.integral = clamp(self.integral, limit)
        candidate = clamp(self.integral + self.gain / self.integral_time * error * dt, limit)
        correction = self.gain * error + candidate
        # conditional integration: do not wind up into the bound
        if (abs(correction) <= limit and abs(requested + correction) <= self.command_limit) \
                or correction * error < 0.0:
            self.integral = candidate
        return clamp(requested + clamp(self.gain * error + self.integral, limit),
                     self.command_limit)


class TurnTrim(Node):

    def __init__(self):
        super().__init__('turn_trim')
        self.declare_parameter('enabled', True)
        self.declare_parameter('input_topic', 'cmd_vel_trim_in')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('gain', 0.25)           # trim per rad/s of rate error
        self.declare_parameter('integral_time', 1.0)   # s for the integral to add as much again
        self.declare_parameter('max_trim', 0.5)        # trim at most this share of the request
        self.declare_parameter('command_limit', 3.0)   # driver clamp (rad/s)
        self.declare_parameter('odom_timeout', 0.5)    # older /odom counts as no measurement (s)
        self.declare_parameter('log_period', 1.0)      # TURN RATE log line every this (s); 0 = off

        param = lambda name: self.get_parameter(name).value
        self.enabled = bool(param('enabled'))
        self.odom_timeout = float(param('odom_timeout'))
        self.log_period = float(param('log_period'))
        self.trim = RateTrim(float(param('gain')), float(param('integral_time')),
                             float(param('max_trim')), float(param('command_limit')))
        self.rate = None
        self.rate_time = None
        self.last_time = None
        self.next_log = 0.0

        self.pub = self.create_publisher(Twist, str(param('output_topic')), 10)
        self.create_subscription(Twist, str(param('input_topic')), self._cmd_callback, 10)
        self.create_subscription(Odometry, str(param('odom_topic')), self._odom_callback, 10)
        self.get_logger().info(
            '%s -> %s, turn rate trim %s (gain %.2f, integral_time %.1f s, max_trim %.2f)'
            % (param('input_topic'), param('output_topic'),
               'on' if self.enabled else 'OFF (pass-through)',
               self.trim.gain, self.trim.integral_time, self.trim.max_trim))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_callback(self, msg):
        rate = msg.twist.twist.angular.z
        if math.isfinite(rate):
            self.rate = rate
            self.rate_time = self.now()

    def _cmd_callback(self, msg):
        now = self.now()
        dt = 0.0 if self.last_time is None else min(0.2, max(0.0, now - self.last_time))
        self.last_time = now
        measured = self.rate
        if self.rate_time is None or now - self.rate_time > self.odom_timeout:
            measured = None
        requested = msg.angular.z
        out = Twist()
        out.linear.x = msg.linear.x
        out.angular.z = self.trim.update(requested, measured, dt) if self.enabled \
            else requested
        self.pub.publish(out)
        if requested and self.log_period > 0.0 and now >= self.next_log:
            self.next_log = now + self.log_period
            self.get_logger().info(
                'TURN RATE requested=%+.3f measured=%s command=%+.3f'
                % (requested, 'stale' if measured is None else '%+.3f' % measured,
                   out.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = TurnTrim()
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
