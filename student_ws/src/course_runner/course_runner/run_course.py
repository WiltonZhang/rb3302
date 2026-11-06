#!/usr/bin/env python3
"""Run one RB3302 graded run: localise from a start square, visit 4 squares.

Usage (robot_init + planning_rpp or planning_astar already running):

    ros2 run course_runner run_course --start S --bearing N --goals A J Q X
    ros2 run course_runner run_course --start S --bearing N --dry-run

Mission inputs are CLI arguments, not ROS parameters, because the ROS YAML
parser turns a bare N/Y into a boolean and N is both a square and a bearing.

Course letters (as in the L08b brief, North up):

        A B C D E F
    W   G H I J K L   E
        M N O P Q R
        S T U V W X

Grid geometry (map frame) comes from ROS parameters; the defaults match
student_ws/maps/rb3302_map.yaml, where North is +x and West is +y.
"""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

ROWS = 4
COLS = 6
LETTERS = [chr(ord('A') + i) for i in range(ROWS * COLS)]
COMPASS = {
    'N': 0.0, 'NE': 45.0, 'E': 90.0, 'SE': 135.0,
    'S': 180.0, 'SW': 225.0, 'W': 270.0, 'NW': 315.0,
}


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def parse_bearing(text):
    """Compass word (N, NE, ...) or degrees clockwise from North."""
    key = text.strip().upper()
    if key in COMPASS:
        return COMPASS[key]
    try:
        return float(key)
    except ValueError:
        raise argparse.ArgumentTypeError(
            'bearing must be one of %s or degrees' % '/'.join(COMPASS))


def parse_letter(text):
    letter = text.strip().upper()
    if letter not in LETTERS:
        raise argparse.ArgumentTypeError('square must be A-X, got %r' % text)
    return letter


class CourseRunner(Node):

    def __init__(self, args):
        super().__init__('course_runner')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('cell_size', 0.60)
        self.declare_parameter('a_center_x', 1.725)
        self.declare_parameter('a_center_y', 2.905)
        self.declare_parameter('north_yaw', 0.0)
        self.declare_parameter('xy_stddev', 0.15)
        self.declare_parameter('yaw_stddev', 0.5)
        self.declare_parameter('spin_angle', 2.0 * math.pi)
        self.declare_parameter('spin_time_allowance', 25.0)

        param = lambda name: self.get_parameter(name).value
        self.frame_id = str(param('frame_id'))
        self.cell = float(param('cell_size'))
        self.a_x = float(param('a_center_x'))
        self.a_y = float(param('a_center_y'))
        self.north_yaw = float(param('north_yaw'))
        self.xy_std = float(param('xy_stddev'))
        self.yaw_std = float(param('yaw_stddev'))
        self.spin_angle = float(param('spin_angle'))
        self.spin_allowance = float(param('spin_time_allowance'))

        self.start = args.start
        self.bearing_deg = args.bearing
        self.goals = args.goals
        self.do_spin = not args.no_spin

        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, 'waypoint_markers', 10)
        self.amcl_stamp = None
        self.amcl_msg = None
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._amcl_callback, 10)
        self.waypoint_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints')
        self.spin_client = ActionClient(self, Spin, 'spin')
        # Nav2 only activates planner/controller/behaviors once AMCL publishes
        # map->odom, i.e. after the initial pose; this service says when it is done.
        self.nav_active_client = self.create_client(
            Trigger, 'lifecycle_manager_navigation/is_active')
        self.create_timer(1.0, self.publish_markers)

    # ------------------------------------------------------------ geometry
    def square_center(self, letter):
        row, col = divmod(LETTERS.index(letter), COLS)
        theta = self.north_yaw
        north = (math.cos(theta), math.sin(theta))
        east = (math.sin(theta), -math.cos(theta))
        x = self.a_x + col * self.cell * east[0] - row * self.cell * north[0]
        y = self.a_y + col * self.cell * east[1] - row * self.cell * north[1]
        return x, y

    def compass_to_yaw(self, degrees):
        return math.atan2(
            math.sin(self.north_yaw - math.radians(degrees)),
            math.cos(self.north_yaw - math.radians(degrees)))

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        # zero stamp = use latest TF, avoids extrapolation errors
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = yaw_to_quaternion(yaw)
        pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
        pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
        return pose

    def goal_poses(self):
        poses = []
        prev_x, prev_y = self.square_center(self.start)
        yaw = self.compass_to_yaw(self.bearing_deg)
        for letter in self.goals:
            x, y = self.square_center(letter)
            if math.hypot(x - prev_x, y - prev_y) > 1e-6:
                yaw = math.atan2(y - prev_y, x - prev_x)
            poses.append(self.make_pose(x, y, yaw))
            prev_x, prev_y = x, y
        return poses

    def log_plan(self):
        x, y = self.square_center(self.start)
        self.get_logger().info(
            'start %s at (%.3f, %.3f), bearing %.0f deg -> map yaw %.2f rad'
            % (self.start, x, y, self.bearing_deg,
               self.compass_to_yaw(self.bearing_deg)))
        for i, letter in enumerate(self.goals, 1):
            gx, gy = self.square_center(letter)
            self.get_logger().info(
                'goal %d: %s at (%.3f, %.3f)' % (i, letter, gx, gy))

    # ------------------------------------------------------------- markers
    def publish_markers(self):
        markers = MarkerArray()
        order = {}
        for i, letter in enumerate(self.goals, 1):
            order.setdefault(letter, []).append(str(i))
        for idx, letter in enumerate(LETTERS):
            x, y = self.square_center(letter)
            text = Marker()
            text.header.frame_id = self.frame_id
            text.ns = 'squares'
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y = x, y
            text.pose.position.z = 0.15
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.text = letter
            if letter in order:
                text.text = '%s (%s)' % (letter, ','.join(order[letter]))
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            markers.markers.append(text)

            if letter in order or letter == self.start:
                disc = Marker()
                disc.header.frame_id = self.frame_id
                disc.ns = 'targets'
                disc.id = idx
                disc.type = Marker.CYLINDER
                disc.action = Marker.ADD
                disc.pose.position.x, disc.pose.position.y = x, y
                disc.pose.orientation.w = 1.0
                # diameter = 2 x goal tolerance region shown for reference
                disc.scale.x = disc.scale.y = 0.12
                disc.scale.z = 0.01
                is_start = letter == self.start and letter not in order
                disc.color.r = 0.1 if is_start else 1.0
                disc.color.g = 0.8 if is_start else 0.5
                disc.color.b = 0.1
                disc.color.a = 0.9
                markers.markers.append(disc)
        self.marker_pub.publish(markers)

    # -------------------------------------------------------- localisation
    def _amcl_callback(self, msg):
        self.amcl_msg = msg
        self.amcl_stamp = time.monotonic()

    def set_initial_pose(self, attempts=10):
        x, y = self.square_center(self.start)
        yaw = self.compass_to_yaw(self.bearing_deg)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.pose = self.make_pose(x, y, yaw).pose
        cov = [0.0] * 36
        cov[0] = cov[7] = self.xy_std ** 2
        cov[35] = self.yaw_std ** 2
        msg.pose.covariance = cov

        for attempt in range(1, attempts + 1):
            while self.initial_pub.get_subscription_count() == 0:
                self.get_logger().info('waiting for AMCL on /initialpose ...')
                rclpy.spin_once(self, timeout_sec=1.0)
            sent = time.monotonic()
            # zero stamp: AMCL uses the latest odom TF instead of failing with
            # 'extrapolation into the future'
            self.initial_pub.publish(msg)
            deadline = sent + 2.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.amcl_stamp is not None and self.amcl_stamp > sent:
                    self.get_logger().info(
                        'AMCL accepted initial pose (attempt %d)' % attempt)
                    return True
            self.get_logger().warn(
                'no /amcl_pose after initial pose, retrying (AMCL active?)')
        return False

    def log_amcl_spread(self, label):
        if self.amcl_msg is None:
            return
        cov = self.amcl_msg.pose.covariance
        p = self.amcl_msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.get_logger().info(
            '%s: AMCL (%.3f, %.3f, %.2f rad), std xy=(%.3f, %.3f) yaw=%.3f'
            % (label, p.position.x, p.position.y, yaw,
               math.sqrt(max(cov[0], 0.0)), math.sqrt(max(cov[7], 0.0)),
               math.sqrt(max(cov[35], 0.0))))

    def wait_for_nav_active(self, timeout=90.0):
        """Block until lifecycle_manager_navigation reports all servers active."""
        start = time.monotonic()
        last_log = 0.0
        while time.monotonic() - start < timeout:
            if self.nav_active_client.wait_for_service(timeout_sec=1.0):
                future = self.nav_active_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.done() and future.result() is not None \
                        and future.result().success:
                    self.get_logger().info(
                        'Nav2 active after %.1f s' % (time.monotonic() - start))
                    return True
            if time.monotonic() - last_log > 3.0:
                last_log = time.monotonic()
                self.get_logger().info('waiting for Nav2 servers to become active ...')
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(
            'Nav2 not active after %.0f s - check lifecycle_manager_navigation '
            'output in the planning terminal' % timeout)
        return False

    # ------------------------------------------------------------- actions
    def _send(self, client, goal, name, feedback_cb=None, retries=10):
        """Send an action goal, retrying while Nav2 is still activating."""
        self.get_logger().info('waiting for %s action server ...' % name)
        client.wait_for_server()
        for _ in range(retries):
            future = client.send_goal_async(goal, feedback_callback=feedback_cb)
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if handle is not None and handle.accepted:
                result_future = handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                return result_future.result()
            self.get_logger().warn('%s goal rejected, retrying in 2 s' % name)
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def localisation_spin(self):
        goal = Spin.Goal()
        goal.target_yaw = self.spin_angle
        goal.time_allowance = Duration(sec=int(self.spin_allowance))
        result = self._send(self.spin_client, goal, 'spin')
        ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        if not ok:
            self.get_logger().warn('localisation spin did not succeed')
        return ok

    def _waypoint_feedback(self, feedback_msg):
        index = feedback_msg.feedback.current_waypoint
        if index != getattr(self, '_last_index', None) and index < len(self.goals):
            self._last_index = index
            self.get_logger().info(
                'heading to %d/%d: %s' % (index + 1, len(self.goals),
                                          self.goals[index]))

    def follow_waypoints(self):
        goal = FollowWaypoints.Goal()
        goal.poses = self.goal_poses()
        result = self._send(
            self.waypoint_client, goal, 'follow_waypoints',
            feedback_cb=self._waypoint_feedback)
        if result is None:
            self.get_logger().error('follow_waypoints was never accepted')
            return False
        missed = []
        for item in result.result.missed_waypoints:
            # Jazzy: WaypointStatus msgs; older distros: plain int indices
            index = getattr(item, 'waypoint_index', item)
            missed.append(self.goals[index] if 0 <= index < len(self.goals)
                          else str(index))
        if missed:
            self.get_logger().error('missed squares: %s' % ', '.join(missed))
        return not missed


def parse_args(argv):
    parser = argparse.ArgumentParser(description='RB3302 graded run')
    parser.add_argument('--start', required=True, type=parse_letter,
                        help='start square, e.g. S')
    parser.add_argument('--bearing', required=True, type=parse_bearing,
                        help='rough start heading: N/E/S/W/NE/... or degrees')
    parser.add_argument('--goals', nargs='+', type=parse_letter, default=[],
                        help='squares to visit in order, e.g. A J Q X')
    parser.add_argument('--no-spin', action='store_true',
                        help='skip the in-place spin used to converge AMCL')
    parser.add_argument('--dry-run', action='store_true',
                        help='only print + show markers, do not move')
    args = parser.parse_args(argv)
    if not args.dry_run and not args.goals:
        parser.error('--goals is required unless --dry-run')
    return args


def main(argv=None):
    argv = sys.argv if argv is None else argv
    rclpy.init(args=argv)
    args = parse_args(rclpy.utilities.remove_ros_args(argv)[1:])
    node = CourseRunner(args)
    node.log_plan()
    try:
        if args.dry_run:
            node.get_logger().info('dry run: showing /waypoint_markers, Ctrl-C to exit')
            rclpy.spin(node)
            return
        t0 = time.monotonic()
        if not node.set_initial_pose():
            node.get_logger().error('AMCL never accepted the initial pose')
            return
        if not node.wait_for_nav_active():
            return
        if node.do_spin:
            node.localisation_spin()
            node.log_amcl_spread('after spin')
        t1 = time.monotonic()
        ok = node.follow_waypoints()
        t2 = time.monotonic()
        node.get_logger().info(
            '%s: waypoints %.1f s, total incl. localisation %.1f s'
            % ('DONE' if ok else 'FINISHED WITH MISSES', t2 - t1, t2 - t0))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
