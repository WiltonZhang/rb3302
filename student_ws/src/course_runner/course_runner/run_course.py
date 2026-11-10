#!/usr/bin/env python3
"""Run one RB3302 graded run: localise from a start square, visit 4 squares.

Usage (robot_init + planning_rpp or planning_astar already running):

    ros2 run course_runner run_course --start S --bearing N --goals A J Q X
    ros2 run course_runner run_course --start S --bearing N --dry-run
    ros2 run course_runner run_course --start S --bearing N --goals A J Q X \
        --avoid P                      # square known to hold the obstacle
    ros2 run course_runner run_course ... --planner nav2   # previous behaviour

Planners:
  grid (default)  A* over the 24 squares (walls from the L08b brief, plus any
                  square the global costmap shows as blocked). The route runs
                  through square centres with rounded corners at squares it
                  passes through, and goes straight in to each goal centre. It
                  is sent straight to the RPP controller (FollowPath), so Nav2
                  does not replan it every second. Before each leg the robot
                  turns on the spot to the leg heading. If a leg cannot be
                  driven, it is replanned, and the last resort is Nav2's own
                  planner (NavigateToPose).
  nav2            Nav2 FollowWaypoints with the NavFn A* planner, as before.

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
import heapq
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import Marker, MarkerArray

try:
    # tf2_ros_py ships with ROS 2; without it the AMCL pose is used instead.
    from rclpy.time import Time
    from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                         LookupException, TransformListener)
    TF_ERRORS = (LookupException, ConnectivityException, ExtrapolationException)
except ImportError:  # pragma: no cover
    Buffer = None
    TF_ERRORS = ()

ROWS = 4
COLS = 6
LETTERS = [chr(ord('A') + i) for i in range(ROWS * COLS)]
COMPASS = {
    'N': 0.0, 'NE': 45.0, 'E': 90.0, 'SE': 135.0,
    'S': 180.0, 'SW': 225.0, 'W': 270.0, 'NW': 315.0,
}
# Interior walls from the L08b brief, as pairs of squares with a wall between
# them. Checked against rb3302_map.pgm (all 38 neighbour pairs agree).
WALLS = {frozenset(pair) for pair in (
    'BH', 'GH', 'HN', 'NO', 'NT', 'IJ', 'JP', 'JK', 'EK', 'KL', 'PV', 'QW')}
# (d_row, d_col) moves: North, South, West, East
MOVES = ((-1, 0), (1, 0), (0, -1), (0, 1))


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def neighbours(letter, blocked):
    """Squares reachable in one move from letter (no wall, not blocked)."""
    row, col = divmod(LETTERS.index(letter), COLS)
    for move, (d_row, d_col) in enumerate(MOVES):
        r, c = row + d_row, col + d_col
        if not (0 <= r < ROWS and 0 <= c < COLS):
            continue
        other = LETTERS[r * COLS + c]
        if frozenset((letter, other)) in WALLS or other in blocked:
            continue
        yield other, move


def grid_astar(start, goal, blocked=(), turn_penalty=0.5):
    """A* over the squares, 4-connected, Manhattan heuristic.

    Each move costs 1, plus turn_penalty when the direction changes, so out of
    several equally short routes the one with the fewest corners wins.
    Returns the list of squares from start to goal, or None.
    """
    goal_r, goal_c = divmod(LETTERS.index(goal), COLS)

    def heuristic(letter):
        r, c = divmod(LETTERS.index(letter), COLS)
        return abs(r - goal_r) + abs(c - goal_c)

    blocked = set(blocked) - {start, goal}
    start_state = (start, None)
    best = {start_state: 0.0}
    parent = {start_state: None}
    heap = [(heuristic(start), 0.0, start, None)]
    while heap:
        _, cost, letter, move = heapq.heappop(heap)
        state = (letter, move)
        if cost > best.get(state, math.inf):
            continue
        if letter == goal:
            route = []
            while state is not None:
                route.append(state[0])
                state = parent[state]
            return route[::-1]
        for other, next_move in neighbours(letter, blocked):
            step = 1.0
            if move is not None and next_move != move:
                step += turn_penalty
            next_state = (other, next_move)
            next_cost = cost + step
            if next_cost < best.get(next_state, math.inf):
                best[next_state] = next_cost
                parent[next_state] = state
                heapq.heappush(
                    heap, (next_cost + heuristic(other), next_cost, other, next_move))
    return None


def corner_curve(turn, tangent, step, smoothness=0.6):
    """Points of a smooth turn by `turn` rad, in a frame starting at (0, 0) heading +x.

    Heading follows turn * (s - a sin(2 pi s) / (2 pi)) for s = 0..1, with
    a = smoothness. Curvature is proportional to 1 - a cos(2 pi s): a = 0 is a
    circular arc (steering jumps to full lock at the start), a = 1 ramps the
    steering from zero to a peak twice the arc's and back to zero. In between
    trades a gentler entry for a tighter middle. The curve is symmetric and
    scaled so it leaves the straight line `tangent` before the corner and
    rejoins the next one `tangent` after it.
    """
    n = 400
    xs, ys = [0.0], [0.0]
    for k in range(n):
        s = (k + 0.5) / n
        heading = turn * (s - smoothness * math.sin(2.0 * math.pi * s) / (2.0 * math.pi))
        xs.append(xs[-1] + math.cos(heading) / n)
        ys.append(ys[-1] + math.sin(heading) / n)
    # distance from the start to the corner for the unit-length curve
    reach = xs[-1] - ys[-1] * math.cos(turn) / math.sin(turn)
    scale = tangent / reach
    m = max(2, int(math.ceil(scale / step)))
    return [(xs[i * n // m] * scale, ys[i * n // m] * scale) for i in range(1, m + 1)]


def rounded_polyline(corners, distance, step, smoothness=0.6):
    """Sample a polyline through corners, with a smooth turn at every inner corner.

    Each turn starts `distance` before the corner and ends `distance` after
    it, so with distance <= half a square it stays inside the square it turns
    in. End points are kept exactly (the goal is the square centre).
    Returns [(x, y, yaw), ...] spaced about step apart.
    """
    if len(corners) < 2:
        return []
    points = [corners[0]]

    def line_to(end):
        sx, sy = points[-1]
        length = math.hypot(end[0] - sx, end[1] - sy)
        n = max(1, int(math.ceil(length / step)))
        for i in range(1, n + 1):
            t = i / n
            points.append((sx + t * (end[0] - sx), sy + t * (end[1] - sy)))

    for i in range(1, len(corners) - 1):
        (x0, y0), (x1, y1), (x2, y2) = corners[i - 1], corners[i], corners[i + 1]
        len_in = math.hypot(x1 - x0, y1 - y0)
        len_out = math.hypot(x2 - x1, y2 - y1)
        u_in = ((x1 - x0) / len_in, (y1 - y0) / len_in)
        u_out = ((x2 - x1) / len_out, (y2 - y1) / len_out)
        cross = u_in[0] * u_out[1] - u_in[1] * u_out[0]
        dot = u_in[0] * u_out[0] + u_in[1] * u_out[1]
        turn = math.atan2(cross, dot)  # +left / -right
        if abs(turn) < 1e-3 or abs(abs(turn) - math.pi) < 1e-3:
            line_to((x1, y1))
            continue
        # capped at half of either neighbouring segment
        tangent = min(distance, len_in / 2.0, len_out / 2.0)
        start = (x1 - u_in[0] * tangent, y1 - u_in[1] * tangent)
        line_to(start)
        for lx, ly in corner_curve(turn, tangent, step, smoothness):
            points.append((start[0] + lx * u_in[0] - ly * u_in[1],
                           start[1] + lx * u_in[1] + ly * u_in[0]))
    line_to(corners[-1])
    return with_headings(points)


def with_headings(points):
    """[(x, y)] -> [(x, y, yaw)], yaw pointing at the next point (last keeps the previous)."""
    poses = []
    yaw = 0.0
    for i, (x, y) in enumerate(points):
        if i + 1 < len(points):
            nx, ny = points[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        poses.append((x, y, yaw))
    return poses


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
        # Square pitch; stretched from the map's 0.60 m to the 0.62 m measured on
        # the course, about the course centre (the corner shared by I, J, O, P).
        self.declare_parameter('cell_size', 0.62)
        self.declare_parameter('center_x', 0.825)
        self.declare_parameter('center_y', 1.405)
        self.declare_parameter('north_yaw', 0.0)
        self.declare_parameter('xy_stddev', 0.15)
        self.declare_parameter('yaw_stddev', 0.5)
        self.declare_parameter('spin_angle', 2.0 * math.pi)
        self.declare_parameter('spin_time_allowance', 25.0)
        # grid planner
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('corner_distance', 0.30)
        self.declare_parameter('corner_smoothness', 0.6)
        self.declare_parameter('path_step', 0.025)
        self.declare_parameter('turn_penalty', 0.5)
        self.declare_parameter('stop_seconds', 3.5)
        self.declare_parameter('align_tolerance', 0.04)
        self.declare_parameter('align_attempts', 3)
        self.declare_parameter('align_time_allowance', 10.0)
        self.declare_parameter('merge_distance', 0.50)
        # AMCL convergence (no-motion updates while stopped)
        self.declare_parameter('settle_yaw_std', 0.04)
        self.declare_parameter('settle_xy_std', 0.03)
        self.declare_parameter('settle_timeout', 4.0)
        self.declare_parameter('blocked_cost', 99)
        self.declare_parameter('replan_attempts', 3)
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'goal_checker')

        param = lambda name: self.get_parameter(name).value
        self.frame_id = str(param('frame_id'))
        self.cell = float(param('cell_size'))
        self.north_yaw = float(param('north_yaw'))
        # square A centre: 1.5 squares North and 2.5 squares West of the centre
        theta = self.north_yaw
        north = (math.cos(theta), math.sin(theta))
        west = (-math.sin(theta), math.cos(theta))
        to_a = ((ROWS - 1) / 2.0 * self.cell, (COLS - 1) / 2.0 * self.cell)
        self.a_x = float(param('center_x')) + to_a[0] * north[0] + to_a[1] * west[0]
        self.a_y = float(param('center_y')) + to_a[0] * north[1] + to_a[1] * west[1]
        self.xy_std = float(param('xy_stddev'))
        self.yaw_std = float(param('yaw_stddev'))
        self.spin_angle = float(param('spin_angle'))
        self.spin_allowance = float(param('spin_time_allowance'))
        self.base_frame = str(param('base_frame'))
        self.corner_distance = float(param('corner_distance'))
        self.corner_smoothness = float(param('corner_smoothness'))
        self.path_step = float(param('path_step'))
        self.turn_penalty = float(param('turn_penalty'))
        self.stop_seconds = float(param('stop_seconds'))
        self.align_tolerance = float(param('align_tolerance'))
        self.align_attempts = int(param('align_attempts'))
        self.align_allowance = float(param('align_time_allowance'))
        self.merge_distance = float(param('merge_distance'))
        self.settle_yaw_std = float(param('settle_yaw_std'))
        self.settle_xy_std = float(param('settle_xy_std'))
        self.settle_timeout = float(param('settle_timeout'))
        self.blocked_cost = int(param('blocked_cost'))
        self.replan_attempts = int(param('replan_attempts'))
        self.controller_id = str(param('controller_id'))
        self.goal_checker_id = str(param('goal_checker_id'))

        self.start = args.start
        self.bearing_deg = args.bearing
        self.goals = args.goals
        self.do_spin = not args.no_spin
        self.blocked = set(args.avoid)
        self.preview = []

        self.costmap = None
        # volatile + reliable matches the costmap publisher whatever its durability
        self.create_subscription(
            OccupancyGrid, 'global_costmap/costmap', self._costmap_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        # shown by the existing 'Plan' display (/plan) in navigation.rviz
        self.path_pub = self.create_publisher(Path, 'plan', 10)
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = None
        if Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

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
        # AMCL runs a filter update without motion when asked (normally it only
        # updates after moving update_min_d / update_min_a).
        self.nomotion_client = self.create_client(Empty, 'request_nomotion_update')
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

    def cell_of(self, x, y):
        """Letter of the square containing map point (x, y), or None."""
        theta = self.north_yaw
        dx, dy = x - self.a_x, y - self.a_y
        east = dx * math.sin(theta) - dy * math.cos(theta)
        south = -(dx * math.cos(theta) + dy * math.sin(theta))
        row, col = int(round(south / self.cell)), int(round(east / self.cell))
        if 0 <= row < ROWS and 0 <= col < COLS:
            return LETTERS[row * COLS + col]
        return None

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
        if self.preview:
            self.publish_path(self.preview)

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

    # --------------------------------------------------------- grid planner
    def _costmap_callback(self, msg):
        self.costmap = msg

    def pause(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def robot_pose(self):
        """(x, y, yaw) of base_frame in the map: TF if possible, else last AMCL pose."""
        if self.tf_buffer is not None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.frame_id, self.base_frame, Time())
                q = t.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return t.transform.translation.x, t.transform.translation.y, yaw
            except TF_ERRORS as exc:
                self.get_logger().debug('TF lookup failed: %s' % exc)
        if self.amcl_msg is None:
            return None
        p = self.amcl_msg.pose.pose
        return (p.position.x, p.position.y,
                2.0 * math.atan2(p.orientation.z, p.orientation.w))

    def costmap_value(self, x, y):
        """Global costmap value at (x, y): 0 free .. 99 inscribed, 100 lethal, -1 unknown."""
        grid = self.costmap
        if grid is None:
            return None
        info = grid.info
        ix = int(math.floor((x - info.origin.position.x) / info.resolution))
        iy = int(math.floor((y - info.origin.position.y) / info.resolution))
        if not (0 <= ix < info.width and 0 <= iy < info.height):
            return None
        return grid.data[iy * info.width + ix]

    def first_blocked_square(self, poses, ignore):
        """First square on the path where the robot centre would be in collision."""
        for x, y, _ in poses:
            value = self.costmap_value(x, y)
            if value is not None and value >= self.blocked_cost:
                letter = self.cell_of(x, y)
                if letter is not None and letter not in ignore:
                    return letter
        return None

    def leg_poses(self, route):
        corners = [self.square_center(letter) for letter in route]
        return rounded_polyline(
            corners, self.corner_distance, self.path_step, self.corner_smoothness)

    def plan_leg(self, here, goal):
        """Grid route here -> goal and its path, avoiding blocked squares."""
        while True:
            route = grid_astar(here, goal, self.blocked, self.turn_penalty)
            if route is None:
                self.get_logger().warn(
                    'no grid route %s -> %s avoiding %s'
                    % (here, goal, ','.join(sorted(self.blocked)) or '-'))
                return None, None
            poses = self.leg_poses(route)
            bad = self.first_blocked_square(poses, ignore={here, goal})
            if bad is None:
                return route, poses
            self.get_logger().warn(
                'square %s is blocked in the global costmap, routing around it' % bad)
            self.blocked.add(bad)

    def make_path(self, poses):
        path = Path()
        path.header.frame_id = self.frame_id
        path.poses = [self.make_pose(x, y, yaw) for x, y, yaw in poses]
        return path

    def publish_path(self, poses):
        self.path_pub.publish(self.make_path(poses))

    def amcl_spread(self):
        """(yaw std, larger of x/y std) of the last AMCL pose, or None."""
        if self.amcl_msg is None:
            return None
        cov = self.amcl_msg.pose.covariance
        return (math.sqrt(max(cov[35], 0.0)),
                math.sqrt(max(cov[0], cov[7], 0.0)))

    def settle_localisation(self, budget):
        """While stopped, ask AMCL for no-motion updates until its spread is
        below settle_yaw_std / settle_xy_std or `budget` seconds pass.

        AMCL normally only updates after moving 5 cm / 0.1 rad, so the heading
        it had when the robot stopped (std ~0.17 rad = 10 deg after the start
        spin in field test 1) is what the next leg would be aligned to. A few
        extra scans while standing still tighten it.
        """
        end = time.monotonic() + budget
        if not self.nomotion_client.wait_for_service(timeout_sec=0.2):
            return self.amcl_spread()
        while time.monotonic() < end:
            sent = time.monotonic()
            self.nomotion_client.call_async(Empty.Request())
            # the update happens on the next scan; wait for its /amcl_pose
            while time.monotonic() < min(end, sent + 1.0):
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.amcl_stamp is not None and self.amcl_stamp > sent:
                    break
            spread = self.amcl_spread()
            if spread is not None and spread[0] <= self.settle_yaw_std \
                    and spread[1] <= self.settle_xy_std:
                return spread
        return self.amcl_spread()

    def turn_to(self, target_yaw):
        """Turn on the spot onto target_yaw (behavior_server Spin, which slows
        down before the target), re-checking the heading after each turn so the
        robot starts each leg along the compass axis, not a few degrees off."""
        for _ in range(self.align_attempts):
            pose = self.robot_pose()
            if pose is None:
                return
            delta = wrap_angle(target_yaw - pose[2])
            if abs(delta) < self.align_tolerance:
                return
            self.get_logger().info('turning %.1f deg onto the leg' % math.degrees(delta))
            goal = Spin.Goal()
            goal.target_yaw = delta
            goal.time_allowance = Duration(sec=int(self.align_allowance))
            result = self._send(self.spin_client, goal, 'spin')
            if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn('turn onto the leg did not finish')
            self.settle_localisation(1.0)
        pose = self.robot_pose()
        if pose is not None:
            self.get_logger().warn(
                'still %.1f deg off the leg heading, RPP will correct'
                % math.degrees(wrap_angle(target_yaw - pose[2])))

    def merge_onto_path(self, poses):
        """Start the leg where the robot actually is.

        The robot stops up to xy_goal_tolerance (4 cm) off a square centre. The
        path keeps the robot's sideways offset at the start and blends it to
        zero along the first straight (cosine blend over merge_distance), so
        the robot drives off along the compass axis and eases onto the centre
        line, instead of pure pursuit aiming it diagonally at the centre line.
        """
        pose = self.robot_pose()
        if pose is None or len(poses) < 2:
            return poses
        x0, y0, yaw0 = poses[0]
        ux, uy = math.cos(yaw0), math.sin(yaw0)
        lateral = -(pose[0] - x0) * uy + (pose[1] - y0) * ux
        if abs(lateral) < 0.005 or abs(lateral) > self.cell / 2.0:
            return poses
        straight = 0
        while straight + 1 < len(poses) and \
                abs(wrap_angle(poses[straight + 1][2] - yaw0)) < 1e-3:
            straight += 1
        last = poses[straight]
        length = min(self.merge_distance,
                     (last[0] - x0) * ux + (last[1] - y0) * uy)
        if length < 0.10:
            return poses
        points = []
        for x, y, _ in poses:
            s = (x - x0) * ux + (y - y0) * uy
            if len(points) <= straight and s < length:
                w = 0.5 * (1.0 + math.cos(math.pi * max(s, 0.0) / length))
                points.append((x - w * lateral * uy, y + w * lateral * ux))
            else:
                points.append((x, y))
        return with_headings(points)

    def follow_path(self, poses):
        goal = FollowPath.Goal()
        goal.path = self.make_path(poses)
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        result = self._send(self.follow_path_client, goal, 'follow_path')
        return result is not None and result.status == GoalStatus.STATUS_SUCCEEDED

    def navigate_to(self, letter):
        """Fallback: Nav2 planner (NavFn A*) + RPP to the square centre."""
        x, y = self.square_center(letter)
        pose = self.robot_pose()
        yaw = math.atan2(y - pose[1], x - pose[0]) if pose is not None else 0.0
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)
        result = self._send(self.navigate_client, goal, 'navigate_to_pose')
        return result is not None and result.status == GoalStatus.STATUS_SUCCEEDED

    def drive_leg(self, here, goal):
        """Drive from square `here` to the centre of `goal`. True if reached."""
        for attempt in range(1, self.replan_attempts + 1):
            if here == goal:
                break
            route, poses = self.plan_leg(here, goal)
            if route is None:
                break
            self.get_logger().info(
                'route %s (attempt %d)' % (' -> '.join(route), attempt))
            # face exactly along the compass axis of the first straight
            self.turn_to(poses[0][2])
            poses = self.merge_onto_path(poses)
            self.publish_path(poses)
            if self.follow_path(poses):
                return True
            self.get_logger().warn('follow_path %s -> %s failed' % (here, goal))
            # give the costmap a moment to mark whatever stopped the robot
            self.pause(1.5)
            pose = self.robot_pose()
            if pose is not None:
                here = self.cell_of(pose[0], pose[1]) or here
        self.get_logger().warn('using the Nav2 planner for %s' % goal)
        return self.navigate_to(goal)

    def log_stop_error(self, letter):
        pose = self.robot_pose()
        if pose is None:
            return
        x, y = self.square_center(letter)
        self.get_logger().info(
            'stopped %.0f mm from the centre of %s (dx=%.3f, dy=%.3f)'
            % (1000.0 * math.hypot(pose[0] - x, pose[1] - y), letter,
               pose[0] - x, pose[1] - y))

    def run_grid(self):
        missed = []
        here = self.start
        for i, goal in enumerate(self.goals, 1):
            self.get_logger().info('heading to %d/%d: %s' % (i, len(self.goals), goal))
            if self.drive_leg(here, goal):
                self.log_stop_error(goal)
                here = goal
                # use the compulsory stop to sharpen AMCL for the next leg
                stop_end = time.monotonic() + self.stop_seconds
                self.settle_localisation(self.stop_seconds)
                self.pause(max(0.0, stop_end - time.monotonic()))
                continue
            self.get_logger().error('could not reach %s' % goal)
            missed.append(goal)
            pose = self.robot_pose()
            if pose is not None:
                here = self.cell_of(pose[0], pose[1]) or here
        if missed:
            self.get_logger().error('missed squares: %s' % ', '.join(missed))
        return not missed

    def preview_routes(self):
        """Dry run: log each leg's grid route and show the whole path on /plan."""
        here = self.start
        self.preview = []
        for goal in self.goals:
            route = grid_astar(here, goal, self.blocked, self.turn_penalty)
            if route is None:
                self.get_logger().warn('no grid route %s -> %s' % (here, goal))
                continue
            self.get_logger().info('leg %s -> %s: %s' % (here, goal, ' -> '.join(route)))
            self.preview.extend(self.leg_poses(route))
            here = goal


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
    parser.add_argument('--planner', choices=('grid', 'nav2'), default='grid',
                        help='grid: square-centre route with rounded corners '
                             '(default); nav2: FollowWaypoints + NavFn A*')
    parser.add_argument('--avoid', nargs='+', type=parse_letter, default=[],
                        help='squares to route around (e.g. the obstacle square)')
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
            if args.planner == 'grid':
                node.preview_routes()
            node.get_logger().info(
                'dry run: showing /waypoint_markers and /plan, Ctrl-C to exit')
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
        if args.planner == 'grid':
            node.settle_localisation(node.settle_timeout)
            node.log_amcl_spread('after settling')
        t1 = time.monotonic()
        if args.planner == 'grid':
            ok = node.run_grid()
        else:
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
