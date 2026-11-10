# RB3302 — session handoff (5–10 Nov 2026)

This is the record of the first Claude session on this project. It covers the task requirements,
the system facts worked out, every change made and why, and the owner's field-test logs (quoted
exactly). The project rules are in `/CLAUDE.md`.

---

## 1. Task requirements (from the course briefs)

**L08b — Planning & Navigation project description (Dr. Jason S. Ku)**
- Course: a 4 × 6 grid of ~60 cm cubes, 24 squares labelled A–X, with each letter centred on the
  floor of its square. North is up in the brief's diagram.

  ```text
                 N
        A  B  C  D  E  F
     W  G  H  I  J  K  L  E
        M  N  O  P  Q  R
        S  T  U  V  W  X
                 S
  ```
- Interior walls (pairs of squares with a wall between them): B|H, G|H, H|N, N|O, N|T, I|J, J|P,
  J|K, E|K, K|L, P|V, Q|W.
- Graded run: you are given a **start square and a rough bearing (±45°)** and a **sequence of 4
  squares to visit**. An obstacle may block one square. The robot must **stop 3 s at each square**
  and **finish within 3 min**.
- 3 graded runs (randomised, one the same for everyone). If the robot significantly moves or
  damages the course, the run is paused and marks may be deducted.
- Scoring: each of the 4 paths is worth 4 points, +1 each for **reaching the square**, being
  **contained in the square**, **covering the label**, and **not touching walls or obstacles**.
  Maximum is 3 × 4 × 4 = 48 pts, plus 2 pts.

**L08c — Final project briefing**
- Robot: Wave Rover differential-drive base (4WD skid-steer), 2D RPLIDAR, Yesense IMU, and an
  onboard ROS 2 computer. To connect: `ssh ubuntu@192.168.50.<GROUP>`.
- `robot_ws` is provided (drivers, sensors, robot model, TF). `student_ws` is where your own work
  goes: `mapping_cartographer`, `planning_astar`, `planning_rpp`.
- Step 1: `ros2 launch robot_init robot.launch.py`. Step 2: teleop with
  `ros2 run waverover_base keyboard_teleop`. Step 3: mapping with
  `ros2 launch mapping_cartographer mapping.launch.py`. Step 4: `rviz2` on the laptop.
- Tasks: 1) map with Cartographer + IMU; 2) localise with AMCL (needs a map and an initial pose);
  3) costmaps (footprint and inflation), A* global planning, and Regulated Pure Pursuit following.
  Concepts to investigate: lookahead, curvature, linear/angular velocity, collision checking, and
  speed regulation.

## 2. System facts worked out

- **Robot size:** measured 18 × 19 cm. The brief doesn't say which side is the length, so the
  Nav2 footprint uses 19 × 19 cm (±0.095 m) with 0.01 m padding. `base_link` is assumed to be at
  the centre of the chassis. The URDF chassis box (30 × 25 cm) in robot_ws is display-only.
- **Driver** (`robot_ws/.../driver.launch.py`, read-only): `/cmd_vel` linear.x and angular.z are
  sent straight to the motor board, clamped at **0.8 m/s and 3.0 rad/s**, with a cmd_vel timeout
  of 0.8 s. Keyboard teleop defaults to 1.5 rad/s, which turns well.
- **Skid-steer turn tracking:** a 360° spin commanded at 1.0 rad/s took about 18 s, so the robot
  actually turns at about ⅓ of the commanded rate.
- **Map:** `student_ws/maps/rb3302_map.yaml` + `.pgm` is the owner's cleaned Cartographer map
  (originally named `rb3302_map_clean.*`). It is 71 × 90 px at 0.05 m/px, with
  `origin: [-0.925, -0.845, 0]`. The walls are drawn 2 px (10 cm) thick on a 12 px (0.60 m)
  pitch, which leaves 50 cm of free space per square.
- **Map frame vs course:** **+x = North, +y = West.** Square A's centre is at
  (1.725, 2.905), and the squares are 0.60 m apart:
  - centre(row r, col c) = (1.725 − 0.6·r, 2.905 − 0.6·c), where r = 0..3 runs N→S and c = 0..5
    runs W→E. For example, S = (−0.075, 2.905), M = (0.525, 2.905), N = (0.525, 2.305),
    X = (−0.075, −0.095), R = (0.525, −0.095), Q = (0.525, 0.505).
  - Bearing → map yaw: N = 0, W = +π/2, S = π, E = −π/2.
  - This was verified against the PGM: all 24 square centres fit the robot, and all 38 pairs of
    neighbouring squares match the brief's wall list. The owner confirmed in RViz that the LiDAR
    scan lines up with the map.
- **ROS param gotcha:** the ROS YAML parser turns a bare `N`/`Y` into a boolean, so mission inputs
  for `course_runner` are CLI arguments, not ROS parameters.
- **Nav2 activation:** the planner and controller costmaps wait for map→odom from AMCL, so the
  Nav2 servers only become active **after** an initial pose is set. `course_runner` handles this.

## 3. What was built (all in `student_ws`)

1. **Map** installed as `maps/rb3302_map.{yaml,pgm}` with `image: rb3302_map.pgm`.
2. **`planning_rpp/config/nav2_params.yaml` and `planning_astar/config/nav2_params.yaml`**: these
   are identical and tuned for the robot and course (see section 5 for the current values).
   Both use NavFn with `use_astar: true` and the RPP controller.
3. **New package `course_runner`** (ament_python; depends on rclpy, geometry_msgs, nav2_msgs,
   action_msgs, builtin_interfaces, visualization_msgs and std_srvs):
   ```bash
   ros2 run course_runner run_course --start S --bearing N --goals A J Q X
   ros2 run course_runner run_course --start S --bearing N --goals A J Q X --dry-run
   ```
   Flow:
   1. Publish the A–X letters and the target discs on `/waypoint_markers` (already in
      `navigation.rviz`).
   2. Publish `/initialpose` with a zero stamp at the start square centre and bearing, with
      σxy = 0.15 m and σyaw = 0.5 rad. Wait for `/amcl_pose` to confirm, retrying up to 10×.
   3. Poll `lifecycle_manager_navigation/is_active` (up to 90 s).
   4. Spin 360° using the `spin` action so AMCL converges (skip with `--no-spin`).
   5. Send `FollowWaypoints` with the 4 square centres. Each waypoint's yaw is the direction of
      travel, which doesn't matter because yaw tolerance is ignored. Nav2's `WaitAtWaypoint`
      pauses 3.5 s on each square.
   6. Log the missed squares and the elapsed time (waypoints only, and total including
      localisation).

   Grid parameters (`cell_size`, `a_center_x/y`, `north_yaw`, `xy_stddev`, `yaw_stddev`,
   `spin_angle`, …) live in `course_runner/config/course.yaml`. Override them with
   `--ros-args --params-file`.
4. **README** updated with a "Graded Run (course_runner)" section.

Reference image of the squares on the map: `docs/claude-handoff/rb3302_map_squares.png`.

## 4. Chronology, field tests and logs

### 4.1 Setup (5–6 Nov)
- The owner provided the course template zip, both PDFs, and the cleaned map.
- I built the map install, the first Nav2 tuning (footprint, inflation 0.30 global / 0.25 local
  with cost_scaling 10, xy tolerance 0.06, yaw ignored, 3.5 s pause, RPP lookahead 0.25 at
  0.22 m/s, AMCL alphas 0.2, a scan obstacle layer on the global costmap) and `course_runner`.
- Commits were pushed to `WiltonZhang/rb3302`, branch `student-ws-nav-tuning`, and merged into
  `main` via PR #2. The original `CoNickel-MC/rb3302` remote is not writable from Claude.
- The owner asked to confirm robot_ws was untouched. It was confirmed identical to the zip. The
  robot_ws entries in `git status` (the `rplidar_ros` submodule and staged `.pyc` files) were
  already there in the original zip.

### 4.2 Owner's RViz check (6 Nov, 11:43)
> "this is the map I have gotten from Rviz which i think is quite accurate."

The screenshot showed the LiDAR scan aligned with the walls, the letters on the right squares,
and the robot at S.

### 4.3 Field test 1 — S → M and S → M → N (6 Nov, 11:59)
> "For start S, Bearing N, goals M, the robot centre is slightly east by around 5 cm, heading is
> around 20 degrees to the north.
> For start S, bearing N, goals M N, the turning at grid M is very slow as it tries to enter grid
> N. I think the robot knows that it has to get there but the resistance is rather high."

Owner's logs:
```text
[controller_server-6] [INFO] [1780676379.522172501] [controller_server]: Passing new path to controller.
[controller_server-6] [ERROR] [1780676380.324009584] [controller_server]: Failed to make progress
[controller_server-6] [WARN] [1780676380.324473021] [controller_server]: [follow_path] [ActionServer] Aborting handle.
[controller_server-6] [INFO] [1780676380.341111719] [local_costmap.local_costmap]: Received request to clear entirely the local_costmap
[controller_server-6] [INFO] [1780676380.347809011] [controller_server]: Received a goal, begin computing control effort.
[controller_server-6] [INFO] [1780676380.548396459] [controller_server]: Passing new path to controller.
... (Passing new path to controller, once per second until ~1780676389.7) ...

[INFO] [1780676217.260283552] [course_runner]: start S at (-0.075, 2.905), bearing 0 deg -> map yaw 0.00 rad
[INFO] [1780676217.260885844] [course_runner]: goal 1: M at (0.525, 2.905)
[INFO] [1780676217.261414750] [course_runner]: goal 2: N at (0.525, 2.305)
[INFO] [1780676217.262104542] [course_runner]: waiting for AMCL on /initialpose ...
  (x5)
[INFO] [1780676221.351938707] [course_runner]: AMCL accepted initial pose (attempt 1)
[INFO] [1780676221.353958603] [course_runner]: waiting for spin action server ...
[INFO] [1780676239.667730731] [course_runner]: after spin: AMCL (-0.055, 2.884, -0.03 rad), std xy=(0.082, 0.066) yaw=0.169
[INFO] [1780676239.672249325] [course_runner]: waiting for follow_waypoints action server ...
[INFO] [1780676239.684725940] [course_runner]: heading to 1/2: M
[INFO] [1780676260.238257807] [course_runner]: heading to 2/2: N
```
Diagnosis:
- **Stuck turn:** RPP's rotate-to-heading limits each command to the *measured* turn rate plus
  `max_angular_accel·dt`. With accel 2.0 that's 0.1 rad/s from standstill, which can't overcome
  tyre scrub, so the robot never starts turning. That deadlock tripped the progress checker
  (`Failed to make progress`) for about 2 min on the M→N leg.
- **The spin took 18 s**, which shows the robot only turns at about ⅓ of the commanded rate.
- **S→M (0.6 m) took about 17 s of driving.** Speed was throttled by cost scaling, approach
  scaling and the minimum speeds.
- **The 5 cm stop offset** comes from the goal tolerance of 6 cm plus AMCL's uncertainty after
  the spin (std of about 8 cm).

→ Commit `f718989`: RPP `max_angular_accel` 2→20, rotate speed 1.0→1.8, rotate min angle
0.6→0.4; smoother angular ±1.2→±2.0 and accel 2.5→4.0; spin max 1.0→1.8; `cost_scaling_dist`
0.20→0.12; approach distance 0.30→0.20; minimum speeds 0.06/0.05→0.10/0.08; progress allowance
10→15 s; AMCL alpha1–4 0.2→0.1; xy tolerance 0.06→0.04.

### 4.4 Field test 2 — "turning really slowly" (6 Nov, 12:34)
> "right now the robot is turning really slowly, which i think the angular velocity published is
> really small. Verify this. If its true, increase the angular velocity and other relevant params"

Verified from the code and settings (not measured on the robot): on an arc, RPP's turn command is
linear speed × curvature. With the heading error below the rotate threshold (23°), that's only
0.3–0.6 rad/s, or about 0.1–0.2 rad/s actual.

→ Commit `d4c9121`: rotate min angle 0.4→0.25 (≈14°), rotate speed 1.8→2.5, smoother angular
±2.0→±2.8 and accel 4→6, spin max/min/acc 1.8/0.3/2.0→2.5/0.6/4.0, regulated min radius
0.35→0.25, regulated min speed 0.10→0.12.

### 4.5 Field test 3 — X → R → Q, spin rejected (6 Nov, 13:34)
```text
[INFO] [1780674731.214388390] [course_runner]: start X at (-0.075, -0.095), bearing 0 deg -> map yaw 0.00 rad
[INFO] [1780674731.215066828] [course_runner]: goal 1: R at (0.525, -0.095)
[INFO] [1780674731.215621671] [course_runner]: goal 2: Q at (0.525, 0.505)
[INFO] [1780674733.324842712] [course_runner]: AMCL accepted initial pose (attempt 1)
[INFO] [1780674733.325424587] [course_runner]: waiting for spin action server ...
[WARN] [1780674733.334029796] [course_runner]: spin goal rejected, retrying in 2 s
[WARN] [1780674735.414741878] [course_runner]: spin goal rejected, retrying in 2 s
[WARN] [1780674737.511217086] [course_runner]: spin goal rejected, retrying in 2 s
[amcl-3] [WARN] [1780674724.129393549] [amcl]: Failed to transform initial pose in time (Lookup would require extrapolation into the future. ...)
[amcl-3] [INFO] [1780674724.129441987] [amcl]: Setting pose (1780674724.129440): -0.060 -0.175 0.002
[amcl-3] [WARN] [1780674733.193494483] [amcl]: Failed to transform initial pose in time (Lookup would require extrapolation into the future. ...)
[amcl-3] [INFO] [1780674733.193542816] [amcl]: Setting pose (1780674733.193541): -0.075 -0.095 0.000
[behavior_server-9] [INFO] [1780674733.327386671] [behavior_server]: Action server is inactive. Rejecting the goal.
  (x3)
```
Diagnosis: the Nav2 servers only activate after AMCL has a pose, and `course_runner` gave up
after 5 × 2 s retries. The "extrapolation" warning was cosmetic, caused by stamping
`/initialpose` with `now()`.

→ Commit `ec801a2`: `course_runner` now waits on `lifecycle_manager_navigation/is_active` (up to
90 s), sends `/initialpose` with a zero stamp, and retries goals 10× instead of 5×. **This has not
been tested on the robot yet.**

## 5. Current state of `nav2_params.yaml` (main @ f0715ab; rpp and astar identical)

| Area | Key values |
|---|---|
| Footprint | ±0.095 m square, padding 0.01 |
| AMCL | alpha1–4 0.1, alpha5 0.2, likelihood_field, laser range 0.15–6.0 m, sigma_hit 0.1, particles 500–3000, update_min_d 0.05, update_min_a 0.1 |
| Goal checker | xy 0.04, yaw 3.14 (ignored), stateful |
| Progress checker | 0.10 m in 15 s |
| RPP | desired_linear_vel 0.22; lookahead 0.25 (0.15–0.35, not velocity-scaled); rotate_to_heading 2.5 rad/s above 0.25 rad; max_angular_accel 20; min approach velocity 0.08 within 0.20 m; regulated min radius 0.25, min speed 0.12; cost scaling distance 0.12, gain 1.0; collision detection on; no reversing |
| Local costmap | 2 × 2 m rolling window in odom frame, 0.05 m resolution, obstacle layer (scan 0.15–2.0 m), inflation 0.25 / cost_scaling 10 |
| Global costmap | static + obstacle (scan 0.15–2.5 m) + inflation 0.30 / cost_scaling 10, track_unknown |
| Planner | NavFn `use_astar: true`, tolerance 0.05, allow_unknown false |
| Behaviour server | spin max 2.5, min 0.6, acc 4.0 rad/s(²) |
| Velocity smoother | OPEN_LOOP, max [0.25, 0, 2.8], accel [0.6, 0, 6.0] |
| Waypoint follower | WaitAtWaypoint 3500 ms, stop_on_failure false |

## 6. Open items / next steps

1. **Rebuild and retest `ec801a2`** on the robot:
   `colcon build --symlink-install --packages-select course_runner`. Expect to see
   `Nav2 active after X s` followed by an accepted spin.
2. **Measure turning with the new values.** Run these three side by side during a turn:
   `ros2 topic echo /cmd_vel_nav_raw --field angular.z`, `/cmd_vel --field angular.z`, and
   `/odom --field twist.twist.angular.z`. If `/cmd_vel` shows about 2.5 but `/odom` shows about
   ⅓ of that, consider a small `student_ws` node that boosts `angular.z` to compensate for the
   skid-steer before it reaches the driver. That needs a remap in both `planning.launch.py`
   files, which is a template change, so **ask the owner first**.
3. **Accuracy:** check the stop offset against the label with xy tolerance at 0.04 and AMCL
   alphas at 0.1. If the offset always points the same way, check where `base_link` actually is
   on the chassis.
4. **Measure the real course pitch** with a tape. The map assumes exactly 0.60 m. If it differs,
   change `cell_size` in `course_runner/config/course.yaml` (and consider re-cleaning the map).
5. **Staged tests still to do:** a full 4-square run timed against 3 min, a start bearing off by
   45° (`--bearing NE` while facing N), and an obstacle blocking a square on the route.
6. Run the same tests with `planning_astar` as well as `planning_rpp`.
7. The owner had uncommitted `cartographer.lua` edits (`use_landmarks = true`, lower
   `min_score`) in the original zip. These are now on `main` via the upload. Don't touch them
   unless asked.

## 7. Second session (10 Nov 2026): field tests and the final version

Section 5 above is the state at `f0715ab`. This section supersedes it.

### 7.1 What each version did on the robot (owner's reports)

| Commit | Change | Result |
|---|---|---|
| `57727f1` / `116bdf3` | grid routes through square centres, 0.30 m rounded corners, Spin before each leg; RPP lookahead 0.20, 0.22 m/s, rotate threshold 0.785, xy tolerance 0.04 | "the robot is able to run now"; legs still start at an angle |
| `c2bb18f`, `04af912` | 0.62 m squares, slower, smoother corners | spun in circles, AMCL lost (std 0.4-0.56 m); reverted |
| `a83d169`, `138f0bd`, `9a9a137` | align 0.08 rad, padding 0.02, LiDAR centring | X->V `follow_path` failed after 25 s, then hung waiting for `navigate_to_pose`; reverted |
| `d797775` | strict straight runs + 3 s server timeout | reverted to `116bdf3` before a run was reported |
| `7f6eeaf` | lookahead 0.15, 0.18 m/s, rotate threshold 0.5, xy tolerance 0.03 | "this run went almost perfectly well", but kept correcting its heading |
| `3023d14`, `dbacc72`, `479a950` | turn speeds cut to 80 % and lower, Spin braking 2.0 | over-turned, then hesitant: stalled and aborted spins, 48-55 s legs |
| `184341e` + `e47eaf1` | local costmap 0.025, lookahead 0.20; dead-end fallback | "a lot more hesitant ... skids a bit forward" |
| `c3c04d6` | `7f6eeaf` values + dead-end fallback | owner's persistent problems: zigzag on straights, stops short of the centre and moves on, corners oscillate into the walls |

Lowering the turn speeds below the `116bdf3` values made things worse every time: near tyre-scrub
the skid-steer stalls, Spin aborts and the run hesitates. Keep turn speeds at the `116bdf3` values.

### 7.2 Findings
- **Arcs cannot be tracked by this base.** It turns at about 1/3-1/2 of the commanded rate, so to
  follow a 0.30 m corner arc RPP has to lag 40-55 deg behind it. That crosses
  `rotate_to_heading_min_angle`, RPP stops to turn at full speed with no slow-down, overshoots,
  and repeats. Simulated with `7f6eeaf` values: 8-13 in-place turns per corner and up to 16.5 cm
  off the path; on the map a robot 11 cm wide of a 0.30 m arc touches the wall.
- **Zigzag on straights:** lookahead 0.15 turns each 2-3 cm AMCL correction into a sharp steer,
  and threshold 0.5 rad lets those become stop-and-turn events.
- **Stopping short:** the goal checker succeeds on entering the xy tolerance circle, so a path
  ending on the centre always leaves the robot that far short (3-4 cm), plus coasting and AMCL.
- **Map:** the owner's original rough map and `rb3302_map.pgm` agree to within half a pixel
  (2.5 cm); outer size 3.65 x 2.45 m matches 6 x 4 squares of 0.60 m. Not a grid problem.

### 7.3 Final version
- `course_runner` drives each leg as **straight runs** between square centres. At every square
  where the route turns, and at each goal, the robot stops on the centre and **turns on the spot
  with Spin** (closed loop on odometry, slows before the target), then RPP follows a dead-straight
  line. `corner_radius: 0.0` selects this; `0.30` brings back rounded corners.
- Every path ends `goal_extension` (0.03 m) past the centre, so with `xy_goal_tolerance` 0.04 the
  robot stops on the centre instead of 4 cm short.
- If the robot still ends more than `centre_tolerance` (0.05 m) from a goal centre, or the leg
  failed but the robot is inside the goal square, it turns towards the centre and drives onto it
  once before the 3.5 s stop, instead of moving on.
- A* `turn_penalty` 1.0: a corner now costs a stop and a turn, about one square of driving.
- `nav2_params.yaml` (rpp and astar identical) = `116bdf3` values except `desired_linear_vel`
  0.18 from `7f6eeaf`. RPP lookahead 0.20 (0.15-0.35), rotate threshold 0.785 at 1.5 rad/s,
  xy tolerance 0.04; Spin 1.8 / 0.6 / 4.0; smoother angular +/-2.8.
- Offline checks: all 552 routes split into compass-straight runs; on the map the robot keeps
  11 cm from the walls 4.5 cm off a straight line and 6.6 cm when turning on the spot 5 cm off a
  centre. Estimated run times 110-178 s; if a long mission gets close to 3 min, raise
  `desired_linear_vel` to 0.22 (the `116bdf3` value).

### 7.4 Owner's run of `6749477`, and smooth corners
> "The robot oscillates a lot when it comes to turning to reach the goal. I want it to smoothly
> turn into the the center of the grids ... the moment it enters the starting of the grid, the
> robot should be turning in already." Also: the recovery turn after over-rotating is too much,
> but it must not stop too much.

Stopping on each corner centre to turn made the turn start only at the centre (or past it after
the forward skid), so from E the robot went into J near its West wall. Rounded corners are back
(`corner_radius` 0.30: the arc runs from the edge the square is entered by to the edge it is left
by), with RPP values chosen in a simulation of the under-turning robot on the map (E-D-J, W-V-U-O,
I-C-D-J; turn ratio 0.35-0.7, lag 0.1-0.3 s, +/-8 deg after Spin, AMCL jumps):

| Parameter | `6749477` | Now | Why |
|---|---|---|---|
| `use_velocity_scaled_lookahead_dist` | false | true | lookahead = speed x 1.5 s: ~0.27 m on straights, 0.12 m on arcs |
| `min_lookahead_dist` | 0.15 | 0.12 | short enough to steer the 0.30 m arc |
| `rotate_to_heading_min_angle` | 0.785 | 1.0 | on an arc the robot lags 40-55 deg; below that RPP stopped to turn |
| `rotate_to_heading_angular_vel` | 1.5 | 1.2 | gentler turn back after an over-rotation (1.0 and 0.8 stalled) |
| `regulated_linear_scaling_min_speed` | 0.12 | 0.08 | arcs driven at 0.08 m/s: 6.6 cm worst off the arc instead of 9 cm |
| Spin `rotational_acc_lim` | 4.0 | 3.0 | brakes from ~31 deg before the target: less over-rotation |

Simulated: no stop-and-turn on any corner (`7f6eeaf` values: ~3 per corner), closest wall 8 cm,
turning starts as the robot enters the corner square, 1.8 steering reversals on a 1.8 m straight
(`7f6eeaf`: 7.1), whole runs 79-162 s. `centre_on` now backs straight onto a goal centre the
robot stopped past (Nav2 `backup`), instead of turning round. `corner_radius: 0.0` keeps the
straight-run mode.

### 7.5 Owner's run of `0cbd2e4`: stuck turning
> "this execution is way worse, the robot just gets stuck at turning"

The six Nav2 values changed in `0cbd2e4` came from a simulation that modelled turning as a fixed
fraction of the command and a clean speed reading. The real robot needs a minimum turn command to
beat tyre scrub, and the driver's speed (`dist_centre` difference at 100 Hz) is jumpy. So:
- `regulated_linear_scaling_min_speed` 0.08: the arc turn command (speed x curvature) was too
  small to turn the robot.
- `use_velocity_scaled_lookahead_dist` true / `min_lookahead_dist` 0.12: the lookahead followed
  the jumpy speed and sat at 0.12 m whenever the robot was slow.
- `rotate_to_heading_angular_vel` 1.2, `rotate_to_heading_min_angle` 1.0, Spin
  `rotational_acc_lim` 3.0: weaker and later turning, and Spin ends turns at low command.

All six are back to `6749477` (= `116bdf3` values except 0.18 m/s from `7f6eeaf`); the corner
arcs (`corner_radius` 0.30) stay, as in `116bdf3` and `7f6eeaf`. `BACKUP_SPEED` 0.05 -> 0.08.
**Lesson: do not tune turn speeds, arc speed or Spin braking below the `116bdf3` values, and do
not pick speed values from a simulation without tyre scrub.**
