# RB3302 Final Project 

This repository contains two ROS 2 workspaces for the RB3302 final project:

- `robot_ws` contains the robot-side nodes used by the lab setup to activate the 2D Lidar, IMU, robot model, and transform definitions.
- `student_ws` contains the mapping and planning packages used in this project.

Both workspaces are located inside `~/rb3302_final_proj_code`.

Packages in this workspace:

```text
mapping_cartographer  - Cartographer algorithm
planning_astar        - Planning with A*
planning_rpp          - Planning with Regulated Pure Pursuit (RPP)
course_runner         - Graded run: start square + bearing, visit 4 squares A-X
```

## Initialise the Robot

Run this from `robot_ws` before mapping or planning.
Keep this terminal running.

Build once:

```bash
cd ~/rb3302_final_proj_code/robot_ws
colcon build --symlink-install
source ~/rb3302_final_proj_code/robot_ws/install/setup.bash
```

Start the robot-side nodes:

```bash
source ~/rb3302_final_proj_code/robot_ws/install/setup.bash
ros2 launch robot_init robot.launch.py
```

Manual driving:

```bash
source ~/rb3302_final_proj_code/robot_ws/install/setup.bash
ros2 run waverover_base keyboard_teleop
```

On your own computer:

```bash
rviz2
```

![RViz](images/laser_rviz2.png)

## Build

```bash
cd ~/rb3302_final_proj_code/student_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Mapping with Cartographer

Config file:

```text
src/mapping_cartographer/config/cartographer.lua
```

Start mapping:

```bash
ros2 launch mapping_cartographer mapping.launch.py
```

Drive the robot around while Cartographer creates the map.

## Save the Map

Keep mapping running, then open another terminal:

```bash 

mkdir -p ~/rb3302_final_proj_code/student_ws/maps

ros2 run nav2_map_server map_saver_cli -f  ~/rb3302_final_proj_code/student_ws/maps/rb3302_map

```

Save maps here:

```text
~/rb3302_final_proj_code/student_ws/maps/rb3302_map.yaml
~/rb3302_final_proj_code/student_ws/maps/rb3302_map.pgm
```

## Before Using A* or Regulated Pure Pursuit (RPP)

Both planning options use Adaptive Monte Carlo Localisation (AMCL) to localise the robot on the saved map.
After starting either planning package, set the robot's starting pose in RViz with `2D Pose Estimate`, then send a goal.

![2D Pose Estimate button](https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcTZRV3ugXlOB_iR_gDu-X5ThKJhgR311uwVFu6B63culC722Dts3-WeHmM&s=10)

## Planning with A*

Config file:

```text
src/planning_astar/config/nav2_params.yaml
```

Start planning:

```bash
ros2 launch planning_astar planning.launch.py
```

## Planning with Regulated Pure Pursuit (RPP)

Config file:

```text
src/planning_rpp/config/nav2_params.yaml
```

Start planning:

```bash
ros2 launch planning_rpp planning.launch.py
```

## Graded Run (course_runner)

With `robot_init` and one planning package running, give the start square,
rough bearing (N/E/S/W, +/-45 deg) and the 4 squares to visit:

```bash
source ~/rb3302_final_proj_code/student_ws/install/setup.bash
ros2 run course_runner run_course --start S --bearing N --goals A J Q X
```

The node sets the AMCL initial pose from the start square (no RViz click
needed), spins once in place to converge AMCL, then drives to each square and
stops 3.5 s on it.

By default (`--planner grid`) each leg is planned with A* over the 24 squares
(walls from the L08b brief, plus squares the global costmap shows as blocked).
The route goes through square centres, with a smooth turn (steering ramps
in and out, starting 0.30 m before the centre) in any square it only passes
through, and goes straight in to the goal centre. The path is sent straight
to the RPP controller (`follow_path`) and shown on `/plan`. Before each leg the
robot turns on the spot until it is within ~2 deg of the compass axis, and
AMCL is refined with no-motion updates during each 3.5 s stop. Squares are
spaced 0.62 m (measured course) about the course centre; `cell_size` in
`course.yaml` sets this. If a leg fails it is
replanned, and the last resort is Nav2's planner (`navigate_to_pose`). If you
know which square holds the obstacle, add `--avoid P`. To use the previous
behaviour (Nav2 waypoint follower with NavFn A*), add `--planner nav2`.

Check the square letters line up with the map before a run (RViz on the
laptop, `WaypointMarkers` display, topic `/waypoint_markers`):

```bash
ros2 run course_runner run_course --start S --bearing N --goals A J Q X --dry-run
```

Grid geometry for `maps/rb3302_map.yaml` (North = +x, West = +y, square A
centre at (1.725, 2.905), 0.60 m squares) is in
`src/course_runner/config/course.yaml`. Override with
`--ros-args --params-file <file>` if the map is replaced.
