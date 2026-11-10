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

Build once after pulling (adds the `odom_sign` executable):

```bash
cd ~/rb3302_final_proj_code/student_ws
colcon build --symlink-install --packages-select course_runner planning_rpp planning_astar
```

On the robot, after `robot_init`, start these in order, each in its own terminal:

```bash
source ~/rb3302_final_proj_code/student_ws/install/setup.bash
ros2 run course_runner odom_sign
```

```bash
source ~/rb3302_final_proj_code/student_ws/install/setup.bash
ros2 launch planning_rpp planning.launch.py raw_odom_topic:=/odom/raw_signed
```

```bash
source ~/rb3302_final_proj_code/student_ws/install/setup.bash
ros2 run course_runner run_course --start S --bearing N --goals A J Q X
```

`odom_sign` puts the direction back on the wheel speed. The board reports it
without a sign, so reversing and turning on the spot read as driving forward,
and the EKF slid the robot across the map while it was really turning in place.
`raw_odom_topic:=/odom/raw_signed` is an existing argument of both planning
launch files, so no launch file is changed. `planning_astar` works the same way.

The runner sets the AMCL initial pose from the start square (no RViz click
needed) and then localises without moving: it asks AMCL for stationary updates
until 95 % of the LiDAR points land on mapped walls (usually a few seconds),
then clears the costmaps. `--localise spin` brings back the 360 deg turn and
`--localise none` skips both. Each leg is an A* route over the 24 squares
through the square centres (see `student_ws/README.md` for the details: corner
arcs, stopping on the centre, replanning around the obstacle).

Check the square letters line up with the map before a run (RViz on the
laptop, `WaypointMarkers` display, topic `/waypoint_markers`):

```bash
ros2 run course_runner run_course --start S --bearing N --goals A J Q X --dry-run
```

Grid geometry for `maps/rb3302_map.yaml` (+x East, +y North, (0, 0) at the
south-west corner of the course, 0.5825 m squares, square A centre at
(0.291, 2.039)) is in `src/course_runner/config/course.yaml` and is also the
default in `run_course.py`. Override with `--ros-args --params-file <file>` if
the map is replaced.
