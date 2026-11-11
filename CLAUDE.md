# RB3302 Final Project — instructions for Claude

These are the owner's (Wilton's) standing rules for this repository. Follow them in every session.

## Role and hard boundaries
- Your role is to finetune the existing ROS 2 (Jazzy) infrastructure for the RB3302 project.
- **Never edit anything under `robot_ws/`.** Only edit code under `student_ws/`.
- `student_ws/maps/rb3302_map.yaml` + `rb3302_map.pgm` is the cleaned map used by RViz (on the
  owner's laptop) and by the Nav2 map_server. Always refer to it by exactly that name
  (never `_clean`).
- Ask for confirmation before changing any part of the original course template beyond
  parameter values — e.g. launch files, package structure, RViz configs. Parameter-only changes
  to `nav2_params.yaml` are the intended tuning surface (the README names that file).
- No external libraries. Use only what ships with ROS 2 / Nav2 / the Python stdlib. If something
  extra is ever genuinely needed, justify it first.
- All code must be consistent with the existing code style (ament_python packages,
  `declare_parameter`, etc.).

## Method (every answer)
1. Restate the request in one line.
2. List which project files you are relying on.
3. Then answer.

## Format
- Code in fenced blocks with the language tag.
- No preamble, no summary of what you are about to do.

## Escalation
- If the request is ambiguous, ask exactly one clarifying question, then wait.
- If two project files conflict, name both and stop.

## Reference material
- Course briefs: `L08b_Project.pdf` (course + scoring) and `L08c_RB3302_Final_Project.pdf`
  (packages + steps). They are NOT in this repo (it is public); the owner keeps them locally.
- Current tuning and the reason for each value: the comments in
  `student_ws/src/planning_rpp/config/nav2_params.yaml` (the `planning_astar` copy is identical).
  Read them before changing anything. Earlier decisions are in the git history.
- No separate docs: tuning changes and their explanations go straight into `nav2_params.yaml`
  (rpp + astar), never into a separate document.

## Git
- Work on a branch, open a PR into `main`; the owner merges.
- Do not commit `__pycache__/`, `.idea/`, `build/`, `install/`, `log/`.
- `robot_ws/src/rplidar_ros` is a git submodule whose contents are not in this repo; leave it.
