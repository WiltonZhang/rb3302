# RB3302 — tuning review and root causes (11 Nov 2026)

A review of every commit from `2267fd7` to `ee11f8c`: what went wrong in each run, which values
changed, and what that did. It also compares the setup with two friends' setups (friend A: the
code that runs on his robot, with its own runner; friend B: the RPP `nav2_params` that was pasted
in). It then lists what this branch (`odom-fix-5825-map`) changes. Nothing here has been run on the
robot yet. Section 8 lists what is checked offline and what is not.

---

## 0. Summary

Most of the bad runs were **not** caused by the turning values themselves. Three faults sat
underneath, and every value change either hid them or made them worse.

1. **The wheel odometry has no sign.** The driver works out `/odom/raw` linear.x from the board's
   wheel distance, which only counts up. Reversing reads as driving forward, and so does turning
   on the spot (one side runs backwards). The EKF in the planning launch adds that "forward"
   speed along the IMU heading. So **every turn on the spot drives the odometry round a circle**
   of radius (wheel speed ÷ real turn rate), about 20–40 cm on this base. A 90° turn moves the
   estimate about 30–50 cm and a 180° turn twice that. This is exactly the
   *"spins on the spot while RViz shows it sliding diagonally across the map"* report (`04af912`)
   and *"moves in circles while RViz showed it flying away"* (`cae339e`, which backed up with
   Nav2 `backup`). Friend A found the same fault and fixed it with a small node. Every commit
   that went badly wrong added turns on the spot or reversing: `c2bb18f` (re-spin until within
   2°), `04af912`, `9a9a137` (backup + 90° turns), `6749477` (Spin at every corner), and
   `0cbd2e4`/`cae339e` (backup).
2. **AMCL was told to trust that odometry.** alpha1–4 went 0.8 → 0.2 → 0.1 on 6 Nov. At 0.1
   the particle cloud spreads so little while the robot moves that the LiDAR cannot pull it back
   when the odometry jumps 30 cm, so AMCL follows the odometry and gets lost (std 0.4–0.56 m).
   `c2bb18f` made this worse by asking for stationary AMCL updates at every stop, which squeezes
   the cloud even tighter just before the next turn.
3. **The obstacle layers were blind.** Neither costmap's `scan` source set
   `max_obstacle_height`. Nav2's default for a source is 0.0 m and the LiDAR sits 9 cm above
   `base_link`, so every scan point was thrown away (friend A hit the same thing: *"every lidar
   point was thrown away, so the box was invisible"*). The local costmap was empty, RPP's
   collision check had nothing to check, and an obstacle box could never be routed around. The
   original zip's params did set it (2.0); it was lost when the repo started from the bare template.

Two smaller faults:

4. **Spin stalls on its last few degrees.** Spin ramps the turn command down to
   `min_rotational_vel` near the target. At 0.6 rad/s that is below the ~0.8–1.0 rad/s this base
   needs to beat tyre scrub (`479a950` stalled at 0.8), so a turn could stop a few degrees short
   and sit there until its time allowance ran out. That is a "stop at the turn" with no failure
   message.
5. **The map was 3 % too big, and AMCL reads it half a cell low.** The cleaned map puts walls on
   a 0.60 m pitch. Friend A measured 58.25 cm centre to centre, and your rough LiDAR map fits
   57–58.6 cm (section 6). Separately, AMCL looks a map cell up by rounding to the nearest cell
   corner, so its poses come out half a cell (2.4–2.5 cm) low in x and y. Friend A measured 2.5 cm
   and aims every goal that much low.

The base itself turns at about ⅓–½ of the commanded rate and stalls below about 1 rad/s. That
is why no turn value worked well. Values below `116bdf3`'s stalled (`3023d14`–`479a950`,
`0cbd2e4`), values above overshot (`d4c9121`), and arcs cannot be tracked closely (`HANDOFF.md`
7.2). This branch keeps `116bdf3`'s turn values and fixes faults 1–5 instead.

### What this branch changes

| Area | Change | Fixes |
|---|---|---|
| `course_runner/odom_sign.py` (new) | `/odom/raw` → `/odom/raw_signed`: speed gets the sign of `/cmd_vel`, 0 when turning on the spot, capped at 1.5× the commanded speed | 1 |
| planning launch | run with the **existing** argument `raw_odom_topic:=/odom/raw_signed` (no launch file edited) | 1 |
| AMCL | alpha 0.4/0.3/0.4/0.4, z_hit 0.8 / z_rand 0.2, update_min_a 0.05 | 2 |
| costmaps | `max_obstacle_height: 2.0` in both scan sources | 3 |
| costmaps | `robot_radius: 0.14` disc instead of the 19 × 19 cm square | dead-end wall hits |
| RPP | cost-regulated slow-down off (both friends run it off) | hesitation once the walls are visible |
| Spin | `min_rotational_vel` 0.6 → 1.0 | 4 |
| goal checker + runner | xy 0.03, paths end 3 cm past the centre, drive forward onto the centre if > 4 cm off | stopping short |
| runner | stand-still localisation (no 360° spin), costmaps cleared after it | your problem 6, fault 2 |
| runner | watch the costmap 1.2 m ahead while driving; replan when a square becomes blocked; walls-only fallback (`e47eaf1`) | obstacle task |
| runner | `aim_offset_x/y` −0.0243 | fault 5 (AMCL half cell) |
| map | friend A's 58.25 cm grid and frame, redrawn in your map's exact style | fault 5 (scale) |
| runner | `wait_for_server` times out (10 s) instead of hanging (`d797775`'s idea) | the run that hung forever |
| params | global costmap `initial_transform_timeout: 600` | Nav2 giving up if the run starts > 60 s after the launch |
| params | desired speed 0.18 and xy 0.03 from `7f6eeaf` (your best run) | — |

Everything else is `116bdf3` (`= ee11f8c`, current `main`).

---

## 1. How to test this branch (in this order)

1. **Confirm fault 1 on your robot (one minute).** With only `robot_init` and teleop running,
   turn on the spot with the keyboard, then drive backwards, while watching:

   ```bash
   ros2 topic echo /odom/raw --field twist.twist.linear.x
   ```

   Positive numbers while turning on the spot or reversing confirm it. (If the numbers are
   already 0 while turning and negative while reversing, `odom_sign` is harmless, but tell me,
   because then the circling has a different cause.)
2. **Build and start** as in `student_ws/README.md` → *Graded Run*: `odom_sign`, then
   `planning_rpp` with `raw_odom_topic:=/odom/raw_signed`, then a `--dry-run`. In RViz check that
   the letters sit in the right squares. The map now has North up and East to the right, like the
   brief. Then `ros2 topic hz /odom/raw_signed` should show about 100 Hz.
3. **Odometry check in RViz:** teleop a turn on the spot. The robot model must stay where it is.
   Before this change it slid round a circle.
4. **Obstacle layer check:** the `LocalCostmap` display must now show the walls around the robot.
   On `main` it was empty.
5. **Short run** `--start S --bearing N --goals M N`, the first field test. Look for no slide
   during turns, a `localised standing still: ... scan on walls 9x %` line, and the `stopped NN mm
   from the centre` lines.
6. **Dead end:** a mission through J (e.g. `--start D --bearing S --goals J Q`). The robot should
   stop on J's centre, then turn there.
7. **Obstacle:** put the box in a square on the route (e.g. V for `X → S`). Expect
   `square V became blocked ahead, replanning around it`.
8. **Full 4-square mission** timed against 3 min, and once more with `--bearing` 45° off.

Please send the full `course_runner` output and the `stopped ... from the centre` lines. Also note
anything RViz shows during turns.

---

## 2. Commit by commit

Results are your reports (from `HANDOFF.md` §4 and §7 and the commit messages). *Why* is the
explanation as now understood. One conflict: you said `c2bb18f` and earlier were best, and
`433ebfd` calls `c2bb18f` "the last version that worked", but `417a300` and `cf607e4` report it
circling. Either way, its extra Spins and stationary updates are the risky part.

| Commit | What changed | What happened | Why |
|---|---|---|---|
| `2267fd7` (6 Nov) | first tuning: 19 × 19 cm footprint, inflation 0.30/0.25, xy 0.06, RPP 0.22 m/s, lookahead 0.25, AMCL alpha 0.8 → 0.2, runner = FollowWaypoints + 360° spin | S→M stopped 5 cm off, 20° off; M→N turn stalled ~2 min ("Failed to make progress"); the spin took 18 s | RPP `max_angular_accel` 2.0 caps a turn command at 0.1 rad/s from standstill, which never beats tyre scrub; the base turns at ~⅓ of the command |
| `f718989` | `max_angular_accel` 20, rotate 1.8 rad/s above 0.4 rad, smoother ±2.0, Spin 1.8, cost/approach slow-downs reduced, **alpha 0.1**, xy 0.04 | "turning really slowly" | on arcs RPP commands only ω = v·κ (0.3–0.6 rad/s), ~0.1–0.2 actual. alpha 0.1 begins trusting the unsigned odometry |
| `d4c9121` | rotate threshold 0.25, rotate 2.5, smoother ±2.8, Spin 2.5/0.6/4.0, min radius 0.25 | overshot every turn (as reported for `57727f1`) | 2.5 rad/s with no slow-down overshoots; a 0.25 rad threshold turns every small AMCL correction into a stop and spin |
| `ec801a2` | wait for Nav2 active; zero-stamp start pose | fixed "spin goal rejected" | Nav2 activates only after AMCL has a pose |
| `57727f1` / `116bdf3` | grid route through centres with 0.30 m arcs; Spin before each leg; threshold 0.785, rotate 1.5, Spin 1.8, decel −10, lookahead 0.20, min radius 0.45 | "the robot is able to run now"; legs start at an angle | the best base: few turns on the spot, and on arcs at v > 0 both wheel sides go forward, so the unsigned speed is right |
| `c2bb18f` | 0.62 m squares; smoother corners; stationary AMCL updates after the spin and at every stop; re-spin until within 0.04 rad (3 tries); 0.15 m/s; rotate 1.2; Spin 1.2/3.0; `update_min_a` 0.05 | turned in circles; AMCL lost (std 0.4–0.56 m); "spun on the spot until Spin timed out" | many extra turns on the spot (faults 1 + 2); stationary updates squeezed the cloud before each turn; Spin max 1.2 and acc 3.0 gave a long low-command tail that stalls (fault 4); 0.62 m is the wrong direction (real ≈ 0.5825) |
| `417a300`, `b6cabbc` | revert `c2bb18f` | — | — |
| `04af912` | `c2bb18f` again with calmer turns; 0.16 m/s | "spin on the spot while RViz showed it sliding diagonally across the map" | the textbook sign of fault 1: forward speed integrated along a turning heading |
| `433ebfd`, `cf607e4` | restore `c2bb18f`, then `116bdf3` | — | — |
| `a83d169` | `align_min_angle` 0.20 → 0.08 | (same batch as below) | more Spins, so more phantom slide |
| `138f0bd` | footprint padding 0.02 | (same batch) | — |
| `9a9a137` | LiDAR centring with `drive_on_heading` / `backup` and 90° turns | X→V `follow_path` failed after 25 s, then the run hung on `navigate_to_pose` | reversing reads as forward (fault 1, double the error); `wait_for_server` had no timeout |
| `d797775` | strict straight runs; 3 s server timeout | reverted before a run | — |
| `328aefe` | back to `116bdf3` | — | — |
| `7f6eeaf` | lookahead 0.15 (0.10–0.25), 0.18 m/s, threshold 0.5, xy 0.03 | **"this run went almost perfectly well"**, but kept correcting its heading | tighter end of leg; the 0.15 m lookahead turns each 2–3 cm AMCL jump into a sharp steer (zigzag) |
| `3023d14` | turn caps to 80 % (1.2 / 1.44 / 2.24) | over-turned, then hesitant | closer to the stall |
| `dbacc72` | threshold 0.785, rotate 1.0, Spin acc 2.0 | legs ending 15–39° off; 48–55 s per 6-square route | rotate command at the stall edge; Spin braking from ~30° means a long tail below the stall |
| `479a950` | another 20 % lower (0.8 / 1.15 / 1.8) | stalled and aborted spins | below the stall |
| `184341e` | local costmap 0.025 m; lookahead back to 0.20 | "a lot more hesitant ... skids a bit forward" | low turn values still in place |
| `e47eaf1` | walls-only fallback when blocked squares leave no route | — | sensible; kept on this branch |
| `c3c04d6` | `7f6eeaf` values + fallback | zigzag on straights; stops short of the centre and moves on; corners oscillate into walls | 0.15 lookahead; the stateful goal checker succeeds 3–4 cm short; the base cannot follow 0.30 m arcs |
| `6749477` | straight runs with Spin at every corner and goal; path ends 3 cm past the centre; drive onto the centre if > 5 cm off | "oscillates a lot when it comes to turning to reach the goal"; turns start at or after the centre; went into J near its west wall | a turn on the spot at **every** corner, each sliding the odometry 30–50 cm (fault 1); the believed position jumps, so RPP steers hard after each turn |
| `0cbd2e4` | arcs start at the square edge; centre by backing up; RPP velocity-scaled lookahead (min 0.12), threshold 1.0, rotate 1.2, min speed 0.08, Spin acc 3.0 | "way worse, the robot just gets stuck at turning" | at 0.08 m/s the arc command ω = v·κ is below the stall; the velocity-scaled lookahead follows the jumpy `/odom` speed |
| `cae339e` | turn values restored; backup speed 0.08 | "move in circles while RViz showed it flying away" | Nav2 `backup` reads as driving forward (fault 1) |
| `ee11f8c` | revert to `116bdf3` | — | — |

---

## 3. What each value does on this robot

| Value | Use | Too low | Too high |
|---|---|---|---|
| RPP `rotate_to_heading_angular_vel` | **1.5** | 1.2 / 1.0 / 0.8 stalled near tyre scrub | 2.5 overshot every turn (RPP does not slow down near the heading) |
| RPP `rotate_to_heading_min_angle` | **0.785** | 0.25 / 0.5: AMCL jitter becomes stop-and-spin | 1.0: turns start too late, robot falls off the arc |
| RPP `max_angular_accel` | **20** | 2.0 (and the 0.8 default): turn never starts | — (smoothing is the velocity smoother's job) |
| RPP `lookahead_dist` | **0.20**, fixed | 0.15: zigzag on straights | 0.6 (template): cuts corners into walls. Velocity-scaled follows the jumpy speed |
| RPP `regulated_linear_scaling_min_speed` | **0.12** | 0.08: the arc turn command is too small, stuck turning | — |
| RPP `desired_linear_vel` | **0.18** (0.22 if time is short) | — | 0.22 is fine; `c2bb18f` thought the run was "faster than needed" |
| goal `xy_goal_tolerance` | **0.03** + 3 cm path extension | — (≤ 0.02 can hunt) | 0.06 / 0.04 stop that far short |
| Spin `max_rotational_vel` | **1.8** | 1.2 makes the slow tail longer | 2.5 overshoots |
| Spin `min_rotational_vel` | **1.0** | 0.6 stalls the last few degrees | — |
| Spin `rotational_acc_lim` | **4.0** | 2.0 / 3.0 brake from further out, so a longer stall-prone tail | — |
| AMCL alpha1–4 | **0.3–0.4** | 0.1 follows the odometry's phantom slides and gets lost | 0.8 (friend A) also works, with a bit more pose jitter |
| AMCL `update_min_a` | **0.05** | — | 0.1 lets the heading lag through a 90° turn |
| scan `max_obstacle_height` | **2.0** | not set (0.0): the layer is blind | — |
| costmap footprint | **robot_radius 0.14** | the 19 × 19 cm square let the corners sweep into walls when turning | > 0.16 starts to block the 48.5 cm drawn gaps |
| velocity smoother angular | **±2.8, accel 6, decel −10** | lower stalls | the driver clamps at 3.0 |

---

## 4. Values side by side

`116bdf3` is current `main`. Friend A pairs his values with his runner (§5) and `odom_sign`.
Friend B's values came from the paste, which lost its indentation, so where each key belongs is
reconstructed.

| Key | `116bdf3` | friend A | friend B | this branch |
|---|---|---|---|---|
| AMCL alpha1/2/3/4 | 0.1/0.1/0.1/0.1 | 0.8 ×4 | 0.4/0.3/0.4/0.4 | **0.4/0.3/0.4/0.4** |
| AMCL z_hit / z_rand / sigma_hit | 0.6 / 0.4 / 0.1 | 0.8 / 0.2 / 0.1 | 0.9 / 0.1 / 0.2 | **0.8 / 0.2** / 0.1 |
| AMCL update_min_d / _a | 0.05 / 0.1 | 0.05 / 0.05 | 0.05 / 0.03 | 0.05 / **0.05** |
| AMCL max_beams / likelihood max dist | 90 / 1.0 | 100 / 0.5 | 60 / 2.0 | 90 / 1.0 |
| controller_frequency | 20 | 20 | 5 | 20 |
| progress checker | 0.10 m in 15 s | 600 s (off; own 60 s timeout) | 0.001 m in 15 s | 0.10 m in 15 s |
| goal checker | Simple, xy 0.04 | Simple, xy 0.02 | Position, xy 0.04 | Simple, **xy 0.03** |
| RPP desired_linear_vel | 0.22 | 0.30 | 0.27 | **0.18** |
| RPP lookahead | 0.20 fixed | 0.25, velocity-scaled 0.15–0.4 (1.2 s) | 0.30 fixed | 0.20 fixed |
| RPP rotate vel / threshold | 1.5 / 0.785 | 2.5 / 0.3 | 1.5 / 1.041 | 1.5 / 0.785 |
| RPP max_angular_accel | 20 | 20 | 1.5 | 20 |
| RPP min radius / min speed | 0.45 / 0.12 | 0.9 / 0.15 | 0.9 / 0.05 | 0.45 / 0.12 |
| RPP approach dist / min speed | 0.20 / 0.08 | 0.6 / 0.10 | 0.6 / 0.05 | 0.20 / 0.08 |
| RPP cost-regulated slow-down | on | off | off | **off** |
| footprint | 19 × 19 cm square + 0.01 | robot_radius 0.13 | 14 × 12 cm square (smaller than the robot) | **robot_radius 0.14** |
| local costmap | 2 × 2 m, 0.05 | 3 × 3 m, 0.05 | 2 × 2 m, 0.04 | 2 × 2 m, 0.05 |
| inflation (local / global) | 0.25 / 0.30, scaling 10 | defaults (0.55, 10) | 0.16, scaling 0.6 | 0.25 / 0.30, scaling 10 |
| scan max_obstacle_height | **not set** | 2.0 | 0.4 / 0.5 | **2.0** |
| global initial_transform_timeout | default 60 s | 600 s | default | **600 s** |
| Spin max / min / acc | 1.8 / 0.6 / 4.0 | defaults | 1.5 / 1.5 / 1.5 (constant speed) | 1.8 / **1.0** / 4.0 |
| smoother max angular / accel / decel | 2.8 / 6 / −10 | 2.5 / 20 / −20 | 1.8 / 1.5 / −1.5 | 2.8 / 6 / −10 |
| smoother linear accel | 0.6 | 0.5 | 0.2 | 0.6 |

What is worth taking from the friends:

- **Both** run AMCL with much higher alphas and RPP's cost slow-down off, and both set
  `max_obstacle_height`. Taken.
- **Friend A**: `odom_sign`, a disc footprint the size of the turning robot, stand-still
  localisation, the costmap clear, the half-cell aim offset and `initial_transform_timeout`. Taken.
  His 2.5 rad/s rotate with a 0.3 rad threshold means many short turns on the spot. That only
  works with the odometry fixed and fits his NavFn paths, so it was not taken (it is what
  `d4c9121` overshot with).
- **Friend B**: Spin at a constant 1.5 rad/s (min = max) also avoids the stall; this branch uses
  min 1.0 to keep Spin's braking. His 5 Hz controller and accel 1.5 suit his runner, not
  `116bdf3`'s values. His footprint is smaller than the robot: do not copy it.

---

## 5. course_runner and friend A's runner

| | `116bdf3` | friend A | this branch |
|---|---|---|---|
| start localisation | 360° Spin (18–25 s), odometry slides during it | stand still, stationary AMCL updates until 95 % of scan points are within 10 cm of a wall (max 50), clear the global costmap, wait for Enter | stand still, same check (max 50 or 30 s), clear both costmaps, no Enter. `--localise spin` / `none` are still there |
| odometry | raw (unsigned) | `odom_sign` → `raw_odom_topic` | `odom_sign` (+ cap at 1.5× command) |
| planning | own grid A* with rounded corners → `follow_path` | `/goal_pose` per square; Nav2's tree replans with NavFn every second | grid kept |
| obstacle | `--avoid`, costmap check at the start of each leg, replan after a failed leg | Nav2 replanning on a global costmap that can see the box | + costmap watched 1.2 m ahead while driving, replan on 2 sightings; walls-only fallback; obstacle layer fixed |
| arriving | `follow_path` success (xy 0.04) | AMCL within 10 cm and stopped for 1 s (xy 0.02) | path ends 3 cm past the centre, xy 0.03, then drive forward onto the centre if > 4 cm off |
| AMCL bias | — | aims 2.5 cm low in x and y | `aim_offset_x/y` −0.0243 |
| dead ends (H, J, K, N) | Spin wherever it stopped | line the rear up with the exit and reverse out one square | your choice: centre first (forward only), then Spin there; the disc footprint lets Spin's collision check refuse a turn that would hit a wall |
| hangs | `wait_for_server` forever | 60 s per square | 10 s server timeout, 15 s progress checker |
| map | 0.60 m grid, +x North | 0.5825 m grid, +x East | 0.5825 m grid, friend A's frame, your drawing style |

The legacy `mission_control` package in `rb3302_final_proj_code_original.zip` is not used. Its
anchor defaults assume yet another orientation (North = −x).

---

## 6. Map

- **Scale.** Friend A measured 58.25 cm centre to centre. Your rough Cartographer map
  (`Downloads/rb3302_map.pgm`, before cleaning) has inner spans of 2.25–2.35 m across 4 squares
  and 3.40–3.50 m across 6. With ~2 cm walls that is a 57.0–58.6 cm pitch; 60 cm does not fit the
  6-square span and 62 cm (`c2bb18f`'s note) fits neither. **Please re-measure across several
  squares, wall centre to wall centre.**
- **New `rb3302_map.pgm` / `.yaml`.** Friend A's map is 84 × 60 px at 0.04854167 m/px,
  `origin [-0.29125, -0.29125, 0]`, +x East, +y North, (0, 0) at the course's south-west corner.
  It is redrawn with the same rules as your old map: every wall 2 px thick, straddling its grid
  line, 1 px past each end; free inside, unknown outside; same YAML fields. The drawing code
  reproduces your old `rb3302_map.pgm` pixel for pixel, and the new map differs from friend A's in
  24 pixels, all at rounded wall ends. Checked: all 24 square centres are free, all 38
  neighbouring pairs match the brief's wall list, and each centre is 0.243 m from the drawn walls.
- **Frame change.** Square A is at (0.29125, 2.03875); bearing N is map yaw +π/2 and E is 0. The
  defaults in `run_course.py` and `course.yaml` match. RViz now shows North up.
- `docs/claude-handoff/rb3302_map_squares.png` is regenerated. The green dot is the map centre;
  the red cross is where the runner aims (−2.4 cm in x and y).
- The thick drawn walls (9.7 cm against ~2 cm real) leave AMCL a few cm of play on each wall. If
  the stop error stays at 3–5 cm with no fixed direction, the next lever is `sigma_hit` (0.1 →
  0.07). If it always points the same way, adjust `aim_offset_x/y`.

---

## 7. Your six problems

| # | Problem | Cause | Change on this branch | Still to watch |
|---|---|---|---|---|
| 1 | robot turns in circles while RViz shows it moving diagonally | fault 1 + 2: unsigned odometry during turns on the spot and reversing, AMCL at alpha 0.1 | `odom_sign`, alpha 0.4, `update_min_a` 0.05, no 360° spin | the echo test in §1 must confirm it |
| 2 | stops at turns | Spin stalling below the scrub threshold (`min_rotational_vel` 0.6); RPP stopping to turn when it lags an arc by > 45°; the progress checker after 15 s of stall | Spin min 1.0; turn values kept at `116bdf3` | if corners still stall, an angular gain node is next (see §8) |
| 3 | does not cover the centre | goal checker succeeds 3–4 cm short; AMCL half-cell bias (2.4 cm); 3 % map scale; AMCL thrown off by the odometry | 3 cm path extension + xy 0.03; drive forward onto the centre if > 4 cm; aim offset; 58.25 cm map; odometry fix | `stopped NN mm` lines |
| 4 | heavy oscillation, back and forth while turning | the believed pose jumping after every turn on the spot (fault 1), plus overshoot or stall from the changing turn values | fault 1 fixed; turn values back to the ones that ran | AMCL jitter on straights: if it zigzags, try lookahead 0.25 |
| 5 | in 3-wall squares it runs along the wall, then hits it when turning out | off-centre stop (AMCL error), square footprint (corners sweep 13.4 cm when turning), blind local costmap (no collision check) | centre first, then turn on the centre; 0.14 m disc footprint; obstacle layer sees the walls, so Spin and RPP refuse turns that would hit | dead-end test in §1 |
| 6 | the 360° spin at the start | — | `--localise still` (default): no motion, a few seconds of stationary AMCL updates | the `scan on walls` percentage in the log |

---

## 8. Checked offline, and not verified

Checked offline (no ROS here; the runner was imported with stubbed ROS modules):

- `run_course.py` and `odom_sign.py` compile.
- All 552 legs route. Every path point stays ≥ 0.213 m from the drawn walls, on the true path
  (aim offset removed). The closest is the end of D→J, 3 cm past J's centre towards its south
  wall; the goal checker stops the robot on the centre before that.
- Geometry: `cell_of(square_center(X)) == X` for all 24 squares; bearings N = +y, E = +x.
- The scan check accepts a correct pose (100 %), and one with the 2.4 cm AMCL bias (100 %).
  It rejects a 30° heading error (58 %) and a 15 cm position error (69 %).
- The obstacle monitor ignores a box more than 1.2 m ahead and stops the leg on the second
  sighting within 1.2 m. A single sighting followed by a clear check does not stop it. The replan
  avoids the square, and the walls-only fallback frees dead-end J when D is falsely blocked.
- `odom_sign` gives 0 while turning on the spot, −0.08 while reversing at 0.08, passes 0.17
  through when driving forward, caps a tight turn at 1.5× the command, and gives 0 after a
  stale command.

Not verified:

- Nothing has run on the robot or in ROS. Build and dry-run first (§1).
- That the board's speed is unsigned on **your** robot (friend A's finding; the echo test checks it).
- The AMCL half-cell offset on your robot (friend A's measurement; the `stopped ... from the
  centre` lines plus a look at the label will tell).
- The real pitch (58.25 cm against your earlier 62 cm note).
- RPP cannot track arcs closely on this base (`HANDOFF.md` 7.2). If corners still swing wide,
  the next step is a small node between the velocity smoother and the driver that scales the
  turn command (`cmd_vel_topic:=...`, another existing launch argument). Ask before adding it.
- `navigation.rviz` is unchanged; the map now shows North up.
