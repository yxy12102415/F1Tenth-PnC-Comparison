# Fitzpatrick local planning with static obstacles

One simulated F1TENTH car follows the archived [44.73 m physical reference](../real_vehicle/routes/Fitzpatrick_fixed_start_arc_straight_v2_speed_profile.csv) in the official headless Gym. This experiment measures avoidance, return to the reference and completion of the lap on a fixed map. The five methods get the same starting pose, reference speed envelope (up to 2 m/s), mapped walls and LiDAR processing. The vehicle starts and finishes at rest. [Scenario diagram](../results/fitzpatrick_local_design_20261120/layout.png) · [Measured results and traces](../results/fitzpatrick_local_design_20261120/campaign/REPORT.md)

## Layout

Here `s` measures distance along the driven reference and positive `d` points left relative to travel. All positions below are obstacle centres, all diameters are in metres.

| Scenario ID | First obstacle (s, d, diameter) | Second obstacle (s, d, diameter) | Comparison |
|---|---|---|---|
| clear | — | — | System and terminal-stop baseline |
| A | 5.98, ~0, 0.10 | — | First historical obstacle in isolation |
| B | 36.00, ~0, 0.10 | — | Second historical obstacle in isolation |
| AB | 5.98, ~0, 0.10 | 36.00, ~0, 0.10 | Historical nominal layout, two separate returns to route |
| single_left | 33.50, +0.18, 0.40 | — | First obstacle of the left-first pair |
| single_right | 33.50, −0.18, 0.40 | — | First obstacle of the right-first pair |
| stagger_left_first | 33.50, +0.18, 0.40 | 36.00, −0.18, 0.40 | Grouped obstacles on the final straight |
| stagger_right_first | 33.50, −0.18, 0.40 | 36.00, +0.18, 0.40 | Mirror the obstacles with the measured walls fixed |

The original [vehicle placement file](../real_vehicle/config/Fitzpatrick_two_obstacles_A_B.json) specifies the intended A/B centres. The optional `AB_recorded` uses the obstacle estimates from the archived vehicle response. The two 0.40 m discs are designed simulation fixtures. Comparing each pair with its own single keeps both the initial encounter and its diameter fixed. The historical A+B layout and the staggered layouts differ in multiple factors. The fixed reference avoids introducing a new curvature or speed profile as a further factor.

The independent [geometric check](../results/fitzpatrick_local_design_20261120/geometry.json) samples the full oriented vehicle on candidate curves at 1 mm intervals between stations 31 and 39. The unmodified reference intersects the discs. Both an alternating-side maneuver and a common-side bypass fit the available space within the steering limit. These witnesses are checks on the experiment design. The online planners only see estimated obstacles.

## Run

From the repository root, on the [configured Gym and native acados environment](SIM_SETUP.md), run one case or repeat the full campaign. The wrapper builds the native simulation executable, runs trials serially and creates a new result directory for every invocation.

```bash
bash Code/sim_ws/run_fitz_local.sh --methods lattice --scenarios stagger_left_first --seeds 11
bash Code/sim_ws/run_fitz_local.sh --seeds 11 29 47
```

### Watch one trial on the desktop

Run the following from the repository root in a graphical session with Python Tk and Matplotlib available.

```bash
bash Code/sim_ws/run_fitz_local.sh --gui
```

The default live example runs Frenet lattice with NMPC on `stagger_left_first`, seed 11. The left pane shows the full map and the right pane magnifies the obstacles. The dashed gray line marks the reference, purple shows the selected local plan, teal traces the vehicle, and red circles show the physical layout. In this example the car drives toward the bottom of the map. S1 appears to the right of the reference on screen (vehicle left) and S2 appears to the left (vehicle right). Watch from about 17 to 23 simulated seconds. After the run, click **Replay approach** to replay from 5 m before the first obstacle or **Replay lap** to watch from the start. Closing the live window ends that trial. Results go to the directory printed in the terminal.

To watch the historical A/B layout instead, run `bash Code/sim_ws/run_fitz_local.sh --gui --scenarios AB`. All five methods accept the same single-trial CLI; the DWA overlay draws its selected speed/steering rollout. Use the headless command for timing comparisons.

The negative control deliberately follows the unmodified reference directly into the test section. A nonzero exit is expected for collision or any unsuccessful trial. The schedule still records all outcomes.

```bash
bash Code/sim_ws/run_fitz_local.sh --methods unmodified_reference --scenarios A B stagger_left_first stagger_right_first --seeds 11
```

## What is comparable

- Hybrid A* searches forward Ackermann motion primitives. Kinematic local RRT* samples poses in the corridor, selects parents and rewires its tree under a finite iteration budget. Frenet lattice enumerates smooth lateral transitions and checks their feasibility. These three send spatial paths through the same timing adapter and native v8 Gym NMPC.
- The car-like timed elastic band optimizes lateral knot offsets and segment durations with a local nonlinear least-squares solve. It keeps its optimized timing when sending a reference to NMPC. This is a reduced benchmark implementation.
- Ackermann DWA samples reachable speed and steering commands every 50 ms, checks forward rollouts and braking, then applies the chosen command directly. End-to-end results therefore describe the **five deployed planning/control combinations**. A new RRT* path request and a DWA command cycle represent different units of work.
- The sensor mapper uses noisy single-agent Gym scans and exact simulator ego pose to subtract the known walls, estimate obstacle discs and retain static observations across a lap. Fixture truth goes to Gym world construction and independent scoring. Each trial starts with an empty obstacle map. There is no ROS graph or connection to a physical car.
- Physics runs at 100 Hz and the decision loop at 20 Hz. Each method runs once per scenario and seed; planner order is shuffled within the block. The three seeds cover scan noise and, for RRT*, search randomness; they share the same physical layout. They do not represent independent tracks. The historical clear/A/B/AB fixtures were also used for engineering checks before the campaign. Treat the designed fixtures as new evaluation conditions.
- Collision scoring includes Gym's TTC collision flag, overlap with injected map pixels and ground-truth vehicle-rectangle-to-disc distance at physics steps. The flag can fire before geometric contact. The scorer checks wall/unknown/out-of-map overlap separately. For each obstacle the car must advance 0.55 m past its station; within 4 m of the final obstacle in a group it must return to within 0.12 m and 0.16 rad of the route for five consecutive decisions. It must then finish the lap within 0.15 m remaining, 0.20 m Euclidean goal distance, 0.18 rad heading error and 0.10 m/s for 0.5 s. Simulation timeout is 55 s.
- Raw per-trial trajectories, commands, sensing/planning/control timings, observed targets at plan requests, plan points, and all completion states are preserved. The report lists completed-lap times, minimum obstacle gaps, rejoin checks and deadline misses. Synchronous Gym advances after each command, so a missed wall-clock budget does not inject actuator delay. Additional maps, hardware timing and localization noise are needed to assess transfer.

## Inspect an existing campaign

```bash
.deps/sim_env/bin/python Code/sim_ws/tools/compare_fitz_layouts.py results/fitzpatrick_local_design_20261120/campaign
OPENBLAS_NUM_THREADS=1 .deps/sim_env/bin/python Code/sim_ws/tools/replay_fitz_local.py results/fitzpatrick_local_design_20261120/campaign
.deps/sim_env/bin/python tools/plot_fitz_local_layout.py results/fitzpatrick_local_design_20261120
```

The paired CSV reports success counts, time added by the second obstacle for seeds that completed both conditions, and passing sides observed at the obstacle stations. The replay tool applies the recorded commands to the archived scenarios and rechecks pose, flags and the minimum physical clearance at 100 Hz. The [campaign source supplement](../results/fitzpatrick_local_design_20261120/reproduction/README.md) freezes the official campaign version. Later instrumentation captures path timing when NMPC rejects a new plan in the same control cycle; the report identifies any missing legacy samples.
