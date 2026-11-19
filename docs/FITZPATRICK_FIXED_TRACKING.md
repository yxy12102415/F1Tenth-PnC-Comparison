# Fitzpatrick single-car tracking benchmark

Five trackers follow the archived v8 44.73154 m route from rest to rest in the official, headless F1TENTH Gym. The route and speed profile are in `real_vehicle/routes/Fitzpatrick_fixed_start_arc_straight_v2_speed_profile.csv`. This test uses the original map walls, a single simulated car, and direct simulator feedback. It uses no ROS topics, vehicle connection, opponent or obstacle-avoidance planner.

## Run

From the repository root on the configured Ubuntu laptop:

```bash
bash Code/sim_ws/run_fitz_tracking.sh
```

To repeat a single centered run or write to a specific new directory:

```bash
bash Code/sim_ws/run_fitz_tracking.sh --methods lqr --cases center --seeds 11
bash Code/sim_ws/run_fitz_tracking.sh --output /tmp/fitz_tracking_reproduction
```

The wrapper regenerates the clearance asset from the tracked map, builds the two native backends when needed and limits numerical-library worker threads to one. It prints the output path and runs the trials serially. The default is a fresh directory under `Code/sim_ws/results/fitz_tracking/`. An existing output directory is protected. Exit status is nonzero for a failed trial, after the remaining scheduled trials have run and the results have been written.

The tested stack uses Python 3.10, classic `f110-gym==0.2.1`, `gym==0.19.0`, NumPy, SciPy, OSQP (tested with 0.6.3), Pillow, PyYAML and Matplotlib. The native backends require a C++14 compiler, Eigen, jsoncpp and the project's acados v0.4.0 build and native-Gym codegen environment. See [laptop build and Gym solver setup](SIM_SETUP.md). MPPI uses the local C++ CPU implementation in `takeover_sim/mppi_control.hpp`. The manifest records hashes of the controller and benchmark sources, native binary, map and route, including local changes.

## Driving task

The speed reference comes from the CSV and peaks at 2 m/s. Progress is projected onto a local window in unwrapped route arclength so the coincident start and finish stay distinct. The 25 reference samples span 1.2 s from the current progress. Every method gets the same reference-generation rule. A delayed vehicle can continue tracking from its actual progress. Reference time is not a fixed external race clock.

Gym integrates at 100 Hz. New commands arrive every five physics steps (20 Hz). All trials start with zero velocity. Start conditions are centered, 0.10 m left with +4 degrees yaw, and 0.10 m right with -4 degrees yaw. Methods are shuffled within each start/seed block with a fixed schedule seed. The two default seeds (11, 29) vary MPPI sampling. Gym scan noise does not feed the controller. With exact state feedback, repeated deterministic-controller runs principally remeasure laptop execution time.

A finish requires at most 0.15 m remaining along the completed lap, at most 0.20 m Euclidean goal error, at most 0.18 rad heading error and speed at most 0.10 m/s continuously for 0.50 s. One official Gym collision or any body overlap with occupied/unknown map cells ends the run. The independent footprint check runs at each physics step. A rejected controller command ends the run. The time limit is 55 simulated seconds.

## Controllers

- **Pure Pursuit** uses speed-dependent lookahead. **LQR** uses a speed-scheduled discrete Riccati gain with curvature feedforward. **Linear MPC** solves a 24-stage constrained linearized kinematic bicycle QP using OSQP with steering and steering-step bounds. These three Python simulation implementations share a speed controller which compensates for Gym speed response. Historical hardware runs used separate controller releases.
- **NMPC** uses the native-Gym v8 joint controller and generated acados solver (acceleration of commanded speed and front steering). The benchmark selects active Gym braking for the timed stop. The real-vehicle profile keeps its physical coast policy. **MPPI** uses the existing native-Gym CPU controller (256 samples, two updates, 24 stages) to track the same reference without an opponent.
- The observers use Gym pose, forward speed and yaw rate. Gym's exposed lateral-speed field is a zero placeholder. NMPC propagates a lateral-speed estimate internally, while the benchmark supplies MPPI with the reported field. The joint controllers also check their predicted footprint. These are comparisons of the specified tracking implementations.
- All use a 2 m/s command cap and a 0.4189 rad steering limit. Benchmark settings request up to ±3 m/s² command acceleration/deceleration, 0.04 rad steering change per tick and 0.15 m/s² acceleration change per tick. The common speed servo and MPPI project speed back into its feasible interval at saturation. Actual command changes are available in the traces.

## Outputs

Each trial has a compressed 20 Hz CSV with measured pose, speed, route progress, tracking errors, applied command, controller timing and footprint events, plus a JSON summary and any native diagnostic log. `trials.json` includes every outcome. `comparison.csv` reports counts over all scheduled trials and mean per-trial metrics over successful runs; an empty metric denotes no finish. P95 is computed within each trial before averaging. `figures/` shows reference and realized trajectories for each controller, using the first seed at each start. No files from earlier vehicle experiments are used as simulated outcomes.

Compute time covers the controller implementation including its internal checks. The second timing column also includes harness call overhead and native JSON IPC. Both omit Gym physics and reference construction. Simulation time advances synchronously, so deadline counts over 45 ms describe laptop compute load and do not inject actuation delay. Language, solver, hardware and differing internal checks affect this comparison.

The C++ MPPI implementation uses the project’s vehicle model. The original implementation notes cite the Apache-2.0 Nav2 optimizer and the MIT-licensed python_simple_mppi example as algorithm references. [Nav2 MPPI](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller) · [python_simple_mppi at the inspected revision](https://github.com/MizuhoAOKI/python_simple_mppi/tree/ee3e3bf97f6098ddb3fe00cbe0eb3698bc2c1326)
