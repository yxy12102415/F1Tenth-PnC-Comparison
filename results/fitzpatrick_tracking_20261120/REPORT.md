# Fitzpatrick fixed-route tracking, 20 November 2026

All five controllers completed all six assigned simulated trials. The run contains 16,579 control updates, zero Gym collisions, zero footprint boundary violations and zero terminal controller failures. Each trial tracked the original 44.7315 m v8 route in the official single-agent Gym. The reference peaks at 2 m/s. Completion includes the final 0.50 s stopped hold.

Three paired start poses crossed with seeds 11 and 29 were run serially. The order was shuffled within each block. The controllers used direct simulator feedback. The seeds change MPPI sampling. Identical deterministic repeats provide repeated timing measurements rather than additional geometric scenarios.

## Results

Each value is the arithmetic mean of the six trial metrics for that controller. Timing is per control call on the laptop (Ryzen 9 5900HX), with P95 calculated separately in each trial. The comparison covers these configurations at the specified speed.

| Controller | Finished | Duration (s) | Lateral RMSE (cm) | Heading RMSE (deg) | Speed RMSE (m/s) | RMS steering rate (rad/s) | Compute P95 (ms) | Call P95 with IPC (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Pure Pursuit | 6/6 | 27.80 | 2.31 | 2.71 | 0.057 | 0.153 | 0.123 | 0.130 |
| LQR | 6/6 | 27.80 | 2.76 | 2.00 | 0.057 | 0.169 | 0.132 | 0.139 |
| Linear MPC | 6/6 | 27.77 | 2.94 | 1.83 | 0.057 | 0.127 | 0.349 | 0.355 |
| NMPC | 6/6 | 27.27 | 1.79 | 1.71 | 0.050 | 0.126 | 1.306 | 1.614 |
| MPPI | 6/6 | 27.53 | 2.23 | 2.07 | 0.052 | 0.127 | 2.845 | 3.166 |

## Sensitivity to starting pose

Full-trial lateral RMSE in cm, averaged over the two seeds. The left and right start conditions displace the car 0.10 m and rotate it 4° in the same respective direction.

| Controller | Center | Left | Right |
|---|---:|---:|---:|
| Pure Pursuit | 1.75 | 2.61 | 2.58 |
| LQR | 2.28 | 3.03 | 2.98 |
| Linear MPC | 2.38 | 3.28 | 3.17 |
| NMPC | 0.81 | 2.28 | 2.29 |
| MPPI | 1.23 | 2.75 | 2.72 |

NMPC had the smallest mean lateral and speed errors for these configurations. Pure Pursuit had the shortest computation time. Linear MPC attained a smaller heading RMSE than Pure Pursuit while exhibiting a larger lateral RMSE. Solver implementations, state feedback and tuning differ. All accepted control calls stayed within the 45 ms internal compute budget in this batch. Execution times exclude simulation and ROS transport.

## Development finding

The first centered NMPC run stopped 0.215 m short. The controller applied the physical braking/coasting rule and Gym's speed servo stopped it before the accepted 0.15 m remaining distance. The fixed-route Gym mode now optimizes the speed command down to zero through the Gym actuator model, with first-step slew bounds matched to the benchmark. The original failure and trace are retained in [diagnostics](diagnostics/nmpc_physical_coast_center_11/summary.json).

## Reproduction and data

See the [runner and acceptance definitions](../../docs/FITZPATRICK_FIXED_TRACKING.md). [Machine-readable comparison](comparison.csv), [all trials](trials.json), [versions, SHA-256 and schedule](manifest.json). Each trial directory contains its raw trace and summary. The nominal first-seed traces are included in the five separate English trajectory plots.

[Pure Pursuit trajectory](figures/pp.png) · [LQR trajectory](figures/lqr.png) · [Linear MPC trajectory](figures/linear_mpc.png) · [NMPC trajectory](figures/nmpc.png) · [MPPI trajectory](figures/mppi.png)
