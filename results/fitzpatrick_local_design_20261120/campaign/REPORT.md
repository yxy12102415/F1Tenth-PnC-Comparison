# Fitzpatrick static local planning

Of 120 planner trials, **112 passed**. The schedule included 0 additional unmodified-reference negative control(s).

A pass requires the scenario’s route-rejoin checks and a stop at the finish. Gym supplies the scans and ego state. The planner receives obstacle estimates derived from the scans. Ground-truth placements are used to construct the scene and score the body clearance.

The A and B cases replay the nominal physical layout. Where selected, AB_recorded uses obstacle-position and diameter estimates from the archived vehicle response. The outcomes below are new simulations.

The two stagger names denote mirrored, simulation-designed obstacle layouts on the Fitzpatrick route. Left and right are relative to the vehicle's reference direction. The following coordinates describe the simulated obstacle centres in the map frame.

| Designed scenario | Obstacle | Route progress (m) | Map x (m) | Map y (m) | Diameter (m) |
|---|---|---:|---:|---:|---:|
| Designed stagger, left obstacle first | S1 | 33.50 | -0.992 | 9.779 | 0.400 |
| Designed stagger, left obstacle first | S2 | 36.00 | -1.352 | 7.279 | 0.400 |
| Designed stagger, right obstacle first | S1 | 33.50 | -1.352 | 9.779 | 0.400 |
| Designed stagger, right obstacle first | S2 | 36.00 | -0.992 | 7.279 | 0.400 |

| Method | Scenario | Passed / run | Collisions | Boundary | Successful time (s) | Min gap (m) | Plan P95 (ms) | Pipeline P95 (ms) | Late / timed ticks |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Hybrid A* | Clear track | 3/3 | 0 | 0 | 27.30 | — | — | 1.93 | 3/1638 |
| Hybrid A* | A only | 3/3 | 0 | 0 | 27.40 | 0.269 | 11.57 | 2.42 | 3/1644 |
| Hybrid A* | B only | 3/3 | 0 | 0 | 27.95 | 0.212 | 20.96 | 2.59 | 3/1677 |
| Hybrid A* | A+B (historical configuration) | 3/3 | 0 | 0 | 28.05 | 0.188 | 19.68 | 3.02 | 3/1682 |
| Hybrid A* | Matched first obstacle, left | 3/3 | 0 | 0 | 28.05 | 0.255 | 18.47 | 2.91 | 3/1683 |
| Hybrid A* | Matched first obstacle, right | 3/3 | 0 | 0 | 27.65 | 0.184 | 16.33 | 2.92 | 3/1659 |
| Hybrid A* | Designed stagger, left obstacle first | 3/3 | 0 | 0 | 28.35 | 0.210 | 32.69 | 3.33 | 3/1707 |
| Hybrid A* | Designed stagger, right obstacle first | 3/3 | 0 | 0 | 28.35 | 0.148 | 24.86 | 3.52 | 3/1702 |
| Frenet lattice | Clear track | 3/3 | 0 | 0 | 27.30 | — | — | 2.02 | 3/1638 |
| Frenet lattice | A only | 2/3 | 0 | 0 | 27.62 | 0.289 | 6.38 | 2.73 | 3/1197 |
| Frenet lattice | B only | 3/3 | 0 | 0 | 27.60 | 0.165 | 6.43 | 2.87 | 3/1656 |
| Frenet lattice | A+B (historical configuration) | 1/3 | 0 | 0 | 27.95 | 0.166 | 6.77 | 3.23 | 3/1063 |
| Frenet lattice | Matched first obstacle, left | 3/3 | 0 | 0 | 27.60 | 0.193 | 6.71 | 2.94 | 3/1656 |
| Frenet lattice | Matched first obstacle, right | 3/3 | 0 | 0 | 27.60 | 0.194 | 6.59 | 2.63 | 3/1656 |
| Frenet lattice | Designed stagger, left obstacle first | 3/3 | 0 | 0 | 28.40 | 0.185 | 11.38 | 3.43 | 3/1704 |
| Frenet lattice | Designed stagger, right obstacle first | 3/3 | 0 | 0 | 28.45 | 0.183 | 10.98 | 3.37 | 3/1707 |
| Kinematic local RRT* | Clear track | 3/3 | 0 | 0 | 27.30 | — | — | 1.69 | 3/1638 |
| Kinematic local RRT* | A only | 3/3 | 0 | 0 | 27.45 | 0.320 | 25.00 | 2.39 | 3/1648 |
| Kinematic local RRT* | B only | 3/3 | 0 | 0 | 27.60 | 0.251 | 21.78 | 2.73 | 3/1657 |
| Kinematic local RRT* | A+B (historical configuration) | 3/3 | 0 | 0 | 27.65 | 0.235 | 31.76 | 2.92 | 3/1661 |
| Kinematic local RRT* | Matched first obstacle, left | 3/3 | 0 | 0 | 27.60 | 0.240 | 22.48 | 3.16 | 3/1657 |
| Kinematic local RRT* | Matched first obstacle, right | 3/3 | 0 | 0 | 27.55 | 0.164 | 24.21 | 3.03 | 3/1653 |
| Kinematic local RRT* | Designed stagger, left obstacle first | 2/3 | 0 | 0 | 28.00 | 0.125 | 25.66 | 3.34 | 3/1541 |
| Kinematic local RRT* | Designed stagger, right obstacle first | 0/3 | 0 | 0 | — | 2.403 | — | 2.06 | 3/1089 |
| Ackermann DWA | Clear track | 3/3 | 0 | 0 | 27.35 | — | 0.67 | 0.81 | 0/1641 |
| Ackermann DWA | A only | 3/3 | 0 | 0 | 27.75 | 0.086 | 0.70 | 0.87 | 0/1665 |
| Ackermann DWA | B only | 3/3 | 0 | 0 | 27.50 | 0.075 | 0.72 | 0.89 | 0/1651 |
| Ackermann DWA | A+B (historical configuration) | 3/3 | 0 | 0 | 27.90 | 0.077 | 0.72 | 0.88 | 0/1675 |
| Ackermann DWA | Matched first obstacle, left | 3/3 | 0 | 0 | 27.45 | 0.147 | 0.68 | 0.85 | 0/1647 |
| Ackermann DWA | Matched first obstacle, right | 3/3 | 0 | 0 | 27.70 | 0.128 | 0.68 | 0.87 | 0/1664 |
| Ackermann DWA | Designed stagger, left obstacle first | 3/3 | 0 | 0 | 28.20 | 0.126 | 0.68 | 0.88 | 0/1688 |
| Ackermann DWA | Designed stagger, right obstacle first | 2/3 | 0 | 0 | 28.15 | 0.123 | 0.72 | 0.95 | 0/1700 |
| Car-like TEB | Clear track | 3/3 | 0 | 0 | 27.30 | — | — | 1.96 | 3/1638 |
| Car-like TEB | A only | 3/3 | 0 | 0 | 27.40 | 0.197 | 14.72 | 2.05 | 3/1644 |
| Car-like TEB | B only | 3/3 | 0 | 0 | 27.35 | 0.170 | 15.80 | 2.05 | 3/1641 |
| Car-like TEB | A+B (historical configuration) | 3/3 | 0 | 0 | 27.45 | 0.170 | 17.53 | 2.65 | 3/1647 |
| Car-like TEB | Matched first obstacle, left | 3/3 | 0 | 0 | 27.35 | 0.175 | 16.98 | 2.46 | 3/1641 |
| Car-like TEB | Matched first obstacle, right | 3/3 | 0 | 0 | 27.35 | 0.176 | 14.93 | 2.41 | 3/1641 |
| Car-like TEB | Designed stagger, left obstacle first | 3/3 | 0 | 0 | 27.45 | 0.149 | 19.08 | 2.53 | 3/1646 |
| Car-like TEB | Designed stagger, right obstacle first | 3/3 | 0 | 0 | 27.45 | 0.196 | 18.39 | 2.81 | 3/1647 |

Successful time is the median completed-lap simulation time. Minimum gap is the smallest continuous-body-to-physical-disc clearance recorded at the physics steps across all outcomes. A dash indicates no observation (for example a clear track has no obstacle gap). The CSV also gives the minimum gap among successful runs, steering-rate RMS and the status counts.

Both P95 columns are medians of within-trial 95th percentiles. The plan column measures new local-path computations, including rejected planner requests, or the DWA decision on each tick. The pipeline column covers all measured ticks, including perception, planning and the downstream controller when used. It excludes Gym stepping. The synchronous simulation counts deadlines over 50 ms without injecting latency. Timings reflect this host and workload.

Hybrid A*, lattice, RRT* and TEB feed the same joint NMPC. TEB retains its segment times. DWA executes its selected speed and steering directly. Lap-time differences include these execution and speed-policy differences. The unmodified reference runs NMPC without obstacle avoidance.

## Planning timing completeness

Path-planning requests are counted from each trial's planning-event log. The audit found 3 missing timing sample(s) and 0 excess sample(s) across 3 trial(s). The CSV reports the counts per group and the number of trials contributing to its planning P95.

Older trials could accept a new path and then terminate on a controller rejection in the same tick, before a command trace sample was written. For each count mismatch the report retains the true request count and excludes the trial's planning P95. The original measurements and pipeline timing are preserved.

| Method | Scenario | Seed | Requests | Recorded timings |
|---|---|---:|---:|---:|
| Kinematic local RRT* | Designed stagger, right obstacle first | 11 | 2 | 1 |
| Kinematic local RRT* | Designed stagger, right obstacle first | 29 | 2 | 1 |
| Kinematic local RRT* | Designed stagger, right obstacle first | 47 | 2 | 1 |

## Outcomes requiring inspection

| Method | Scenario | Seed | Outcome | Progress (m) | Detail |
|---|---|---:|---|---:|---|
| Kinematic local RRT* | Designed stagger, right obstacle first | 11 | controller_rejection | 30.59 | joint_mpc_solver_4 |
| Frenet lattice | A+B (historical configuration) | 29 | controller_rejection | 34.76 | joint_mpc_nonlinear_infeasible eq=0.002086 ineq=0.000000 |
| Kinematic local RRT* | Designed stagger, left obstacle first | 29 | planner_rejection | 35.84 | no feasible local path |
| Kinematic local RRT* | Designed stagger, right obstacle first | 29 | controller_rejection | 30.60 | joint_mpc_solver_4 |
| Frenet lattice | A only | 47 | controller_rejection | 6.35 | joint_mpc_nonlinear_infeasible eq=0.001359 ineq=0.000000 |
| Frenet lattice | A+B (historical configuration) | 47 | controller_rejection | 6.35 | joint_mpc_nonlinear_infeasible eq=0.001359 ineq=0.000000 |
| Ackermann DWA | Designed stagger, right obstacle first | 47 | missing_rejoin | 44.73 | — |
| Kinematic local RRT* | Designed stagger, right obstacle first | 47 | controller_rejection | 30.59 | joint_mpc_solver_4 |

## Two-obstacle trajectories

Each course plot uses map coordinates and shows the actual vehicle-centre traces. Obstacles are drawn to scale. Crosses mark the end of unsuccessful traces.

- Hybrid A*, A+B (historical configuration) — [course](figures/hybrid_astar_AB.png)
- Hybrid A*, Designed stagger, left obstacle first — [course](figures/hybrid_astar_stagger_left_first.png) · [obstacle detail](figures/hybrid_astar_stagger_left_first_detail.png)
- Hybrid A*, Designed stagger, right obstacle first — [course](figures/hybrid_astar_stagger_right_first.png) · [obstacle detail](figures/hybrid_astar_stagger_right_first_detail.png)
- Frenet lattice, A+B (historical configuration) — [course](figures/lattice_AB.png)
- Frenet lattice, Designed stagger, left obstacle first — [course](figures/lattice_stagger_left_first.png) · [obstacle detail](figures/lattice_stagger_left_first_detail.png)
- Frenet lattice, Designed stagger, right obstacle first — [course](figures/lattice_stagger_right_first.png) · [obstacle detail](figures/lattice_stagger_right_first_detail.png)
- Kinematic local RRT*, A+B (historical configuration) — [course](figures/rrt_star_AB.png)
- Kinematic local RRT*, Designed stagger, left obstacle first — [course](figures/rrt_star_stagger_left_first.png) · [obstacle detail](figures/rrt_star_stagger_left_first_detail.png)
- Kinematic local RRT*, Designed stagger, right obstacle first — [course](figures/rrt_star_stagger_right_first.png) · [obstacle detail](figures/rrt_star_stagger_right_first_detail.png)
- Ackermann DWA, A+B (historical configuration) — [course](figures/dwa_AB.png)
- Ackermann DWA, Designed stagger, left obstacle first — [course](figures/dwa_stagger_left_first.png) · [obstacle detail](figures/dwa_stagger_left_first_detail.png)
- Ackermann DWA, Designed stagger, right obstacle first — [course](figures/dwa_stagger_right_first.png) · [obstacle detail](figures/dwa_stagger_right_first_detail.png)
- Car-like TEB, A+B (historical configuration) — [course](figures/teb_AB.png)
- Car-like TEB, Designed stagger, left obstacle first — [course](figures/teb_stagger_left_first.png) · [obstacle detail](figures/teb_stagger_left_first_detail.png)
- Car-like TEB, Designed stagger, right obstacle first — [course](figures/teb_stagger_right_first.png) · [obstacle detail](figures/teb_stagger_right_first_detail.png)

Each trial directory contains the full compressed trace, summary and planning-event records. The manifest records the run order, geometry, software hashes and evaluation thresholds.
