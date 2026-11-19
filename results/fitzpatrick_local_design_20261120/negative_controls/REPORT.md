# Fitzpatrick static local planning

Of 0 planner trials, **0 passed**. The schedule included 4 additional unmodified-reference negative control(s).

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
| Unmodified reference (negative control) | A only | 0/1 | 1 | 0 | — | 0.154 | — | 1.71 | 1/82 |
| Unmodified reference (negative control) | B only | 0/1 | 1 | 0 | — | 0.174 | — | 1.74 | 1/412 |
| Unmodified reference (negative control) | Designed stagger, left obstacle first | 0/1 | 1 | 0 | — | 0.182 | — | 1.82 | 1/385 |
| Unmodified reference (negative control) | Designed stagger, right obstacle first | 0/1 | 1 | 0 | — | 0.182 | — | 1.94 | 1/385 |

Successful time is the median completed-lap simulation time. Minimum gap is the smallest continuous-body-to-physical-disc clearance recorded at the physics steps across all outcomes. A dash indicates no observation (for example a clear track has no obstacle gap). The CSV also gives the minimum gap among successful runs, steering-rate RMS and the status counts.

Both P95 columns are medians of within-trial 95th percentiles. The plan column measures new local-path computations, including rejected planner requests, or the DWA decision on each tick. The pipeline column covers all measured ticks, including perception, planning and the downstream controller when used. It excludes Gym stepping. The synchronous simulation counts deadlines over 50 ms without injecting latency. Timings reflect this host and workload.

Hybrid A*, lattice, RRT* and TEB feed the same joint NMPC. TEB retains its segment times. DWA executes its selected speed and steering directly. Lap-time differences include these execution and speed-policy differences. The unmodified reference runs NMPC without obstacle avoidance.

## Planning timing completeness

Path-planning requests are counted from each trial's planning-event log. The audit found 0 missing timing sample(s) and 0 excess sample(s) across 0 trial(s). The CSV reports the counts per group and the number of trials contributing to its planning P95.

## Outcomes requiring inspection

| Method | Scenario | Seed | Outcome | Progress (m) | Detail |
|---|---|---:|---|---:|---|
| Unmodified reference (negative control) | A only | 11 | collision | 5.46 | — |
| Unmodified reference (negative control) | B only | 11 | collision | 35.51 | — |
| Unmodified reference (negative control) | Designed stagger, left obstacle first | 11 | collision | 32.85 | — |
| Unmodified reference (negative control) | Designed stagger, right obstacle first | 11 | collision | 32.85 | — |

## Two-obstacle trajectories

Each course plot uses map coordinates and shows the actual vehicle-centre traces. Obstacles are drawn to scale. Crosses mark the end of unsuccessful traces.

- Unmodified reference (negative control), Designed stagger, left obstacle first — [course](figures/unmodified_reference_stagger_left_first.png) · [obstacle detail](figures/unmodified_reference_stagger_left_first_detail.png)
- Unmodified reference (negative control), Designed stagger, right obstacle first — [course](figures/unmodified_reference_stagger_right_first.png) · [obstacle detail](figures/unmodified_reference_stagger_right_first_detail.png)

Each trial directory contains the full compressed trace, summary and planning-event records. The manifest records the run order, geometry, software hashes and evaluation thresholds.
