# Fitzpatrick timing sensitivity

Single car on the 44.73 m reference route. Each cell gives completed / scheduled trials and mean per-lap P95 absolute lateral error among completed laps. See comparison.csv for all-run progress and outcomes. A missing error means no stopped finish.

| Method | Synchronous | Mean | Shuffled | Ordered |
|---|---|---|---|---|
| PP | 4/6 · 4.58 cm | 0/6 · n/a | 2/6 · 4.54 cm | 3/6 · 4.49 cm |
| LQR | 6/6 · 5.96 cm | 0/6 · n/a | 1/6 · 6.28 cm | 2/6 · 5.40 cm |
| Linear MPC | 6/6 · 6.61 cm | 0/6 · n/a | 0/6 · n/a | 2/6 · 6.54 cm |
| NMPC | 6/6 · 4.67 cm | 6/6 · 4.57 cm | 6/6 · 4.50 cm | 6/6 · 4.06 cm |
| MPPI | 6/6 · 5.21 cm | 6/6 · 5.69 cm | 6/6 · 5.84 cm | 6/6 · 5.70 cm |

## Paired outcome

Ordered minus shuffled P95 lateral error in centimetres, averaged over exactly matched starts and seeds for which both methods of scheduling finished. Positive values mean greater error with ordered delays.

- PP: n/a (0/6 matched pairs).
- LQR: n/a (0/6 matched pairs).
- Linear MPC: n/a (0/6 matched pairs).
- NMPC: -0.442 cm (6/6 matched pairs).
- MPPI: -0.138 cm (6/6 matched pairs).

## Common interior section (5–35 m)

At 20 Hz, compute per-run P95 absolute lateral error for this section, including runs that stopped outside the finish tolerance later. Each cell is the mean in cm and number of trials that traversed the section. Raw markers and horizontal mean lines appear in the figure.

| Method | Synchronous | Mean | Shuffled | Ordered |
|---|---|---|---|---|
| PP | 2.61 (6) | 2.65 (6) | 2.70 (6) | 2.69 (6) |
| LQR | 3.14 (6) | 3.43 (6) | 3.40 (6) | 3.34 (6) |
| Linear MPC | 3.26 (6) | 3.57 (6) | 3.54 (6) | 3.59 (6) |
| NMPC | 1.37 (6) | 2.20 (6) | 1.89 (6) | 1.82 (6) |
| MPPI | 1.72 (6) | 2.08 (6) | 1.96 (6) | 1.77 (6) |

## All trials

Outcomes: {'finished': 74, 'time_limit': 46}. Gym collision flags: 0. Body / map boundary flags: 0.

- Linear MPC: time_limit (16 runs). 
- LQR: time_limit (15 runs). 
- PP: time_limit (15 runs). 

At termination 40/46 time-limit runs were at or below the finish speed tolerance but outside the goal distance tolerance. Terminal goal distances for all time-limit runs spanned 13.9–26.3 cm.

## Effective command delivery

Scheduled bursts and shuffled tuples can produce different orderings at the receiver. Counts below cover the actually executed part of each trial, including the stopped tail.

| Profile | Released | Applied | Superseded | Pending at end | Longest consecutive releases with delay >50 ms |
|---|---:|---:|---:|---:|---:|
| Synchronous | 17729 | 17729 | 0 | 0 | 0 |
| Mean | 26450 | 26450 | 0 | 0 | 0 |
| Shuffled | 24819 | 19962 | 4856 | 1 | 4 |
| Ordered | 22651 | 22077 | 571 | 3 | 10 |

Synthetic schedules are shared across controllers. Different completion times give different exposure lengths. Timing is imposed in simulated time; laptop wall time is diagnostic only. Physical prediction error requires separate held-out vehicle runs and hardware timing records. These counts do not represent independent physical sessions or a general controller ranking.

[Interior tracking figure](interior_error.png) · [Interior tracking data](interior_section.csv) · [Completion figure](completion.png) · [All trial summaries](trials.json) · [Schedule and checksums](manifest.json)
