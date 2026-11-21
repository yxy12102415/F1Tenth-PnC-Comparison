# Fitzpatrick timing sensitivity

MPCC selects its own progress speed along the route. This separate experiment uses the vehicle command envelope across all four timing profiles. Host solve time is measured independently of the synthetic command-arrival delay.

Single car on the 44.73 m reference route. Each cell gives completed / scheduled trials and mean per-lap P95 absolute lateral error among completed laps. See comparison.csv for all-run progress and outcomes. A missing error means no stopped finish.

| Method | Synchronous | Mean | Shuffled | Ordered |
|---|---|---|---|---|
| MPCC | 6/6 · 2.81 cm | 4/6 · 24.16 cm | 0/6 · n/a | 0/6 · n/a |

## Paired outcome

Ordered minus shuffled P95 lateral error in centimetres, averaged over exactly matched starts and seeds for which both methods of scheduling finished. Positive values mean greater error with ordered delays.

- MPCC: n/a (0/6 matched pairs).

## Common interior section (5–35 m)

At 20 Hz, compute per-run P95 absolute lateral error for this section, including runs that stopped outside the finish tolerance later. Each cell is the mean in cm and number of trials that traversed the section. Raw markers and horizontal mean lines appear in the figure.

| Method | Synchronous | Mean | Shuffled | Ordered |
|---|---|---|---|---|
| MPCC | 2.15 (6) | 21.13 (6) | 19.39 (6) | 16.88 (6) |

## All trials

Outcomes: {'finished': 10, 'time_limit': 14}. Gym collision flags: 0. Body / map boundary flags: 0.

- MPCC: time_limit (14 runs). 

At termination 14/14 time-limit runs were at or below the finish speed tolerance but outside the goal distance tolerance. Terminal goal distances for all time-limit runs spanned 21.6–46.1 cm.

## Effective command delivery

Scheduled bursts and shuffled tuples can produce different orderings at the receiver. Counts below cover the actually executed part of each trial, including the stopped tail.

| Profile | Released | Applied | Superseded | Pending at end | Longest consecutive releases with delay >50 ms |
|---|---:|---:|---:|---:|---:|
| Synchronous | 3036 | 3036 | 0 | 0 | 0 |
| Mean | 4310 | 4310 | 0 | 0 | 0 |
| Shuffled | 6600 | 5304 | 1296 | 0 | 4 |
| Ordered | 6600 | 6432 | 168 | 0 | 10 |

Synthetic schedules are shared across controllers. Different completion times give different exposure lengths. Timing is imposed in simulated time; laptop wall time is diagnostic only. Physical prediction error requires separate held-out vehicle runs and hardware timing records. These counts do not represent independent physical sessions or a general controller ranking.

[Interior tracking figure](interior_error.png) · [Interior tracking data](interior_section.csv) · [Completion figure](completion.png) · [All trial summaries](trials.json) · [Schedule and checksums](manifest.json)
