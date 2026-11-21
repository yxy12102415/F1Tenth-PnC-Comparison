# Fixed-reference tracking under synthetic timing disturbances

This single-car simulation evaluates five controller implementations on the 44.73 m Fitzpatrick route. The question is how sensor age and command delivery affect following the same speed profile and reaching a stopped finish. No timed moving-vehicle record linking pose acquisition, a control decision and final command reception was available in the project archive. The timing inputs below are synthetic. A separate physical test on held-out days is needed to measure prediction accuracy.

## Reproduce

From the repository root in the [configured Gym environment](SIM_SETUP.md):

```bash
# 120 trials, saved in a new timestamped directory.
bash Code/sim_ws/run_fitz_timing.sh
# Inspect one paired block with a new output directory.
bash Code/sim_ws/run_fitz_timing.sh --methods pp nmpc --cases center --seeds 11 --output /tmp/fitz_timing_pair
```

The full schedule is five methods × four timing profiles × three standing starts × two seeds (11 and 29). The initial pose is centered or displaced laterally by ±0.10 m with a corresponding ±4° heading offset. Within each seed and start, the execution order is shuffled. The runner finishes the schedule and writes all outputs before returning status 1 if any trial failed. To check or regenerate a report from an existing run:

```bash
.deps/sim_env/bin/python Code/sim_ws/tools/audit_fitz_timing.py results/fitzpatrick_timing_20261122
.deps/sim_env/bin/python Code/sim_ws/tools/report_fitz_timing.py results/fitzpatrick_timing_20261122
```

## Timing model

Each control release occurs every 50 ms. Each channel is scheduled in steps of 10 ms, matching Gym physics. The table gives (pose age, odometry age, release-to-command arrival) at a control release, in milliseconds.

| Profile | Assigned timing | Construction |
|---|---|---|
| Synchronous | (0, 0, 0) | Ideal reference condition for this command envelope. |
| Fixed mean | (30, 20, 30) | Exact channel means of the full 55 s schedules. |
| Shuffled | 75% (20, 10, 10), 25% (60, 50, 90) | Shuffle complete tuples in each two-second time window, with the same counts as the ordered schedule. |
| Ordered | Same two tuples and counts | Place the ten late ticks consecutively at a seeded offset in each complete 40-release window. |

The schedule is paired by seed across methods and starts, without using hidden vehicle state to place delays. Matching distributions applies to the assigned schedule, including each time window. Early termination truncates exposure. On the first few ticks each sensor starts from the known stationary initial state. Deliveries that lose a race to a newer command are discarded, so effective command ages and application counts can differ between shuffled and ordered profiles. Neither equal applied-delay histograms nor placement matched by track curvature is assumed.

Pose and odometry are sampled independently from simulated history. The observed pose determines the control reference, while the current Gym pose is reserved for scoring. During the delay the car advances under its last available command. Simultaneous command arrivals use the latest sequence. The modeled freshness budget is 0.50 s. Real laptop solve times are logged separately and are not added to the injected delay. The scheduled command delay represents effective availability at the receiver; CPU preemption, a serial solve backlog, DDS transport and the complete vehicle supervisor are not modeled. Native controllers retain their own host-side runtime and validity checks.

## Model and evaluation

The Gym speed setpoint is limited to 2.00 m/s, nominal acceleration to +1.0/−0.8 m/s² and steering to ±0.269 rad. The intended steering increment limit is 0.04 rad per release. Slew limits at the sender do not imply identical increments at the receiver after skipped commands. This optional envelope is applied inside the controllers. Gym vehicle dynamics, speed actuation and active terminal braking differ from the physical car's zero-current coast. State feedback has timing disturbances with no added pose noise. These results are a controlled timing-sensitivity test.

The trial stops at Gym collision, occupied-map footprint overlap, stale input/command, controller rejection, 55 s elapsed, or a valid stopped finish. The finish requires ≤0.15 m along-route remaining, ≤0.20 m Euclidean distance, ≤0.18 rad heading error and ≤0.10 m/s for 0.50 s. Scoring uses absolute lateral error at 20 Hz. The report gives per-condition completion counts, mean per-lap P95 on completed runs and an additional 5–35 m interior section. The interior measure excludes the long stopped tail of a trial that reached the end but missed its finish tolerance. Ordered-versus-shuffled full-lap contrasts use only matched pairs that completed both conditions. Seeds and deterministic repeats describe these scenarios; they do not constitute independent hardware sessions.

Each trial has a 20 Hz truth trace, a per-release command/event CSV and a JSON summary. The campaign has source and binary hashes, complete assigned schedules, an all-outcome table, matched contrasts, figures and a report. The original five-controller measurement and free-progress MPCC study use separate protocols.

[Campaign report](../results/fitzpatrick_timing_20261122/REPORT.md) · [Paired timing schedules](../results/fitzpatrick_timing_20261122/schedules/) · [Fixed-route protocol](FITZPATRICK_FIXED_TRACKING.md)

## Findings on the frozen build, November 22, 2026

All 120 scheduled trials ran and traversed the common 5–35 m scoring section. Seventy-four met every standing-finish criterion, with zero simulated collisions or boundary overlaps. Of the 46 time-limit outcomes, the final trace sample missed only position in 33 cases, only heading in six, and both in seven. All 46 had stopped. PP, LQR, and linear MPC share the same longitudinal servo, so their terminal performance cannot be attributed solely to their steering algorithms. Their synchronous runs here also use tighter command limits than the original fixed-tracking campaign.

NMPC and MPPI each completed 24/24. In the common section, mean per-run P95 absolute error changed from 1.37 to 2.20 cm for NMPC and from 1.72 to 2.08 cm for MPPI between synchronous and fixed-mean timing. Ordered versus shuffled was a smaller difference for both. The latest-command policy dropped 19.6% of issued commands under shuffled timing and 2.5% under ordered timing over these runs. This demonstrates an interaction between the schedule and the receiver policy, without establishing how either reproduces the car. The imposed time blocks are not aligned to track curvature. The small number of seeds and one route limit generalization.

The archived formal run used 45 unchanged source, map and binary fingerprints. An earlier development sweep, during which native construction-time parameter validation was finalized, remains locally in `Code/sim_ws/results/fitz_timing_20261122_preliminary` and is excluded from the formal counts. [Build integrity](../results/fitzpatrick_timing_20261122/integrity.json) · [Raw-data audit](../results/fitzpatrick_timing_20261122/audit.txt).

## Supplementary free-progress MPCC campaign

Run the MPCC supplement from the repository root. The default combination of four timing profiles, three starts and two seeds schedules 24 trials. The original 120 fixed-reference trials remain the comparison set for PP, LQR, linear MPC, NMPC and MPPI. [MPCC timing report](../results/fitzpatrick_mpcc_timing_20261122/REPORT.md)

```bash
bash Code/sim_ws/run_fitz_timing.sh --methods mpcc
```

MPCC selects its own virtual progress speed along the same route. Compare timing profiles within MPCC using finish outcomes, elapsed time, and lateral error. Its deviation from the CSV speed profile measures a difference in driving pace. The optimizer uses the same commissioned speed, acceleration and steering command bounds as the timing study. It predicts a kinematic bicycle with a smoothed Gym speed response. The Gym plant actively brakes at the end. Physical v8 uses different vehicle and actuator dynamics and a zero-current terminal coast. The earlier free-progress campaign used the wider Gym command envelope, so it answers a different question.

Each per-release `controller_ms` is laptop wall time for the MPCC step, including reference-geometry preparation and optimization. `compute_p95_ms` is the within-trial 95th percentile of returned step times. The field `over_budget_steps` counts those above 45 ms. The count `over_period_requests` measures **assigned synthetic delivery delays** above the 50 ms release period. A solve exceeding 50 ms on the laptop does not delay the scheduled Gym event by that amount. Interpreting either count as a Jetson deadline measurement would require an instrumented Jetson run.
