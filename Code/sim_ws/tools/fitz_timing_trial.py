"""Closed-loop timing experiment. All transport events use integer Gym physics ticks."""
import csv
import gzip
import heapq
import math
import time

import numpy as np

from fitz_tracking_plant import PHYSICS_DT, Plant
import run_fitz_tracking as tracking


TICK_MS = round(1000 * PHYSICS_DT)
CONTROL_STEPS = round(tracking.DT / PHYSICS_DT)
STALE_STEPS = round(.50 / PHYSICS_DT)
TRACE_COLUMNS = ("t_s", "x", "y", "yaw", "v", "vy", "yaw_rate", "s_m", "cte_m",
                 "heading_error_rad", "ref_v_mps", "ref_x", "ref_y", "cmd_v_mps",
                 "cmd_delta_rad", "applied_command_id", "collision", "boundary")
COMMAND_COLUMNS = ("id", "release_step", "requested_pose_age_steps", "requested_odom_age_steps",
                   "pose_source_step", "odom_source_step", "delay_steps", "due_step", "applied_step",
                   "observed_s_m", "nominal_speed_mps", "nominal_steer_rad", "controller_ms",
                   "call_ms", "disposition")


def trial(route, method, case, seed, folder, timeout, schedule):
    """Run one fresh trial; schedule gives causal input ages and downstream delivery delay.

    The stateful controller runs once per 50 ms release. Completion/transport latency
    is represented by the schedule; measured host compute time is diagnostic only.
    The queue models downstream availability, not CPU preemption or a serial solver.
    """
    steps = round(timeout / PHYSICS_DT)
    releases = (steps + CONTROL_STEPS - 1) // CONTROL_STEPS
    if steps < CONTROL_STEPS or len(schedule) < releases:
        raise ValueError("need at least one control interval and a timing row per release")
    for sample in schedule[:releases]:
        delays = (sample.pose_age_steps, sample.odom_age_steps, sample.command_delay_steps)
        if any(not isinstance(d, (int, np.integer)) or d < 0 for d in delays):
            raise ValueError("timing delays must be nonnegative integer physics steps")

    folder.mkdir(parents=True, exist_ok=False)
    offset, angle = tracking.CASES[case]
    plant = Plant(route, initial_offset=offset, initial_heading=math.radians(angle), seed=seed)
    state = plant.state
    history = [state]
    goal = route.at_s(route.length)
    s, _ = route.project(state, 0.)
    observed_s = 0.
    pose_source = odom_source = 0
    active_id = -1
    speed = steer = 0.
    queued, commands, samples = [], [], []
    goal_samples = late_run = longest_late_run = 0
    reason, detail = "time_limit", ""
    limits = tracking.VEHICLE_PARAMS
    try:
        with tracking.controller(method, seed, folder, route, vehicle_envelope=True) as solve:
            for tick in range(steps):
                if tick % CONTROL_STEPS == 0:
                    index = tick // CONTROL_STEPS
                    sample = schedule[index]
                    pose_age, odom_age, delay = map(int, (sample.pose_age_steps,
                                                          sample.odom_age_steps,
                                                          sample.command_delay_steps))
                    # Latest delivered source wins; startup uses the known resting initial state.
                    pose_source = max(pose_source, tick - pose_age)
                    odom_source = max(odom_source, tick - odom_age)
                    observation = {k: history[pose_source][k] for k in ("x", "y", "yaw")}
                    observation.update({k: history[odom_source][k] for k in ("v", "vy", "yaw_rate")})
                    observed_s, _ = route.project(observation, observed_s)
                    row = dict(id=index, release_step=tick, requested_pose_age_steps=pose_age,
                               requested_odom_age_steps=odom_age, pose_source_step=pose_source,
                               odom_source_step=odom_source, delay_steps=delay, due_step=tick+delay,
                               observed_s_m=observed_s, disposition="pending")
                    commands.append(row)
                    late_run = late_run + 1 if delay > CONTROL_STEPS else 0
                    longest_late_run = max(longest_late_run, late_run)
                    if tick - min(pose_source, odom_source) > STALE_STEPS:
                        reason, detail = "stale_stop", "observation exceeded freshness budget"
                        row["disposition"] = "rejected_stale_observation"
                        break
                    reference = route.window(observed_s)
                    start = time.perf_counter()
                    try:
                        v, delta, runtime = solve(observation, reference, route.length-observed_s)
                        if not all(math.isfinite(q) for q in (v, delta, runtime)):
                            raise ValueError("nonfinite command or timing")
                        if not (-1e-5 <= v <= limits["max_speed"] + 1e-5 and
                                abs(delta) <= limits["max_steer"] + 1e-5):
                            raise ValueError("command outside configured actuator envelope")
                        row.update(nominal_speed_mps=v, nominal_steer_rad=delta, controller_ms=runtime)
                    except (RuntimeError, ValueError, OSError) as exc:
                        reason, detail = "controller_failure", str(exc)
                        row["disposition"] = reason
                        break
                    finally:
                        row["call_ms"] = (time.perf_counter() - start) * 1000.
                    heapq.heappush(queued, (tick + delay, index))

                # A simultaneous set of arrivals is consumed atomically. Older sequence
                # numbers (including arrivals overtaken by a previous delivery) are dropped.
                candidates = []
                while queued and queued[0][0] <= tick:
                    _, index = heapq.heappop(queued)
                    candidates.append(index)
                if candidates:
                    newest = max(active_id, max(candidates))
                    for index in candidates:
                        row = commands[index]
                        if index != newest:
                            row["disposition"] = "superseded"
                        elif tick - min(row["pose_source_step"], row["odom_source_step"]) > STALE_STEPS:
                            row["disposition"] = "expired"
                        else:
                            active_id = index
                            row.update(disposition="applied", applied_step=tick)
                            speed, steer = row["nominal_speed_mps"], row["nominal_steer_rad"]

                # Source freshness applies while holding the last command as well.
                active_source = (min(commands[active_id]["pose_source_step"],
                                     commands[active_id]["odom_source_step"]) if active_id >= 0 else 0)
                if tick - active_source > STALE_STEPS:
                    reason, detail = "stale_stop", "held command exceeded source freshness budget"
                    break
                state = plant.step_physics(speed, steer)
                history.append(state)
                contact = state["collision"] or state["boundary"]
                if (tick + 1) % CONTROL_STEPS != 0 and not contact:
                    continue
                s, cte = route.project(state, s)
                ref = route.at_s(s)
                heading = tracking.wrap(state["yaw"] - ref[2])
                samples.append((plant.sim_time, state["x"], state["y"], state["yaw"], state["v"],
                                state["vy"], state["yaw_rate"], s, cte, heading, ref[3], ref[0], ref[1],
                                speed, steer, active_id, int(state["collision"]), int(state["boundary"])))
                if contact:
                    reason = "collision" if state["collision"] else "boundary"
                    break
                at_goal = (s >= route.length - tracking.END["remaining_m"]
                           and math.hypot(state["x"]-goal[0], state["y"]-goal[1]) <= tracking.END["distance_m"]
                           and abs(tracking.wrap(state["yaw"]-goal[2])) <= tracking.END["yaw_rad"]
                           and abs(state["v"]) <= tracking.END["speed_mps"])
                goal_samples = goal_samples + 1 if at_goal else 0
                if goal_samples >= round(tracking.END["dwell_s"] / tracking.DT):
                    reason = "finished"
                    break
    finally:
        plant.close()
    with gzip.open(folder / "commands.csv.gz", "wt", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COMMAND_COLUMNS)
        writer.writeheader()
        writer.writerows(commands)
    with gzip.open(folder / "trace.csv.gz", "wt", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(TRACE_COLUMNS)
        writer.writerows(samples)

    applied = [c for c in commands if c["disposition"] == "applied"]
    result = dict(method=method, case=case, seed=seed, status=reason, detail=detail,
                  simulated_s=plant.sim_time, progress_m=s, samples=len(samples),
                  requests=len(commands), applied_commands=len(applied),
                  superseded_commands=sum(c["disposition"] == "superseded" for c in commands),
                  expired_commands=sum(c["disposition"] == "expired" for c in commands),
                  pending_commands=sum(c["disposition"] == "pending" for c in commands),
                  over_period_requests=sum(c["delay_steps"] > CONTROL_STEPS for c in commands),
                  max_consecutive_over_period=longest_late_run,
                  collision=bool(state["collision"]), boundary=bool(state["boundary"]),
                  final_speed_mps=state["v"], final_distance_m=math.hypot(state["x"]-goal[0], state["y"]-goal[1]))
    runtimes = [c["controller_ms"] for c in commands if "controller_ms" in c]
    if runtimes:
        result.update(compute_p95_ms=float(np.percentile(runtimes, 95)),
                      over_budget_steps=sum(v > tracking.BUDGET_MS for v in runtimes))
    call_times = [c["call_ms"] for c in commands if "call_ms" in c]
    if call_times:
        result["call_p95_ms"] = float(np.percentile(call_times, 95))
    if applied:
        ages = [(c["applied_step"]-c["pose_source_step"])*TICK_MS for c in applied]
        result.update(mean_applied_pose_age_ms=float(np.mean(ages)),
                      applied_pose_age_p95_ms=float(np.percentile(ages, 95)))
    if samples:
        data = dict(zip(TRACE_COLUMNS, np.asarray(samples).T))
        def rms(a):
            return float(np.sqrt(np.mean(np.square(a))))
        result.update(cte_p95_cm=float(np.percentile(abs(data["cte_m"]), 95)) * 100,
                      cte_rmse_cm=rms(data["cte_m"]) * 100,
                      heading_rmse_deg=math.degrees(rms(data["heading_error_rad"])),
                      speed_rmse_mps=rms(data["v"] - data["ref_v_mps"]),
                      steer_rate_rms_rad_s=rms(np.diff(data["cmd_delta_rad"], prepend=0.)/tracking.DT),
                      peak_actual_speed_mps=float(max(data["v"])))
    tracking.save(folder / "summary.json", result)
    return result
