"""Car-like TEB and native Ackermann DWA for the single-car static Gym study.

Both are local benchmark implementations. TEB optimizes a band in a route-
normal coordinate chart, including positive time intervals; it does not wrap
ROS teb_local_planner. DWA samples reachable forward-speed/steering commands.
Neither implementation receives scenario metadata or simulator obstacle truth.
"""

import math

import numpy as np
from scipy.optimize import least_squares


WHEELBASE = 0.3302
STEER_LIMIT = 0.4189
MARGIN = 0.065
DT = 0.05


def _wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _node_yaw(points, start_yaw, end_yaw):
    central = points[2:] - points[:-2]
    return np.r_[start_yaw, np.arctan2(central[:, 1], central[:, 0]), end_yaw]


def plan_teb(route, state, start_s, goal_s, scene, obstacles,
             max_curvature=math.tan(STEER_LIMIT)/WHEELBASE, acceleration_limit=2.5):
    """Optimize lateral knot positions *and* travel times in a timed elastic band.

    Longitudinal stations are fixed and yaw at interior knots follows the local
    tangent. The first and last knots are fixed. Residuals include curvature,
    initial/final tangent, acceleration, traversal time, progress speed,
    lateral displacement and obstacle separation. A dense body check rejects
    infeasible solutions independently of the fitted obstacle cost.
    """
    from scipy.interpolate import CubicHermiteSpline

    stations = np.linspace(start_s, goal_s, max(8, int(np.ceil((goal_s-start_s)/.40))+1))
    refs = route.at_s(stations)
    normals = np.column_stack((-np.sin(refs[:, 2]), np.cos(refs[:, 2])))
    first = np.array([state['x'], state['y']])
    initial_offset = (first-refs[0, :2]) @ normals[0]
    spacing = np.diff(stations)
    n = len(stations)
    obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 3)
    # Project scan clusters into the band coordinate chart to initialize
    # candidate homotopies, and to construct a smooth obstacle residual.
    obstacle_s = []
    obstacle_d = []
    local_obstacles = []
    for ox, oy, radius in obstacles:
        segment = refs[1:, :2]-refs[:-1, :2]
        frac = np.clip(np.sum((np.array([ox, oy])-refs[:-1, :2])*segment, axis=1)
                       / np.sum(segment**2, axis=1), 0., 1.)
        projected = refs[:-1, :2] + frac[:, None]*segment
        idx = np.argmin(np.sum((projected-[ox, oy])**2, axis=1))
        # Previously passed obstacles stay in the static map. Only a cluster
        # close to this band needs to initialize a lateral avoidance maneuver.
        # The final swept-body check below still receives *all* known obstacles.
        if np.linalg.norm(projected[idx]-[ox, oy]) > .50+radius:
            continue
        local_obstacles.append((ox, oy, radius))
        obstacle_s.append(stations[idx]+frac[idx]*spacing[idx])
        obstacle_d.append((np.array([ox, oy])-projected[idx]) @ normals[idx])
    obstacle_s, obstacle_d = np.asarray(obstacle_s), np.asarray(obstacle_d)
    v_target = np.clip(.5*(refs[:-1, 3]+refs[1:, 3]), .20, 2.)
    dt_ref = spacing/v_target
    curv_limit = max_curvature
    identity_offsets = np.zeros((n, n-2))
    identity_offsets[1:-1] = np.eye(n-2)

    def geometry(offsets):
        points = refs[:, :2]+offsets[:, None]*normals
        points[0] = first
        return points

    def evaluate_band(params, side, jac=False):
        offsets = np.r_[initial_offset, params[:n-2], 0.]
        points = geometry(offsets)
        edge = np.diff(points, axis=0)
        lengths = np.linalg.norm(edge, axis=1).clip(.02)
        times = lengths/1.96 + params[n-2:]
        headings = np.arctan2(edge[:, 1], edge[:, 0])
        rates = lengths/times
        # Convert a lateral offset increment to a length and heading change.
        if jac:
            pt_jac = normals[:, :, None]*identity_offsets[:, None, :]
            edge_jac = np.diff(pt_jac, axis=0)
            length_jac = np.sum(edge[:, :, None]*edge_jac, axis=1)/lengths[:, None]
            heading_jac = ((edge[:, 0, None]*edge_jac[:, 1]
                            - edge[:, 1, None]*edge_jac[:, 0])/lengths[:, None]**2)
            len_jac = np.pad(length_jac, ((0, 0), (0, n-1)))
            angle_jac = np.pad(heading_jac, ((0, 0), (0, n-1)))
            time_jac = len_jac/1.96 + np.pad(np.eye(n-1), ((0, 0), (n-2, 0)))
            offset_jac = np.pad(identity_offsets, ((0, 0), (0, n-1)))
            rate_jac = len_jac/times[:, None]-time_jac*(rates/times)[:, None]
        rows = []
        turn = _wrap(np.diff(headings))
        curvature = 2.*turn/(lengths[:-1]+lengths[1:])
        if jac:
            turn_jac = np.diff(angle_jac, axis=0)
            curv_jac = (2.*turn_jac-curvature[:, None]*(len_jac[:-1]+len_jac[1:]))/(lengths[:-1]+lengths[1:])[:, None]
        rows.extend((.6*offset_jac[1:-1] if jac else .6*offsets[1:-1],
                     .9*curv_jac if jac else .9*curvature,
                     .65*np.diff(curv_jac, axis=0) if jac else .65*np.diff(curvature),
                     4.*angle_jac[[0,-1]] if jac else 4.*_wrap(headings[[0,-1]]-[state['yaw'], refs[-1, 2]]),
                     .9*rate_jac if jac else .9*(rates-v_target),
                     .28*time_jac if jac else .28*times))
        # Dynamic obstacles are outside this benchmark. For each static object,
        # impose a soft side-specific displacement where the bodies could overlap.
        for obs_s, obs_d, (_, _, radius) in zip(obstacle_s, obstacle_d, local_obstacles):
            ds = np.abs(stations-obs_s)
            shoulder = np.maximum(1.-np.maximum(0., ds-.38)/.48, 0.)
            required = shoulder*max(0., .25+radius+MARGIN+side*obs_d)
            violation = required-side*offsets
            active = (violation > 0.) & (shoulder > 0.)
            rows.append(-22.*side*active[:, None]*offset_jac if jac else 22.*active*violation)
        accel_change = np.diff(rates, prepend=max(0., state['v']))
        accel = accel_change/times
        if jac:
            da = np.diff(rate_jac, axis=0, prepend=np.zeros((1, 2*n-3)))/times[:, None]-time_jac*(accel/times)[:, None]
        rows.extend((5.*(np.maximum(abs(curvature)-curv_limit*.92, 0.) if not jac else
                          (np.sign(curvature)*(abs(curvature)>curv_limit*.92))[:, None]*curv_jac),
                     3.*(np.maximum(abs(accel)-acceleration_limit, 0.) if not jac else
                          (np.sign(accel)*(abs(accel)>acceleration_limit))[:, None]*da),
                     8.*(np.maximum(rates-2., 0.) if not jac else (rates>2.)[:, None]*rate_jac)))
        return np.concatenate(rows, axis=0)

    subdivisions = max(2, int(np.ceil(max(spacing)/.055)))
    dense_s = np.linspace(start_s, goal_s, (n-1)*subdivisions+1)
    lower = np.r_[np.full(n-2, -1.8), np.zeros(n-1)]
    upper = np.r_[np.full(n-2, 1.8), np.maximum(dt_ref*5., 1.)]
    best_cost, best = np.inf, None
    for side in (1., -1.):
        init = initial_offset*(1.-(stations-start_s)/(goal_s-start_s))
        for obs_s, obs_d, (_, _, radius) in zip(obstacle_s, obstacle_d, local_obstacles):
            sigma = .80
            init += (side*(.31+MARGIN+radius)+obs_d)*np.exp(-.5*((stations-obs_s)/sigma)**2)
        initial_lengths = np.linalg.norm(np.diff(geometry(init), axis=0), axis=1)
        x0 = np.r_[init[1:-1].clip(-1.7, 1.7), np.maximum(.001, dt_ref-initial_lengths/1.96)]
        result = least_squares(lambda p: evaluate_band(p, side), x0,
                               jac=lambda p: evaluate_band(p, side, True),
                               bounds=(lower, upper), max_nfev=20, ftol=2e-4, xtol=2e-4)
        off = np.r_[initial_offset, result.x[:n-2], 0.]
        points = geometry(off)
        yaw = _node_yaw(points, state['yaw'], refs[-1, 2])
        tangent = np.column_stack((np.cos(yaw), np.sin(yaw)))
        curve = CubicHermiteSpline(stations, points, tangent)
        dense = curve(dense_s)
        dxy = curve(dense_s, 1)
        ddxy = curve(dense_s, 2)
        curvature = (dxy[:, 0]*ddxy[:, 1]-dxy[:, 1]*ddxy[:, 0])/np.linalg.norm(dxy, axis=1).clip(.02)**3
        poses = np.column_stack((dense, np.arctan2(dxy[:, 1], dxy[:, 0])))
        clearance = scene.clearance(poses, obstacles)
        if np.min(clearance) < MARGIN or np.max(abs(curvature)) > curv_limit:
            continue
        # Interpolate optimized intervals by actual spline arc (not station).
        # This preserves every optimized knot timestamp and avoids artificial
        # speed pulses between knots when the path changes lateral offset.
        knot_durations = np.linalg.norm(np.diff(points, axis=0), axis=1)/1.96 + result.x[n-2:]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
        knot_arc = arc[::subdivisions]
        if np.max(np.diff(knot_arc)/knot_durations) > 2.:
            continue
        times = np.interp(arc, knot_arc, np.r_[0., np.cumsum(knot_durations)])
        cost = result.cost-.25*min(float(np.min(clearance)), .25)
        if cost < best_cost:
            best, best_cost = np.column_stack((poses, times)), cost
    return best


def dwa_command(route, state, s, scene, obstacles, prev_speed, prev_steer, on_rollout=None,
                wheelbase=WHEELBASE, steer_limit=STEER_LIMIT, acceleration=3., deceleration=3.,
                require_safe_rollout=False):
    """Sample velocity/steer pairs inside a dynamic window and apply the first command.

    The window includes commanded speed and steering slew limits. Rollouts use
    a bounded-acceleration bicycle at the actual measured speed. Test full-body
    clearance on each arc including braking distance; steer and speed remain
    constant commands over the excursion. The route supplies progress, heading
    and terminal-speed references, not an externally planned avoidance path.
    """
    remaining = max(0., route.length-s)
    if remaining < .025:
        return max(0., prev_speed-deceleration*DT), float(np.clip(0., prev_steer-.8*DT, prev_steer+.8*DT))
    goal_v = route.at_s(min(s+.12, route.length))[3]
    if remaining < 1.5:
        goal_v = route.at_s(s)[3]
    min_v, max_v = max(0., prev_speed-deceleration*DT), min(2., prev_speed+acceleration*DT)
    min_delta, max_delta = max(-steer_limit, prev_steer-.8*DT), min(steer_limit, prev_steer+.8*DT)
    vs = np.unique(np.r_[np.linspace(min_v, max_v, 4), np.clip(goal_v, min_v, max_v)])
    ds = np.linspace(min_delta, max_delta, 9)
    speeds, steer = np.meshgrid(vs, ds)
    speeds, steer = speeds.ravel(), steer.ravel()
    # Constant-command bicycle arcs using acceleration-limited integrated distance.
    v0 = max(0., state['v'])
    ts = np.linspace(.05, 1.45, 20)
    rates = np.where(speeds >= v0, acceleration, deceleration)[:, None]
    ramp = np.minimum(ts, abs(speeds[:, None]-v0)/rates)
    dist = speeds[:, None]*ts - .5*np.sign(speeds-v0)[:, None]*rates*ramp**2
    before_target = ts < abs(speeds[:, None]-v0)/rates
    dist = np.where(before_target, v0*ts + .5*np.sign(speeds-v0)[:, None]*rates*ts**2, dist)
    curvature = np.tan(steer)/wheelbase
    turn = dist*curvature[:, None]
    heading = state['yaw'] + turn
    mid = state['yaw']+.5*turn
    chord = dist*np.sinc(.5*turn/np.pi)
    x = state['x']+chord*np.cos(mid)
    y = state['y']+chord*np.sin(mid)
    ref = route.at_s(np.minimum(route.length, s+dist))
    lateral = -(x-ref[:, :, 0])*np.sin(ref[:, :, 2]) + (y-ref[:, :, 1])*np.cos(ref[:, :, 2])
    angle = _wrap(heading-ref[:, :, 2])
    poses = np.stack((x, y, heading), axis=-1)
    clearance = scene.clearance(poses.reshape(-1, 3), obstacles).reshape(dist.shape)
    min_clear = clearance.min(axis=1)
    first_block = np.min(np.where(clearance < MARGIN, dist, np.inf), axis=1)
    stop_dist = np.maximum(speeds, v0)**2/(2.*deceleration)+.10*np.maximum(speeds, v0)
    safe = (min_clear >= MARGIN) & (first_block > stop_dist)
    # Immediate tracking is weighted more strongly than the end of the arc;
    # collision and stopping feasibility still see the full prediction horizon.
    # Bias headings towards intercepting the reference only when the measured
    # forward corridor is open. With a blocked corridor, let arc clearance
    # determine the detour instead of steering back into the detected obstacle.
    centerline_probe = route.at_s(s+np.linspace(0., 3., 17))[:, :3]
    corridor_clear = np.min(scene.clearance(centerline_probe, obstacles)) > MARGIN+.10
    tracking_weight = np.exp(-ts/(.25 if corridor_clear else .65))
    tracking_weight /= tracking_weight.sum()
    align = angle
    if corridor_clear:
        align = angle + np.arctan(1.4*lateral)*np.clip((remaining-dist)/1.3, 0., 1.)
    cost = (1.6*(lateral**2 @ tracking_weight) + 1.8*(align**2 @ tracking_weight)
            + .40*(speeds-goal_v)**2 + .12*(steer-prev_steer)**2
            + 8.*np.maximum(.22-min_clear, 0.)**2)
    if remaining < 1.2:
        at_goal_yaw = state['yaw'] + curvature*remaining
        cost += .95*(1.-remaining/1.2)*_wrap(at_goal_yaw-route.yaw[-1])**2
    if not safe.any():
        if require_safe_rollout:
            raise ValueError("no collision-free DWA rollout")
        # Active braking; preserve the best remaining clearance during deceleration.
        slowest = speeds <= min_v+1e-8
        idx = np.argmax(np.where(slowest, min_clear, -np.inf))
    else:
        idx = np.argmin(np.where(safe, cost, np.inf))
    if on_rollout is not None:
        on_rollout(poses[idx])
    return float(speeds[idx]), float(steer[idx])
