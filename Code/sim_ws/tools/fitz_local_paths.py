"""Shared encounter policy, spatial lattice and trajectory adapters for static planning."""
import math

import numpy as np

MARGIN = .065
DETECTION_LOOKAHEAD = 4.2
REJOIN_LENGTH = 2.1
GROUP_DISTANCE = 3.0
DT = .05


def reference_station(route, xy):
    # Local continuous projection onto the fixed reference polyline.
    fractions = np.clip(np.sum((xy-route.xy[:-1])*route.edges, axis=1) / route.edge_squared, 0., 1.)
    residual = xy - (route.xy[:-1] + fractions[:, None]*route.edges)
    j = np.argmin(np.sum(residual**2, axis=1))
    return route.s[j] + fractions[j]*(route.s[j+1]-route.s[j])


def encounter(route, s, obstacles):
    """Group visible, route-blocking observations that need a common rejoin goal."""
    candidates = []
    for x, y, radius in obstacles:
        station = reference_station(route, np.array([x, y]))
        if -.20 <= station-s <= DETECTION_LOOKAHEAD + GROUP_DISTANCE:
            center = route.at_s(station)
            if math.hypot(center[0]-x, center[1]-y) <= .32 + radius:
                candidates.append(station)
    candidates.sort()
    if not candidates or candidates[0]-s > DETECTION_LOOKAHEAD:
        return []
    group = candidates[:1]
    for station in candidates[1:]:
        if station-group[-1] > GROUP_DISTANCE:
            break
        # Repeated scan hypotheses for one object do not define a new gate.
        if station-group[-1] > .35:
            group.append(station)
    return group


def smoothstep(q):
    q = np.clip(q, 0., 1.)
    return q**3 * (10. + q*(-15. + 6.*q))


def plan_lattice(route, state, s, goal_s, threats, scene, obstacles, max_curvature=math.tan(.4189)/.3302):
    """Terminal-offset sampling with smooth entry, passing and return segments."""
    stations = np.linspace(s, goal_s, max(3, math.ceil((goal_s-s)/.035)+1))
    x, y, yaw, _ = route.at_s(stations).T
    normal = np.column_stack((-np.sin(yaw), np.cos(yaw)))
    d0 = np.dot([state["x"]-x[0], state["y"]-y[0]], normal[0])
    slope = np.tan(np.clip(math.remainder(state["yaw"]-yaw[0], 2*math.pi), -.65, .65))
    candidates, scores = [], []
    obstacle_s = threats[0]
    last_s = threats[-1]
    for half_plateau in (.25, .50):
        available = obstacle_s-half_plateau-s
        if available <= .25:
            continue
        for entry in (.65, .95):
            length = available*entry
            origin = obstacle_s-half_plateau-length
            # A brief Hermite correction matches measured yaw at the start.
            correction_q = np.clip((stations-s)/min(1., available), 0., 1.)
            initial = d0*(1.-smoothstep(correction_q)) + slope*(stations-s)*(1.-correction_q)**3
            entry_shape = smoothstep((stations-origin)/length)
            exit_shape = 1.-smoothstep((stations-last_s-half_plateau)/max(.1, goal_s-last_s-half_plateau))
            transition = smoothstep((stations-obstacle_s-half_plateau)/max(.1,last_s-obstacle_s-2*half_plateau))
            offsets = (-.36, .36, -.48, .48, -.62, .62, -.78, .78)
            for offset, exit_offset in ((a,b) for a in offsets for b in ((a,-a) if len(threats)>1 else (a,))):
                lateral = initial + entry_shape*exit_shape*(offset+(exit_offset-offset)*transition)
                xy = np.column_stack((x, y)) + lateral[:, None]*normal
                chord = np.linalg.norm(np.diff(xy, axis=0), axis=1)
                arc = np.r_[0., np.cumsum(chord)]
                heading = np.unwrap(np.arctan2(np.gradient(xy[:, 1]), np.gradient(xy[:, 0])))
                curvature = np.gradient(heading, arc)
                if np.max(np.abs(curvature)) > max_curvature:
                    continue
                path = np.column_stack((xy, heading))
                candidates.append(path)
                scores.append(.30*(arc[-1]-(goal_s-s)) + .24*np.trapz(curvature**2, arc) + .20*abs(offset))
    if not candidates:
        return None
    paths = np.stack(candidates)
    gaps = scene.clearance(paths.reshape(-1, 3), obstacles).reshape(paths.shape[:2]).min(axis=1)
    costs = np.where(gaps >= MARGIN, np.asarray(scores) - .9*np.minimum(.23, gaps), np.inf)
    return paths[np.argmin(costs)] if np.isfinite(costs).any() else None


class Maneuver:
    """Arc-length projection and time-preserving NMPC references for an accepted plan."""
    def __init__(self, raw, route, start_s, goal_s):
        raw = np.asarray(raw, dtype=float)
        if raw.ndim != 2 or raw.shape[1] not in (3, 4) or len(raw)<2 or not np.isfinite(raw).all():
            raise ValueError("invalid planner output")
        self.goal_s = goal_s
        self.index = 0
        keep = np.r_[True, np.linalg.norm(np.diff(raw[:, :2], axis=0), axis=1) > 1.e-6]
        if not keep.all():
            raise ValueError("zero-length segment in moving maneuver")
        # Continue the controller horizon along the reference after this local goal.
        tail_s = route.s[(route.s > goal_s+.025) & (route.s < goal_s+3.1)]
        tail = route.at_s(tail_s)
        joined = np.vstack((raw[:, :3], tail[:, :3]))
        yaw = np.unwrap(joined[:, 2])
        edges = np.diff(joined[:, :2], axis=0)
        chord = np.linalg.norm(edges, axis=1)
        if min(chord) < 1e-6:
            raise ValueError("duplicate points at maneuver join")
        self.arc = np.r_[0., np.cumsum(chord)]
        self.xy, self.yaw, self.edges = joined[:, :2], yaw, edges
        self.end_index = len(raw)-1
        self.planned = raw
        self.dense_arc = np.linspace(0., self.arc[self.end_index],
                                     int(np.ceil(self.arc[self.end_index]/.025))+1)
        self.dense_pose = self.poses_at(self.dense_arc)
        base_station = np.interp(self.arc[:len(raw)], [0., self.arc[self.end_index]], [start_s, goal_s])
        cap = route.at_s(np.r_[base_station, tail_s])[:, 3]
        if raw.shape[1] == 4:
            dt = np.diff(raw[:, 3])
            if np.any(dt <= 0):
                raise ValueError("TEB segment times must increase")
            self.teb = True
            self.times = np.r_[0., np.cumsum(np.r_[dt, 2*chord[len(dt):] /
                                          np.maximum(.05, cap[len(dt):-1]+cap[len(dt)+1:])])]
            self.segment_speed = chord/np.diff(self.times)
        else:
            self.teb = False
            curvature = np.abs(np.gradient(yaw, self.arc))
            v = np.minimum(cap, np.sqrt(1.25/np.maximum(.01, curvature)))
            for i in range(len(v)-2, -1, -1):
                v[i] = min(v[i], math.sqrt(v[i+1]**2+2.*1.0*chord[i]))
            for i in range(1, len(v)):
                v[i] = min(v[i], math.sqrt(v[i-1]**2+2.*1.0*chord[i-1]))
            self.times = np.r_[0., np.cumsum(2*chord/np.maximum(.05, v[:-1]+v[1:]))]
            self.speeds = v

    def poses_at(self, a):
        return np.column_stack([np.interp(a, self.arc, val) for val in (self.xy[:, 0], self.xy[:, 1], self.yaw)])

    def reference(self, state):
        point = np.array([state["x"], state["y"]])
        lo = max(0, self.index-4)
        hi = min(len(self.edges), max(self.index+2, self.arc.searchsorted(self.arc[self.index]+.9)))
        edges = self.edges[lo:hi]
        alpha = np.clip(np.sum((point-self.xy[lo:hi])*edges, axis=1)/np.sum(edges**2, axis=1), 0., 1.)
        distances = np.linalg.norm(self.xy[lo:hi]+alpha[:, None]*edges-point, axis=1)
        k = np.argmin(distances)
        self.index = j = lo+k
        arc = self.arc[j] + alpha[k]*(self.arc[j+1]-self.arc[j])
        now = np.interp(arc, self.arc, self.times)
        sample_t = np.minimum(self.times[-1], now+np.arange(25)*DT)
        a = np.interp(sample_t, self.times, self.arc)
        if self.teb:
            idx = np.clip(np.searchsorted(self.times, sample_t, side="right")-1, 0, len(self.segment_speed)-1)
            speed = self.segment_speed[idx]
        else:
            speed = np.interp(a, self.arc, self.speeds)
        return np.column_stack((self.poses_at(a), speed)), float(distances[k]), arc


class PathPlanner:
    def __init__(self, method, route, scene, seed, max_curvature=None, acceleration_limit=None):
        self.max_curvature = max_curvature
        self.acceleration_limit = acceleration_limit
        self.method, self.route, self.scene = method, route, scene
        self.rng = np.random.default_rng(seed)
        self.cached = None
        self.events = []

    def reference(self, state, s, obstacles, now):
        from fitz_local_search import plan_search
        from fitz_local_teb_dwa import plan_teb

        threats = encounter(self.route, s, obstacles)
        goal = min(threats[-1]+REJOIN_LENGTH, self.route.length) if threats else None
        if self.cached is not None:
            if s >= self.cached.goal_s:
                self.cached = None
            else:
                ref, distance, arc = self.cached.reference(state)
                future = self.cached.dense_pose[self.cached.dense_arc >= arc-.025]
                extend = goal is not None and goal > self.cached.goal_s+.45
                if not extend and distance <= .30 and (not len(future) or np.min(self.scene.clearance(future, obstacles)) >= MARGIN):
                    return ref, False
                self.cached = None
        if not threats:
            return self.route.window(s), False
        event = dict(t=now, s=float(s), threat_stations=threats, goal_s=float(goal),
                     observed=obstacles.tolist())
        self.events.append(event)
        limits = {} if self.max_curvature is None else {"max_curvature": self.max_curvature}
        if self.method == "lattice":
            path = plan_lattice(self.route, state, s, goal, threats, self.scene, obstacles, **limits)
        elif self.method == "teb":
            if self.acceleration_limit is not None:
                limits["acceleration_limit"] = self.acceleration_limit
            path = plan_teb(self.route, state, s, goal, self.scene, obstacles, **limits)
        else:
            stats = {}
            path = plan_search(self.method, self.route, state, s, goal, self.scene, obstacles,
                               self.rng, stats=stats, **limits)
            event["search"] = stats
        if path is None:
            raise ValueError("no feasible local path")
        candidate = Maneuver(path, self.route, s, goal)
        min_gap = float(np.min(self.scene.clearance(candidate.dense_pose, obstacles)))
        event["planned_clearance_m"] = min_gap
        if min_gap < MARGIN-1e-6:
            raise ValueError("local path failed dense body validation")
        if np.linalg.norm(path[0, :2]-[state["x"],state["y"]]) > .05 or np.linalg.norm(
                path[-1, :2]-self.route.at_s(goal)[:2]) > .10:
            raise ValueError("local path endpoints invalid")
        event["path"] = np.round(path, 6).tolist()
        self.cached = candidate
        return candidate.reference(state)[0], True
