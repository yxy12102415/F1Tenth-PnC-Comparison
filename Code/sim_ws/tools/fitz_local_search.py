"""Forward Ackermann searches for the Fitzpatrick static-obstacle benchmark.

Coordinates are map metres and heading radians. The local RRT* samples in a
route corridor, steers to SE(2) poses, chooses a parent and rewires nodes. Its
finite work budget and directed corridor are part of this benchmark variant.
"""
from dataclasses import dataclass
from heapq import heappop, heappush
import math
import time

import numpy as np


# Reserve room below the Gym steering limit (0.4189 rad, wheelbase 0.3302 m).
MAX_CURVATURE = 1.25
SAMPLE_STEP = 0.04
BODY_MARGIN = 0.065
HYBRID_LIMIT = 240
RRT_LIMIT = 40


def _wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def _hermite(start, end, max_curvature=MAX_CURVATURE):
    """Endpoint-tangent cubic steering connector; reject tight and looping edges."""
    delta = end[:2] - start[:2]
    chord = np.hypot(*delta)
    if chord < 0.06:
        return None
    start_tan = np.array([math.cos(start[2]), math.sin(start[2])])
    end_tan = np.array([math.cos(end[2]), math.sin(end[2])])
    direction = delta / chord
    if min(start_tan @ direction, end_tan @ direction) < 0.25:
        return None

    # Length-scaled Hermite derivatives produce an SE(2) connector. Bound
    # sampling spacing from the derivative control points of the Bezier form.
    v0 = chord * start_tan
    v1 = chord * end_tan
    max_speed = max(chord, np.linalg.norm(3 * delta - v0 - v1))
    count = max(3, math.ceil(max_speed / SAMPLE_STEP))
    t = np.linspace(0., 1., count + 1)[:, None]
    a = v0 + v1 - 2 * delta
    b = 3 * delta - 2 * v0 - v1
    positions = start[:2] + ((a*t + b)*t + v0)*t
    velocity = (3*a*t + 2*b)*t + v0
    acceleration = 6*a*t + 2*b
    speed_sq = np.einsum("ij,ij->i", velocity, velocity)
    if np.min(speed_sq) < 1e-8:
        return None
    curvature = ((velocity[:, 0]*acceleration[:, 1] - velocity[:, 1]*acceleration[:, 0]) /
                 speed_sq**1.5)
    if np.max(np.abs(curvature)) > max_curvature:
        return None
    headings = np.arctan2(velocity[:, 1], velocity[:, 0])
    return np.column_stack((positions, headings))


def _edge_cost(path, scene, obstacles):
    if path is None:
        return np.inf
    clearance = np.asarray(scene.clearance(path, obstacles)).reshape(-1)
    if not np.isfinite(clearance).all() or np.min(clearance) < BODY_MARGIN:
        return np.inf
    ds = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    turn = np.abs(_wrap(np.diff(path[:, 2])))
    # Penalize low clearance per metre. Prefer gentle paths even when
    # multiple candidates satisfy the identical hard footprint margin.
    clearance_cost = 9.0 * np.maximum(0., 0.29 - np.minimum(clearance[:-1], clearance[1:]))**2
    return float(np.sum(ds*(1 + clearance_cost) + 0.16*turn*turn/np.maximum(ds, 1e-8)))


@dataclass
class _Node:
    pose: np.ndarray
    parent: int
    edge: np.ndarray
    cost: float
    key: object = None


def _assemble(nodes, index, final_edge):
    edges = [final_edge[1:]]
    while index:
        node = nodes[index]
        edges.append(node.edge[1:])
        index = node.parent
    edges.append(nodes[0].pose[None, :])
    return np.concatenate(edges[::-1], axis=0)


def _arc(pose, curvature, length=0.46):
    s = np.linspace(0., length, math.ceil(length/SAMPLE_STEP) + 1)
    angle = pose[2] + curvature*s
    if abs(curvature) < 1e-9:
        dx, dy = s*math.cos(pose[2]), s*math.sin(pose[2])
    else:
        dx = (np.sin(angle) - math.sin(pose[2])) / curvature
        dy = (math.cos(pose[2]) - np.cos(angle)) / curvature
    return np.column_stack((pose[0] + dx, pose[1] + dy, angle))


def _bin(pose):
    return (round(pose[0]/0.14), round(pose[1]/0.14),
            round(float(_wrap(pose[2]))/math.radians(8)) % 45)


def _hybrid(start, goal, scene, obstacles, stats, started, max_curvature):
    root_key = _bin(start)
    nodes = [_Node(start, -1, None, 0., root_key)]
    queue = [(float(np.linalg.norm(goal[:2] - start[:2])), 0)]
    visited = {root_key: 0.}
    best = None
    best_cost = np.inf
    first_iteration = None
    for iteration in range(HYBRID_LIMIT):
        if not queue:
            break
        _, index = heappop(queue)
        node = nodes[index]
        if node.cost > visited[node.key] + 1e-9:
            continue

        connection = _hermite(node.pose, goal, max_curvature)
        total = node.cost + _edge_cost(connection, scene, obstacles)
        if total < best_cost:
            best, best_cost = (index, connection), total
            if first_iteration is None:
                first_iteration = iteration
                stats["first_solution_s"] = time.perf_counter() - started
                stats["first_solution_cost"] = total

        if first_iteration is not None and iteration - first_iteration >= 18:
            break
        for curvature in (-1., -.50, 0., .50, 1.):
            edge = _arc(node.pose, curvature)
            pose = edge[-1]
            key = _bin(pose)
            ds = np.linalg.norm(goal[:2] - pose[:2])
            if node.cost + .46 >= visited.get(key, np.inf) or node.cost + .46 + ds >= best_cost:
                continue
            cost = node.cost + _edge_cost(edge, scene, obstacles)
            if cost >= visited.get(key, np.inf):
                continue
            visited[key] = cost
            new_index = len(nodes)
            nodes.append(_Node(pose, index, edge, cost, key))
            priority = cost + 1.10*ds + .12*abs(_wrap(goal[2] - pose[2]))
            heappush(queue, (priority, new_index))

    stats.update(nodes=len(nodes), solution_cost=float(best_cost) if best else None)
    return None if best is None else _assemble(nodes, *best)


def _update_branch_costs(nodes, children, index, change):
    pending = [index]
    while pending:
        current = pending.pop()
        nodes[current].cost += change
        pending.extend(children[current])


def _rrt(start, goal, route, start_s, goal_s, scene, obstacles, rng, stats, started, max_curvature):
    root = _Node(start, -1, None, 0.)
    nodes = [root]
    children = [set()]
    poses = np.empty((RRT_LIMIT + 1, 3))
    progress = np.empty(RRT_LIMIT + 1)
    poses[0], progress[0] = start, start_s
    terminal = {}
    direct = _hermite(start, goal, max_curvature)
    direct_cost = _edge_cost(direct, scene, obstacles)
    if np.isfinite(direct_cost):
        terminal[0] = (direct, direct_cost)
        stats["first_solution_s"] = time.perf_counter() - started
        stats["first_solution_cost"] = direct_cost
    span = goal_s - start_s
    rewires = 0
    # Sample SE(2) endpoints near the known route; every fourth sample is
    # biased toward its tangent, and samples still explore both passage sides.
    for i in range(RRT_LIMIT):
        sample_s = start_s + rng.uniform(.06, .94) * span
        ref = route.at_s(sample_s)
        lateral = rng.uniform(-1.05, 1.05)
        sample = np.array([ref[0] - lateral*math.sin(ref[2]),
                           ref[1] + lateral*math.cos(ref[2]),
                           ref[2] + (0. if i % 4 == 0 else rng.uniform(-.42, .42))])
        n = len(nodes)
        distances = np.linalg.norm(poses[:n, :2] - sample[:2], axis=1)
        # Bounded step size and forward route progress give a directed tree.
        radius = min(3.5, max(1.6, 7.*(math.log(n + 2)/(n + 1))**(1./3.)))
        near = np.flatnonzero((distances < radius) & (distances > .20))
        parents = near[progress[near] < sample_s - .15]
        best_parent, best_edge, best_cost = None, None, np.inf
        for j in sorted(parents, key=lambda k: nodes[k].cost + distances[k]):
            if nodes[j].cost + distances[j] >= best_cost:
                break
            proposal = _hermite(poses[j], sample, max_curvature)
            cost = nodes[j].cost + _edge_cost(proposal, scene, obstacles)
            if cost < best_cost:
                best_parent, best_edge, best_cost = j, proposal, cost
        if best_parent is None:
            continue
        poses[n], progress[n] = sample, sample_s
        nodes.append(_Node(sample, best_parent, best_edge, best_cost))
        children.append(set())
        children[best_parent].add(n)
        # Reconnect to the exact original neighbour pose. Keeping endpoint
        # heading intact leaves its entire downstream subtree kinematically valid.
        for j in near[progress[near] > sample_s + .15]:
            old = nodes[j]
            if best_cost + distances[j] >= old.cost:
                continue
            proposal = _hermite(sample, old.pose, max_curvature)
            new_cost = best_cost + _edge_cost(proposal, scene, obstacles)
            if new_cost < old.cost - 1e-8:
                children[old.parent].remove(j)
                children[n].add(j)
                old.parent, old.edge = n, proposal
                _update_branch_costs(nodes, children, j, new_cost - old.cost)
                rewires += 1
        conn = _hermite(sample, goal, max_curvature)
        last_cost = _edge_cost(conn, scene, obstacles)
        if np.isfinite(last_cost):
            if not terminal:
                stats["first_solution_s"] = time.perf_counter() - started
                stats["first_solution_cost"] = best_cost + last_cost
            terminal[n] = (conn, last_cost)

    stats.update(nodes=len(nodes), rewires=rewires)
    if not terminal:
        return None
    index = min(terminal, key=lambda j: nodes[j].cost + terminal[j][1])
    stats["solution_cost"] = nodes[index].cost + terminal[index][1]
    return _assemble(nodes, index, terminal[index][0])


def plan_search(method, route, state, start_s, goal_s, scene, obstacles, rng, stats=None, max_curvature=MAX_CURVATURE):
    """Plan a forward local SE(2) path or return None if the work budget expires."""
    started = time.perf_counter()
    if stats is None:
        stats = {}
    if method not in ("hybrid_astar", "rrt_star"):
        raise ValueError(f"Unknown search planner {method!r}")
    start = np.array([state["x"], state["y"], state["yaw"]], dtype=float)
    goal = np.asarray(route.at_s(goal_s))[:3]
    if goal_s <= start_s or np.min(scene.clearance(np.vstack((start, goal)), obstacles)) < BODY_MARGIN:
        return None
    if method == "hybrid_astar":
        return _hybrid(start, goal, scene, obstacles, stats, started, max_curvature)
    return _rrt(start, goal, route, start_s, goal_s, scene, obstacles, rng, stats, started, max_curvature)
