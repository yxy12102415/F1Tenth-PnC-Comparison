#!/usr/bin/env python3
"""Replay archived Fitzpatrick control commands in Gym and audit 100 Hz clearances.

Usage: python3 Code/sim_ws/tools/replay_fitz_local.py CAMPAIGN [CAMPAIGN ...]
No planner, tracker, or obstacle detector runs. Scenario PNGs/YAML and nominal
circle positions come from the campaign archive. A launch/solver rejection has
no corresponding command to execute; only the recorded prefix can be replayed.
"""
import argparse
from collections import Counter
import csv
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np

from fitz_local_scene import LocalPlant
from fitz_tracking_plant import CONTROL_DT, PHYSICS_DT, MAP_BASE, ROUTE_CSV, Route

TOL = 1.e-7


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    return json.loads(path.read_text())


def signed_body_gaps(poses, placement, length, width):
    """Closest point on oriented rectangle to circle, signed inside the box."""
    dx = placement['center_xy'][0] - poses[:, 0]
    dy = placement['center_xy'][1] - poses[:, 1]
    c = np.cos(poses[:, 2]); s = np.sin(poses[:, 2])
    x = c*dx + s*dy; y = c*dy - s*dx
    px = np.clip(x, -length/2., length/2.)
    py = np.clip(y, -width/2., width/2.)
    inside = (x == px) & (y == py)
    radial = np.hypot(x-px, y-py)
    radial[inside] = -np.minimum(length/2.-np.abs(x[inside]), width/2.-np.abs(y[inside]))
    return radial - placement['desired_diameter_m']/2.


def close_optional(a, b):
    if a is None or b is None:
        return a is None and b is None
    return math.isfinite(a) and math.isfinite(b) and abs(a-b) <= TOL


def replay_trial(root, route, row, placements):
    method, case, seed = row['method'], row['scenario'], row['seed']
    folder = root/f'{method}_{case}_{seed}'
    summary_path, trace_path = folder/'summary.json', folder/'trace.csv.gz'
    summary = read_json(summary_path)
    errors = []
    if row != summary:
        errors.append('trial summary differs from campaign trials.json')
    plant = LocalPlant(route, root/'maps'/f'Fitzpatrick_{case}', seed=seed)
    physics, n_commands = [], 0
    max_position_error = max_yaw_error = max_speed_error = max_time_error = 0.
    flag_diff_count = 0
    try:
        length, width = plant.params['length'], plant.params['width']
        with gzip.open(trace_path, 'rt', newline='') as trace:
            for original in csv.DictReader(trace):
                speed = float(original['speed_cmd']); steer = float(original['steer_cmd'])
                if not (math.isfinite(speed) and math.isfinite(steer)):
                    raise ValueError(f'nonfinite command at sample {n_commands}')
                state = plant.step(speed, steer)
                physics.extend(plant.physics_poses)
                n_commands += 1
                dx = state['x']-float(original['x']); dy = state['y']-float(original['y'])
                yaw_error = abs(math.remainder(state['yaw']-float(original['yaw']), 2.*math.pi))
                sample_error = [math.hypot(dx,dy), yaw_error, abs(state['v']-float(original['v'])),
                                abs(plant.sim_time-float(original['t']))]
                if not all(math.isfinite(value) for value in sample_error):
                    raise ValueError(f'nonfinite state at sample {n_commands}')
                max_position_error = max(max_position_error, sample_error[0]); max_yaw_error = max(max_yaw_error, sample_error[1])
                max_speed_error = max(max_speed_error, sample_error[2]); max_time_error = max(max_time_error, sample_error[3])
                # Older archives store only the combined collision + wall flags.
                flags = ('collision', 'boundary', 'gym_collision', 'raster_obstacle_contact')
                if any(bool(state[name]) != bool(int(original[name])) for name in flags if name in original):
                    flag_diff_count += 1
        end_state = plant.state
        sim_time = plant.sim_time
    finally:
        plant.close()
    if max(max_position_error, max_yaw_error, max_speed_error, max_time_error)>TOL:
        errors.append('trajectory state/time differs from 20 Hz command trace')
    if flag_diff_count:
        errors.append(f'{flag_diff_count} collision/boundary flag mismatches')
    if n_commands!=row['command_ticks'] or not close_optional(row['simulated_s'],sim_time):
        errors.append('final simulated time or command count differs from summary')
    if len(physics)!=round(sim_time/PHYSICS_DT):
        errors.append('missing physics samples')
    expected_events={event['name']:event for event in row['encounters']}
    if len(expected_events)!=len(placements) or set(expected_events)!={p['name'] for p in placements}:
        errors.append('missing, duplicated or extra encounter in summary')
    poses = np.asarray(physics,dtype=float).reshape(-1,3)
    gaps = {}
    for p in placements:
        expected = expected_events.get(p['name'], {})
        if expected.get('center')!=p['center_xy'] or not close_optional(expected.get('radius'),p['desired_diameter_m']/2.):
            errors.append(f"{p['name']} geometry differs from manifest")
        gap = float(np.min(signed_body_gaps(poses, p, length, width))) if len(poses) else None
        gaps[p['name']] = gap
        if not close_optional(gap,expected.get('minimum')):
            errors.append(f"{p['name']} minimum gap mismatch (expected {expected.get('minimum')}, got {gap})")
    minimum = min((g for g in gaps.values() if g is not None),default=None)
    if not close_optional(minimum, row['min_body_gap_m']):
        errors.append('overall minimum gap differs from summary')
    collided = bool(end_state['collision']) or (minimum is not None and minimum<=0.)
    if collided!=row['collision'] or collided!=(row['status']=='collision') or bool(end_state['boundary'])!=row['boundary']:
        errors.append('final collision/boundary outcome differs from summary')
    return dict(method=method, scenario=case, seed=seed, recorded_status=row['status'],
                commands=n_commands, physics_samples=len(physics), trace_sha256=digest(trace_path),
                summary_sha256=digest(summary_path), max_position_error_m=max_position_error,
                max_yaw_error_rad=max_yaw_error, max_speed_error_mps=max_speed_error,
                max_timestamp_error_s=max_time_error, flag_mismatches=flag_diff_count,
                per_obstacle_min_body_gap_m=gaps, min_body_gap_m=minimum,
                collision=collided, boundary=bool(end_state['boundary']), valid=not errors, errors=errors)


def verify_asset(path, recorded):
    candidates = {v for k,v in recorded.items() if Path(k).name==path.name}
    if not candidates or digest(path) not in candidates:
        raise ValueError(f'file absent or changed from manifest: {path}')


def atomic_json(path, data):
    payload = json.dumps(data,indent=2,allow_nan=False)+'\n'
    if path.exists() and path.read_text()==payload:
        return
    with tempfile.NamedTemporaryFile(mode='w',dir=path.parent,delete=False) as out:
        name = out.name
        try:
            out.write(payload)
            out.flush()
            os.fsync(out.fileno())
        except BaseException:
            os.unlink(name)
            raise
    try:
        os.replace(name, path)
    finally:
        Path(name).unlink(missing_ok=True)


def replay_campaign(root):
    report = {'schema_version':1, 'physics_dt':PHYSICS_DT, 'tolerance':TOL,
              'validation_scope':'deterministic replay of executed commands only', 'trials':[], 'errors':[]}
    errors, trials = report['errors'], report['trials']
    try:
        sources = [root/'manifest.json', root/'trials.json']
        before = [digest(path) for path in sources]
        report['manifest_sha256'],report['trials_sha256'] = before
        manifest, rows = map(read_json, sources)
        schedule = [tuple(k) for k in manifest['schedule']]
        actual = [(r['method'],r['scenario'],r['seed']) for r in rows]
        report['scheduled_trials'] = len(schedule)
        if not schedule or Counter(actual)!=Counter(schedule) or len(schedule)!=len(set(schedule)):
            errors.append('missing, extra or duplicate scheduled/result trials')
        if {p.parent.name for p in root.glob('*/summary.json')}!={f'{m}_{c}_{s}' for m,c,s in schedule}:
            errors.append('extra or missing summary directories')
        if manifest['command_dt']!=CONTROL_DT or manifest['physics_dt']!=PHYSICS_DT:
            raise ValueError('archive uses an unsupported control/physics time step')
        recorded = manifest['source_sha256']
        verify_asset(ROUTE_CSV, recorded)
        report['base_assets_missing_from_original_manifest'] = []
        for path in (MAP_BASE.with_suffix('.png'), MAP_BASE.with_suffix('.yaml')):
            if any(Path(k).name == path.name for k in recorded):
                verify_asset(path, recorded)
            else:
                # Legacy pilots hashed every rendered world, but excluded the
                # obstacle-free base map used by their wall contact checker.
                report['base_assets_missing_from_original_manifest'].append(
                    dict(file=path.name, replay_sha256=digest(path)))
        for case in {c for _,c,_ in schedule}:
            for ext in ('.png','.yaml'):
                verify_asset(root/'maps'/f'Fitzpatrick_{case}{ext}',manifest['source_sha256'])
        route = Route()
        for row in rows:
            try:
                trial = replay_trial(root,route,row,manifest['placements'][row['scenario']])
            except Exception as exc:
                trial = dict(method=row['method'],scenario=row['scenario'],seed=row['seed'],valid=False,errors=[str(exc)])
            trials.append(trial)
        if before!=[digest(path) for path in sources]:
            errors.append('archive was modified while replay was running; rerun after producers stop')
    except Exception as exc:
        errors.append(str(exc))
    report['checked_trials'] = len(trials)
    report['physics_samples'] = sum(t.get('physics_samples',0) for t in trials)
    report['valid'] = not errors and all(t['valid'] for t in trials)
    atomic_json(root/'replay_validation.json',report)
    failed = [f"{t['method']}/{t['scenario']}/{t['seed']}: {t['errors']}" for t in trials if not t['valid']]
    print(f"{root}: {'PASS' if report['valid'] else 'FAIL'}, {len(trials)} trial(s), {report['physics_samples']} physics poses",flush=True)
    for msg in errors+failed:
        print('  '+msg,flush=True)
    return report['valid']


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaigns',type=Path,nargs='+')
    results = [replay_campaign(path.resolve()) for path in parser.parse_args().campaigns]
    raise SystemExit(not all(results))
