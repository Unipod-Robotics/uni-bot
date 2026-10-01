"""Collect per-trial and per-goal metrics from an experiment's results directory.

trials.csv  one row per trial (mapping or navigation)
goals.csv   one row per navigation goal

Ground truth: maps_gt/<world>.yaml (scan plane) for map quality, maps_gt/<world>_traversable.yaml
for SPL shortest paths and clearance.
"""
import json
import math
import os
from functools import lru_cache

import numpy as np
import pandas as pd
import yaml

from ubot_bench.analysis import metrics as M
from ubot_bench.grid import Grid
from ubot_bench.planning import ROBOT_RADIUS, clearance_field, shortest_path_length
from ubot_bench.profiles import STACKS, load_profiles


def _share(pkg, *p):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(pkg), *p)


@lru_cache(maxsize=None)
def gt_plane(world):
    return Grid.load(_share('ubot_worlds', 'maps_gt', f'{world}.yaml'))


@lru_cache(maxsize=None)
def gt_trav(world):
    return Grid.load(_share('ubot_worlds', 'maps_gt', f'{world}_traversable.yaml'))


@lru_cache(maxsize=None)
def feasible(world):
    return clearance_field(gt_trav(world)) >= ROBOT_RADIUS


@lru_cache(maxsize=None)
def mission(world):
    with open(_share('ubot_worlds', 'missions', f'{world}.yaml')) as f:
        return yaml.safe_load(f)


@lru_cache(maxsize=None)
def observable(world, rng_max):
    route = np.asarray(mission(world)['mapping_route'])
    return M.observable_mask(gt_plane(world), route, rng_max)


@lru_cache(maxsize=4096)
def spl_shortest(world, sx, sy, gx, gy):
    return shortest_path_length(gt_trav(world), feasible(world), (sx, sy), (gx, gy))


def parse_id(tid):
    world, condition, stack, seed, phase = tid.split('__')
    return dict(world=world, condition=condition, stack=stack, seed=int(seed[1:]), phase=phase)


def _cpu(tdir):
    try:
        with open(os.path.join(tdir, 'resources.json')) as f:
            r = json.load(f)
    except (OSError, ValueError):
        return {}
    est = sum(v['cpu_mean'] for k, v in r.items() if k not in ('gazebo', 'other'))
    return {'cpu_estimation_pct': est,
            'cpu_slam_pct': r.get('slam_toolbox', {}).get('cpu_mean', math.nan),
            'cpu_amcl_pct': r.get('amcl', {}).get('cpu_mean', math.nan),
            'rss_estimation_mb': sum(v['rss_mb_max'] for k, v in r.items()
                                     if k not in ('gazebo', 'other'))}


def collect(exp_dir):
    profiles = load_profiles()
    trial_rows, goal_rows = [], []
    for tid in sorted(os.listdir(exp_dir)):
        tdir = os.path.join(exp_dir, tid)
        rpath = os.path.join(tdir, 'result.json')
        if not os.path.isdir(tdir) or not os.path.exists(rpath):
            continue
        meta = parse_id(tid)
        with open(rpath) as f:
            res = json.load(f)
        prof = profiles[STACKS[meta['stack']]]
        row = {'trial': tid, **meta, 'status': res.get('status'),
               'price_usd': prof.get('price_usd'), 'sensor_range_m': prof['range_max'],
               'sim_duration_s': res.get('sim_duration_s'),
               'wall_duration_s': res.get('wall_duration_s'),
               'contacts': len(res.get('contacts', [])), **_cpu(tdir)}
        gt = os.path.join(tdir, 'gt.tum')
        if meta['phase'] == 'mapping' and res.get('status') in ('ok', 'timeout'):
            s = M.trajectory_metrics(gt, os.path.join(tdir, 'slam.tum'))
            align = s.pop('_align', None)
            row.update({f'slam_{k}': v for k, v in s.items()})
            o = M.trajectory_metrics(gt, os.path.join(tdir, 'odom.tum'))
            o.pop('_align', None)
            row.update({f'odom_{k}': v for k, v in o.items()})
            mpath = os.path.join(tdir, 'map.yaml')
            if align and os.path.exists(mpath):
                row.update(M.map_metrics(gt_plane(meta['world']), Grid.load(mpath), align,
                                         observable(meta['world'], float(prof['range_max']))))
            row['completed_route'] = res.get('status') == 'ok'
        elif meta['phase'] == 'navigation' and res.get('status') == 'ok':
            a = M.trajectory_metrics(gt, os.path.join(tdir, 'amcl.tum'))
            a.pop('_align', None)
            row.update({f'loc_{k}': v for k, v in a.items()})
            goals = res.get('goals', [])
            spl_terms = []
            for g in goals:
                sx, sy = g['start_xy']
                shortest = spl_shortest(meta['world'], round(sx, 2), round(sy, 2),
                                        round(g['goal'][0], 2), round(g['goal'][1], 2))
                spl = (float(g['success']) * shortest / max(shortest, g['gt_path_m'])
                       if math.isfinite(shortest) and shortest > 0 else math.nan)
                spl_terms.append(spl)
                goal_rows.append({'trial': tid, **meta, 'goal': g['index'],
                                  'success': g['success'], 'nav_status': g['nav_status'],
                                  # position-only success: separates "reached the place" from the
                                  # final in-place turn, which shifts the base ~0.25 m (pivot)
                                  'success_pos': bool(g['final_err_m'] <= 0.25),
                                  'nav_says_success': g['nav_status'] == 'SUCCEEDED',
                                  'time_s': g['time_s'], 'gt_path_m': g['gt_path_m'],
                                  'shortest_m': shortest, 'spl': spl,
                                  'final_err_m': g['final_err_m'],
                                  'final_err_rad': g['final_err_rad'],
                                  'recoveries': g['recoveries'], 'contacts': g['contacts'],
                                  'reset_after': g.get('reset_after', False)})
            if goals:
                row['success_rate'] = float(np.mean([g['success'] for g in goals]))
                row['success_pos_rate'] = float(np.mean([g['final_err_m'] <= 0.25 for g in goals]))
                row['spl'] = float(np.nanmean(spl_terms))
                row['time_per_goal_s'] = float(np.mean([g['time_s'] for g in goals]))
                row['recoveries'] = int(sum(g['recoveries'] for g in goals))
                row['resets'] = int(sum(g.get('reset_after', False) for g in goals))
                row['false_success'] = int(sum(g['nav_status'] == 'SUCCEEDED' and
                                               not g['success'] for g in goals))
        trial_rows.append(row)
    trials = pd.DataFrame(trial_rows)
    goals = pd.DataFrame(goal_rows)
    trials.to_csv(os.path.join(exp_dir, 'trials.csv'), index=False)
    goals.to_csv(os.path.join(exp_dir, 'goals.csv'), index=False)
    return trials, goals
