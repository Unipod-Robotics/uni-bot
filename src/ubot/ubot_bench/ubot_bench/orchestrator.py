"""bench: run a benchmark experiment (a set of trials) headless, isolated and resumable.

  ros2 run ubot_bench bench run pilot            # experiments/pilot.yaml
  ros2 run ubot_bench bench run sim_core --parallel 3
  ros2 run ubot_bench bench list sim_core        # show the trial matrix and what is done

A trial is (world, condition, stack, seed, phase). A navigation trial uses the map made by the
mapping trial with the same (world, condition, stack, seed), so the pairing is exact.

Each trial runs in its own ROS_DOMAIN_ID and GZ_PARTITION (nothing is shared with other trials
or with an interactive sim), in its own process group, and is killed completely at the end.
Outputs: <results>/<experiment>/<trial_id>/{result.json, *.tum, map.*, resources.json, *.log}.
A trial with a result.json whose status is not an infrastructure failure is never re-run.

Experiment file (experiments/<name>.yaml):
  name, worlds, stacks, conditions, seeds, phases (default both), parallel (default 2),
  slam_profile (default | tight, default 'default'), trial_timeout_s
"""
import argparse
import itertools
import json
import os
import signal
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import yaml

INFRA_FAIL = {'startup_timeout', 'crashed', 'runner_timeout', None}
# 'stuck' (the ground-truth mapping follower hit something) is a protocol failure: kept, reported,
# excluded from analysis, never silently retried.
ESTIMATION_PROCS = ('slam_toolbox', 'amcl', 'ekf_node', 'scan_model', 'depth_to_scan',
                    'controller_server', 'planner_server', 'bt_navigator', 'behavior_server',
                    'map_server', 'velocity_smoother', 'collision_monitor', 'smoother_server')


def share(*p):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('ubot_bench'), *p)


def results_root():
    return os.environ.get('UBOT_BENCH_RESULTS', os.path.expanduser('~/uni-bot/bench_results'))


def code_version():
    """Commit of the ~/uni-bot workspace repo and whether it had uncommitted changes."""
    repo = os.path.expanduser('~/uni-bot')
    try:
        head = subprocess.run(['git', '-C', repo, 'rev-parse', 'HEAD'], capture_output=True,
                              text=True, check=True).stdout.strip()
        dirty = bool(subprocess.run(['git', '-C', repo, 'status', '--porcelain', 'src/ubot'],
                                    capture_output=True, text=True).stdout.strip())
        return {'commit': head, 'dirty': dirty,
                'models_commit': open(os.path.join(repo, 'src/ubot/ubot_worlds/models_external',
                                                   '.commit')).read().strip()}
    except (OSError, subprocess.CalledProcessError):
        return {'commit': 'unknown'}


def load_experiment(name):
    path = name if os.path.exists(name) else share('experiments', f'{name}.yaml')
    with open(path) as f:
        exp = yaml.safe_load(f)
    exp.setdefault('phases', ['mapping', 'navigation'])
    exp.setdefault('parallel', 2)
    return exp


def trial_id(world, condition, stack, seed, phase):
    return f'{world}__{condition}__{stack}__s{seed:02d}__{phase}'


def trials(exp):
    out = []
    for phase in exp['phases']:
        for world, condition, stack, seed in itertools.product(
                exp['worlds'], exp['conditions'], exp['stacks'], exp['seeds']):
            if condition == 'low_light' and stack != 'OAKD':
                continue                     # C4 only affects camera stacks
            out.append(dict(world=world, condition=condition, stack=stack, seed=int(seed),
                            phase=phase, id=trial_id(world, condition, stack, int(seed), phase)))
    return out


def read_status(tdir):
    try:
        with open(os.path.join(tdir, 'result.json')) as f:
            return json.load(f).get('status')
    except (OSError, ValueError):
        return None


class Slots:
    """Unique ROS_DOMAIN_IDs (120..219) for concurrently running trials."""

    def __init__(self):
        self.free = list(range(120, 220))
        self.lock = threading.Lock()

    def take(self):
        with self.lock:
            return self.free.pop(0)

    def give(self, d):
        with self.lock:
            self.free.append(d)


def kill_partition(partition):
    """SIGKILL every process of this user tagged with the trial's GZ_PARTITION."""
    me = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open(f'/proc/{pid}/environ', 'rb') as f:
                if f'GZ_PARTITION={partition}'.encode() in f.read().split(b'\0'):
                    os.kill(int(pid), signal.SIGKILL)
        except (OSError, PermissionError):
            pass


def sample_resources(partition, stop, out):
    """CPU (% of one core) and RSS per process name for the trial's processes, every 2 s."""
    import psutil
    samples = {}
    procs = {}
    while not stop.is_set():
        for p in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if p.pid not in procs:
                    env = p.environ()
                    if env.get('GZ_PARTITION') != partition:
                        continue
                    procs[p.pid] = p
                    p.cpu_percent(None)
            except (psutil.Error, OSError):
                continue
        stop.wait(2.0)
        for pid, p in list(procs.items()):
            try:
                cmd = ' '.join(p.cmdline())
                name = next((n for n in ESTIMATION_PROCS if n in cmd), None) or \
                    ('gazebo' if 'gz sim' in cmd else 'other')
                cpu, rss = p.cpu_percent(None), p.memory_info().rss / 2 ** 20
                s = samples.setdefault(name, {'cpu': [], 'rss_mb': []})
                s['cpu'].append(cpu)
                s['rss_mb'].append(rss)
            except (psutil.Error, OSError):
                procs.pop(pid, None)
    summary = {k: {'cpu_mean': sum(v['cpu']) / max(len(v['cpu']), 1),
                   'rss_mb_max': max(v['rss_mb'] or [0])} for k, v in samples.items()}
    with open(out, 'w') as f:
        json.dump(summary, f, indent=2)


def run_trial(t, exp_dir, slots, log):
    tdir = os.path.join(exp_dir, t['id'])
    os.makedirs(tdir, exist_ok=True)
    args = [f"world:={t['world']}", f"stack:={t['stack']}", f"condition:={t['condition']}",
            f"seed:={t['seed']}", f"phase:={t['phase']}", 'headless:=true',
            f"slam_profile:={exp.get('slam_profile', 'default')}"]
    if t['phase'] == 'navigation':
        map_yaml = os.path.join(exp_dir, t['id'].replace('__navigation', '__mapping'), 'map.yaml')
        if not os.path.exists(map_yaml):
            with open(os.path.join(tdir, 'result.json'), 'w') as f:
                json.dump({'status': 'no_map', 'phase': 'navigation'}, f)
            log(f"{t['id']}: no map from its mapping trial, skipped")
            return 'no_map'
        args.append(f'map:={map_yaml}')
    domain = slots.take()
    partition = f"bench_{t['id']}_{os.getpid()}"
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), GZ_PARTITION=partition,
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    env.setdefault('__EGL_VENDOR_LIBRARY_FILENAMES',
                   '/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
    runner = 'mapping_runner' if t['phase'] == 'mapping' else 'mission_runner'
    stop = threading.Event()
    res_thread = threading.Thread(target=sample_resources,
                                  args=(partition, stop, os.path.join(tdir, 'resources.json')))
    t0 = time.time()
    status = None
    with open(os.path.join(tdir, 'launch.log'), 'w') as llog, \
            open(os.path.join(tdir, 'runner.log'), 'w') as rlog:
        launch = subprocess.Popen(['ros2', 'launch', 'ubot_bench', 'bench_sim.launch.py', *args],
                                  env=env, stdout=llog, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        res_thread.start()
        try:
            run = subprocess.run(['ros2', 'run', 'ubot_bench', runner, '--world', t['world'],
                                  '--out', tdir], env=env, stdout=rlog,
                                 stderr=subprocess.STDOUT, timeout=exp.get('trial_timeout_s', 7200),
                                 start_new_session=True)
            status = read_status(tdir) or ('crashed' if run.returncode else None)
        except subprocess.TimeoutExpired:
            status = 'runner_timeout'
        finally:
            stop.set()
            try:
                os.killpg(launch.pid, signal.SIGINT)
                launch.wait(timeout=20)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                pass
            kill_partition(partition)
            res_thread.join(timeout=10)
            slots.give(domain)
    if status in INFRA_FAIL and read_status(tdir) is None:
        with open(os.path.join(tdir, 'result.json'), 'w') as f:
            json.dump({'status': status or 'crashed', 'phase': t['phase']}, f)
    log(f"{t['id']}: {status} ({time.time() - t0:.0f} s)")
    return status


exp = {}


def check_installed_missions():
    """Abort if ubot_worlds' installed missions/maps differ from the source tree (it installs
    copies, so forgetting to rebuild after make_missions silently runs old routes)."""
    import filecmp
    from ament_index_python.packages import get_package_share_directory
    src = os.path.expanduser('~/uni-bot/src/ubot/ubot_worlds')
    inst = get_package_share_directory('ubot_worlds')
    stale = []
    for sub in ('missions', 'maps_gt', 'worlds'):
        for root, _, files in os.walk(os.path.join(src, sub)):
            for fn in files:
                a = os.path.join(root, fn)
                b = os.path.join(inst, os.path.relpath(a, src))
                if not os.path.exists(b) or not filecmp.cmp(a, b, shallow=False):
                    stale.append(os.path.relpath(a, src))
    if stale:
        raise SystemExit(f'ubot_worlds install is stale ({len(stale)} files, e.g. {stale[:3]}); '
                         'run: colcon build --packages-select ubot_worlds')


def cmd_run(a):
    global exp
    check_installed_missions()
    exp = load_experiment(a.experiment)
    exp_dir = os.path.join(results_root(), exp['name'])
    os.makedirs(exp_dir, exist_ok=True)
    # One entry per invocation, so a resumed experiment keeps the commit of every run.
    prev = {}
    try:
        with open(os.path.join(exp_dir, 'experiment.yaml')) as f:
            prev = yaml.safe_load(f) or {}
    except OSError:
        pass
    runs = prev.get('runs', [])
    if not runs and prev.get('code_version'):
        runs.append({'started': 'first run', **prev['code_version']})
    runs.append({'started': time.strftime('%Y-%m-%d %H:%M:%S'), **code_version()})
    exp['code_version'] = runs[-1]
    exp['runs'] = runs
    with open(os.path.join(exp_dir, 'experiment.yaml'), 'w') as f:
        yaml.safe_dump(exp, f, sort_keys=False)
    all_trials = trials(exp)
    lock = threading.Lock()
    logf = open(os.path.join(exp_dir, 'orchestrator.log'), 'a')

    def log(msg):
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        with lock:
            print(line, flush=True)
            logf.write(line + '\n')
            logf.flush()
    slots = Slots()
    parallel = a.parallel or exp['parallel']
    # Pair scheduling: each (world, condition, stack, seed) runs mapping then, if a map was saved,
    # its navigation trial straight away, so complete results accumulate from the first hour.
    maps = {t['id'].replace('__mapping', ''): t for t in all_trials if t['phase'] == 'mapping'}
    navs = {t['id'].replace('__navigation', ''): t for t in all_trials if t['phase'] == 'navigation'}
    keys = [k for k in maps] + [k for k in navs if k not in maps]

    def run_with_retry(t):
        if read_status(os.path.join(exp_dir, t['id'])) not in INFRA_FAIL:
            return
        for attempt in range(2):                     # one retry for infrastructure failures
            run_trial(t, exp_dir, slots, log)
            if read_status(os.path.join(exp_dir, t['id'])) not in INFRA_FAIL:
                return
            if attempt == 0:
                log(f"{t['id']}: infrastructure failure, retrying once")
                os.remove(os.path.join(exp_dir, t['id'], 'result.json'))

    def run_pair(key):
        if key in maps:
            run_with_retry(maps[key])
        if key in navs:
            run_with_retry(navs[key])

    todo = [k for k in keys if any(
        read_status(os.path.join(exp_dir, t['id'])) in INFRA_FAIL
        for t in (maps.get(k), navs.get(k)) if t)]
    log(f'{len(todo)} of {len(keys)} (world, condition, stack, seed) pairs to run, '
        f'{parallel} in parallel')
    with ThreadPoolExecutor(max_workers=parallel) as pool:
        list(pool.map(run_pair, todo))
    log('done')


def cmd_list(a):
    e = load_experiment(a.experiment)
    exp_dir = os.path.join(results_root(), e['name'])
    ts = trials(e)
    counts = {}
    for t in ts:
        s = read_status(os.path.join(exp_dir, t['id'])) or 'pending'
        counts[s] = counts.get(s, 0) + 1
        if a.verbose:
            print(f"{t['id']:70s} {s}")
    print(f"{e['name']}: {len(ts)} trials {counts} -> {exp_dir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('experiment')
    r.add_argument('--parallel', type=int)
    r.set_defaults(fn=cmd_run)
    ls = sub.add_parser('list')
    ls.add_argument('experiment')
    ls.add_argument('-v', '--verbose', action='store_true')
    ls.set_defaults(fn=cmd_list)
    a = ap.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
