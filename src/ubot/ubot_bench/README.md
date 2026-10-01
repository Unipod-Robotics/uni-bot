# ubot_bench: low-cost indoor navigation benchmark

This package holds the simulation half of the study described in [docs/PROTOCOL.md](docs/PROTOCOL.md).
To rebuild the whole setup from scratch, and for a log of every change made, see
[docs/REPRODUCE.md](docs/REPRODUCE.md). To stop experiments before a shutdown and continue them
afterwards, see REPRODUCE.md section 5.1 (`scripts/stop_bench.sh`, `scripts/run_experiment.sh`).
It covers sensor noise models, ground-truth maps and trajectories, the mapping and navigation
runners, an experiment orchestrator, and the metrics and statistics pipeline. The worlds live
in `ubot_worlds`.

## One-time setup
```bash
cd ~/uni-bot
src/ubot/ubot_worlds/scripts/fetch_models.sh          # AWS world models, pinned commit
conda deactivate                                      # conda breaks colcon's python
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
python3 -m venv --system-site-packages .venv-bench    # analysis deps (evo, statsmodels)
.venv-bench/bin/pip install -r src/ubot/ubot_bench/requirements-bench.txt
```
`ubot_worlds` and `ubot_description` install copies of their files, not symlinks, so rebuild
them after editing a world, mission or URDF.

## Ground truth and missions (already generated and committed)
```bash
ros2 run ubot_bench gt_map --world small_house        # maps_gt/<world>.yaml + _traversable
ros2 run ubot_bench make_missions --world small_house # missions/<world>.yaml + .png preview
```

## Run experiments
Close any interactive Gazebo first: it competes for CPU.
```bash
source ~/uni-bot/install/setup.bash
ros2 run ubot_bench bench run pilot                   # experiments/pilot.yaml
ros2 run ubot_bench bench run sim_core --parallel 3
ros2 run ubot_bench bench list sim_core -v            # progress per trial
```
- Every trial is isolated (its own `ROS_DOMAIN_ID` and `GZ_PARTITION`), headless, and resumable.
- Results go to `~/uni-bot/bench_results/<experiment>/`, or to `$UBOT_BENCH_RESULTS` if set.

## Analyse
```bash
~/uni-bot/.venv-bench/bin/python -m ubot_bench.analysis.report pilot
```
This writes `analysis/` into the experiment folder: tables (CSV and LaTeX), statistics and
paper figures.

## One trial by hand
```bash
ros2 launch ubot_bench bench_sim.launch.py world:=arena_5x5 stack:=MS200 phase:=mapping headless:=false
ros2 run ubot_bench mapping_runner --world arena_5x5 --out /tmp/t1
ros2 launch ubot_bench bench_sim.launch.py world:=arena_5x5 stack:=MS200 phase:=navigation map:=/tmp/t1/map.yaml
ros2 run ubot_bench mission_runner --world arena_5x5 --out /tmp/t1_nav
```

## Stacks
Stacks map to `ubot_description/config/sensor_profiles.yaml`:

| Stack | Sensor |
|---|---|
| `REF` | Hokuyo UST-10LX (sim only) |
| `MS200` | Oradar MS200 |
| `LD06` | LDROBOT LD06 |
| `OAKD` | OAK-D Lite depth → scan |

The odometry-only baseline is logged in every mapping trial (`odom.tum`).
