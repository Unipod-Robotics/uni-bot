#!/bin/bash
# Run one benchmark experiment and then its analysis; meant to be started as a user service so it
# survives closing terminals / editor sessions:
#
#   systemd-run --user --collect --unit=bench-<exp> --working-directory=$HOME/uni-bot \
#       ~/uni-bot/src/ubot/ubot_bench/scripts/run_experiment.sh <exp> [parallel]
#
# Log: ~/uni-bot/bench_results/<exp>.log    Status: systemctl --user status bench-<exp>
# (no `set -u`: the ROS setup scripts reference unset variables)
EXP=$1
PAR=${2:-}
mkdir -p "$HOME/uni-bot/bench_results"
exec >> "$HOME/uni-bot/bench_results/$EXP.log" 2>&1
source /opt/ros/jazzy/setup.bash
source "$HOME/uni-bot/install/setup.bash"
echo "=== bench run $EXP $(date)"
ros2 run ubot_bench bench run "$EXP" ${PAR:+--parallel $PAR}
echo "=== analysis $(date)"
"$HOME/uni-bot/.venv-bench/bin/python" -m ubot_bench.analysis.report "$EXP"
echo "=== done $(date)"
