#!/bin/bash
# Stop running benchmark experiments safely (before a shutdown, or to free the machine).
#   ~/uni-bot/src/ubot/ubot_bench/scripts/stop_bench.sh
# 1. stops every user service named bench-* (the orchestrators started via run_experiment.sh)
# 2. kills every trial process (each is tagged GZ_PARTITION=bench_<trial> in its environment)
# 3. lists trial folders left without a final status; `bench run <exp>` re-runs those on resume.
for u in $(systemctl --user list-units --type=service --state=active --plain --no-legend 'bench-*' | awk '{print $1}'); do
  echo "stopping $u"; systemctl --user stop "$u"
done
sleep 2
n=0
for d in /proc/[0-9]*; do
  if { tr '\0' '\n' < "$d/environ"; } 2>/dev/null | grep -q '^GZ_PARTITION=bench_'; then
    kill -9 "${d#/proc/}" 2>/dev/null && n=$((n+1))
  fi
done
echo "killed $n trial processes"
for r in "$HOME"/uni-bot/bench_results/*/; do
  for t in "$r"*__*/; do
    [ -d "$t" ] || continue
    s=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('status'))" "$t/result.json" 2>/dev/null)
    case "$s" in ''|None|startup_timeout|crashed|runner_timeout) echo "unfinished (re-run on resume): ${t#$HOME/}";; esac
  done
done
