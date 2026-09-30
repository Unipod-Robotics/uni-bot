"""bench_analyze: metrics, statistics, figures and LaTeX tables for one experiment.

  ~/uni-bot/.venv-bench/bin/python -m ubot_bench.analysis.report pilot
  (or: ros2 run ubot_bench bench_analyze pilot, if evo/statsmodels are importable)

Writes into <results>/<experiment>/analysis/:
  trials.csv, goals.csv          (also in the experiment dir)
  summary.csv                    median / IQR / bootstrap CI per cell and metric
  success.csv                    success rates with Wilson CIs, false successes, collisions
  pairwise_vs_ref.csv            Wilcoxon vs REF, Holm, Cliff's delta, Friedman per cell
  success_tests.csv              McNemar vs REF, Cochran's Q
  mixed_models.json              LMM robustness check per metric
  power.csv                      paired n for a 20% effect (meaningful on the pilot)
  figures/*.pdf|png, tables/*.tex
"""
import argparse
import json
import os

import pandas as pd

from ubot_bench.analysis import figures as F
from ubot_bench.analysis import stats as S
from ubot_bench.analysis.collect import collect


def latex_summary(summ, metric, label, out, digits=3):
    d = summ[(summ['metric'] == metric) & (summ['condition'] == 'nominal')]
    if d.empty:
        return
    worlds = [w for w in ['arena_5x5', 'small_house', 'bookstore', 'small_warehouse']
              if w in set(d['world'])]
    stacks = [s for s in F.STACK_ORDER if s in set(d['stack'])]
    lines = [r'\begin{tabular}{l' + 'c' * len(worlds) + '}', r'\toprule',
             'Stack & ' + ' & '.join(w.replace('_', r'\_') for w in worlds) + r' \\',
             r'\midrule']
    for s in stacks:
        cells = []
        for w in worlds:
            r = d[(d['world'] == w) & (d['stack'] == s)]
            cells.append('--' if r.empty else
                         f"{r['median'].iloc[0]:.{digits}f} [{r['q1'].iloc[0]:.{digits}f}, "
                         f"{r['q3'].iloc[0]:.{digits}f}]")
        lines.append(f'{F.LABELS[s]} & ' + ' & '.join(cells) + r' \\')
    lines += [r'\bottomrule', r'\end{tabular}',
              f'% {label}: median [IQR] over seeds, nominal condition']
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, f'{metric}.tex'), 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('experiment')
    ap.add_argument('--results', default=os.environ.get(
        'UBOT_BENCH_RESULTS', os.path.expanduser('~/uni-bot/bench_results')))
    a = ap.parse_args()
    exp_dir = os.path.join(a.results, a.experiment)
    out = os.path.join(exp_dir, 'analysis')
    os.makedirs(out, exist_ok=True)

    trials, goals = collect(exp_dir)
    trials.to_csv(os.path.join(out, 'trials.csv'), index=False)
    goals.to_csv(os.path.join(out, 'goals.csv'), index=False)
    mapping = trials[trials['phase'] == 'mapping']
    nav = trials[trials['phase'] == 'navigation']

    summ = pd.concat([S.summary(mapping), S.summary(nav)], ignore_index=True)
    summ.to_csv(os.path.join(out, 'summary.csv'), index=False)
    pw = pd.concat([S.pairwise_vs_ref(mapping), S.pairwise_vs_ref(nav)], ignore_index=True)
    pw.to_csv(os.path.join(out, 'pairwise_vs_ref.csv'), index=False)
    S.power_table(pd.concat([mapping, nav])).to_csv(os.path.join(out, 'power.csv'), index=False)
    if not goals.empty:
        succ = S.success_summary(goals)
        succ.to_csv(os.path.join(out, 'success.csv'), index=False)
        S.success_tests(goals).to_csv(os.path.join(out, 'success_tests.csv'), index=False)
    else:
        succ = pd.DataFrame()
    mm = [S.mixed_model(mapping, m) for m in ('slam_ate_rmse', 'map_f1') if m in mapping]
    mm += [S.mixed_model(nav, m) for m in ('spl', 'loc_ate_rmse') if m in nav]
    with open(os.path.join(out, 'mixed_models.json'), 'w') as f:
        json.dump([m for m in mm if m], f, indent=2, default=str)

    figs = os.path.join(out, 'figures')
    if not mapping.empty:
        F.per_world_strip(mapping, 'slam_ate_rmse', 'SLAM ATE RMSE (m)', figs, 'slam_ate')
        F.per_world_strip(mapping, 'map_f1', 'map F1 (walls, 10 cm)', figs, 'map_f1')
        F.cost_pareto(mapping, 'slam_ate_rmse', 'SLAM ATE RMSE (m), median [IQR]', figs)
        F.degradation_heatmap(mapping, 'slam_ate_rmse', figs, 'degradation_ate',
                              'SLAM ATE relative to nominal (median paired ratio)')
    if not nav.empty:
        F.per_world_strip(nav, 'spl', 'SPL', figs, 'spl')
        F.per_world_strip(nav, 'loc_ate_rmse', 'AMCL ATE RMSE (m)', figs, 'loc_ate')
    if not succ.empty:
        F.success_bars(succ, figs)
    tabs = os.path.join(out, 'tables')
    for m, lab in (('slam_ate_rmse', 'SLAM ATE RMSE (m)'), ('map_f1', 'Map F1'),
                   ('spl', 'SPL'), ('loc_ate_rmse', 'AMCL ATE RMSE (m)')):
        latex_summary(summ, m, lab, tabs)
    print(f'{a.experiment}: {len(trials)} trials, {len(goals)} goals -> {out}')


if __name__ == '__main__':
    main()
