"""Paired comparison of two experiments that share worlds, stacks, conditions and seeds
(e.g. sim_core vs sim_core_tight: SLAM thresholds 0.5 m / 0.5 rad vs 0.1 m / 0.1 rad).

  ~/uni-bot/.venv-bench/bin/python -m ubot_bench.analysis.compare sim_core sim_core_tight

Each experiment must already have analysis/trials.csv (run analysis.report first). Writes
<results>/compare_<A>__<B>.csv: per (world, condition, stack, metric) the paired median
difference B - A, Wilcoxon signed-rank p, Holm-corrected p (per metric) and Cliff's delta.
"""
import argparse
import math
import os

import pandas as pd
from scipy import stats as st

from ubot_bench.analysis.stats import TRIAL_METRICS, cliffs_delta, holm


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('a')
    ap.add_argument('b')
    ap.add_argument('--results', default=os.environ.get(
        'UBOT_BENCH_RESULTS', os.path.expanduser('~/uni-bot/bench_results')))
    args = ap.parse_args()
    ta = pd.read_csv(os.path.join(args.results, args.a, 'analysis', 'trials.csv'))
    tb = pd.read_csv(os.path.join(args.results, args.b, 'analysis', 'trials.csv'))
    key = ['world', 'condition', 'stack', 'seed', 'phase']
    m = ta.merge(tb, on=key, suffixes=('_a', '_b'))
    rows = []
    for metric in TRIAL_METRICS:
        if f'{metric}_a' not in m:
            continue
        block = []
        for (w, c, s), g in m.groupby(['world', 'condition', 'stack']):
            pair = g[[f'{metric}_a', f'{metric}_b']].dropna()
            if pair.empty:
                continue
            d = pair[f'{metric}_b'] - pair[f'{metric}_a']
            p = math.nan
            if len(pair) >= 3 and d.abs().sum() > 0:
                p = st.wilcoxon(pair[f'{metric}_b'], pair[f'{metric}_a']).pvalue
            block.append({'metric': metric, 'world': w, 'condition': c, 'stack': s,
                          'n_pairs': len(pair), f'median_{args.a}': pair[f'{metric}_a'].median(),
                          f'median_{args.b}': pair[f'{metric}_b'].median(),
                          'median_diff_b_minus_a': d.median(),
                          'cliffs_delta': cliffs_delta(pair[f'{metric}_b'], pair[f'{metric}_a']),
                          'p_wilcoxon': p})
        for b_, ph in zip(block, holm([b_['p_wilcoxon'] for b_ in block])):
            b_['p_holm'] = ph
        rows.extend(block)
    out = os.path.join(args.results, f'compare_{args.a}__{args.b}.csv')
    pd.DataFrame(rows).to_csv(out, index=False)
    print(f'{len(rows)} paired comparisons -> {out}')


if __name__ == '__main__':
    main()
