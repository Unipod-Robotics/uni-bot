"""Statistical analysis plan (PROTOCOL.md, "Statistics"), implemented once for sim and real data.

Design: paired. Within a (world, condition) cell every stack runs the same seeds, so stacks are
compared seed-by-seed.

  summary        median, IQR and a percentile-bootstrap 95% CI of the median (10,000 resamples,
                 fixed RNG) per (world, condition, stack); Wilson 95% CI for success rates
  omnibus        Friedman test across stacks per cell (complete seeds only)
  pairwise       Wilcoxon signed-rank, each budget stack vs REF, per cell; Holm correction over
                 all comparisons of one metric; effect size Cliff's delta (paired sign) and the
                 median of paired differences
  success        Cochran's Q across stacks on per-(seed, goal) outcomes; McNemar exact vs REF
  mixed model    value ~ C(stack, Treatment('REF')) * C(condition), random intercept per world
                 (statsmodels MixedLM), reported as a robustness check
  power          from pilot data: paired n needed to detect a 20% change of the REF median at
                 alpha 0.05 and power 0.8 (paired t, / 0.955 Wilcoxon ARE correction)
"""
import itertools
import math

import numpy as np
import pandas as pd
from scipy import stats

RNG = np.random.default_rng(20260930)
TRIAL_METRICS = ['slam_ate_rmse', 'slam_ate_anchor_rmse', 'slam_rpe_t_1m', 'slam_rpe_r_1m',
                 'map_f1', 'map_precision', 'map_recall', 'wall_offset_mean', 'free_iou',
                 'odom_ate_anchor_rmse', 'loc_ate_rmse', 'success_rate', 'spl',
                 'time_per_goal_s', 'recoveries', 'contacts', 'false_success',
                 'cpu_estimation_pct']
LOWER_IS_BETTER = {'slam_ate_rmse', 'slam_ate_anchor_rmse', 'slam_rpe_t_1m', 'slam_rpe_r_1m',
                   'wall_offset_mean', 'odom_ate_anchor_rmse', 'loc_ate_rmse', 'time_per_goal_s',
                   'recoveries', 'contacts', 'false_success', 'cpu_estimation_pct'}


def boot_median_ci(x, n=10000, alpha=0.05):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size < 2:
        return (math.nan, math.nan)
    idx = RNG.integers(0, x.size, size=(n, x.size))
    meds = np.median(x[idx], axis=1)
    return tuple(np.quantile(meds, [alpha / 2, 1 - alpha / 2]))


def wilson(k, n, z=1.959964):
    if n == 0:
        return (math.nan, math.nan)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (c - h, c + h)


def cliffs_delta(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    if a.size == 0 or b.size == 0:
        return math.nan
    gt = (a[:, None] > b[None, :]).sum()
    lt = (a[:, None] < b[None, :]).sum()
    return float((gt - lt) / (a.size * b.size))


def holm(pvals):
    p = np.asarray(pvals, dtype=float)
    out = np.full(p.shape, np.nan)
    ok = np.isfinite(p)
    order = np.argsort(p[ok])
    m = ok.sum()
    adj = np.empty(m)
    running = 0.0
    for rank, i in enumerate(order):
        running = max(running, (m - rank) * p[ok][i])
        adj[i] = min(1.0, running)
    out[ok] = adj
    return out


def summary(trials, metrics=None):
    rows = []
    metrics = [m for m in (metrics or TRIAL_METRICS) if m in trials]
    for (w, c, s), g in trials.groupby(['world', 'condition', 'stack']):
        for m in metrics:
            x = g[m].dropna().astype(float)
            if x.empty:
                continue
            lo, hi = boot_median_ci(x)
            rows.append({'world': w, 'condition': c, 'stack': s, 'metric': m, 'n': len(x),
                         'median': x.median(), 'q1': x.quantile(.25), 'q3': x.quantile(.75),
                         'ci_lo': lo, 'ci_hi': hi, 'mean': x.mean()})
    return pd.DataFrame(rows)


def success_summary(goals):
    rows = []
    for (w, c, s), g in goals.groupby(['world', 'condition', 'stack']):
        k, n = int(g['success'].sum()), len(g)
        lo, hi = wilson(k, n)
        rows.append({'world': w, 'condition': c, 'stack': s, 'successes': k, 'goals': n,
                     'rate': k / n if n else math.nan, 'ci_lo': lo, 'ci_hi': hi,
                     'false_success': int((g['nav_says_success'] & ~g['success']).sum()),
                     'collisions': int((g['contacts'] > 0).sum())})
    return pd.DataFrame(rows)


def pairwise_vs_ref(trials, ref='REF', metrics=None):
    rows = []
    metrics = [m for m in (metrics or TRIAL_METRICS) if m in trials]
    for m in metrics:
        block = []
        for (w, c), g in trials.groupby(['world', 'condition']):
            wide = g.pivot_table(index='seed', columns='stack', values=m, aggfunc='first')
            if ref not in wide:
                continue
            stacks = [s for s in wide.columns if s != ref]
            complete = wide.dropna()
            fried_p = math.nan
            if len(stacks) >= 2 and len(complete) >= 3:
                try:
                    fried_p = stats.friedmanchisquare(*[complete[s] for s in wide.columns]).pvalue
                except ValueError:
                    pass
            for s in stacks:
                pair = wide[[ref, s]].dropna()
                p = math.nan
                if len(pair) >= 3 and (pair[s] - pair[ref]).abs().sum() > 0:
                    p = stats.wilcoxon(pair[s], pair[ref], zero_method='wilcox').pvalue
                block.append({'metric': m, 'world': w, 'condition': c, 'stack': s, 'ref': ref,
                              'n_pairs': len(pair), 'median_ref': pair[ref].median(),
                              'median_stack': pair[s].median(),
                              'median_diff': (pair[s] - pair[ref]).median(),
                              'cliffs_delta': cliffs_delta(pair[s], pair[ref]),
                              'p_wilcoxon': p, 'p_friedman_cell': fried_p})
        if block:
            adj = holm([b['p_wilcoxon'] for b in block])
            for b, pa in zip(block, adj):
                b['p_holm'] = pa
            rows.extend(block)
    return pd.DataFrame(rows)


def success_tests(goals, ref='REF'):
    """Cochran's Q across stacks and exact McNemar vs REF, on per-(seed, goal) outcomes."""
    rows = []
    for (w, c), g in goals.groupby(['world', 'condition']):
        wide = g.pivot_table(index=['seed', 'goal'], columns='stack', values='success',
                             aggfunc='first').dropna().astype(int)
        if ref not in wide or wide.shape[1] < 2:
            continue
        q_p = math.nan
        if wide.shape[1] >= 3:
            from statsmodels.stats.contingency_tables import cochrans_q
            q_p = float(cochrans_q(wide.values).pvalue)
        for s in [x for x in wide.columns if x != ref]:
            from statsmodels.stats.contingency_tables import mcnemar
            b = int(((wide[ref] == 1) & (wide[s] == 0)).sum())
            cc = int(((wide[ref] == 0) & (wide[s] == 1)).sum())
            p = float(mcnemar([[0, b], [cc, 0]], exact=True).pvalue) if b + cc else 1.0
            rows.append({'world': w, 'condition': c, 'stack': s, 'n': len(wide),
                         'ref_only': b, 'stack_only': cc, 'p_mcnemar': p, 'p_cochran_q': q_p})
    out = pd.DataFrame(rows)
    if not out.empty:
        out['p_holm'] = holm(out['p_mcnemar'])
    return out


def mixed_model(trials, metric, ref='REF'):
    import statsmodels.formula.api as smf
    d = trials[['world', 'condition', 'stack', metric]].dropna()
    if d['world'].nunique() < 2 or d['stack'].nunique() < 2:
        return None
    terms = f"C(stack, Treatment('{ref}'))"
    if d['condition'].nunique() > 1:
        terms += ' * C(condition)'
    try:
        fit = smf.mixedlm(f'{metric} ~ {terms}', d, groups=d['world']).fit(reml=True)
    except Exception as e:                                  # singular fits on tiny pilots
        return {'metric': metric, 'error': str(e)}
    return {'metric': metric, 'params': fit.params.to_dict(), 'pvalues': fit.pvalues.to_dict(),
            'converged': bool(fit.converged)}


def power_table(trials, ref='REF', effect=0.20, alpha=0.05, power=0.8, metrics=None):
    """Paired n to detect a change of `effect` x median(REF), per (world, metric, stack)."""
    z = stats.norm.ppf(1 - alpha / 2) + stats.norm.ppf(power)
    rows = []
    metrics = [m for m in (metrics or TRIAL_METRICS) if m in trials]
    for m in metrics:
        for (w, c), g in trials.groupby(['world', 'condition']):
            wide = g.pivot_table(index='seed', columns='stack', values=m, aggfunc='first')
            if ref not in wide:
                continue
            for s in [x for x in wide.columns if x != ref]:
                pair = wide[[ref, s]].dropna()
                if len(pair) < 3:
                    continue
                sd = float((pair[s] - pair[ref]).std(ddof=1))
                delta = effect * abs(float(pair[ref].median()))
                n = math.ceil((z * sd / delta) ** 2 / 0.955) if delta > 0 and sd > 0 else math.nan
                rows.append({'metric': m, 'world': w, 'condition': c, 'stack': s,
                             'n_pilot': len(pair), 'sd_diff': sd, 'target_delta': delta,
                             'n_required': n})
    return pd.DataFrame(rows)


def all_pairs(stacks):
    return list(itertools.combinations(stacks, 2))
