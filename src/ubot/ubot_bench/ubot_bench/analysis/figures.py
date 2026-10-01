"""Paper figures (PDF + PNG). Stack identity always has a fixed colour AND a visible label or
marker shape (the palette's light slots are below 3:1 contrast, so colour is never the only cue).

Palette: categorical slots 1-4 of the reference data-viz palette in fixed order, validated for
4 adjacent series (see the validator output in REPRODUCE.md).
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

STACK_ORDER = ['REF', 'MS200', 'LD06', 'OAKD']
COLORS = {'REF': '#2a78d6', 'MS200': '#eb6834', 'LD06': '#1baf7a', 'OAKD': '#eda100'}
MARKERS = {'REF': 'o', 'MS200': 's', 'LD06': '^', 'OAKD': 'v'}
LABELS = {'REF': 'REF (UST-10LX)', 'MS200': 'MS200', 'LD06': 'LD06', 'OAKD': 'OAK-D Lite'}
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e6e5e0'


def _style():
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8,
                         'axes.edgecolor': '#b9b8b2', 'axes.labelcolor': INK2,
                         'xtick.color': INK2, 'ytick.color': INK2, 'axes.titlesize': 9,
                         'axes.titlecolor': INK, 'axes.titlelocation': 'left'})


def _clean(ax):
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.grid(axis='y', color=GRID, linewidth=0.6, zorder=0)
    ax.tick_params(length=0)


def _save(fig, out, name):
    os.makedirs(out, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(out, f'{name}.{ext}'), dpi=300, bbox_inches='tight',
                    facecolor='white')
    plt.close(fig)


def per_world_strip(trials, metric, ylabel, out, name, condition='nominal', log=False):
    """One panel per world: per-stack points (one per seed) + median bar."""
    _style()
    d = trials[(trials['condition'] == condition) & trials[metric].notna()]
    worlds = [w for w in ['arena_5x5', 'small_house', 'bookstore', 'small_warehouse']
              if w in set(d['world'])]
    if not worlds:
        return
    fig, axes = plt.subplots(1, len(worlds), figsize=(1.8 * len(worlds) + 0.6, 2.3),
                             sharey=True, squeeze=False)
    rng = np.random.default_rng(0)
    for ax, w in zip(axes[0], worlds):
        stacks = [s for s in STACK_ORDER if s in set(d[d['world'] == w]['stack'])]
        for i, s in enumerate(stacks):
            v = d[(d['world'] == w) & (d['stack'] == s)][metric].astype(float).values
            ax.scatter(i + rng.uniform(-0.15, 0.15, v.size), v, s=10, color=COLORS[s],
                       marker=MARKERS[s], alpha=0.75, linewidths=0, zorder=3)
            ax.hlines(np.median(v), i - 0.3, i + 0.3, color=INK, linewidth=1.4, zorder=4)
        ax.set_xticks(range(len(stacks)), [LABELS[s].split(' ')[0] for s in stacks],
                      rotation=35, ha='right')
        ax.set_title(w.replace('_', ' '))
        if log:
            ax.set_yscale('log')
        _clean(ax)
    axes[0][0].set_ylabel(ylabel)
    _save(fig, out, name)


def success_bars(success, out, name='success_rate', condition='nominal'):
    _style()
    d = success[success['condition'] == condition]
    worlds = [w for w in ['arena_5x5', 'small_house', 'bookstore', 'small_warehouse']
              if w in set(d['world'])]
    if not worlds:
        return
    fig, axes = plt.subplots(1, len(worlds), figsize=(1.8 * len(worlds) + 0.6, 2.3),
                             sharey=True, squeeze=False)
    for ax, w in zip(axes[0], worlds):
        g = d[d['world'] == w].set_index('stack')
        stacks = [s for s in STACK_ORDER if s in g.index]
        for i, s in enumerate(stacks):
            r = g.loc[s]
            ax.bar(i, r['rate'], width=0.6, color=COLORS[s], zorder=3)
            ax.errorbar(i, r['rate'], yerr=[[r['rate'] - r['ci_lo']], [r['ci_hi'] - r['rate']]],
                        color=INK, capsize=2, linewidth=0.9, zorder=4)
        ax.set_xticks(range(len(stacks)), [LABELS[s].split(' ')[0] for s in stacks],
                      rotation=35, ha='right')
        ax.set_ylim(0, 1.05)
        ax.set_title(w.replace('_', ' '))
        _clean(ax)
    axes[0][0].set_ylabel('success rate (Wilson 95% CI)')
    _save(fig, out, name)


def cost_pareto(trials, metric, ylabel, out, name='cost_vs_accuracy'):
    """Price vs median metric over all nominal trials, one marker per stack, direct labels."""
    _style()
    d = trials[(trials['condition'] == 'nominal') & trials[metric].notna()]
    if d.empty:
        return
    fig, ax = plt.subplots(figsize=(3.4, 2.4))
    for s in STACK_ORDER:
        v = d[d['stack'] == s]
        if v.empty:
            continue
        x = float(v['price_usd'].iloc[0])
        med = float(v[metric].median())
        q1, q3 = float(v[metric].quantile(.25)), float(v[metric].quantile(.75))
        ax.errorbar(x, med, yerr=[[med - q1], [q3 - med]], fmt=MARKERS[s], color=COLORS[s],
                    markersize=7, capsize=2, markeredgecolor='white', zorder=3)
        ax.annotate(LABELS[s], (x, med), textcoords='offset points', xytext=(6, 4),
                    fontsize=7, color=INK)
    ax.set_xscale('log')
    ax.set_xlabel('sensor price, USD (log)')
    ax.set_ylabel(ylabel)
    _clean(ax)
    _save(fig, out, name)


def degradation_heatmap(trials, metric, out, name='degradation', label=None):
    """Median of metric under each condition relative to nominal, per stack (all worlds pooled
    by paired ratio per world/seed). Sequential single hue: darker = worse."""
    _style()
    base = trials[trials['condition'] == 'nominal'].set_index(['world', 'stack', 'seed'])[metric]
    conds = [c for c in ['glass', 'dynamic', 'degraded', 'low_light']
             if c in set(trials['condition'])]
    stacks = [s for s in STACK_ORDER if s in set(trials['stack'])]
    if not conds or base.empty:
        return
    mat = np.full((len(stacks), len(conds)), np.nan)
    for j, c in enumerate(conds):
        cur = trials[trials['condition'] == c].set_index(['world', 'stack', 'seed'])[metric]
        ratio = (cur / base.reindex(cur.index)).dropna()
        for i, s in enumerate(stacks):
            r = ratio[ratio.index.get_level_values('stack') == s]
            if not r.empty:
                mat[i, j] = float(np.median(r))
    fig, ax = plt.subplots(figsize=(0.9 * len(conds) + 1.4, 0.35 * len(stacks) + 0.8))
    im = ax.imshow(mat, cmap='Blues', aspect='auto')
    for i in range(len(stacks)):
        for j in range(len(conds)):
            if np.isfinite(mat[i, j]):
                ax.text(j, i, f'{mat[i, j]:.2f}x', ha='center', va='center', fontsize=7,
                        color='white' if mat[i, j] > np.nanmedian(mat) else INK)
    ax.set_xticks(range(len(conds)), conds)
    ax.set_yticks(range(len(stacks)), [LABELS[s] for s in stacks])
    ax.set_title(label or f'{metric} relative to nominal (median paired ratio)')
    fig.colorbar(im, ax=ax, shrink=0.8)
    _save(fig, out, name)
