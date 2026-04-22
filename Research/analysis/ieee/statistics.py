"""Bootstrap confidence intervals, paired significance tests, and
effect sizes for the IEEE-Transactions campaign. Consumed by
`plot_ablation_bars.py` and the table generators.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd
from scipy import stats


@dataclass(frozen=True)
class BootstrapResult:
    mean: float
    lo: float
    hi: float


def bootstrap_ci(samples: np.ndarray, n_boot: int = 10_000,
                 alpha: float = 0.05,
                 rng: np.random.Generator | None = None) -> BootstrapResult:
    """Percentile bootstrap confidence interval of the mean."""
    if rng is None:
        rng = np.random.default_rng(0)
    x = np.asarray(samples, dtype=float)
    if x.size == 0:
        return BootstrapResult(np.nan, np.nan, np.nan)
    idx = rng.integers(0, x.size, size=(n_boot, x.size))
    means = x[idx].mean(axis=1)
    lo = float(np.quantile(means, alpha / 2.0))
    hi = float(np.quantile(means, 1.0 - alpha / 2.0))
    return BootstrapResult(mean=float(x.mean()), lo=lo, hi=hi)


def paired_wilcoxon(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """Paired Wilcoxon signed-rank test. Returns (statistic, p-value).

    Both inputs must be the same length; caller is responsible for
    pairing samples on (seed, scenario, configuration-pair).
    """
    if len(a) != len(b) or len(a) < 3:
        return float("nan"), float("nan")
    result = stats.wilcoxon(a, b, zero_method="pratt", alternative="two-sided",
                             nan_policy="omit")
    return float(result.statistic), float(result.pvalue)


def cohen_d_paired(a: np.ndarray, b: np.ndarray) -> float:
    d = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    sd = d.std(ddof=1)
    if sd == 0.0:
        return 0.0
    return float(d.mean() / sd)


def benjamini_hochberg(pvalues: np.ndarray, q: float = 0.05) -> np.ndarray:
    """Return a boolean array indicating which tests pass the BH
    procedure at false-discovery-rate q.
    """
    p = np.asarray(pvalues, dtype=float)
    n = p.size
    order = np.argsort(p)
    thresholds = (np.arange(1, n + 1) / n) * q
    passed_sorted = p[order] <= thresholds
    # Largest index k where passed_sorted[k] is True; everything ≤ k passes.
    if not np.any(passed_sorted):
        mask = np.zeros(n, dtype=bool)
    else:
        k = int(np.max(np.where(passed_sorted)[0]))
        mask_sorted = np.zeros(n, dtype=bool)
        mask_sorted[: k + 1] = True
        mask = np.empty(n, dtype=bool)
        mask[order] = mask_sorted
    return mask


def pairwise_comparison_table(df: pd.DataFrame, metric: str,
                               configs: list[str],
                               pair_by: list[str]) -> pd.DataFrame:
    """Build a long-form table of pairwise differences between configs.

    Parameters
    ----------
    df:        dataframe with columns including `config`, the metric,
               and each key in `pair_by`.
    metric:    column to compare.
    configs:   list of config names (each pair compared).
    pair_by:   keys used to join two configs row-wise.

    Returns a dataframe with one row per (configA, configB, scenario)
    triple carrying the paired difference, Wilcoxon p-value, Cohen's d,
    and BH pass flag.
    """
    rows = []
    for i, a in enumerate(configs):
        for b in configs[i + 1:]:
            da = df[df.config == a].set_index(pair_by)[metric]
            db = df[df.config == b].set_index(pair_by)[metric]
            idx = da.index.intersection(db.index)
            if len(idx) < 3:
                continue
            va = da.loc[idx].values
            vb = db.loc[idx].values
            stat, pval = paired_wilcoxon(va, vb)
            d = cohen_d_paired(va, vb)
            rows.append({
                "config_a": a,
                "config_b": b,
                "n_pairs": len(idx),
                "mean_diff": float(np.mean(va - vb)),
                "p_wilcoxon": pval,
                "cohen_d": d,
            })
    table = pd.DataFrame(rows)
    if not table.empty:
        table["bh_pass"] = benjamini_hochberg(table["p_wilcoxon"].values)
    return table
