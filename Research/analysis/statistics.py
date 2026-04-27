"""Descriptive statistics helpers for the campaign post-processing.

The current campaign is single-run-per-cell (deterministic) so
inferential tests that require paired samples are not used; the
helpers here are limited to descriptive summaries.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BootstrapResult:
    mean: float
    lo: float
    hi: float


def bootstrap_ci(samples: np.ndarray, n_boot: int = 10_000,
                 alpha: float = 0.05,
                 rng: np.random.Generator | None = None) -> BootstrapResult:
    """Percentile bootstrap confidence interval of the mean.

    Useful when a summary is aggregated across an axis with more than
    one sample (for example, across the five fault scenarios within
    one controller configuration). Returns (mean, mean, mean) when
    only a single sample is supplied.
    """
    if rng is None:
        rng = np.random.default_rng(0)
    x = np.asarray(samples, dtype=float)
    if x.size == 0:
        return BootstrapResult(np.nan, np.nan, np.nan)
    if x.size == 1:
        return BootstrapResult(float(x[0]), float(x[0]), float(x[0]))
    idx = rng.integers(0, x.size, size=(n_boot, x.size))
    means = x[idx].mean(axis=1)
    lo = float(np.quantile(means, alpha / 2.0))
    hi = float(np.quantile(means, 1.0 - alpha / 2.0))
    return BootstrapResult(mean=float(x.mean()), lo=lo, hi=hi)
