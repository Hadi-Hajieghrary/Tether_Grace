from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np


@dataclass
class FaultMetrics:
    fault_time: float
    window_seconds: float
    pre_fault_rmse: float = 0.0
    post_fault_rmse: float = 0.0
    rmse_ratio: float = 0.0
    peak_deviation: float = 0.0
    recovery_time: float = 0.0
    nominal_rmse: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def compute_fault_metrics(
    times: np.ndarray,
    errors: np.ndarray,
    fault_time: float,
    window: float = 5.0,
    peak_window: float = 2.0,
    recovery_multiplier: float = 1.5,
    recovery_hold: float = 0.5,
    nominal_start_time: float = 8.0,
) -> FaultMetrics:
    metric = FaultMetrics(fault_time=fault_time, window_seconds=window)

    if times.size == 0 or errors.size == 0:
        return metric

    nominal_mask = times >= nominal_start_time
    if np.any(nominal_mask):
        metric.nominal_rmse = float(np.sqrt(np.mean(errors[nominal_mask] ** 2)))

    if fault_time >= 1e9:
        metric.pre_fault_rmse = metric.nominal_rmse
        metric.post_fault_rmse = metric.nominal_rmse
        metric.rmse_ratio = 1.0 if metric.nominal_rmse > 0.0 else 0.0
        metric.peak_deviation = float(np.max(errors[nominal_mask])) if np.any(nominal_mask) else 0.0
        metric.recovery_time = 0.0
        return metric

    pre_mask = (times >= max(nominal_start_time, fault_time - window)) & (times < fault_time)
    post_mask = (times >= fault_time) & (times < fault_time + window)

    if np.any(pre_mask):
        metric.pre_fault_rmse = float(np.sqrt(np.mean(errors[pre_mask] ** 2)))
    if np.any(post_mask):
        metric.post_fault_rmse = float(np.sqrt(np.mean(errors[post_mask] ** 2)))

    if metric.pre_fault_rmse > 1e-9:
        metric.rmse_ratio = metric.post_fault_rmse / metric.pre_fault_rmse

    peak_mask = (times >= fault_time) & (times < fault_time + peak_window)
    if np.any(peak_mask):
        metric.peak_deviation = float(np.max(errors[peak_mask]))

    threshold = recovery_multiplier * metric.pre_fault_rmse
    metric.recovery_time = window
    for index, current_time in enumerate(times):
        if current_time < fault_time:
            continue
        if errors[index] >= threshold:
            continue

        sustained_mask = (times >= current_time) & (times < current_time + recovery_hold)
        if np.any(sustained_mask) and np.all(errors[sustained_mask] < threshold):
            metric.recovery_time = float(current_time - fault_time)
            break

    return metric
