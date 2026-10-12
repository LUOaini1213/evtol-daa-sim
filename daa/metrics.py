"""Alerting and outcome metrics with confidence intervals."""
from __future__ import annotations

import math
from numbers import Integral

import numpy as np
from scipy import stats


def wilson(k, n, level=0.95):
    """Wilson score interval for k successes out of n. Returns (p, lo, hi)."""
    if n == 0:
        return (float("nan"), float("nan"), float("nan"))
    z = stats.norm.ppf(0.5 + level / 2.0)
    p = k / n
    den = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (p, max(0.0, centre - half), min(1.0, centre + half))


def alert_lead_time(levels, onset, dt, min_level=2):
    """Lead time of the alert episode (level >= min_level) that started most
    recently at or before loss-of-well-clear onset. NaN if none or no onset."""
    mask = levels >= min_level
    prev = np.zeros_like(mask)
    prev[:, 1:] = mask[:, :-1]
    start = mask & ~prev
    t = np.arange(levels.shape[1]) * dt
    on = np.where(np.isnan(onset), -np.inf, onset)
    ok = start & (t[None, :] <= on[:, None])
    t_start = np.where(ok, t[None, :], -np.inf).max(axis=1)
    good = np.isfinite(t_start) & np.isfinite(on)
    return np.where(good, np.where(good, on, 0.0) - np.where(good, t_start, 0.0), np.nan)


def alert_outcomes(levels, hazardous, onset, dt, t_req, min_level=2):
    """Per-encounter booleans: missed_or_late (hazardous only), nuisance
    (non-hazardous only), plus lead time."""
    lead = alert_lead_time(levels, onset, dt, min_level)
    alerted = (levels >= min_level).any(axis=1)
    missed_or_late = hazardous & (np.isnan(lead) | (lead < t_req))
    nuisance = ~hazardous & alerted
    return dict(lead=lead, alerted=alerted, missed_or_late=missed_or_late, nuisance=nuisance)


def rate(mask, subset=None, level=0.95):
    mask = np.asarray(mask, bool)
    if subset is not None:
        mask = mask[np.asarray(subset, bool)]
    k, n = int(mask.sum()), int(mask.size)
    p, lo, hi = wilson(k, n, level)
    return dict(k=k, n=n, p=p, lo=lo, hi=hi)


def _paired(x, y, *, binary=False):
    x, y = np.asarray(x, float), np.asarray(y, float)
    if x.ndim != 1 or y.ndim != 1:
        raise ValueError("paired inputs must be one-dimensional")
    if x.size == 0 or x.size != y.size:
        raise ValueError("paired inputs must have equal, nonempty lengths")
    if not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError("paired inputs must be finite")
    if binary:
        if not np.isin(x, [0., 1.]).all() or not np.isin(y, [0., 1.]).all():
            raise ValueError("paired McNemar outcomes must be binary")
        return x.astype(bool), y.astype(bool)
    return x, y


def _bootstrap_inputs(x, y, n_boot, level):
    if isinstance(n_boot, bool) or not isinstance(n_boot, Integral) or n_boot < 1:
        raise ValueError("n_boot must be a positive integer")
    if not math.isfinite(level) or not 0. < level < 1.:
        raise ValueError("confidence level must be finite and strictly between 0 and 1")
    return _paired(x, y)


def mcnemar_exact(x, y):
    """Exact two-sided McNemar test on paired booleans."""
    x, y = _paired(x, y, binary=True)
    b = int((x & ~y).sum())
    c = int((~x & y).sum())
    if b + c == 0:
        return dict(b=b, c=c, p=1.0)
    return dict(b=b, c=c, p=float(stats.binomtest(min(b, c), b + c, 0.5).pvalue))


def paired_bootstrap_diff(x, y, n_boot, seed, level=0.95):
    """CI for mean(x) - mean(y) with paired resampling."""
    x, y = _bootstrap_inputs(x, y, n_boot, level)
    n = x.size
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, n, size=(n_boot, n))
    d = x[idx].mean(1) - y[idx].mean(1)
    a = (1 - level) / 2
    return dict(diff=float(x.mean() - y.mean()), lo=float(np.quantile(d, a)), hi=float(np.quantile(d, 1 - a)))


def paired_bootstrap_ratio(x, y, n_boot, seed, level=0.95):
    """CI for mean(x)/mean(y) with paired resampling (risk ratio)."""
    x, y = _bootstrap_inputs(x, y, n_boot, level)
    n = x.size
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, n, size=(n_boot, n))
    num = x[idx].mean(1)
    den = y[idx].mean(1)
    with np.errstate(divide="ignore", invalid="ignore"):
        r = np.where(den > 0, num / den, np.nan)
    r = r[np.isfinite(r)]
    a = (1 - level) / 2
    est = float(x.mean() / y.mean()) if y.mean() > 0 else float("nan")
    if r.size == 0:
        return dict(ratio=est, lo=float("nan"), hi=float("nan"))
    return dict(ratio=est, lo=float(np.quantile(r, a)), hi=float(np.quantile(r, 1 - a)))
