"""Tiered alerting strategies.

Alert levels: 0 none, 1 preventive, 2 corrective, 3 warning. The level is the
highest tier whose condition holds (same convention as DAIDALUS alerters).

Strategies
  * "cpa"     : CPA thresholds -- inside the cylinder now, or horizontal time
                to CPA in (0, T_i] with horizontal miss distance <= Dh and
                vertical separation at that time <= Dv_i.
  * "wcv"     : predicted well-clear violation within the tier's alerting time
                (DAIDALUS WCV_TAUMOD interval test), TTHR from config (0 s).
  * "wcv_tau" : same as "wcv" with a modified-tau buffer TTHR > 0.
"""
from __future__ import annotations

import numpy as np

from . import geometry as g

STRATEGIES = ("cpa", "wcv", "wcv_tau")
LEVEL_NAMES = ("none", "preventive", "corrective", "warning")


def tier_params(cfg, strategy):
    """Return list of dicts (D, Z, TTHR, T) per tier for a strategy."""
    a = cfg["alerting"]
    if strategy == "cpa":
        return [dict(D=a["cpa"]["Dh"], Z=t["Dv"], TTHR=None, T=t["T"]) for t in a["cpa"]["tiers"]]
    tthr = a["wcv"]["TTHR"] if strategy == "wcv" else a["wcv_tau"]["TTHR"]
    if strategy not in ("wcv", "wcv_tau"):
        raise ValueError(strategy)
    return [dict(D=a["wcv"]["DTHR"], Z=t["ZTHR"], TTHR=tthr, T=t["T"]) for t in a["wcv"]["tiers"]]


def condition(strategy, s, v, D, Z, T, TTHR=0.0):
    """Boolean alert condition of one tier (vectorised)."""
    if strategy not in STRATEGIES:
        raise ValueError(f"unknown alert strategy: {strategy}")
    TTHR = g._parameter(TTHR, "TTHR")
    if strategy == "cpa":
        return g.cpa_condition(s, v, D, Z, T)
    hit, _, _ = g.wcv_interval(s, v, D, Z, TTHR, T)
    return hit


def alert_level(strategy, s, v, tiers):
    """Vectorised tiered alert level for relative states s, v [..., 3]."""
    if strategy not in STRATEGIES:
        raise ValueError(f"unknown alert strategy: {strategy}")
    s, v = g._state_array(s), g._state_array(v)
    level = np.zeros(s.shape[:-1], dtype=np.int8)
    for i, t in enumerate(tiers, start=1):
        hit = condition(strategy, s, v, t["D"], t["Z"], t["T"], 0.0 if t["TTHR"] is None else t["TTHR"])
        level = np.where(hit, np.int8(i), level)
    return level


def alert_level_scalar(strategy, s, v, tiers):
    """Scalar reference used for cross-checking against C++."""
    if strategy not in STRATEGIES:
        raise ValueError(f"unknown alert strategy: {strategy}")
    s, v = g._scalar_state(s), g._scalar_state(v)
    level = 0
    for i, t in enumerate(tiers, start=1):
        tau = g._parameter(0.0 if t["TTHR"] is None else t["TTHR"], "TTHR", scalar=True)
        if strategy == "cpa":
            hit = g.cpa_condition_scalar(s, v, t["D"], t["Z"], t["T"])
        else:
            hit = g.wcv_interval_scalar(s, v, t["D"], t["Z"], tau, t["T"]) is not None
        if hit:
            level = i
    return level


class AlertFilter:
    """Online persistence/hysteresis filter for alert levels.

    A level L is raised when the raw level was >= L in at least M of the last
    N cycles, and is held until the raw level has stayed below L for H
    consecutive cycles. M = N = 1, H = 1 reproduces the raw levels. The
    values used (M=2, N=4, H=5 s) are taken from the DAIDALUS configuration
    (alerting_m, alerting_n, hysteresis_time), but this is a simplified
    scheme, not DAIDALUS's hysteresis algorithm.
    """

    def __init__(self, n, M=1, N=1, H=1, levels=3):
        self.M, self.N, self.H, self.levels = int(M), int(N), int(H), levels
        self.hist = np.zeros((levels, n, self.N), dtype=bool)
        self.since = np.full((levels, n), 10**9)
        self.on = np.zeros((levels, n), dtype=bool)
        self.k = 0

    def update(self, raw):
        raw = np.asarray(raw)
        out = np.zeros(raw.shape, dtype=np.int8)
        for i in range(self.levels):
            L = i + 1
            r = raw >= L
            self.hist[i, :, self.k % self.N] = r
            self.since[i] = np.where(r, 0, self.since[i] + 1)
            raise_ok = self.hist[i].sum(axis=1) >= self.M
            self.on[i] = raise_ok | (self.on[i] & (self.since[i] < self.H))
            out = np.where(self.on[i], np.int8(L), out)
        self.k += 1
        return out


def filter_levels(levels, M=1, N=1, H=1):
    """Apply AlertFilter along the time axis of levels [n, K]."""
    f = AlertFilter(levels.shape[0], M, N, H)
    out = np.empty_like(levels)
    for k in range(levels.shape[1]):
        out[:, k] = f.update(levels[:, k])
    return out


def episodes_before(levels, t_end, dt, min_level=2):
    """Number of alert episodes (level >= min_level) starting at or before t_end."""
    m = levels >= min_level
    prev = np.zeros_like(m)
    prev[:, 1:] = m[:, :-1]
    start = m & ~prev
    t = np.arange(levels.shape[1]) * dt
    end = np.where(np.isnan(t_end), np.inf, t_end)
    return (start & (t[None, :] <= end[:, None])).sum(axis=1)


def first_time_at_or_above(levels, L, dt):
    """First time (s) the level reaches >= L along the last axis; NaN if never."""
    ok = levels >= L
    any_ok = ok.any(axis=-1)
    idx = ok.argmax(axis=-1)
    return np.where(any_ok, idx * dt, np.nan)
