"""Closest-point-of-approach and well-clear-violation geometry.

Conventions: relative state s = p_intruder - p_ownship, v = v_intruder - v_ownship,
components (x, y, z) in metres and m/s, z up. Both aircraft fly straight lines
over the prediction horizon (the usual state-based DAA assumption).

Two implementations are kept on purpose:
  * scalar functions (pure Python floats) that mirror cpp/daa_core.hpp line by line,
  * vectorised numpy functions used by the simulator.
Tests check that the two agree with each other and with the C++ build.

The well-clear-violation (WCV) test follows the WCV_TAUMOD definition in NASA
DAIDALUS (WCV_tvar.java / WCV_TAUMOD.java, commit 1c7b58e):
  horizontal WCV  <=>  |s_h| <= DTHR  or  (dcpa <= DTHR and 0 <= tau_mod <= TTHR),
  tau_mod = (DTHR^2 - |s_h|^2) / (s_h . v_h)   when s_h . v_h < 0,
  vertical WCV    <=>  |s_z| <= ZTHR           (TCOA = 0 throughout this project),
  WCV = horizontal WCV and vertical WCV.
"""
from __future__ import annotations

import math

import numpy as np

EPS = 1e-12  # m^2/s^2: relative horizontal speed^2 below this is treated as zero
INF = math.inf


def _has_bool(value):
    if isinstance(value, (bool, np.bool_)):
        return True
    return isinstance(value, (list, tuple)) and any(_has_bool(x) for x in value)


def _real_array(value, name):
    """Public inputs must be finite real numbers, not coercible text or bools."""
    try:
        raw = np.asarray(value)
        if _has_bool(value) or raw.dtype.kind not in "iuf":
            raise ValueError(f"{name} must contain real numbers")
        result = raw.astype(float, copy=False)
        if not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must be finite")
        return result
    except (TypeError, OverflowError) as exc:
        raise ValueError(f"{name} must contain finite real numbers") from exc


def _state_array(value, name="state"):
    result = _real_array(value, name)
    if result.ndim < 1 or result.shape[-1] != 3:
        raise ValueError(f"{name} must have shape [..., 3]")
    return result


def _scalar_state(value):
    result = _state_array(value)
    if result.shape != (3,):
        raise ValueError("scalar state must have exactly three components")
    return tuple(float(x) for x in result)


def _parameter(value, name, scalar=False):
    result = _real_array(value, name)
    if np.any(result < 0) or (scalar and result.ndim != 0):
        raise ValueError(f"{name} must be finite and nonnegative" + (" scalar" if scalar else ""))
    return float(result) if scalar else result


def _finite_calculation(*values):
    if not all(np.all(np.isfinite(value)) for value in values):
        raise ValueError("geometry exceeds the finite arithmetic range")


# ---------------------------------------------------------------- scalar ----
def cpa_scalar(s, v):
    """Return (tcpa, hmd, vmd) for the horizontal CPA.

    tcpa is the unclamped horizontal time of closest approach (negative when
    diverging). hmd and vmd are evaluated at max(tcpa, 0).
    """
    sx, sy, sz = _scalar_state(s)
    vx, vy, vz = _scalar_state(v)
    a = vx * vx + vy * vy
    sv = sx * vx + sy * vy
    tcpa = -sv / a if a > EPS else 0.0
    t = tcpa if tcpa > 0.0 else 0.0
    hx = sx + t * vx
    hy = sy + t * vy
    hmd = math.sqrt(hx * hx + hy * hy)
    vmd = sz + t * vz
    _finite_calculation(a, sv, tcpa, hmd, vmd)
    return tcpa, hmd, vmd


def vertical_interval_scalar(sz, vz, Z):
    """Times t with |sz + t*vz| <= Z, as (lo, hi) or None."""
    sz, vz, _ = _scalar_state((sz, vz, 0.0))
    Z = _parameter(Z, "Z", scalar=True)
    if abs(vz) <= 1e-9:
        return (-INF, INF) if abs(sz) <= Z else None
    t1 = (-Z - sz) / vz
    t2 = (Z - sz) / vz
    _finite_calculation(t1, t2)
    return (t1, t2) if t1 <= t2 else (t2, t1)


def horizontal_interval_scalar(sx, sy, vx, vy, D, TTHR):
    """Times t with horizontal WCV (tau_mod form), as (lo, hi) or None."""
    sx, sy, _ = _scalar_state((sx, sy, 0.0))
    vx, vy, _ = _scalar_state((vx, vy, 0.0))
    D = _parameter(D, "D", scalar=True)
    TTHR = _parameter(TTHR, "TTHR", scalar=True)
    a = vx * vx + vy * vy
    ss = sx * sx + sy * sy
    D2 = D * D
    _finite_calculation(a, ss, D2)
    if a <= EPS:
        return (-INF, INF) if ss <= D2 else None
    sv = sx * vx + sy * vy
    # Cross-track distance avoids subtracting the large along-track terms in
    # sv**2 - a*(ss-D**2), which can make an exact tangent look disjoint.
    speed = math.sqrt(a)
    miss = abs(sx * (vy / speed) - sy * (vx / speed))
    _finite_calculation(sv, miss)
    if miss > D:
        return None
    half_width2 = (D - miss) * (D + miss) / a
    tcpa = -sv / a
    half_tau = TTHR / 2.0
    t_out = tcpa + math.sqrt(half_width2)
    t_in = tcpa - half_tau - math.sqrt(half_tau * half_tau + half_width2)
    _finite_calculation(t_in, t_out)
    return (t_in, t_out)


def wcv_interval_scalar(s, v, D, Z, TTHR, T):
    """First/last time in [0, T] of predicted WCV, or None if no violation."""
    s, v = _scalar_state(s), _scalar_state(v)
    D, Z, TTHR, T = (_parameter(x, name, scalar=True)
                     for x, name in zip((D, Z, TTHR, T), ("D", "Z", "TTHR", "T")))
    h = horizontal_interval_scalar(s[0], s[1], v[0], v[1], D, TTHR)
    if h is None:
        return None
    vi = vertical_interval_scalar(s[2], v[2], Z)
    if vi is None:
        return None
    lo = max(h[0], vi[0], 0.0)
    hi = min(h[1], vi[1], T)
    if lo > hi:
        return None
    return (lo, hi)


def cpa_condition_scalar(s, v, Dh, Dv, T):
    """CPA-threshold test: inside the cylinder now, or 0 < tcpa <= T with
    hmd <= Dh and |vmd| <= Dv."""
    s, v = _scalar_state(s), _scalar_state(v)
    Dh, Dv, T = (_parameter(x, name, scalar=True)
                 for x, name in zip((Dh, Dv, T), ("Dh", "Dv", "T")))
    tcpa, hmd, vmd = cpa_scalar(s, v)
    inside = (s[0] * s[0] + s[1] * s[1] <= Dh * Dh) and abs(s[2]) <= Dv
    return inside or (0.0 < tcpa <= T and hmd <= Dh and abs(vmd) <= Dv)


# ------------------------------------------------------------ vectorised ----
def _split(s):
    s = _state_array(s)
    return s[..., 0], s[..., 1], s[..., 2]


def cpa(s, v):
    """Vectorised cpa_scalar. s, v: arrays [..., 3]."""
    sx, sy, sz = _split(s)
    vx, vy, vz = _split(v)
    a = vx * vx + vy * vy
    sv = sx * vx + sy * vy
    moving = a > EPS
    with np.errstate(divide="ignore", invalid="ignore"):
        tcpa = np.where(moving, -sv / np.where(moving, a, 1.0), 0.0)
    t = np.where(tcpa > 0.0, tcpa, 0.0)
    hx = sx + t * vx
    hy = sy + t * vy
    hmd = np.sqrt(hx * hx + hy * hy)
    vmd = sz + t * vz
    _finite_calculation(a, sv, tcpa, hmd, vmd)
    return tcpa, hmd, vmd


def wcv_interval(s, v, D, Z, TTHR, T):
    """Vectorised wcv_interval_scalar.

    Returns (hit, lo, hi); lo/hi are NaN where hit is False. D, Z, TTHR, T may
    be scalars or arrays broadcastable to s[..., 0].
    """
    sx, sy, sz = _split(s)
    vx, vy, vz = _split(v)
    D, Z, TTHR, T = (_parameter(x, name) for x, name in
                     zip((D, Z, TTHR, T), ("D", "Z", "TTHR", "T")))

    # horizontal
    a = vx * vx + vy * vy
    ss = sx * sx + sy * sy
    D2 = D * D
    still = a <= EPS
    a_safe = np.where(still, 1.0, a)
    sv = sx * vx + sy * vy
    speed = np.sqrt(a_safe)
    miss = np.abs(sx * (vy / speed) - sy * (vx / speed))
    _finite_calculation(a, ss, D2, sv, miss)
    half_width2 = np.maximum(D - miss, 0.0) * (D + miss) / a_safe
    tcpa = -sv / a_safe
    half_tau = TTHR / 2.0
    t_out = tcpa + np.sqrt(half_width2)
    t_in = tcpa - half_tau - np.sqrt(half_tau * half_tau + half_width2)
    _finite_calculation(t_in, t_out)
    h_ok = np.where(still, ss <= D2, miss <= D)
    h_lo = np.where(still, -np.inf, t_in)
    h_hi = np.where(still, np.inf, t_out)

    # vertical
    flat = np.abs(vz) <= 1e-9
    vz_safe = np.where(flat, 1.0, vz)
    t1 = (-Z - sz) / vz_safe
    t2 = (Z - sz) / vz_safe
    _finite_calculation(t1, t2)
    v_ok = np.where(flat, np.abs(sz) <= Z, True)
    v_lo = np.where(flat, -np.inf, np.minimum(t1, t2))
    v_hi = np.where(flat, np.inf, np.maximum(t1, t2))

    lo = np.maximum(np.maximum(h_lo, v_lo), 0.0)
    hi = np.minimum(np.minimum(h_hi, v_hi), T)
    hit = h_ok & v_ok & (lo <= hi)
    lo = np.where(hit, lo, np.nan)
    hi = np.where(hit, hi, np.nan)
    return hit, lo, hi


def cpa_condition(s, v, Dh, Dv, T):
    """Vectorised cpa_condition_scalar."""
    Dh, Dv, T = (_parameter(x, name) for x, name in zip((Dh, Dv, T), ("Dh", "Dv", "T")))
    sx, sy, sz = _split(s)
    tcpa, hmd, vmd = cpa(s, v)
    inside = (sx * sx + sy * sy <= Dh * Dh) & (np.abs(sz) <= Dv)
    ahead = (tcpa > 0.0) & (tcpa <= T) & (hmd <= Dh) & (np.abs(vmd) <= Dv)
    return inside | ahead


def segment_violation(rel, dt, D, Z):
    """Continuous-time check of a sampled relative trajectory against a
    cylinder (TTHR = 0), assuming linear relative motion between samples.

    rel: [N, K, 3] relative positions at t_k = k*dt.
    Returns (violated [N], onset time [N] (NaN if never)).
    """
    rel = _state_array(rel, "trajectory")
    dt = _parameter(dt, "dt", scalar=True)
    if rel.ndim != 3 or rel.shape[1] < 1 or dt <= 0:
        raise ValueError("trajectory must have shape [N, K>=1, 3] and dt must be positive")
    if rel.shape[1] == 1:
        hit, lo, _ = wcv_interval(rel[:, 0, :], np.zeros_like(rel[:, 0, :]), D, Z, 0.0, 0.0)
        return hit, lo
    s = rel[:, :-1, :]
    v = (rel[:, 1:, :] - rel[:, :-1, :]) / dt
    hit, lo, _ = wcv_interval(s, v, D, Z, 0.0, dt)
    k = np.arange(s.shape[1]) * dt
    onset_k = np.where(hit, k[None, :] + lo, np.inf)
    onset = onset_k.min(axis=1)
    violated = np.isfinite(onset)
    return violated, np.where(violated, onset, np.nan)
