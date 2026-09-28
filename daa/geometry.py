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


# ---------------------------------------------------------------- scalar ----
def cpa_scalar(s, v):
    """Return (tcpa, hmd, vmd) for the horizontal CPA.

    tcpa is the unclamped horizontal time of closest approach (negative when
    diverging). hmd and vmd are evaluated at max(tcpa, 0).
    """
    sx, sy, sz = s
    vx, vy, vz = v
    a = vx * vx + vy * vy
    sv = sx * vx + sy * vy
    tcpa = -sv / a if a > EPS else 0.0
    t = tcpa if tcpa > 0.0 else 0.0
    hx = sx + t * vx
    hy = sy + t * vy
    hmd = math.sqrt(hx * hx + hy * hy)
    vmd = sz + t * vz
    return tcpa, hmd, vmd


def vertical_interval_scalar(sz, vz, Z):
    """Times t with |sz + t*vz| <= Z, as (lo, hi) or None."""
    if abs(vz) <= 1e-9:
        return (-INF, INF) if abs(sz) <= Z else None
    t1 = (-Z - sz) / vz
    t2 = (Z - sz) / vz
    return (t1, t2) if t1 <= t2 else (t2, t1)


def horizontal_interval_scalar(sx, sy, vx, vy, D, TTHR):
    """Times t with horizontal WCV (tau_mod form), as (lo, hi) or None."""
    a = vx * vx + vy * vy
    ss = sx * sx + sy * sy
    D2 = D * D
    if a <= EPS:
        return (-INF, INF) if ss <= D2 else None
    sv = sx * vx + sy * vy
    disc_c = sv * sv - a * (ss - D2)
    if disc_c < 0.0:  # dcpa > DTHR: never horizontally in violation
        return None
    t_out = (-sv + math.sqrt(disc_c)) / a
    b = 2.0 * sv + TTHR * a
    c = ss + TTHR * sv - D2
    disc_t = b * b - 4.0 * a * c
    if disc_t < 0.0:  # impossible when disc_c >= 0; round-off guard
        disc_t = 0.0
    t_in = (-b - math.sqrt(disc_t)) / (2.0 * a)
    return (t_in, t_out)


def wcv_interval_scalar(s, v, D, Z, TTHR, T):
    """First/last time in [0, T] of predicted WCV, or None if no violation."""
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
    tcpa, hmd, vmd = cpa_scalar(s, v)
    inside = (s[0] * s[0] + s[1] * s[1] <= Dh * Dh) and abs(s[2]) <= Dv
    return inside or (0.0 < tcpa <= T and hmd <= Dh and abs(vmd) <= Dv)


# ------------------------------------------------------------ vectorised ----
def _split(s):
    s = np.asarray(s, dtype=float)
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
    return tcpa, hmd, vmd


def wcv_interval(s, v, D, Z, TTHR, T):
    """Vectorised wcv_interval_scalar.

    Returns (hit, lo, hi); lo/hi are NaN where hit is False. D, Z, TTHR, T may
    be scalars or arrays broadcastable to s[..., 0].
    """
    sx, sy, sz = _split(s)
    vx, vy, vz = _split(v)
    D = np.asarray(D, dtype=float)
    Z = np.asarray(Z, dtype=float)
    TTHR = np.asarray(TTHR, dtype=float)
    T = np.asarray(T, dtype=float)

    # horizontal
    a = vx * vx + vy * vy
    ss = sx * sx + sy * sy
    D2 = D * D
    still = a <= EPS
    a_safe = np.where(still, 1.0, a)
    sv = sx * vx + sy * vy
    disc_c = sv * sv - a * (ss - D2)
    with np.errstate(invalid="ignore"):
        t_out = (-sv + np.sqrt(np.where(disc_c < 0.0, 0.0, disc_c))) / a_safe
        b = 2.0 * sv + TTHR * a
        c = ss + TTHR * sv - D2
        disc_t = b * b - 4.0 * a * c
        disc_t = np.where(disc_t < 0.0, 0.0, disc_t)
        t_in = (-b - np.sqrt(disc_t)) / (2.0 * a_safe)
    h_ok = np.where(still, ss <= D2, disc_c >= 0.0)
    h_lo = np.where(still, -np.inf, t_in)
    h_hi = np.where(still, np.inf, t_out)

    # vertical
    flat = np.abs(vz) <= 1e-9
    vz_safe = np.where(flat, 1.0, vz)
    t1 = (-Z - sz) / vz_safe
    t2 = (Z - sz) / vz_safe
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
    s = rel[:, :-1, :]
    v = (rel[:, 1:, :] - rel[:, :-1, :]) / dt
    hit, lo, _ = wcv_interval(s, v, D, Z, 0.0, dt)
    k = np.arange(s.shape[1]) * dt
    onset_k = np.where(hit, k[None, :] + lo, np.inf)
    onset = onset_k.min(axis=1)
    violated = np.isfinite(onset)
    return violated, np.where(violated, onset, np.nan)
