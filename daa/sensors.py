"""Surveillance model: noisy intruder reports, Gilbert burst dropouts and an
alpha-beta tracker. Ownship state is assumed known exactly.

Cooperative intruders (eVTOL, GA) report position and velocity (ADS-B-like);
non-cooperative intruders (drones) report position only and the tracker
estimates velocity. All error levels are assumptions (see config).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .encounters import INTRUDERS

COOPERATIVE = {"evtol": True, "drone": False, "ga": True}


@dataclass
class Track:
    pos: np.ndarray       # [N,K+1,3]
    vel: np.ndarray       # [N,K+1,3]
    received: np.ndarray  # [N,K+1] bool
    condition: str


def alpha_beta_gains(lam):
    """Steady-state alpha-beta gains for tracking index lam = sigma_a*dt^2/sigma_n
    (constant-velocity model with white acceleration noise). Satisfies
    beta = 2(2-alpha) - 4 sqrt(1-alpha) and lam^2 = beta^2/(1-alpha); the
    Riccati-iteration check is in tests/test_sensors.py."""
    lam = np.asarray(lam, float)
    r = np.sqrt(lam * lam + 8.0 * lam)
    alpha = -(lam * lam + 8.0 * lam - (lam + 4.0) * r) / 8.0
    beta = (lam * lam + 4.0 * lam - lam * r) / 4.0
    return alpha, beta


def gilbert_mask(rng, n, k, p_gb, p_bg):
    """True = report received. Starts in the 'good' state."""
    rec = np.ones((n, k), dtype=bool)
    if p_gb <= 0.0:
        return rec
    u = rng.uniform(size=(n, k))
    state = np.zeros(n, dtype=bool)  # False = good
    for j in range(1, k):
        state = np.where(state, u[:, j] >= p_bg, u[:, j] < p_gb)
        rec[:, j] = ~state
    return rec


def make_track(enc, cfg, condition, seed):
    sc = cfg["sensors"]
    cond = sc["conditions"][condition]
    n, K1 = enc.int_pos.shape[:2]
    dt = enc.dt
    if cond["noise_scale"] == 0.0 and cond["p_gb"] == 0.0:
        return Track(enc.int_pos.copy(), enc.int_vel.copy(), np.ones((n, K1), bool), condition)

    rng = np.random.default_rng(seed)
    coop = np.array([COOPERATIVE[x] for x in INTRUDERS])[enc.intruder]
    k = cond["noise_scale"]
    sh = np.where(coop, sc["cooperative"]["sigma_h"], sc["noncooperative"]["sigma_h"]) * k
    sz = np.where(coop, sc["cooperative"]["sigma_z"], sc["noncooperative"]["sigma_z"]) * k
    sv = np.where(coop, sc["cooperative"]["sigma_v"], 0.0) * k
    sig_p = np.stack([sh, sh, sz], -1)[:, None, :]
    meas_p = enc.int_pos + rng.standard_normal(enc.int_pos.shape) * sig_p
    meas_v = enc.int_vel + rng.standard_normal(enc.int_vel.shape) * sv[:, None, None]
    rec = gilbert_mask(rng, n, K1, cond["p_gb"], cond["p_bg"])
    rec[:, :2] = True  # track initiation uses the first two reports

    perf = cfg["performance"]
    typ = np.array(INTRUDERS)[enc.intruder]
    g_h = np.array([perf[x]["gust_sigma_h"] for x in typ])
    g_z = np.array([perf[x]["gust_sigma_z"] for x in typ])
    acc = np.stack([g_h, g_h, g_z], -1) * np.sqrt(2.0 / perf["gust_tau"])
    noise = np.maximum(np.stack([sh, sh, sz], -1), 1e-9)
    alpha, beta = alpha_beta_gains(acc * dt * dt / noise)
    pos = np.empty_like(meas_p)
    vel = np.empty_like(meas_p)
    pos[:, 0] = meas_p[:, 0]
    # Noncooperative velocity is unknown at the first report: use a zero prior.
    # The two-position estimate becomes available only at the second report.
    vel[:, 0] = np.where(coop[:, None], meas_v[:, 0], 0.0)
    c = coop[:, None]
    for j in range(1, K1):
        pp = pos[:, j - 1] + vel[:, j - 1] * dt
        r = meas_p[:, j] - pp
        got = rec[:, j][:, None]
        pos[:, j] = np.where(got, pp + alpha * r, pp)
        v_ab = vel[:, j - 1] + (beta / dt) * r
        vel[:, j] = np.where(got, np.where(c, meas_v[:, j], v_ab), vel[:, j - 1])
        if j == 1:  # both initiation reports are received by construction
            pos[:, j] = np.where(c, pos[:, j], meas_p[:, j])
            vel[:, j] = np.where(c, vel[:, j], (meas_p[:, j] - meas_p[:, 0]) / dt)
    return Track(pos, vel, rec, condition)
