"""Synthetic pairwise encounter generator and truth trajectories.

Each encounter is specified at the horizontal CPA of the *undisturbed*
straight-line trajectories (time t_cpa0, horizontal miss distance hmd0,
vertical offset vmd0 at that time) and back-propagated to t = 0. The ownship
is a level eVTOL heading along +x. Heading angles are counter-clockwise from
+x, so a right turn is a negative heading change.

Wind: a steady uniform wind moves both aircraft equally and cancels out of the
relative motion, so it is not simulated. Gusts/turbulence are modelled as an
independent Ornstein-Uhlenbeck velocity disturbance on each aircraft.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import geometry as g

GEOMETRIES = ("head_on", "crossing", "overtaking", "vertical")
INTRUDERS = ("evtol", "drone", "ga")


@dataclass
class EncounterSet:
    n: int
    dt: float
    t: np.ndarray                    # [K+1]
    geometry: np.ndarray             # [N] index into GEOMETRIES
    intruder: np.ndarray             # [N] index into INTRUDERS
    own_p0: np.ndarray               # [N,3]
    own_speed: np.ndarray            # [N]
    own_psi: np.ndarray              # [N]
    own_w: np.ndarray                # [N,K,3] ownship gust velocity per step
    int_pos: np.ndarray              # [N,K+1,3] intruder truth positions
    int_vel: np.ndarray              # [N,K+1,3] intruder truth velocity (used over step k)
    own_pos: np.ndarray              # [N,K+1,3] unmitigated ownship truth positions
    own_vel: np.ndarray              # [N,K+1,3]
    params: dict = field(default_factory=dict)   # t_cpa0, hmd0, vmd0, ...
    stats: dict = field(default_factory=dict)


def _ou(rng, n, k, sigma_h, sigma_z, tau, dt):
    """Stationary OU velocity disturbance [n, k, 3]; sigma arrays shape [n]."""
    phi = np.exp(-dt / tau)
    sig = np.stack([sigma_h, sigma_h, sigma_z], axis=-1)  # [n,3]
    w = np.empty((n, k, 3))
    w[:, 0, :] = rng.standard_normal((n, 3)) * sig
    q = np.sqrt(1.0 - phi * phi)
    xi = rng.standard_normal((n, k, 3))
    for j in range(1, k):
        w[:, j, :] = phi * w[:, j - 1, :] + q * sig * xi[:, j, :]
    return w


def _sample_params(rng, m, cfg):
    ec = cfg["encounter"]
    perf = cfg["performance"]
    geo = rng.choice(len(GEOMETRIES), size=m, p=[ec["geometry_mix"][k] for k in GEOMETRIES])
    mix = np.array([ec["intruder_mix"][k] for k in INTRUDERS])
    typ = rng.choice(len(INTRUDERS), size=m, p=mix / mix.sum())
    lo, hi = perf["evtol"]["speed"]
    v_o = rng.uniform(lo, hi, m)
    v_i = np.empty(m)
    vzmax_i = np.empty(m)
    for j, name in enumerate(INTRUDERS):
        sel = typ == j
        a, b = perf[name]["speed"]
        v_i[sel] = rng.uniform(a, b, sel.sum())
        vzmax_i[sel] = perf[name]["vz_max"]
    deg = np.pi / 180.0
    psi_i = np.empty(m)
    vz_i = rng.uniform(-0.5, 0.5, m)
    side = rng.choice([-1.0, 1.0], m)
    u = rng.uniform(0, 1, m)
    s = geo == 0
    psi_i[s] = np.pi + (u[s] * 30.0 - 15.0) * deg
    s = geo == 1
    psi_i[s] = side[s] * (30.0 + 120.0 * u[s]) * deg
    s = geo == 2
    psi_i[s] = (u[s] * 40.0 - 20.0) * deg
    s = geo == 3
    psi_i[s] = u[s] * 2 * np.pi
    vz_i[s] = rng.choice([-1.0, 1.0], s.sum()) * rng.uniform(0.5, 1.0, s.sum()) * vzmax_i[s]
    t_cpa0 = rng.uniform(*ec["t_cpa_range"], m)
    hmd0 = rng.uniform(0.0, ec["hmd_max"], m) * rng.choice([-1.0, 1.0], m)
    vmd0 = rng.uniform(-ec["vmd_max"], ec["vmd_max"], m)
    h0 = rng.uniform(*ec["ownship_alt_range"], m)
    return dict(geometry=geo, intruder=typ, v_o=v_o, v_i=v_i, psi_i=psi_i, vz_i=vz_i,
                t_cpa0=t_cpa0, hmd0=hmd0, vmd0=vmd0, h0=h0)


def generate(cfg, n=None, seed=None):
    ec = cfg["encounter"]
    sim = cfg["simulation"]
    perf = cfg["performance"]
    hz = cfg["hazard"]["well_clear"]
    n = int(n or ec["n_encounters"])
    seed = int(ec["seed"] if seed is None else seed)
    dt = float(sim["dt"])
    K = int(round(sim["horizon"] / dt))
    horizon = K * dt
    rng = np.random.default_rng(seed)

    accepted = {k: [] for k in ("geometry", "intruder", "v_o", "v_i", "psi_i", "vz_i",
                                "t_cpa0", "hmd0", "vmd0", "h0", "s0", "vrel")}
    tried = 0
    reasons = {"slow_closure": 0, "early_or_initial_loss": 0, "intruder_altitude": 0}
    got = 0
    while got < n:
        m = 2 * (n - got) + 64
        p = _sample_params(rng, m, cfg)
        tried += m
        vo = np.stack([p["v_o"], np.zeros(m), np.zeros(m)], -1)
        vi = np.stack([p["v_i"] * np.cos(p["psi_i"]), p["v_i"] * np.sin(p["psi_i"]), p["vz_i"]], -1)
        vrel = vi - vo
        sp = np.hypot(vrel[:, 0], vrel[:, 1])
        ok_speed = sp >= ec["min_rel_speed"]
        sp_safe = np.where(ok_speed, sp, 1.0)
        nrm = np.stack([-vrel[:, 1] / sp_safe, vrel[:, 0] / sp_safe], -1)
        s_cpa_h = p["hmd0"][:, None] * nrm
        s0_h = s_cpa_h - p["t_cpa0"][:, None] * vrel[:, :2]
        z_i0 = p["h0"] + p["vmd0"] - p["vz_i"] * p["t_cpa0"]
        s0 = np.concatenate([s0_h, (z_i0 - p["h0"])[:, None]], -1)
        # undisturbed onset must leave room for a 55 s alert
        hit, lo, _ = g.wcv_interval(s0, vrel, hz["DTHR"], hz["ZTHR"], 0.0, horizon)
        ok_onset = ~hit | (lo >= ec["min_onset_time"])
        z_end = z_i0 + p["vz_i"] * np.minimum(p["t_cpa0"] + 30.0, horizon)
        ok_alt = (np.minimum(z_i0, z_end) >= 30.0) & (np.maximum(z_i0, z_end) <= 1200.0)
        reasons["slow_closure"] += int((~ok_speed).sum())
        reasons["early_or_initial_loss"] += int((ok_speed & ~ok_onset).sum())
        reasons["intruder_altitude"] += int((ok_speed & ok_onset & ~ok_alt).sum())
        keep = np.flatnonzero(ok_speed & ok_onset & ok_alt)[: n - got]
        for k in ("geometry", "intruder", "v_o", "v_i", "psi_i", "vz_i", "t_cpa0", "hmd0", "vmd0", "h0"):
            accepted[k].append(p[k][keep])
        accepted["s0"].append(s0[keep])
        accepted["vrel"].append(vrel[keep])
        got += keep.size
    P = {k: np.concatenate(v) for k, v in accepted.items()}

    # truth trajectories with gusts
    sig_h_i = np.array([perf[x]["gust_sigma_h"] for x in INTRUDERS])[P["intruder"]]
    sig_z_i = np.array([perf[x]["gust_sigma_z"] for x in INTRUDERS])[P["intruder"]]
    tau = perf["gust_tau"]
    w_i = _ou(rng, n, K, sig_h_i, sig_z_i, tau, dt)
    w_o = _ou(rng, n, K, np.full(n, perf["evtol"]["gust_sigma_h"]),
              np.full(n, perf["evtol"]["gust_sigma_z"]), tau, dt)

    own_p0 = np.stack([np.zeros(n), np.zeros(n), P["h0"]], -1)
    vo_nom = np.stack([P["v_o"], np.zeros(n), np.zeros(n)], -1)
    vi_nom = vo_nom + P["vrel"]
    int_p0 = own_p0 + P["s0"]

    # unannounced intruder turns (dynamic-obstacle behaviour the straight-line
    # predictor cannot anticipate)
    im = ec["intruder_manoeuvre"]
    turns = rng.uniform(size=n) < im["prob"]
    t_m = P["t_cpa0"] - rng.uniform(*im["window_before_cpa"], n)
    dpsi = np.radians(rng.uniform(*im["dpsi_deg"], n)) * rng.choice([-1.0, 1.0], n)
    tk = np.arange(K) * dt
    ramp = np.clip((tk[None, :] - t_m[:, None]) / im["duration"], 0.0, 1.0)
    hdg = P["psi_i"][:, None] + np.where(turns[:, None], dpsi[:, None] * ramp, 0.0)
    vi_path = np.stack([P["v_i"][:, None] * np.cos(hdg), P["v_i"][:, None] * np.sin(hdg),
                        np.repeat(vi_nom[:, None, 2], K, 1)], -1)

    own_vel = vo_nom[:, None, :] + w_o
    int_vel = vi_path + w_i
    own_pos = np.concatenate([own_p0[:, None, :], own_p0[:, None, :] + np.cumsum(own_vel * dt, axis=1)], 1)
    int_pos = np.concatenate([int_p0[:, None, :], int_p0[:, None, :] + np.cumsum(int_vel * dt, axis=1)], 1)
    # pad velocities to K+1 samples (last sample repeats)
    own_vel = np.concatenate([own_vel, own_vel[:, -1:, :]], 1)
    int_vel = np.concatenate([int_vel, int_vel[:, -1:, :]], 1)

    params = {k: P[k] for k in ("t_cpa0", "hmd0", "vmd0", "h0", "v_o", "v_i", "psi_i", "vz_i")}
    params.update(intruder_turn=turns, turn_time=np.where(turns, t_m, np.nan),
                  turn_dpsi_deg=np.where(turns, np.degrees(dpsi), 0.0))
    stats = {"tried": tried, "accepted": n, "rejected": reasons, "seed": seed}
    return EncounterSet(n=n, dt=dt, t=np.arange(K + 1) * dt, geometry=P["geometry"],
                        intruder=P["intruder"], own_p0=own_p0, own_speed=P["v_o"],
                        own_psi=np.zeros(n), own_w=w_o, int_pos=int_pos, int_vel=int_vel,
                        own_pos=own_pos, own_vel=own_vel, params=params, stats=stats)


def truth_outcome(own_pos, int_pos, dt, cfg):
    """Loss of well clear and NMAC on truth trajectories (continuous-time check)."""
    rel = int_pos - own_pos
    wc = cfg["hazard"]["well_clear"]
    nm = cfg["hazard"]["nmac"]
    lowc, onset = g.segment_violation(rel, dt, wc["DTHR"], wc["ZTHR"])
    nmac, _ = g.segment_violation(rel, dt, nm["DTHR"], nm["ZTHR"])
    return lowc, onset, nmac
