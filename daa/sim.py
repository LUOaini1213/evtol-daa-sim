"""Open-loop alerting and closed-loop avoidance simulation.

Ownship dynamics (per 1 s step, explicit Euler):
  heading slews toward the command at most omega = g*tan(bank_max)/V,
  speed slews toward the command at most a_x,
  vertical rate slews toward clip(k_z*(z_target - z_nom), +-vz_max) at most a_z,
  position += (commanded-state velocity + gust) * dt.
The gust sequence is the same one used for the unmitigated trajectory, so the
"none" policy reproduces the open-loop ownship path (checked in tests).

Manoeuvre policies
  none      : no manoeuvre (baseline)
  heading   : heading change only, candidates +-10..90 deg
  vertical  : altitude change only, candidates +-40..200 m within floor/ceiling
  combined  : heading x altitude grid
On a warning-level alert every non-"none" policy escalates once to an
emergency manoeuvre: heading x altitude x {current speed, v_min} at full
performance, picking the candidate with the largest predicted separation.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import alerting as al
from .encounters import truth_outcome

G0 = 9.80665
K_Z = 0.5  # 1/s altitude-capture gain (assumption)
POLICIES = ("none", "heading", "vertical", "combined")
REPLAN_PERIOD = 5  # cycles between re-checks of a committed manoeuvre (assumption)
ALTITUDE_EPS = 1e-9  # metres/vertical-rate round-off, not an operating tolerance


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def turn_rate(V, bank_deg):
    return G0 * np.tan(np.radians(bank_deg)) / np.maximum(V, 1.0)


def _braking_rate(headroom, acceleration, dt):
    """Largest rate stoppable inside headroom with this explicit-Euler order.

    A rate q flies one full step before its next acceleration applies. With
    delta=a*dt and n positive steps, distance=dt*(n*q-delta*n*(n-1)/2).
    Inverting the triangular breakpoints avoids a continuous-time braking
    approximation that would omit the first held-velocity step.
    """
    delta = acceleration * dt
    distance = np.maximum(headroom, 0.0) / dt
    n = np.maximum(1.0, np.ceil((np.sqrt(1.0 + 8.0 * distance / delta) - 1.0) / 2.0))
    return (distance + delta * n * (n - 1.0) / 2.0) / n


def _bounded_vertical_rate(vz, requested, z, z_next, perf, dt, band):
    """Constrain commanded altitude without moving it or exceeding acceleration.

    The OU gust remains an unbounded external disturbance on truth position.
    This guard certifies the commanded Euler path, not unknown future gusts.
    """
    floor, ceiling = np.asarray(band, dtype=float)
    a, vmax = float(perf["a_z"]), float(perf["vz_max"])
    vz, requested, z, z_next = (np.asarray(value, dtype=float) for value in (vz, requested, z, z_next))
    if (not np.isfinite([floor, ceiling, a, vmax, dt]).all() or floor >= ceiling
            or a <= 0 or vmax <= 0 or dt <= 0):
        raise ValueError("altitude band and dynamics must be finite, ordered and positive")
    if (not np.isfinite(z).all() or not np.isfinite(z_next).all() or not np.isfinite(vz).all()
            or not np.isfinite(requested).all()
            or np.any(z < floor - ALTITUDE_EPS) or np.any(z > ceiling + ALTITUDE_EPS)
            or np.any(z_next < floor - ALTITUDE_EPS) or np.any(z_next > ceiling + ALTITUDE_EPS)):
        raise ValueError("altitude state/held-velocity step is infeasible within the band")
    lower = np.maximum.reduce([vz - a * dt, np.full_like(vz, -vmax),
                              -_braking_rate(z_next - floor, a, dt)])
    upper = np.minimum.reduce([vz + a * dt, np.full_like(vz, vmax),
                              _braking_rate(ceiling - z_next, a, dt)])
    if np.any(lower > upper + ALTITUDE_EPS):
        raise ValueError("altitude state cannot brake inside the band within acceleration limits")
    result = np.clip(requested, lower, np.maximum(upper, lower))
    return result, np.abs(result - requested) > ALTITUDE_EPS


def step_dynamics(psi, V, vz, znom, psi_cmd, V_cmd, z_tgt, perf, dt, *,
                  altitude_band=None, return_limited=False):
    """Advance ownship commanded state one step. Returns new state and the
    velocity flown during the step (state at the start of the step)."""
    if altitude_band is not None and (not np.isfinite(dt) or dt <= 0):
        raise ValueError("bounded altitude step requires a finite positive dt")
    vel = np.stack([V * np.cos(psi), V * np.sin(psi), vz], -1)
    om = turn_rate(V, perf["bank_max_deg"]) * dt
    psi_n = psi + np.clip(wrap(psi_cmd - psi), -om, om)
    V_n = V + np.clip(V_cmd - V, -perf["a_x"] * dt, perf["a_x"] * dt)
    vz_cmd = np.clip(K_Z * (z_tgt - znom), -perf["vz_max"], perf["vz_max"])
    vz_n = vz + np.clip(vz_cmd - vz, -perf["a_z"] * dt, perf["a_z"] * dt)
    znom_n = znom + vz * dt
    limited = np.zeros_like(vz_n, dtype=bool)
    if altitude_band is not None:
        vz_n, limited = _bounded_vertical_rate(vz, vz_n, znom, znom_n, perf, dt, altitude_band)
    values = (psi_n, V_n, vz_n, znom_n, vel)
    return values + (limited,) if return_limited else values


# ------------------------------------------------------------ open loop ----
def relative_states(enc, track, own_pos=None, own_vel=None):
    own_pos = enc.own_pos if own_pos is None else own_pos
    own_vel = enc.own_vel if own_vel is None else own_vel
    return track.pos - own_pos, track.vel - own_vel


def open_loop_levels(enc, track, strategy, tiers, filt=None):
    """Alert levels [N, K+1] on the unmitigated trajectories. filt = dict(M, N, H)
    in cycles, or None for raw levels."""
    s, v = relative_states(enc, track)
    lv = al.alert_level(strategy, s, v, tiers)
    if filt:
        lv = al.filter_levels(lv, filt["M"], filt["N"], filt["H"])
    return lv


# ------------------------------------------------------------- planner -----
def candidate_set(kind, cfg):
    """Return arrays (dpsi_rad, dz_m, slow_flag, cost) of candidates."""
    sim = cfg["simulation"]
    hd = np.array(sim["heading_candidates_deg"], float)
    az = np.array(sim["altitude_candidates_m"], float)
    if kind == "heading":
        d = np.concatenate([[0.0], -hd, hd])  # negative = right turn first
        dz = np.zeros_like(d)
        slow = np.zeros_like(d, bool)
        cost = np.abs(d) + 0.1 * (d > 0)       # prefer right turn on ties (14 CFR 91.113)
    elif kind == "vertical":
        dz = np.concatenate([[0.0], az, -az])
        d = np.zeros_like(dz)
        slow = np.zeros_like(d, bool)
        cost = np.abs(dz) + 0.1 * (dz < 0)     # prefer climb on ties
    elif kind == "combined":
        hh = np.array([0, -20, 20, -40, 40, -60, 60, -90, 90], float)
        zz = np.array([0, 80, -80, 160, -160], float)
        d, dz = [a.ravel() for a in np.meshgrid(hh, zz)]
        slow = np.zeros_like(d, bool)
        cost = np.abs(d) / 30.0 + np.abs(dz) / 80.0 + 0.01 * (d > 0)
    elif kind == "emergency":
        hh = np.array([0, -30, 30, -60, 60, -90, 90], float)
        zz = np.array([0, 100, -100, 200, -200], float)
        ss = np.array([0, 1], float)
        d, dz, sl = [a.ravel() for a in np.meshgrid(hh, zz, ss)]
        slow = sl.astype(bool)
        cost = np.abs(d) / 30.0 + np.abs(dz) / 80.0 + 0.5 * slow + 0.01 * (d > 0)
    else:
        raise ValueError(kind)
    return np.radians(d), dz, slow, cost


def _select(mmin, cost, *, maximize=False, keep_current=False):
    """Cheapest candidate with predicted min separation ratio >= 1; if none,
    the candidate with the largest ratio (cheapest among ties). Emergency selection
    maximizes the ratio even if a cheaper clear candidate exists. A clear current
    command is retained during rechecks, as specified by the replanning policy."""
    free = mmin >= 1.0
    any_free = free.any(axis=1)
    pick_free = np.where(free, cost[None, :], np.inf).argmin(axis=1)
    best = mmin.max(axis=1, keepdims=True)
    pick_best = np.where(mmin >= best - 1e-9, cost[None, :], np.inf).argmin(axis=1)
    pick = pick_best if maximize else np.where(any_free, pick_free, pick_best)
    if keep_current:
        pick = np.where(free[:, 0], 0, pick)
    return pick, any_free


def plan(kind, cfg, psi, V, vz, znom, z_true, pos, cmd, p_int, v_int, delay_steps, include_current=False):
    """Choose a manoeuvre for J encounters at once.

    psi, V, vz, znom, z_true: [J]; pos, p_int, v_int: [J,3]; cmd = (psi_cmd,
    V_cmd, z_tgt) currently in force, each [J]. With include_current, the
    command in force is candidate 0 at zero cost ("keep" = it was chosen).
    Returns dict of arrays [J].
    """
    perf = cfg["performance"]["evtol"]
    sim = cfg["simulation"]
    hz = cfg["hazard"]["well_clear"]
    b = sim["plan_buffer"]["value"]
    Dp, Zp = b * hz["DTHR"], b * hz["ZTHR"]
    dt = sim["dt"]
    L = int(round(sim["plan_lookahead"]["value"] / dt))
    dpsi, dz, slow, cost = candidate_set(kind, cfg)
    J, C = psi.shape[0], dpsi.shape[0]

    tgt_psi = wrap(psi[:, None] + dpsi[None, :])
    tgt_z = znom[:, None] + dz[None, :]
    feasible = ((tgt_z >= sim["floor_m"]["value"] - ALTITUDE_EPS)
                & (tgt_z <= sim["ceiling_m"]["value"] + ALTITUDE_EPS))
    tgt_V = np.where(slow[None, :], perf["v_min_emergency"], V[:, None])
    if include_current:
        tgt_psi = np.concatenate([cmd[0][:, None], tgt_psi], 1)
        tgt_V = np.concatenate([cmd[1][:, None], tgt_V], 1)
        tgt_z = np.concatenate([cmd[2][:, None], tgt_z], 1)
        current_feasible = ((cmd[2] >= sim["floor_m"]["value"] - ALTITUDE_EPS)
                            & (cmd[2] <= sim["ceiling_m"]["value"] + ALTITUDE_EPS))
        feasible = np.concatenate([current_feasible[:, None], feasible], 1)
        dpsi, dz = np.concatenate([[0.0], dpsi]), np.concatenate([[0.0], dz])
        slow, cost = np.concatenate([[False], slow]), np.concatenate([[-1.0], cost])
        C += 1

    P = np.repeat(pos[:, None, :], C, 1)
    ps = np.repeat(psi[:, None], C, 1)
    Vs = np.repeat(V[:, None], C, 1)
    vzs = np.repeat(vz[:, None], C, 1)
    zn = np.repeat(znom[:, None], C, 1)
    mmin = np.full((J, C), np.inf)
    band = (sim["floor_m"]["value"], sim["ceiling_m"]["value"])
    band_limited = np.zeros((J, C), bool)
    for i in range(L):
        if i < delay_steps:
            pc, vc, zc = cmd[0][:, None], cmd[1][:, None], cmd[2][:, None]
        else:
            pc, vc, zc = tgt_psi, tgt_V, tgt_z
        ps, Vs, vzs, zn, vel, limited_step = step_dynamics(
            ps, Vs, vzs, zn, pc, vc, zc, perf, dt, altitude_band=band, return_limited=True)
        band_limited |= limited_step
        P = P + vel * dt
        if i + 1 >= delay_steps:
            s = p_int[:, None, :] + v_int[:, None, :] * ((i + 1) * dt) - P
            m = np.maximum(np.hypot(s[..., 0], s[..., 1]) / Dp, np.abs(s[..., 2]) / Zp)
            mmin = np.minimum(mmin, m)
    objective = dict(maximize=kind == "emergency", keep_current=include_current)
    if np.any(~feasible.any(axis=1)):
        raise ValueError("no altitude-band-feasible candidate")
    pick, any_free = _select(np.where(feasible, mmin, -np.inf), cost, **objective)
    pick_unconstrained, _ = _select(mmin, cost, **objective)
    r = np.arange(J)
    # Target exclusion or actual controller braking constrained this decision.
    limited = (pick != pick_unconstrained) | band_limited[r, pick]
    mmin = np.where(feasible, mmin, -np.inf)
    return dict(psi_cmd=tgt_psi[r, pick], V_cmd=tgt_V[r, pick], z_tgt=tgt_z[r, pick],
                dpsi=dpsi[pick], dz=dz[pick], slow=slow[pick], free=any_free,
                mmin=mmin[r, pick], limited=limited,
                keep=(pick == 0) if include_current else np.zeros(J, bool))


# --------------------------------------------------------- closed loop -----
@dataclass
class ClosedLoopResult:
    lowc: np.ndarray
    onset: np.ndarray
    nmac: np.ndarray
    manoeuvred: np.ndarray
    emergency: np.ndarray
    first_plan_t: np.ndarray
    max_dpsi_deg: np.ndarray
    max_dz_m: np.ndarray
    floor_limited: np.ndarray
    plan_free: np.ndarray
    n_replans: np.ndarray
    levels: np.ndarray
    own_pos: np.ndarray


def closed_loop(enc, track, strategy, tiers, policy, cfg, filt=None, response_delay=None,
                replan=True):
    """Run the closed loop. replan: while a corrective-or-higher alert persists and
    nothing is pending, re-check the committed manoeuvre every REPLAN_PERIOD s
    against the latest track and re-plan if it is no longer predicted clear."""
    if policy not in POLICIES:
        raise ValueError(policy)
    perf = cfg["performance"]["evtol"]
    sim = cfg["simulation"]
    dt = enc.dt
    N, K1 = enc.int_pos.shape[:2]
    delay = sim["response_delay"]["value"] if response_delay is None else response_delay
    d = int(round(delay / dt))
    filt_obj = al.AlertFilter(N, filt["M"], filt["N"], filt["H"]) if filt else None
    w = np.concatenate([enc.own_w, enc.own_w[:, -1:, :]], 1)

    psi = enc.own_psi.copy()
    V = enc.own_speed.copy()
    vz = np.zeros(N)
    znom = enc.own_p0[:, 2].copy()
    psi0, z0 = psi.copy(), znom.copy()
    cmd = [psi.copy(), V.copy(), znom.copy()]
    pos = enc.own_p0.copy()
    own_pos = np.empty((N, K1, 3))
    own_pos[:, 0] = pos
    raw = np.zeros((N, K1), np.int8)
    levels = np.zeros((N, K1), np.int8)
    stage = np.zeros(N, np.int8)
    pend_step = np.full(N, -1)
    pend = [np.zeros(N), np.zeros(N), np.zeros(N)]
    manoeuvred = np.zeros(N, bool)
    first_plan_t = np.full(N, np.nan)
    floor_limited = np.zeros(N, bool)
    plan_free = np.ones(N, bool)
    max_dpsi = np.zeros(N)
    max_dz = np.zeros(N)
    last_plan_k = np.full(N, -10**6)
    n_replans = np.zeros(N, int)

    for k in range(K1):
        vel_state = np.stack([V * np.cos(psi), V * np.sin(psi), vz], -1)
        own_v = vel_state + w[:, k]
        s = track.pos[:, k] - pos
        v = track.vel[:, k] - own_v
        raw[:, k] = al.alert_level(strategy, s, v, tiers)
        levels[:, k] = filt_obj.update(raw[:, k]) if filt_obj else raw[:, k]
        lvl = levels[:, k]

        if policy != "none":
            need_e = (lvl == 3) & (stage < 2)
            need_c = (lvl >= 2) & (stage == 0) & ~need_e
            for kind, need, st in (("emergency", need_e, 2), (policy, need_c, 1)):
                idx = np.flatnonzero(need)
                if idx.size == 0:
                    continue
                p = plan(kind, cfg, psi[idx], V[idx], vz[idx], znom[idx], pos[idx, 2], pos[idx],
                         [c[idx] for c in cmd], track.pos[idx, k], track.vel[idx, k], d)
                pend[0][idx], pend[1][idx], pend[2][idx] = p["psi_cmd"], p["V_cmd"], p["z_tgt"]
                pend_step[idx] = k + d
                last_plan_k[idx] = k
                stage[idx] = st
                first_plan_t[idx] = np.where(np.isnan(first_plan_t[idx]), k * dt, first_plan_t[idx])
                floor_limited[idx] |= p["limited"]
                plan_free[idx] &= p["free"]
                manoeuvred[idx] |= (p["dpsi"] != 0) | (p["dz"] != 0) | p["slow"]
            if replan:
                chk = (stage >= 1) & (lvl >= 2) & (pend_step < 0) & (k - last_plan_k >= REPLAN_PERIOD)                     & ~need_e & ~need_c
                for kind, st in ((policy, 1), ("emergency", 2)):
                    idx = np.flatnonzero(chk & (stage == st))
                    if idx.size == 0:
                        continue
                    p = plan(kind, cfg, psi[idx], V[idx], vz[idx], znom[idx], pos[idx, 2], pos[idx],
                             [c[idx] for c in cmd], track.pos[idx, k], track.vel[idx, k], d,
                             include_current=True)
                    last_plan_k[idx] = k
                    ch = idx[~p["keep"]]
                    sel = ~p["keep"]
                    pend[0][ch], pend[1][ch], pend[2][ch] = p["psi_cmd"][sel], p["V_cmd"][sel], p["z_tgt"][sel]
                    pend_step[ch] = k + d
                    n_replans[ch] += 1
                    floor_limited[ch] |= p["limited"][sel]
                    plan_free[ch] &= p["free"][sel]
                    manoeuvred[ch] |= (p["dpsi"][sel] != 0) | (p["dz"][sel] != 0) | p["slow"][sel]
            act = pend_step == k
            if act.any():
                for c, pv in zip(cmd, pend):
                    c[act] = pv[act]
                pend_step[act] = -1

        if k == K1 - 1:
            break
        pos = pos + own_v * dt
        psi, V, vz, znom, _, bounded = step_dynamics(
            psi, V, vz, znom, cmd[0], cmd[1], cmd[2], perf, dt,
            altitude_band=(sim["floor_m"]["value"], sim["ceiling_m"]["value"]), return_limited=True)
        floor_limited |= bounded
        own_pos[:, k + 1] = pos
        max_dpsi = np.maximum(max_dpsi, np.abs(np.degrees(wrap(psi - psi0))))
        max_dz = np.maximum(max_dz, np.abs(znom - z0))

    lowc, onset, nmac = truth_outcome(own_pos, enc.int_pos, dt, cfg)
    return ClosedLoopResult(lowc=lowc, onset=onset, nmac=nmac, manoeuvred=manoeuvred,
                            emergency=stage == 2, first_plan_t=first_plan_t,
                            max_dpsi_deg=max_dpsi, max_dz_m=max_dz, floor_limited=floor_limited,
                            plan_free=plan_free, n_replans=n_replans, levels=levels, own_pos=own_pos)
