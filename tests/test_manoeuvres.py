"""Ownship dynamics stay within the performance table; planner behaviour."""
import numpy as np
import pytest

from daa import alerting as al
from daa import encounters as E
from daa import sensors as S
from daa import sim


@pytest.fixture(scope="module")
def small(cfg):
    enc = E.generate(cfg, n=300, seed=7)
    tr = S.make_track(enc, cfg, "nominal", 8)
    return enc, tr


def test_step_dynamics_respects_limits(cfg):
    perf = cfg["performance"]["evtol"]
    rng = np.random.default_rng(1)
    n = 2000
    psi = rng.uniform(-np.pi, np.pi, n)
    V = rng.uniform(*perf["speed"], n)
    vz = rng.uniform(-perf["vz_max"], perf["vz_max"], n)
    zn = rng.uniform(150, 600, n)
    psi_c = rng.uniform(-np.pi, np.pi, n)
    V_c = rng.uniform(10, 90, n)
    z_c = rng.uniform(0, 1000, n)
    p2, V2, vz2, zn2, vel = sim.step_dynamics(psi, V, vz, zn, psi_c, V_c, z_c, perf, 1.0)
    om = np.degrees(sim.turn_rate(V, perf["bank_max_deg"]))
    assert (np.abs(np.degrees(sim.wrap(p2 - psi))) <= om + 1e-9).all()
    assert (np.abs(V2 - V) <= perf["a_x"] + 1e-12).all()
    assert (np.abs(vz2 - vz) <= perf["a_z"] + 1e-12).all()
    assert (np.abs(vz2) <= perf["vz_max"] + 1e-12).all()
    assert np.allclose(np.hypot(vel[:, 0], vel[:, 1]), V)


def test_turn_rate_from_bank():
    # 9.80665*tan(30 deg)/60 m/s = 0.094366 rad/s = 5.4067 deg/s
    assert np.degrees(sim.turn_rate(60.0, 30.0)) == pytest.approx(5.4067, abs=1e-3)


def test_turn_takes_the_short_way(cfg):
    perf = cfg["performance"]["evtol"]
    psi = np.array([np.radians(170.0)])
    p2, *_ = sim.step_dynamics(psi, np.array([60.0]), np.zeros(1), np.full(1, 300.0),
                               np.array([np.radians(-170.0)]), np.array([60.0]), np.full(1, 300.0), perf, 1.0)
    assert np.degrees(sim.wrap(p2 - psi))[0] > 0  # +20 deg is shorter than -340 deg


def test_none_policy_reproduces_open_loop(cfg, small):
    enc, tr = small
    r = sim.closed_loop(enc, tr, "wcv", al.tier_params(cfg, "wcv"), "none", cfg)
    assert np.allclose(r.own_pos, enc.own_pos, atol=1e-6)
    assert not r.manoeuvred.any()
    haz, _, _ = E.truth_outcome(enc.own_pos, enc.int_pos, enc.dt, cfg)
    assert (r.lowc == haz).all()


@pytest.mark.parametrize("policy", ["heading", "vertical", "combined"])
def test_closed_loop_bounds(cfg, small, policy):
    enc, tr = small
    perf = cfg["performance"]["evtol"]
    r = sim.closed_loop(enc, tr, "wcv", al.tier_params(cfg, "wcv"), policy, cfg)
    step = np.diff(r.own_pos, axis=1) - enc.own_w  # commanded-state velocity per step
    vz = step[..., 2]
    assert np.abs(vz).max() <= perf["vz_max"] + 1e-9
    assert np.abs(np.diff(vz, axis=1)).max() <= perf["a_z"] + 1e-9
    hdg = np.unwrap(np.arctan2(step[..., 1], step[..., 0]), axis=1)
    V = np.hypot(step[..., 0], step[..., 1])
    om = np.degrees(sim.turn_rate(V[:, :-1], perf["bank_max_deg"]))
    assert (np.abs(np.degrees(np.diff(hdg, axis=1))) <= om + 1e-6).all()
    assert np.abs(np.diff(V, axis=1)).max() <= perf["a_x"] + 1e-9
    if policy == "heading":
        assert r.max_dz_m[~r.emergency].max(initial=0.0) < 1e-9  # no altitude change unless escalated
    if policy == "vertical":
        assert r.max_dpsi_deg[~r.emergency].max(initial=0.0) < 1e-9


def test_vertical_candidates_respect_floor(cfg):
    # ownship 20 m above the floor, intruder descending onto it from above:
    # every chosen target must stay within [floor, ceiling]
    floor = cfg["simulation"]["floor_m"]["value"]
    J = 1
    p = sim.plan("vertical", cfg, np.zeros(J), np.full(J, 60.0), np.zeros(J), np.full(J, floor + 20.0),
                 np.full(J, floor + 20.0), np.array([[0.0, 0.0, floor + 20.0]]),
                 [np.zeros(J), np.full(J, 60.0), np.full(J, floor + 20.0)],
                 np.array([[3000.0, 0.0, floor + 60.0]]), np.array([[-10.0, 0.0, -0.5]]), 5)
    assert p["z_tgt"][0] >= floor
    assert p["dz"][0] > 0  # must climb: descending is outside the band


def test_head_on_prefers_right_turn(cfg):
    J = 1
    p = sim.plan("heading", cfg, np.zeros(J), np.full(J, 60.0), np.zeros(J), np.full(J, 300.0),
                 np.full(J, 300.0), np.array([[0.0, 0.0, 300.0]]),
                 [np.zeros(J), np.full(J, 60.0), np.full(J, 300.0)],
                 np.array([[5000.0, 0.0, 300.0]]), np.array([[-60.0, 0.0, 0.0]]), 5)
    assert p["free"][0]
    assert p["dpsi"][0] < 0  # right turn (clockwise) in this frame


def test_manoeuvre_reduces_loss_on_small_set(cfg, small):
    enc, tr = small
    base = sim.closed_loop(enc, tr, "wcv", al.tier_params(cfg, "wcv"), "none", cfg)
    r = sim.closed_loop(enc, tr, "wcv", al.tier_params(cfg, "wcv"), "heading", cfg)
    assert r.lowc.sum() < 0.5 * base.lowc.sum()


def test_response_delay_is_respected(cfg, small):
    enc, tr = small
    d = 10.0
    r = sim.closed_loop(enc, tr, "wcv", al.tier_params(cfg, "wcv"), "heading", cfg, response_delay=d)
    step = np.diff(r.own_pos, axis=1) - enc.own_w
    hdg = np.degrees(np.arctan2(step[..., 1], step[..., 0]))
    idx = np.flatnonzero(r.manoeuvred)
    assert idx.size > 10
    for i in idx:
        k0 = int(r.first_plan_t[i] / enc.dt)
        # command applied at k0+d; heading first changes over the step after that
        assert np.abs(hdg[i, : k0 + int(d) + 1]).max() < 1e-6
        assert np.abs(hdg[i, k0 + int(d) + 1:]).max() > 1.0


def test_replan_keeps_clear_manoeuvre(cfg):
    # the command in force already avoids the intruder -> re-plan keeps it
    J = 1
    own = np.array([[0.0, 0.0, 300.0]])
    cmd = [np.array([np.radians(-60.0)]), np.full(J, 60.0), np.full(J, 300.0)]
    p = sim.plan("heading", cfg, np.array([np.radians(-60.0)]), np.full(J, 60.0), np.zeros(J), np.full(J, 300.0),
                 np.full(J, 300.0), own, cmd, np.array([[5000.0, 0.0, 300.0]]), np.array([[-60.0, 0.0, 0.0]]), 5,
                 include_current=True)
    assert p["keep"][0]
    # command in force flies straight at the intruder -> re-plan changes it
    cmd = [np.zeros(J), np.full(J, 60.0), np.full(J, 300.0)]
    p = sim.plan("heading", cfg, np.zeros(J), np.full(J, 60.0), np.zeros(J), np.full(J, 300.0),
                 np.full(J, 300.0), own, cmd, np.array([[5000.0, 0.0, 300.0]]), np.array([[-60.0, 0.0, 0.0]]), 5,
                 include_current=True)
    assert not p["keep"][0] and p["dpsi"][0] < 0
