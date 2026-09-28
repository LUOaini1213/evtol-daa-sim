"""Metric computation, surveillance model and encounter generator."""
import numpy as np
import pytest

from daa import encounters as E
from daa import geometry as g
from daa import metrics as M
from daa import sensors as S


# ---------------------------------------------------------------- metrics
def test_wilson_known_values():
    p, lo, hi = M.wilson(0, 10)
    assert (p, lo) == (0.0, 0.0) and hi == pytest.approx(0.27753, abs=1e-5)
    p, lo, hi = M.wilson(5, 10)
    assert lo == pytest.approx(0.23659, abs=1e-5) and hi == pytest.approx(0.76341, abs=1e-5)
    assert np.isnan(M.wilson(0, 0)[0])


def test_lead_time_and_missed():
    # onset at 30 s; enc0: alert 5..40 -> lead 25; enc1: alert 2..3 and 20..40 -> lead 10 (latest
    # episode); enc2: no alert -> missed; enc3 non-hazardous with alert -> nuisance
    K = 50
    lv = np.zeros((4, K), np.int8)
    lv[0, 5:41] = 2
    lv[1, 2:4] = 2
    lv[1, 20:41] = 3
    lv[3, 10:12] = 2
    onset = np.array([30.0, 30.0, 30.0, np.nan])
    haz = ~np.isnan(onset)
    o = M.alert_outcomes(lv, haz, onset, 1.0, 25.0)
    assert o["lead"][0] == 25.0 and o["lead"][1] == 10.0 and np.isnan(o["lead"][2])
    assert list(o["missed_or_late"]) == [False, True, True, False]
    assert list(o["nuisance"]) == [False, False, False, True]


def test_alert_after_onset_does_not_count():
    lv = np.zeros((1, 50), np.int8)
    lv[0, 31:] = 2
    o = M.alert_outcomes(lv, np.array([True]), np.array([30.0]), 1.0, 25.0)
    assert np.isnan(o["lead"][0]) and o["missed_or_late"][0]


def test_mcnemar_and_bootstrap():
    x = np.array([1, 1, 1, 1, 1, 1, 0, 0, 0, 0], bool)
    y = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0], bool)
    r = M.mcnemar_exact(x, y)
    assert (r["b"], r["c"]) == (6, 0) and r["p"] == pytest.approx(2 * 0.5 ** 6)
    d = M.paired_bootstrap_diff(x, y, 500, 0)
    assert d["diff"] == pytest.approx(0.6) and d["lo"] <= 0.6 <= d["hi"]
    rr = M.paired_bootstrap_ratio(np.array([1, 0, 0, 0] * 5), np.array([1, 1, 0, 0] * 5), 500, 0)
    assert rr["ratio"] == pytest.approx(0.5)
    assert M.mcnemar_exact(y, y)["p"] == 1.0


# ---------------------------------------------------------------- sensors
def test_alpha_beta_gains_match_riccati():
    dt, sa, sn = 1.0, 0.5, 20.0
    F = np.array([[1, dt], [0, 1]])
    Q = sa ** 2 * np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]])
    H = np.array([[1.0, 0.0]])
    P = np.eye(2) * 1e4
    for _ in range(5000):
        Pp = F @ P @ F.T + Q
        K = Pp @ H.T / (H @ Pp @ H.T + sn ** 2)
        P = (np.eye(2) - K @ H) @ Pp
    a, b = S.alpha_beta_gains(sa * dt * dt / sn)
    assert a == pytest.approx(K[0, 0], rel=1e-6)
    assert b == pytest.approx(K[1, 0] * dt, rel=1e-6)


def test_gilbert_loss_fraction():
    rng = np.random.default_rng(3)
    rec = S.gilbert_mask(rng, 400, 2000, 0.05, 0.2)
    assert 1 - rec.mean() == pytest.approx(0.05 / 0.25, abs=0.01)
    assert S.gilbert_mask(rng, 3, 10, 0.0, 1.0).all()


def test_ideal_track_is_truth_and_noise_scales(cfg):
    enc = E.generate(cfg, n=200, seed=5)
    t0 = S.make_track(enc, cfg, "ideal", 1)
    assert np.array_equal(t0.pos, enc.int_pos) and np.array_equal(t0.vel, enc.int_vel)
    err = {}
    for c in ("nominal", "degraded"):
        t = S.make_track(enc, cfg, c, 1)
        err[c] = np.sqrt(np.mean((t.pos - enc.int_pos)[:, 20:, :2] ** 2))
    assert 0 < err["nominal"] < err["degraded"]


# ------------------------------------------------------------- encounters
def test_encounter_construction_hits_requested_cpa(cfg):
    enc = E.generate(cfg, n=400, seed=11)
    # undisturbed relative motion from t=0 must reproduce the sampled CPA spec
    s0 = enc.int_pos[:, 0] - enc.own_pos[:, 0]
    vo = np.stack([enc.params["v_o"], 0 * enc.params["v_o"], 0 * enc.params["v_o"]], -1)
    vi = np.stack([enc.params["v_i"] * np.cos(enc.params["psi_i"]),
                   enc.params["v_i"] * np.sin(enc.params["psi_i"]), enc.params["vz_i"]], -1)
    tcpa, hmd, vmd = g.cpa(s0, vi - vo)
    assert np.allclose(tcpa, enc.params["t_cpa0"])
    assert np.allclose(hmd, np.abs(enc.params["hmd0"]))
    assert np.allclose(vmd, enc.params["vmd0"])


def test_encounter_constraints_and_determinism(cfg):
    a = E.generate(cfg, n=300, seed=2)
    b = E.generate(cfg, n=300, seed=2)
    assert np.array_equal(a.int_pos, b.int_pos) and np.array_equal(a.own_w, b.own_w)
    wc = cfg["hazard"]["well_clear"]
    s0 = a.int_pos[:, 0] - a.own_pos[:, 0]
    inside0 = (np.hypot(s0[:, 0], s0[:, 1]) <= wc["DTHR"]) & (np.abs(s0[:, 2]) <= wc["ZTHR"])
    assert not inside0.any()
    lo, hi = cfg["performance"]["evtol"]["speed"]
    assert ((a.own_speed >= lo) & (a.own_speed <= hi)).all()
    assert set(np.unique(a.geometry)) <= set(range(len(E.GEOMETRIES)))
