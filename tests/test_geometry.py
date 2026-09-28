"""CPA and WCV geometry against hand-worked cases."""
import math

import numpy as np
import pytest

from daa import geometry as g

D, Z = 670.56, 137.16


def both_wcv(s, v, D, Z, TTHR, T):
    """Run scalar and vectorised versions and check they agree."""
    rs = g.wcv_interval_scalar(s, v, D, Z, TTHR, T)
    hit, lo, hi = g.wcv_interval(np.array(s), np.array(v), D, Z, TTHR, T)
    assert bool(hit) == (rs is not None)
    if rs is not None:
        assert float(lo) == rs[0] and float(hi) == rs[1]
    return rs


def test_head_on_cpa():
    # intruder 1000 m ahead closing at 50 m/s, 30 m above, climbing 1 m/s relative
    tcpa, hmd, vmd = g.cpa_scalar((1000.0, 0.0, 30.0), (-50.0, 0.0, 1.0))
    assert tcpa == pytest.approx(20.0)
    assert hmd == pytest.approx(0.0)
    assert vmd == pytest.approx(50.0)


def test_crossing_cpa_right_angle():
    # ownship east 50 m/s; intruder at (1000,-1000) flying north 50 m/s -> v_rel=(-50,50)
    tcpa, hmd, _ = g.cpa_scalar((1000.0, -1000.0, 0.0), (-50.0, 50.0, 0.0))
    assert tcpa == pytest.approx(20.0)
    assert hmd == pytest.approx(0.0, abs=1e-9)


def test_offset_cpa():
    tcpa, hmd, _ = g.cpa_scalar((1000.0, 300.0, 0.0), (-50.0, 0.0, 0.0))
    assert tcpa == pytest.approx(20.0)
    assert hmd == pytest.approx(300.0)


def test_diverging_cpa_is_now():
    tcpa, hmd, vmd = g.cpa_scalar((1000.0, 0.0, 40.0), (50.0, 0.0, 5.0))
    assert tcpa == pytest.approx(-20.0)
    assert hmd == pytest.approx(1000.0)
    assert vmd == pytest.approx(40.0)  # evaluated at t = 0, not at negative time


def test_zero_relative_speed():
    tcpa, hmd, vmd = g.cpa_scalar((300.0, 400.0, 10.0), (0.0, 0.0, 0.0))
    assert (tcpa, hmd, vmd) == (0.0, 500.0, 10.0)


def test_vectorised_cpa_matches_scalar():
    rng = np.random.default_rng(0)
    s = rng.normal(0, 2000, (500, 3))
    v = rng.normal(0, 40, (500, 3))
    v[:5, :2] = 0.0
    tc, h, vm = g.cpa(s, v)
    for i in range(500):
        assert (tc[i], h[i], vm[i]) == g.cpa_scalar(tuple(s[i]), tuple(v[i]))


def test_wcv_head_on_interval():
    # enters when 1000-50t = 670.56 -> t = 6.5888; exits at 1000-50t = -670.56 -> t = 33.4112
    r = both_wcv((1000.0, 0.0, 0.0), (-50.0, 0.0, 0.0), D, Z, 0.0, 55.0)
    assert r == pytest.approx((6.5888, 33.4112))


def test_wcv_lookahead_clips():
    r = both_wcv((1000.0, 0.0, 0.0), (-50.0, 0.0, 0.0), D, Z, 0.0, 20.0)
    assert r == pytest.approx((6.5888, 20.0))
    assert both_wcv((1000.0, 0.0, 0.0), (-50.0, 0.0, 0.0), D, Z, 0.0, 6.0) is None


def test_wcv_offset_circle():
    # |s|<=500 when x^2 + 300^2 <= 500^2 -> x in [-400, 400] -> t in [12, 28]
    r = both_wcv((1000.0, 300.0, 0.0), (-50.0, 0.0, 0.0), 500.0, Z, 0.0, 100.0)
    assert r == pytest.approx((12.0, 28.0))


def test_wcv_taumod_extends_entry():
    # TTHR = 10 s: violation starts when x^2 - 500x - 160000 <= 0 with x = 1000-50t,
    # i.e. x <= 250 + sqrt(222500) = 721.699...; exit unchanged (circle exit).
    x = 250.0 + math.sqrt(222500.0)
    r = both_wcv((1000.0, 300.0, 0.0), (-50.0, 0.0, 0.0), 500.0, Z, 10.0, 100.0)
    assert r[0] == pytest.approx((1000.0 - x) / 50.0)
    assert r[1] == pytest.approx(28.0)
    # tau_mod at the entry time equals TTHR (DAIDALUS definition)
    t = r[0]
    sx, sy = 1000.0 - 50.0 * t, 300.0
    taumod = (500.0 ** 2 - (sx * sx + sy * sy)) / (sx * -50.0)
    assert taumod == pytest.approx(10.0)


def test_wcv_taumod_needs_dcpa_within_D():
    # hmd 600 > DTHR 500: no violation whatever TTHR is
    assert both_wcv((1000.0, 600.0, 0.0), (-50.0, 0.0, 0.0), 500.0, Z, 60.0, 100.0) is None


def test_wcv_vertical_overlap():
    # vertical |200-10t| <= 137.16 -> [6.284, 33.716]; horizontal [6.5888, 33.4112]
    r = both_wcv((1000.0, 0.0, 200.0), (-50.0, 0.0, -10.0), D, Z, 0.0, 100.0)
    assert r == pytest.approx((6.5888, 33.4112))
    r = both_wcv((1000.0, 0.0, 400.0), (-50.0, 0.0, -10.0), D, Z, 0.0, 100.0)
    assert r == pytest.approx((26.284, 33.4112))


def test_wcv_vertical_never():
    assert both_wcv((1000.0, 0.0, 200.0), (-50.0, 0.0, 0.0), D, Z, 0.0, 100.0) is None


def test_wcv_already_inside():
    r = both_wcv((100.0, 0.0, 0.0), (0.0, 0.0, 0.0), D, Z, 0.0, 55.0)
    assert r == (0.0, 55.0)
    r = both_wcv((100.0, 0.0, 0.0), (10.0, 0.0, 0.0), D, Z, 35.0, 55.0)
    assert r == pytest.approx((0.0, 55.0))  # exits at (670.56-100)/10 = 57.056 s, clipped to T


def test_cpa_misses_vertical_timing_case_that_wcv_catches():
    s, v = (1000.0, 0.0, 400.0), (-50.0, 0.0, -10.0)
    # at horizontal CPA (t=20) vertical separation is 200 m > Dv -> CPA logic says no alert
    assert not g.cpa_condition_scalar(s, v, D, Z, 55.0)
    assert g.wcv_interval_scalar(s, v, D, Z, 0.0, 55.0) is not None


def test_cpa_gated_by_tcpa_in_slow_overtake():
    # closure 5 m/s: tcpa = 400 s, well-clear lost at (2000-670.56)/5 = 265.9 s
    s, v = (2000.0, 0.0, 0.0), (-5.0, 0.0, 0.0)
    assert not g.cpa_condition_scalar(s, v, D, Z, 55.0)
    # 30 s before loss: WCV logic alerts, CPA logic still does not (tcpa = 164 s > 55 s)
    s30 = (670.56 + 150.0, 0.0, 0.0)
    assert g.wcv_interval_scalar(s30, v, D, Z, 0.0, 55.0) is not None
    assert not g.cpa_condition_scalar(s30, v, D, Z, 55.0)
    # once inside the cylinder the CPA logic alerts
    assert g.cpa_condition_scalar((600.0, 0.0, 0.0), v, D, Z, 55.0)


def test_segment_violation_catches_between_samples():
    # relative path passes through the NMAC cylinder between two 1 s samples
    rel = np.array([[[200.0, 0.0, 0.0], [-200.0, 0.0, 0.0]]])
    viol, onset = g.segment_violation(rel, 1.0, 152.4, 30.48)
    assert viol[0] and onset[0] == pytest.approx((200.0 - 152.4) / 400.0)
    rel = np.array([[[200.0, 0.0, 50.0], [-200.0, 0.0, 50.0]]])
    viol, onset = g.segment_violation(rel, 1.0, 152.4, 30.48)
    assert not viol[0] and np.isnan(onset[0])


@pytest.mark.parametrize("TTHR", [0.0, 20.0])
def test_vectorised_wcv_matches_scalar(TTHR):
    rng = np.random.default_rng(int(TTHR) + 5)
    s = rng.uniform(-3000, 3000, (3000, 3))
    s[:, 2] /= 10
    v = rng.uniform(-100, 100, (3000, 3))
    v[:, 2] /= 10
    v[:100, 2] = 0.0
    v[100:200, :2] = 0.0
    hit, lo, hi = g.wcv_interval(s, v, D, Z, TTHR, 60.0)
    for i in range(len(s)):
        r = g.wcv_interval_scalar(tuple(s[i]), tuple(v[i]), D, Z, TTHR, 60.0)
        assert bool(hit[i]) == (r is not None)
        if r is not None:
            assert (lo[i], hi[i]) == r
    assert hit.sum() > 100


def test_cpa_condition_boundaries_inclusive():
    v = (-50.0, 0.0, 0.0)
    assert g.cpa_condition_scalar((2750.0, 0.0, 0.0), v, D, Z, 55.0)        # tcpa == T
    assert not g.cpa_condition_scalar((2750.0, 0.0, 0.0), v, D, Z, 54.999)
    assert g.cpa_condition_scalar((2750.0, 670.56, 0.0), v, D, Z, 55.0)     # hmd == Dh
    assert g.cpa_condition_scalar((2750.0, 0.0, 137.16), v, D, Z, 55.0)     # |vmd| == Dv
    assert not g.cpa_condition_scalar((2750.0, 0.0, 137.17), v, D, Z, 55.0)
