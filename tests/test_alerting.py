"""Alert tiers and the persistence/hysteresis filter."""
import numpy as np
import pytest

from daa import alerting as al


@pytest.fixture
def wcv_tiers(cfg):
    return al.tier_params(cfg, "wcv")


def lvl(strategy, s, v, tiers):
    a = al.alert_level_scalar(strategy, s, v, tiers)
    b = int(al.alert_level(strategy, np.array(s), np.array(v), tiers))
    assert a == b
    return a


def test_tier_values_from_config(cfg):
    t = al.tier_params(cfg, "wcv")
    assert [x["T"] for x in t] == [55.0, 55.0, 25.0]
    assert [x["Z"] for x in t] == [213.36, 137.16, 137.16]
    assert all(x["D"] == 670.56 and x["TTHR"] == 0.0 for x in t)
    assert all(x["TTHR"] == 20.0 for x in al.tier_params(cfg, "wcv_tau"))
    with pytest.raises(ValueError):
        al.tier_params(cfg, "nope")


def test_wcv_tiers_by_time_to_loss(wcv_tiers):
    v = (-50.0, 0.0, 0.0)
    # loss starts at (x - 670.56)/50 s
    assert lvl("wcv", (670.56 + 50 * 70, 0.0, 0.0), v, wcv_tiers) == 0   # 70 s away
    assert lvl("wcv", (670.56 + 50 * 40, 0.0, 0.0), v, wcv_tiers) == 2   # 40 s -> corrective
    assert lvl("wcv", (670.56 + 50 * 20, 0.0, 0.0), v, wcv_tiers) == 3   # 20 s -> warning


def test_preventive_only_with_larger_vertical_band(wcv_tiers):
    # 180 m vertical offset held constant: inside 213.36 (preventive) but not 137.16
    assert lvl("wcv", (670.56 + 50 * 40, 0.0, 180.0), (-50.0, 0.0, 0.0), wcv_tiers) == 1


def test_cpa_tiers(cfg):
    t = al.tier_params(cfg, "cpa")
    v = (-50.0, 0.0, 0.0)
    assert lvl("cpa", (50 * 70.0, 0.0, 0.0), v, t) == 0     # tcpa 70 s
    assert lvl("cpa", (50 * 40.0, 0.0, 0.0), v, t) == 2     # tcpa 40 s
    assert lvl("cpa", (50 * 20.0, 0.0, 0.0), v, t) == 3     # tcpa 20 s
    assert lvl("cpa", (50 * 40.0, 700.0, 0.0), v, t) == 0   # hmd 700 > 670.56


def test_filter_identity_when_trivial():
    raw = np.array([[0, 2, 0, 3, 3, 1, 0, 2]], np.int8)
    assert (al.filter_levels(raw, 1, 1, 1) == raw).all()


def test_filter_m_of_n_raise_and_hold():
    raw = np.array([[0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], np.int8)
    # single-cycle blip is suppressed by 2-of-4
    assert al.filter_levels(raw, 2, 4, 5).max() == 0
    raw = np.array([[0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0]], np.int8)
    out = al.filter_levels(raw, 2, 4, 5)[0]
    # raised at k=3 (2 of last 4), held until raw has been below for 5 cycles (k=8)
    assert list(out) == [0, 0, 0, 2, 2, 2, 2, 2, 0, 0, 0, 0]


def test_filter_levels_independent():
    raw = np.array([[3, 3, 2, 2, 2, 0, 0, 0, 0, 0]], np.int8)
    out = al.filter_levels(raw, 2, 4, 3)[0]
    assert list(out) == [0, 3, 3, 3, 2, 2, 2, 0, 0, 0]


def test_episodes_and_first_time():
    lv = np.array([[0, 2, 2, 0, 3, 0, 0, 2], [0, 0, 0, 0, 0, 0, 0, 0]], np.int8)
    assert list(al.episodes_before(lv, np.array([5.0, np.nan]), 1.0)) == [2, 0]
    ft = al.first_time_at_or_above(lv, 3, 1.0)
    assert ft[0] == 4.0 and np.isnan(ft[1])


def test_cpa_alerts_are_subset_of_wcv_alerts(cfg):
    # with equal thresholds and TTHR = 0, a CPA alert implies a WCV alert: at t = tcpa
    # (<= T) the intruder is inside both the horizontal and vertical bands
    rng = np.random.default_rng(4)
    s = np.column_stack([rng.uniform(-4000, 4000, 20000), rng.uniform(-4000, 4000, 20000), rng.uniform(-300, 300, 20000)])
    v = np.column_stack([rng.uniform(-120, 120, 20000), rng.uniform(-120, 120, 20000), rng.uniform(-8, 8, 20000)])
    lc = al.alert_level("cpa", s, v, al.tier_params(cfg, "cpa"))
    lw = al.alert_level("wcv", s, v, al.tier_params(cfg, "wcv"))
    assert (lw >= lc).all()
    assert (lw > lc).any()  # and WCV is strictly larger somewhere
