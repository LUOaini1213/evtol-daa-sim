"""Independent boundary oracles and public-input contracts for the DAA core."""
from decimal import Decimal, localcontext
import math
import subprocess

import numpy as np
import pytest

from daa import alerting as al
from daa import geometry as g


def decimal_horizontal(s, v, D, tau, horizon):
    """80-digit oracle using exact input doubles, not the production formula."""
    with localcontext() as ctx:
        ctx.prec = 80
        sx, sy = (Decimal.from_float(float(x)) for x in s[:2])
        vx, vy = (Decimal.from_float(float(x)) for x in v[:2])
        radius = Decimal.from_float(float(D))
        buffer = Decimal.from_float(float(tau))
        end = Decimal.from_float(float(horizon))
        a, sv = vx * vx + vy * vy, sx * vx + sy * vy
        ss = sx * sx + sy * sy
        disc = sv * sv - a * (ss - radius * radius)
        if disc < 0:
            return None
        b = 2 * sv + buffer * a
        c = ss + buffer * sv - radius * radius
        lo = max(Decimal(0), (-b - (b * b - 4 * a * c).sqrt()) / (2 * a))
        hi = min(end, (-sv + disc.sqrt()) / a)
        return None if lo > hi else (float(lo), float(hi))


BOUNDARIES = [
    ((2000., 670.56 + offset, 0.), (-speed, 0., 0.), 670.56)
    for speed in (40., 50., 56., 80., 112.) for offset in (-1e-6, 0., 1e-6)
] + [((250., -500. + offset, 0.), (3., 4., 0.), 500.) for offset in (-1e-6, 0., 1e-6)]


@pytest.mark.parametrize("s,v,D", BOUNDARIES)
@pytest.mark.parametrize("tau", [0., 20.])
def test_wcv_boundary_matches_high_precision_oracle(s, v, D, tau):
    expected = decimal_horizontal(s, v, D, tau, 120.)
    scalar = g.wcv_interval_scalar(s, v, D, 137.16, tau, 120.)
    hit, lo, hi = g.wcv_interval(s, v, D, 137.16, tau, 120.)
    assert bool(hit) == (expected is not None)
    assert (scalar is not None) == (expected is not None)
    if expected is not None:
        assert scalar == pytest.approx(expected, abs=1e-9, rel=1e-12)
        assert (float(lo), float(hi)) == scalar


def test_default_tangent_preserves_cpa_subset(cfg):
    s, v = (2000., 670.56, 0.), (-80., 0., 0.)
    cpa = al.alert_level_scalar("cpa", s, v, al.tier_params(cfg, "cpa"))
    assert cpa == 3
    for strategy in ("wcv", "wcv_tau"):
        tiers = al.tier_params(cfg, strategy)
        assert al.alert_level_scalar(strategy, s, v, tiers) >= cpa
        assert int(al.alert_level(strategy, s, v, tiers)) >= cpa


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, True, "1", 1j])
@pytest.mark.parametrize("which", ["s", "v"])
def test_scalar_and_vector_reject_invalid_state(bad, which):
    state = [1000., 0., 0.]
    velocity = [-50., 0., 0.]
    (state if which == "s" else velocity)[0] = bad
    for call in (g.cpa_scalar, g.cpa):
        with pytest.raises(ValueError):
            call(state, velocity)
    for call in (g.wcv_interval_scalar, g.wcv_interval):
        with pytest.raises(ValueError):
            call(state, velocity, 670.56, 137.16, 0., 55.)


@pytest.mark.parametrize("state", [[], [1., 2.], [1., 2., 3., 4.], [[1., 2.], [3., 4.]], 1.])
def test_bad_state_shape_rejected(state):
    for call in (g.cpa_scalar, g.cpa):
        with pytest.raises(ValueError):
            call(state, [0., 0., 0.])


@pytest.mark.parametrize("which", range(4))
@pytest.mark.parametrize("bad", [-1., math.nan, math.inf, True, "1"])
def test_wcv_parameter_validation_before_any_early_return(which, bad):
    params = [670.56, 137.16, 0., 55.]
    params[which] = bad
    # Disconnected/vertically separated states must not conceal illegal inputs.
    for call in (g.wcv_interval_scalar, g.wcv_interval):
        with pytest.raises(ValueError):
            call([1e4, 1e4, 1e4], [0., 0., 0.], *params)


@pytest.mark.parametrize("bad", [-1., math.nan, math.inf, True, "1"])
def test_cpa_parameter_validation(bad):
    for which in range(3):
        params = [670.56, 137.16, 55.]
        params[which] = bad
        for call in (g.cpa_condition_scalar, g.cpa_condition):
            with pytest.raises(ValueError):
                call([0., 0., 0.], [0., 0., 0.], *params)


def test_zero_thresholds_and_broadcast_arrays_remain_supported():
    assert g.wcv_interval_scalar([0., 0., 0.], [0., 0., 0.], 0., 0., 0., 0.) == (0., 0.)
    s = np.array([[[0., 0., 0.]], [[1., 0., 0.]]])
    v = np.zeros((1, 3, 3))
    hit, _, _ = g.wcv_interval(s, v, np.array([0., 1., 2.]), 0., 0., 0.)
    assert hit.tolist() == [[True, True, True], [False, True, True]]
    with pytest.raises(ValueError):
        g.cpa(np.zeros((2, 3)), np.zeros((3, 3)))


@pytest.mark.parametrize("bad", [math.nan, math.inf, -1., True, "1"])
def test_cpa_tier_still_validates_ignored_tau(bad):
    tiers = [dict(D=670.56, Z=137.16, T=55., TTHR=bad)]
    for call in (al.alert_level, al.alert_level_scalar):
        with pytest.raises(ValueError):
            call("cpa", [0., 0., 0.], [0., 0., 0.], tiers)


def test_single_sample_trajectory_checks_initial_separation():
    rel = np.array([[[0., 0., 0.]], [[1000., 0., 0.]]])
    hit, onset = g.segment_violation(rel, 1., 670.56, 137.16)
    assert hit.tolist() == [True, False]
    assert onset[0] == 0. and np.isnan(onset[1])
    for bad in (0., -1., math.nan, math.inf, True, "1"):
        with pytest.raises(ValueError):
            g.segment_violation(rel, bad, 670.56, 137.16)
    with pytest.raises(ValueError):
        g.segment_violation(np.empty((2, 0, 3)), 1., 670.56, 137.16)


@pytest.mark.parametrize("strategy", ["", "WCV", "unknown"])
def test_alert_api_rejects_unknown_strategy(strategy):
    tiers = [dict(D=670.56, Z=137.16, TTHR=0., T=55.)]
    with pytest.raises(ValueError):
        al.condition(strategy, [0., 0., 0.], [0., 0., 0.], 1., 1., 1.)
    for call in (al.alert_level, al.alert_level_scalar):
        with pytest.raises(ValueError):
            call(strategy, [0., 0., 0.], [0., 0., 0.], tiers)


def test_cpp_wcv_high_precision_boundaries(cpp_cli):
    cases = [(s, v, D, tau) for s, v, D in BOUNDARIES for tau in (0., 20.)]
    lines = [" ".join(map(str, (*s, *v, D, 137.16, tau, 120.))) for s, v, D, tau in cases]
    result = subprocess.run([str(cpp_cli), "wcv"], input="\n".join(lines) + "\n", capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    actual = result.stdout.splitlines()
    assert len(actual) == len(cases)
    for line, (s, v, D, tau) in zip(actual, cases):
        fields = line.split()
        expected = decimal_horizontal(s, v, D, tau, 120.)
        assert (fields[0] == "1") == (expected is not None)
        if expected is not None:
            assert tuple(map(float, fields[1:])) == pytest.approx(expected, abs=1e-9, rel=1e-12)


@pytest.mark.parametrize("mode,row", [
    ("cpa", "nan 0 0 -50 0 0"), ("cpa", "inf 0 0 -50 0 0"),
    ("cpa", "100 0 0 -50 0"), ("cpa", "100 0 0 -50 0 0 garbage"),
    ("cpa", "100oops 0 0 -50 0 0"),
    ("wcv", "100 0 0 -50 0 0 -1 137.16 0 55"),
    ("wcv", "100 0 0 -50 0 0 670.56 137.16 nan 55"),
    ("wcv", "100 0 0 -50 0 0 670.56 137.16 0"),
])
def test_cpp_invalid_rows_fail_explicitly(cpp_cli, mode, row):
    result = subprocess.run([str(cpp_cli), mode], input=row + "\n", capture_output=True, text=True)
    assert result.returncode != 0
    assert not result.stdout.strip()
    assert result.stderr.strip()


@pytest.mark.parametrize("bad", ["nan", "-1", "not-a-number", "10oops"])
def test_cpp_invalid_alert_parameter_fails(cpp_cli, bad):
    args = [str(cpp_cli), "alert", "wcv", bad, "0", "213.36", "55", "137.16", "55", "137.16", "25"]
    result = subprocess.run(args, input="0 0 0 0 0 0\n", capture_output=True, text=True)
    assert result.returncode != 0
    assert not result.stdout.strip()
    assert result.stderr.strip()
