"""Deterministic controller/encounter controls, not a new Monte-Carlo evaluation."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from daa import alerting, sim


def controlled_step(cfg, z, vz, target, dt=1.0):
    one = np.ones(1)
    return sim.step_dynamics(np.zeros(1), 60 * one, vz * one, z * one,
                             np.zeros(1), 60 * one, target * one,
                             cfg["performance"]["evtol"], dt, altitude_band=(150., 600.),
                             return_limited=True)


@pytest.mark.parametrize("start,target", [(190., 150.), (560., 600.)])
@pytest.mark.parametrize("dt", [.25, .5, 1., 2.])
def test_legal_boundary_targets_remain_inside_band_without_teleport_or_extra_acceleration(cfg, start, target, dt):
    perf = cfg["performance"]["evtol"]
    z, vz = start, 0.
    intervention = False
    for _ in range(int(80 / dt)):
        _, _, next_vz, next_z, velocity, limited = controlled_step(cfg, z, vz, target, dt)
        assert next_z[0] == pytest.approx(z + vz * dt, abs=1e-9)
        assert velocity[0, 2] == vz
        assert 150 - 1e-9 <= next_z[0] <= 600 + 1e-9
        assert abs(next_vz[0]) <= perf["vz_max"] + 1e-9
        assert abs(next_vz[0] - vz) <= perf["a_z"] * dt + 1e-9
        intervention |= limited[0]
        z, vz = float(next_z[0]), float(next_vz[0])
    assert intervention == (dt >= 1.)
    assert z == pytest.approx(target, abs=1e-7)


@pytest.mark.parametrize("sign", [-1., 1.])
def test_exact_discrete_braking_boundary_has_a_hand_calculated_trajectory(cfg, sign):
    # Rate 5, decrement 2.45: held steps travel 5 + 2.55 + .1 = 7.65 m.
    boundary = 150. if sign < 0 else 600.
    z, vz = boundary - sign * 7.65, sign * 5.
    expected = [(boundary - sign * 2.65, sign * 2.55),
                (boundary - sign * .1, sign * .1), (boundary, 0.)]
    for expected_z, expected_vz in expected:
        _, _, next_vz, next_z, _, _ = controlled_step(cfg, z, vz, boundary)
        assert next_z[0] == pytest.approx(expected_z, abs=1e-9)
        assert next_vz[0] == pytest.approx(expected_vz, abs=1e-9)
        z, vz = float(next_z[0]), float(next_vz[0])


@pytest.mark.parametrize("z", [150., 600.])
def test_stationary_boundary_does_not_create_a_vertical_manoeuvre(cfg, z):
    _, _, rate, height, _, limited = controlled_step(cfg, z, 0., z)
    assert rate[0] == 0 and height[0] == z and not limited[0]


@pytest.mark.parametrize("z,vz", [(155., -5.), (595., 5.), (149., 0.), (601., 0.)])
def test_impossible_initial_or_held_velocity_state_is_rejected_instead_of_clipped(cfg, z, vz):
    with pytest.raises(ValueError, match="altitude state"):
        controlled_step(cfg, z, vz, 300.)


@pytest.mark.parametrize("dt", [0., -1., np.inf, np.nan])
def test_invalid_bounded_step_time_is_rejected(cfg, dt):
    with pytest.raises(ValueError, match="finite|positive|altitude"):
        controlled_step(cfg, 300., 0., 300., dt)


def make_plan(cfg, z=190., intruder_z=315., delay=5, include_current=False, current_target=None, x=4200.):
    one, zero = np.ones(1), np.zeros(1)
    return sim.plan("vertical", cfg, zero, 60 * one, zero, z * one, z * one,
                    np.array([[0., 0., z]]),
                    [zero, 60 * one, (z if current_target is None else current_target) * one],
                    np.array([[x, 0., intruder_z]]), np.array([[-60., 0., 0.]]), delay,
                    include_current=include_current)


@pytest.mark.parametrize("z,intruder_z,target", [(190., 315., 150.), (560., 435., 600.)])
@pytest.mark.parametrize("delay", [0, 5, 10])
def test_actual_planner_uses_the_bounded_controller_and_reports_its_intervention(cfg, z, intruder_z, target, delay):
    choice = make_plan(cfg, z, intruder_z, delay)
    assert choice["z_tgt"][0] == target and choice["free"][0]
    assert choice["limited"][0]


def test_recheck_cannot_keep_an_out_of_band_target_merely_because_separation_is_clear(cfg):
    choice = make_plan(cfg, z=300., intruder_z=300., current_target=100.,
                       include_current=True, x=50000.)
    assert not choice["keep"][0]
    assert 150 <= choice["z_tgt"][0] <= 600
    normal = make_plan(cfg, z=300., intruder_z=300., include_current=True, x=50000.)
    assert normal["keep"][0] and not normal["limited"][0]


def test_a_target_outside_the_band_is_not_returned_as_a_clear_candidate(cfg):
    # A 40m descent from 151m is not an eligible command even if its safety
    # controller would saturate at 150m and create slightly more separation.
    choice = make_plan(cfg, z=151., intruder_z=315.)
    assert 150 <= choice["z_tgt"][0] <= 600
    assert not choice["free"][0]


@pytest.mark.parametrize("z", [150. - 1e-12, 600. + 1e-12])
def test_sub_nanometre_roundoff_at_a_stationary_boundary_is_not_an_infeasible_plan(cfg, z):
    choice = make_plan(cfg, z=z, intruder_z=300., x=50000.)
    assert abs(choice["dz"][0]) < 1e-9
    assert choice["free"][0]


@pytest.mark.parametrize("delay", [0, 5, 10])
def test_delayed_plan_does_not_hide_an_infeasible_current_braking_state(cfg, delay):
    one, zero = np.ones(1), np.zeros(1)
    with pytest.raises(ValueError, match="altitude state"):
        sim.plan("heading", cfg, zero, 60 * one, -5 * one, 155 * one, 155 * one,
                 np.array([[0., 0., 155.]]), [zero, 60 * one, 300 * one],
                 np.array([[50000., 0., 300.]]), np.zeros((1, 3)), delay)


def encounter_control(z, intruder_z, dt=1.0, gust=0.):
    t = np.arange(int(100 / dt) + 1) * dt
    own = np.stack([60 * t, np.zeros_like(t), np.full_like(t, z)], -1)[None, ...]
    intruder = np.stack([7200 - 60 * t, np.zeros_like(t), np.full_like(t, intruder_z)], -1)[None, ...]
    track = SimpleNamespace(pos=intruder, vel=np.broadcast_to([-60., 0., 0.], intruder.shape).copy())
    enc = SimpleNamespace(dt=dt, int_pos=intruder, own_psi=np.zeros(1), own_speed=np.array([60.]),
                          own_p0=own[:, 0, :], own_w=np.broadcast_to([0., 0., gust], (1, len(t)-1, 3)).copy())
    return enc, track, own


@pytest.mark.parametrize("z,intruder_z", [(190., 315.), (560., 435.)])
@pytest.mark.parametrize("dt,delay", [(1., 0.), (1., 5.), (.5, 5.), (1., 10.)])
def test_real_closed_loop_respects_band_rate_acceleration_and_response_delay(cfg, z, intruder_z, dt, delay):
    cfg = deepcopy(cfg)
    cfg["simulation"]["dt"] = dt
    enc, track, own = encounter_control(z, intruder_z, dt)
    result = sim.closed_loop(enc, track, "wcv", alerting.tier_params(cfg, "wcv"), "vertical", cfg,
                             response_delay=delay)
    heights = result.own_pos[0, :, 2]
    assert heights.min() >= 150 - 1e-9 and heights.max() <= 600 + 1e-9
    rates = np.diff(heights) / dt
    assert np.abs(rates).max() <= cfg["performance"]["evtol"]["vz_max"] + 1e-9
    assert np.abs(np.diff(rates)).max() <= cfg["performance"]["evtol"]["a_z"] * dt + 1e-9
    assert np.array_equal(result.own_pos[:, :int(delay/dt)+2, :], own[:, :int(delay/dt)+2, :])
    assert result.floor_limited[0] == (dt >= 1.)


def test_truth_gust_is_not_clipped_to_manufacture_altitude_compliance(cfg):
    enc, track, _ = encounter_control(300., 315., gust=10.)
    result = sim.closed_loop(enc, track, "wcv", alerting.tier_params(cfg, "wcv"), "none", cfg)
    # None has commanded altitude 300; the unbounded disturbance remains in truth.
    assert result.own_pos[0, -1, 2] == 1300.
    assert result.max_dz_m[0] == 0
