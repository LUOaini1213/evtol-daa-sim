"""Independent regressions for causal tracking, policy and paired-data contracts."""
from copy import deepcopy
import hashlib
import importlib.util
from pathlib import Path
import os
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from daa import alerting, encounters, metrics, sensors, sim


def track_input(future_dx=10., intruder=1, samples=3):
    pos = np.array([[[1000., 0., 300.], [1000.+future_dx, 0., 300.], [1000., 0., 300.]]])[:, :samples]
    return SimpleNamespace(int_pos=pos, int_vel=np.zeros_like(pos), intruder=np.array([intruder]), dt=1.,
                           own_pos=np.broadcast_to([0., 0., 300.], pos.shape), own_vel=np.zeros_like(pos))


@pytest.mark.parametrize("intruder", [0, 1, 2])
def test_track_and_alert_prefix_do_not_read_future(cfg, intruder):
    a, b = track_input(10., intruder), track_input(-10., intruder)
    ta = sensors.make_track(a, cfg, "nominal", 7)
    tb = sensors.make_track(b, cfg, "nominal", 7)
    np.testing.assert_array_equal(ta.pos[:, 0], tb.pos[:, 0])
    np.testing.assert_array_equal(ta.vel[:, 0], tb.vel[:, 0])
    tiers = alerting.tier_params(cfg, "wcv")
    assert sim.open_loop_levels(a, ta, "wcv", tiers)[0, 0] == sim.open_loop_levels(b, tb, "wcv", tiers)[0, 0]


@pytest.mark.parametrize("samples", [1, 3])
def test_noncooperative_initialization_is_causal(cfg, samples):
    track = sensors.make_track(track_input(samples=samples), cfg, "nominal", 7)
    np.testing.assert_array_equal(track.vel[:, 0], np.zeros((1, 3)))
    if samples > 1:
        # The two-point estimate becomes available at the second report, not at t=0.
        np.testing.assert_allclose(track.vel[:, 1], track.pos[:, 1] - track.pos[:, 0])
        assert np.linalg.norm(track.vel[:, 1]) > 0


def test_ideal_track_still_exposes_truth(cfg):
    enc = track_input()
    enc.int_vel[:] = [3., 4., 5.]
    track = sensors.make_track(enc, cfg, "ideal", 7)
    np.testing.assert_array_equal(track.pos, enc.int_pos)
    np.testing.assert_array_equal(track.vel, enc.int_vel)


def _score_candidate(cfg, x, heading, dz, slow):
    """Scalar reference Euler rollout; does not call the production planner/dynamics."""
    perf = cfg["performance"]["evtol"]
    pos = np.array([0., 0., 300.])
    psi, speed, vz, z = 0., 60., 0., 300.
    minimum = float("inf")
    for i in range(90):
        velocity = np.array([speed*np.cos(psi), speed*np.sin(psi), vz])
        pc, vc, zc = (0., 60., 300.) if i < 5 else (
            np.radians(heading), perf["v_min_emergency"] if slow else 60., 300.+dz)
        delta = (pc-psi+np.pi) % (2*np.pi) - np.pi
        omega = 9.80665*np.tan(np.radians(30.))/max(speed, 1.)
        psi += np.clip(delta, -omega, omega)
        speed += np.clip(vc-speed, -2., 2.)
        vz_next = vz + np.clip(np.clip(.5*(zc-z), -5., 5.)-vz, -2.45, 2.45)
        z += vz
        vz = vz_next
        pos += velocity
        if i+1 >= 5:
            rel = np.array([x-60.*(i+1), 0., 300.]) - pos
            minimum = min(minimum, max(np.hypot(*rel[:2])/(1.2*670.56), abs(rel[2])/(1.2*137.16)))
    return minimum


def _plan(cfg, kind, x=3000., include_current=False):
    return sim.plan(kind, cfg, np.zeros(1), np.full(1, 60.), np.zeros(1), np.full(1, 300.),
                    np.full(1, 300.), np.array([[0., 0., 300.]]),
                    [np.zeros(1), np.full(1, 60.), np.full(1, 300.)],
                    np.array([[x, 0., 300.]]), np.array([[-60., 0., 0.]]), 5,
                    include_current=include_current)


@pytest.mark.parametrize("x", [2500., 3000.])
def test_emergency_maximizes_feasible_predicted_separation(cfg, x):
    assert alerting.alert_level("wcv", np.array([[x, 0., 0.]]), np.array([[-120., 0., 0.]]),
                                alerting.tier_params(cfg, "wcv"))[0] == 3
    scores = [_score_candidate(cfg, x, h, dz, slow)
              for h in (0., -30., 30., -60., 60., -90., 90.)
              for dz in (0., 100., -100., 200.) for slow in (False, True)]
    selected = _plan(cfg, "emergency", x)
    actual = _score_candidate(cfg, x, np.degrees(selected["dpsi"][0]), selected["dz"][0], selected["slow"][0])
    assert selected["mmin"][0] == pytest.approx(actual, abs=1e-12)
    assert actual == pytest.approx(max(scores), abs=1e-12)
    # Equally protective choices use the existing low-cost/right-turn tie rule.
    assert np.degrees(selected["dpsi"][0]) == pytest.approx(-90.)
    assert selected["dz"][0] == 0 and not selected["slow"][0]


def test_corrective_cost_preference_and_clear_replan_remain(cfg):
    ordinary = _plan(cfg, "heading", 5000.)
    assert ordinary["free"][0] and abs(np.degrees(ordinary["dpsi"][0])) < 90.
    # Current route already clear; emergency recheck must not chase gratuitous separation.
    assert _plan(cfg, "emergency", 20000., include_current=True)["keep"][0]


@pytest.mark.parametrize("function", [metrics.mcnemar_exact, metrics.paired_bootstrap_diff, metrics.paired_bootstrap_ratio])
@pytest.mark.parametrize("x,y", [([1., 1.], [0., 0., 100.]), ([[1.], [0.]], [0., 0.]),
                                ([], []), ([np.nan], [0.]), ([1.], [np.inf])])
def test_paired_statistics_reject_invalid_pair_arrays(function, x, y):
    extra = () if function is metrics.mcnemar_exact else (20, 0)
    with pytest.raises(ValueError, match="paired|one-dimensional|equal|nonempty|finite"):
        function(x, y, *extra)


@pytest.mark.parametrize("function", [metrics.paired_bootstrap_diff, metrics.paired_bootstrap_ratio])
@pytest.mark.parametrize("n_boot", [0, -1, True, 1.5])
def test_bootstrap_requires_positive_integer_draw_count(function, n_boot):
    with pytest.raises(ValueError, match="n_boot"):
        function([1., 1.], [1., 1.], n_boot, 0)


def test_valid_paired_statistics_have_independent_answers():
    assert metrics.paired_bootstrap_diff([1., 1.], [0., 0.], 20, 0) == {"diff": 1., "lo": 1., "hi": 1.}
    assert metrics.paired_bootstrap_ratio([1., 1.], [2., 2.], 20, 0) == {"ratio": .5, "lo": .5, "hi": .5}
    result = metrics.mcnemar_exact([True]*6+[False]*4, [False]*10)
    assert result == {"b": 6, "c": 0, "p": 2*.5**6}


def test_generator_checks_nominal_altitude_through_full_horizon(cfg):
    enc = encounters.generate(cfg, n=3, seed=7)
    nominal_end = enc.int_pos[:, 0, 2] + enc.params["vz_i"]*enc.t[-1]
    assert np.all(np.minimum(enc.int_pos[:, 0, 2], nominal_end) >= 30.)
    assert np.all(np.maximum(enc.int_pos[:, 0, 2], nominal_end) <= 1200.)
    assert enc.t[-1] == 200.


def test_generator_does_not_clip_stochastic_gusts(cfg):
    # The acceptance contract is nominal; the OU process remains an unbounded disturbance.
    noisy = deepcopy(cfg)
    for name in encounters.INTRUDERS:
        noisy["performance"][name]["gust_sigma_z"] = 1000.
    enc = encounters.generate(noisy, n=3, seed=7)
    assert np.any((enc.int_pos[..., 2] < 30.) | (enc.int_pos[..., 2] > 1200.))


@pytest.mark.parametrize("bad", [0, -1, True, False, 1.5, 2., np.nan, np.inf, "3"])
def test_generator_rejects_invalid_count_before_sampling(cfg, monkeypatch, bad):
    def forbidden_sampling(*args):
        pytest.fail("invalid n reached encounter generation")
    monkeypatch.setattr(encounters, "_sample_params", forbidden_sampling)
    with pytest.raises(ValueError, match="positive integer"):
        encounters.generate(cfg, n=bad, seed=7)


@pytest.mark.parametrize("bad", [0, -1, True, 2.5])
def test_generator_validates_configured_default_count(cfg, monkeypatch, bad):
    config = deepcopy(cfg)
    config["encounter"]["n_encounters"] = bad
    monkeypatch.setattr(encounters, "_sample_params", lambda *a: pytest.fail("invalid default reached sampling"))
    with pytest.raises(ValueError, match="positive integer"):
        encounters.generate(config, n=None, seed=7)


def test_generator_none_uses_valid_configured_count(cfg):
    config = deepcopy(cfg)
    config["encounter"]["n_encounters"] = 3
    assert encounters.generate(config, n=None, seed=7).n == 3
    assert encounters.generate(config, n=np.int64(1), seed=7).n == 1


@pytest.mark.parametrize("arg", ["0", "-1", "1.5"])
def test_evaluation_cli_rejects_bad_count_without_creating_outputs(tmp_path, arg):
    script = Path(__file__).resolve().parents[1] / "scripts/run_eval.py"
    output, docs = tmp_path / "results", tmp_path / "docs"
    result = subprocess.run([sys.executable, "-B", str(script), "--n", arg,
                             "--out", str(output), "--docs", str(docs)], cwd=tmp_path,
                            env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONIOENCODING="utf-8"),
                            capture_output=True, text=True, encoding="utf-8", timeout=30)
    assert result.returncode == 2 and "error:" in result.stderr
    assert not output.exists() and not docs.exists()


def _load_evaluator():
    path = Path(__file__).resolve().parents[1] / "scripts/run_eval.py"
    spec = importlib.util.spec_from_file_location("_behavior_run_eval", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_runtime_hashes_cover_exact_uncommitted_sources_and_detect_drift(tmp_path, monkeypatch):
    module = _load_evaluator()
    files = {"daa/z.py": b"z=1\r\n", "daa/__init__.py": b"", "config/default.json": b"{}",
             "scripts/run_eval.py": b"print('evaluation')\n"}
    for rel, contents in files.items():
        path = tmp_path / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(contents)
    monkeypatch.setattr(module, "ROOT", tmp_path)
    snapshot = module.runtime_source_sha256()
    assert list(snapshot) == sorted(files)
    assert snapshot == {p: hashlib.sha256(contents).hexdigest() for p, contents in files.items()}
    module.verify_runtime_sources(snapshot)
    (tmp_path / "daa/z.py").write_bytes(b"z=2\r\n")
    with pytest.raises(RuntimeError, match="no results published"):
        module.verify_runtime_sources(snapshot)
    (tmp_path / "daa/z.py").write_bytes(files["daa/z.py"])
    (tmp_path / "daa/new.py").write_bytes(b"new=1\n")
    with pytest.raises(RuntimeError, match="no results published"):
        module.verify_runtime_sources(snapshot)


def test_evaluator_rejects_invalid_configured_count_before_mkdir(cfg, tmp_path, monkeypatch):
    module = _load_evaluator()
    config = deepcopy(cfg)
    config["encounter"]["n_encounters"] = False
    monkeypatch.setattr(module, "load_config", lambda: config)
    output = tmp_path / "output"
    with pytest.raises(SystemExit) as exc:
        module.main(["--out", str(output), "--docs", str(tmp_path / "docs")])
    assert exc.value.code == 2
    assert not output.exists() and not (tmp_path / "docs").exists()
