"""Python and C++ implementations must agree on the same inputs.

Inputs are written with repr() (shortest round-trip form) and outputs are read
back from %.17g, so the comparison is on exact doubles. Alert levels and hit
flags must match exactly; times/distances are checked for exact equality and
the maximum absolute difference is also reported.
"""
import subprocess

import numpy as np
import pytest

from daa import alerting as al
from daa import geometry as g


def _cases(n=20000, seed=123):
    rng = np.random.default_rng(seed)
    s = np.column_stack([rng.uniform(-4000, 4000, n), rng.uniform(-4000, 4000, n), rng.uniform(-400, 400, n)])
    v = np.column_stack([rng.uniform(-120, 120, n), rng.uniform(-120, 120, n), rng.uniform(-10, 10, n)])
    # edge cases: zero relative velocity, zero vertical rate, exact boundaries, diverging, inside
    edge_s = [(1000, 0, 0), (670.56, 0, 0), (0, 670.56, 137.16), (100, 0, 0), (100, 0, 0),
              (1000, 300, 0), (1000, 0, 400), (2000, 0, 0), (1000, 0, 0), (0, 0, 0), (-500, 500, -137.16),
              (2750, 0, 0), (1250, 0, 0), (2750, 670.56, 137.16), (1000, 670.56, 0), (2750, 0, 213.36)]
    edge_v = [(-50, 0, 0), (-10, 0, 0), (0, -10, 0), (0, 0, 0), (10, 0, 0),
              (-50, 0, 0), (-50, 0, -10), (-5, 0, 0), (50, 0, 0), (0, 0, 0), (25, -25, 2),
              (-50, 0, 0), (-50, 0, 0), (-50, 0, 0), (-50, 0, 0), (-50, 0, 0)]
    s = np.vstack([np.array(edge_s, float), s])
    v = np.vstack([np.array(edge_v, float), v])
    s[30:230, 2] = 0.0
    v[230:430, 2] = 0.0
    v[430:530, :2] = 0.0
    return s, v


def _run(exe, args, lines):
    r = subprocess.run([str(exe)] + args, input="\n".join(lines) + "\n", capture_output=True,
                       text=True, check=True)
    return r.stdout.strip().splitlines()


def _vec(s, v):
    return " ".join(repr(float(x)) for x in (*s, *v))


def test_cpa_agrees(cpp_cli):
    s, v = _cases()
    out = _run(cpp_cli, ["cpa"], [_vec(a, b) for a, b in zip(s, v)])
    assert len(out) == len(s)
    maxdiff = 0.0
    for a, b, line in zip(s, v, out):
        c = [float(x) for x in line.split()]
        p = g.cpa_scalar(tuple(a), tuple(b))
        maxdiff = max(maxdiff, max(abs(x - y) for x, y in zip(c, p)))
    assert maxdiff == 0.0


@pytest.mark.parametrize("TTHR", [0.0, 20.0, 35.0])
def test_wcv_interval_agrees(cpp_cli, TTHR):
    s, v = _cases(seed=int(TTHR) + 1)
    rng = np.random.default_rng(int(TTHR))
    D = rng.uniform(150, 1500, len(s))
    Z = rng.uniform(30, 300, len(s))
    T = rng.uniform(0, 120, len(s))
    lines = [_vec(a, b) + f" {D[i]!r} {Z[i]!r} {TTHR!r} {T[i]!r}" for i, (a, b) in enumerate(zip(s, v))]
    out = _run(cpp_cli, ["wcv"], lines)
    assert len(out) == len(lines)
    hits = 0
    maxdiff = 0.0
    for i, line in enumerate(out):
        f = line.split()
        py = g.wcv_interval_scalar(tuple(s[i]), tuple(v[i]), D[i], Z[i], TTHR, T[i])
        assert (f[0] == "1") == (py is not None), i
        if py is not None:
            hits += 1
            maxdiff = max(maxdiff, abs(float(f[1]) - py[0]), abs(float(f[2]) - py[1]))
    assert hits > 1000  # the sample exercises the violation branch
    assert maxdiff == 0.0


@pytest.mark.parametrize("strategy", ["cpa", "wcv", "wcv_tau"])
def test_alert_levels_agree(cpp_cli, cfg, strategy):
    tiers = al.tier_params(cfg, strategy)
    s, v = _cases(seed=99)
    args = ["alert", "cpa" if strategy == "cpa" else "wcv", repr(tiers[0]["D"]), repr(float(tiers[0]["TTHR"] or 0.0))]
    for t in tiers:
        args += [repr(float(t["Z"])), repr(float(t["T"]))]
    out = [int(x) for x in _run(cpp_cli, args, [_vec(a, b) for a, b in zip(s, v)])]
    py_vec = al.alert_level(strategy, s, v, tiers)
    py_scalar = [al.alert_level_scalar(strategy, tuple(a), tuple(b), tiers) for a, b in zip(s, v)]
    assert out == py_scalar == [int(x) for x in py_vec]
    assert len(set(out)) == 4  # all four levels occur in the sample
    if strategy == "cpa":
        # exact boundaries (tcpa == T, hmd == Dh, |vmd| == Dv) are inclusive in both languages
        assert out[11:16] == [2, 3, 2, 3, 1]


def test_cli_rejects_bad_usage(cpp_cli):
    r = subprocess.run([str(cpp_cli), "alert", "xyz"], capture_output=True, text=True)
    assert r.returncode == 2
