"""Mutation check: plant deliberate bugs one at a time and confirm the test
suite fails for each. A suite that never fails is not evidence.

    python scripts/mutation_check.py            # all mutants, writes results/mutation_report.json
    python scripts/mutation_check.py --only 3   # one mutant (0-based index)

Each mutant runs in a fresh copy of the repository (in a temp dir), so the
working tree is never modified. The unmutated copy must pass first.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# (file, original text, mutated text, description)
MUTANTS = [
    ("daa/geometry.py", "    tcpa = -sv / a if a > EPS else 0.0", "    tcpa = sv / a if a > EPS else 0.0", "scalar tcpa sign flipped"),
    ("daa/geometry.py", "    t = np.where(tcpa > 0.0, tcpa, 0.0)", "    t = tcpa", "vectorised CPA not clamped to t >= 0"),
    ("daa/geometry.py", "    t_out = (-sv + math.sqrt(disc_c)) / a", "    t_out = (-sv - math.sqrt(disc_c)) / a", "scalar circle exit uses wrong root"),
    ("daa/geometry.py", "        b = 2.0 * sv + TTHR * a", "        b = 2.0 * sv - TTHR * a", "vectorised tau_mod coefficient sign"),
    ("daa/geometry.py", "    t1 = (-Z - sz) / vz_safe", "    t1 = (-Z + sz) / vz_safe", "vectorised vertical interval bound"),
    ("daa/geometry.py", "    hi = np.minimum(np.minimum(h_hi, v_hi), T)", "    hi = np.minimum(h_hi, v_hi)", "lookahead time ignored"),
    ("daa/geometry.py", "    ahead = (tcpa > 0.0) & (tcpa <= T)", "    ahead = (tcpa > 0.0) & (tcpa <= 2 * T)", "CPA time threshold doubled (vectorised)"),
    ("daa/geometry.py", "    onset = onset_k.min(axis=1)", "    onset = onset_k.max(axis=1)", "loss onset takes last segment"),
    ("daa/alerting.py", "        level = np.where(hit, np.int8(i), level)", "        level = np.where(hit, np.int8(1), level)", "tier index lost"),
    ("daa/alerting.py", "            raise_ok = self.hist[i].sum(axis=1) >= self.M", "            raise_ok = self.hist[i].sum(axis=1) > self.M", "M-of-N off by one"),
    ("daa/alerting.py", "(self.since[i] < self.H)", "(self.since[i] <= self.H)", "hold time off by one"),
    ("daa/metrics.py", "    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den", "    half = z * math.sqrt(p * (1 - p) / n) / den", "Wilson half-width term dropped"),
    ("daa/metrics.py", "(lead < t_req)", "(lead <= t_req)", "required lead boundary"),
    ("daa/metrics.py", "    ok = start & (t[None, :] <= on[:, None])", "    ok = start", "alerts after onset counted"),
    ("daa/sim.py", "    om = turn_rate(V, perf[\"bank_max_deg\"]) * dt", "    om = 2.0 * turn_rate(V, perf[\"bank_max_deg\"]) * dt", "turn rate limit doubled"),
    ("daa/sim.py", "    vz_n = vz + np.clip(vz_cmd - vz, -perf[\"a_z\"] * dt, perf[\"a_z\"] * dt)", "    vz_n = vz_cmd", "vertical acceleration unbounded"),
    ("daa/sim.py", "        cost = np.abs(d) + 0.1 * (d > 0)       # prefer right turn", "        cost = np.abs(d) + 0.1 * (d < 0)       # prefer right turn", "left-turn preference"),
    ("daa/sim.py", "    feasible = (tgt_z >= sim[\"floor_m\"][\"value\"])", "    feasible = (tgt_z >= 0.0)", "altitude floor ignored"),
    ("daa/sim.py", "                pend_step[idx] = k + d", "                pend_step[idx] = k", "response delay ignored"),
    ("daa/sensors.py", "    beta = (lam * lam + 4.0 * lam - lam * r) / 4.0", "    beta = (lam * lam + 4.0 * lam - lam * r) / 2.0", "alpha-beta gain formula"),
    ("daa/sensors.py", "        state = np.where(state, u[:, j] >= p_bg, u[:, j] < p_gb)", "        state = np.where(state, u[:, j] < p_bg, u[:, j] < p_gb)", "dropout recovery inverted"),
    ("daa/encounters.py", "        s0_h = s_cpa_h - p[\"t_cpa0\"][:, None] * vrel[:, :2]", "        s0_h = s_cpa_h + p[\"t_cpa0\"][:, None] * vrel[:, :2]", "encounter back-propagation sign"),
    ("cpp/daa_core.hpp", "    const double t_out = (-sv + std::sqrt(disc_c)) / a;", "    const double t_out = (-sv + std::sqrt(disc_c)) / (a + 1e-9);", "C++ circle exit perturbed by 1e-9"),
    ("cpp/daa_core.hpp", "        if (hit) level = static_cast<int>(i) + 1;", "        if (hit) level = static_cast<int>(i);", "C++ tier off by one"),
    ("cpp/daa_core.hpp", "    return inside || (0.0 < c.tcpa && c.tcpa <= T && c.hmd <= Dh", "    return inside || (0.0 < c.tcpa && c.tcpa < T && c.hmd <= Dh", "C++ CPA time boundary < instead of <="),
]

IGNORE = shutil.ignore_patterns(".git", "results", "build", "docs", "__pycache__", ".pytest_cache")


def run_suite(tree, timeout=900):
    t = time.time()
    r = subprocess.run([sys.executable, "-m", "pytest", "-x", "-q", "-p", "no:cacheprovider", "tests"],
                       cwd=tree, capture_output=True, text=True, timeout=timeout)
    tail = (r.stdout.strip().splitlines() or [""])[-1]
    return r.returncode, tail, round(time.time() - t, 1)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, default=None)
    ap.add_argument("--out", default=str(ROOT / "results" / "mutation_report.json"))
    args = ap.parse_args(argv)
    sel = list(enumerate(MUTANTS)) if args.only is None else [(args.only, MUTANTS[args.only])]
    report = dict(mutants=[], baseline=None)
    with tempfile.TemporaryDirectory(prefix="daa_mut_") as tmp:
        base = Path(tmp) / "base"
        shutil.copytree(ROOT, base, ignore=IGNORE)
        code, tail, secs = run_suite(base)
        report["baseline"] = dict(returncode=code, summary=tail, seconds=secs)
        print(f"baseline: rc={code} {tail} ({secs}s)", flush=True)
        if code != 0:
            print("unmutated suite fails; aborting", file=sys.stderr)
            return 2
        for i, (rel, old, new, desc) in sel:
            tree = Path(tmp) / f"m{i}"
            shutil.copytree(base, tree, ignore=IGNORE)
            f = tree / rel
            src = f.read_text(encoding="utf-8")
            if src.count(old) != 1:
                print(f"[{i}] pattern not found exactly once in {rel}: {desc}", file=sys.stderr)
                return 3
            f.write_text(src.replace(old, new), encoding="utf-8")
            code, tail, secs = run_suite(tree)
            caught = code != 0
            report["mutants"].append(dict(index=i, file=rel, description=desc, caught=caught,
                                          pytest_summary=tail, seconds=secs))
            print(f"[{i:2d}] {'CAUGHT ' if caught else 'SURVIVED'} {rel}: {desc} -> {tail} ({secs}s)", flush=True)
            shutil.rmtree(tree, ignore_errors=True)
    n = len(report["mutants"])
    k = sum(m["caught"] for m in report["mutants"])
    report["caught"], report["total"] = k, n
    print(f"caught {k}/{n}")
    if args.only is None:
        Path(args.out).write_text(json.dumps(report, indent=1), encoding="utf-8")
    return 0 if k == n else 1


if __name__ == "__main__":
    sys.exit(main())
