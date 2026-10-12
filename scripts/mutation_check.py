"""Mutation check: plant deliberate bugs one at a time and confirm the test
suite fails for each. A suite that never fails is not evidence.

    python scripts/mutation_check.py            # all mutants, writes results/mutation_report.json
    python scripts/mutation_check.py --only 3   # one mutant (0-based index)

Each mutant runs in a fresh copy of the repository (in a temp dir), so the
working tree is never modified. The unmutated copy must pass first.
JUnit and full build/test logs survive beside --out in *_artifacts/run_*.
Without --out, --only writes mutation_report.only-N.json, preserving the full report.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# (file, original text, mutated text, description)
MUTANTS = [
    ("daa/geometry.py", "    tcpa = -sv / a if a > EPS else 0.0", "    tcpa = sv / a if a > EPS else 0.0", "scalar tcpa sign flipped"),
    ("daa/geometry.py", "    t = np.where(tcpa > 0.0, tcpa, 0.0)", "    t = tcpa", "vectorised CPA not clamped to t >= 0"),
    ("daa/geometry.py", "    t_out = tcpa + math.sqrt(half_width2)", "    t_out = tcpa - math.sqrt(half_width2)", "scalar circle exit uses wrong root"),
    ("daa/geometry.py", "    t_in = tcpa - half_tau - np.sqrt(half_tau * half_tau + half_width2)", "    t_in = tcpa + half_tau - np.sqrt(half_tau * half_tau + half_width2)", "vectorised tau_mod entry sign"),
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
    ("daa/sim.py", "tgt_z >= sim[\"floor_m\"][\"value\"] - ALTITUDE_EPS", "tgt_z >= 0.0", "altitude target floor ignored"),
    ("daa/sim.py", "                pend_step[idx] = k + d", "                pend_step[idx] = k", "response delay ignored"),
    ("daa/sensors.py", "    beta = (lam * lam + 4.0 * lam - lam * r) / 4.0", "    beta = (lam * lam + 4.0 * lam - lam * r) / 2.0", "alpha-beta gain formula"),
    ("daa/sensors.py", "        state = np.where(state, u[:, j] >= p_bg, u[:, j] < p_gb)", "        state = np.where(state, u[:, j] < p_bg, u[:, j] < p_gb)", "dropout recovery inverted"),
    ("daa/encounters.py", "        s0_h = s_cpa_h - p[\"t_cpa0\"][:, None] * vrel[:, :2]", "        s0_h = s_cpa_h + p[\"t_cpa0\"][:, None] * vrel[:, :2]", "encounter back-propagation sign"),
    ("cpp/daa_core.hpp", "    const double t_out = tcpa + std::sqrt(half_width2);", "    const double t_out = tcpa + std::sqrt(half_width2) + 1e-9;", "C++ circle exit perturbed by 1e-9"),
    ("cpp/daa_core.hpp", "        if (hit) level = static_cast<int>(i) + 1;", "        if (hit) level = static_cast<int>(i);", "C++ tier off by one"),
    ("cpp/daa_core.hpp", "    return inside || (0.0 < c.tcpa && c.tcpa <= T && c.hmd <= Dh", "    return inside || (0.0 < c.tcpa && c.tcpa < T && c.hmd <= Dh", "C++ CPA time boundary < instead of <="),
    ("daa/sensors.py", "    vel[:, 0] = np.where(coop[:, None], meas_v[:, 0], 0.0)", "    vel[:, 0] = np.where(coop[:, None], meas_v[:, 0], (meas_p[:, 1] - meas_p[:, 0]) / dt)", "noncooperative initial velocity leaks future report"),
    ("daa/sim.py", '    objective = dict(maximize=kind == "emergency", keep_current=include_current)', "    objective = dict(maximize=False, keep_current=include_current)", "emergency chooses cheapest clear instead of maximum separation"),
    ("daa/encounters.py", '        z_end = z_i0 + p["vz_i"] * horizon', '        z_end = z_i0 + p["vz_i"] * np.minimum(p["t_cpa0"] + 30.0, horizon)', "generator altitude check stops before full horizon"),
    ("daa/metrics.py", "    if x.size == 0 or x.size != y.size:", "    if x.size == 0:", "paired metrics accept unequal input lengths"),
]

IGNORE = shutil.ignore_patterns(".git", "results", "build", "docs", "__pycache__", ".pytest_cache", ".venv", "venv")


def _run_command(command, *, cwd, env, timeout):
    """Own a process group/tree and reap it before returning a timeout."""
    isolation = ({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW}
                 if os.name == "nt" else {"start_new_session": True})
    process = subprocess.Popen(command, cwd=cwd, env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True, encoding="utf-8",
                               errors="replace", **isolation)
    evidence = dict(pid=process.pid, timed_out=False)
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        evidence["timed_out"] = True
        if os.name == "nt":
            # PID comes only from the Popen above; never kill by image/name.
            killed = subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                                    capture_output=True, text=True, encoding="oem",
                                    errors="replace", timeout=30,
                                    creationflags=subprocess.CREATE_NO_WINDOW)
            evidence.update(tree_kill_returncode=killed.returncode,
                            tree_kill_output=killed.stdout + killed.stderr)
        else:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            evidence["tree_kill_returncode"] = 0
        stdout, stderr = process.communicate(timeout=30)
        evidence["returncode"] = process.returncode
        exc = subprocess.TimeoutExpired(command, timeout, output=stdout, stderr=stderr)
        exc.process_evidence = evidence
        raise exc
    evidence["returncode"] = process.returncode
    completed = subprocess.CompletedProcess(command, process.returncode, stdout, stderr)
    completed.process_evidence = evidence
    return completed


def _classify_junit(path, returncode):
    """Pytest JUnit uses failure for call failures and error for collection/setup/teardown."""
    result = dict(status="invalid", tests=0, failures=0, errors=0, skipped=0,
                  failure_cases=[], reason="missing or malformed JUnit")
    try:
        root = ET.parse(path).getroot()
        suites = list(root.iter("testsuite"))
        cases = list(root.iter("testcase"))
        if not suites or not cases:
            return result
        for suite in suites:
            direct = suite.findall("testcase")
            # The pytest writer emits leaf suites with exact testcase totals.
            if not suite.findall("testsuite"):
                actual = (len(direct), sum(c.find("failure") is not None for c in direct),
                          sum(c.find("error") is not None for c in direct),
                          sum(c.find("skipped") is not None for c in direct))
                declared = tuple(int(suite.attrib[k]) for k in ("tests", "failures", "errors", "skipped"))
                if actual != declared:
                    result["reason"] = "inconsistent JUnit counts"
                    return result
        result.update(tests=len(cases),
                      failures=sum(c.find("failure") is not None for c in cases),
                      errors=sum(c.find("error") is not None for c in cases),
                      skipped=sum(c.find("skipped") is not None for c in cases))
        result["failure_cases"] = [
            dict(name=c.get("classname", "") + "::" + c.get("name", ""),
                 phase="call", message=c.find("failure").get("message", ""))
            for c in cases if c.find("failure") is not None]
    except (OSError, ET.ParseError, KeyError, ValueError):
        return result
    if result["errors"] or result["skipped"]:
        result["reason"] = "JUnit contains errors or skipped tests"
    elif returncode == 0 and result["failures"] == 0:
        result.update(status="passed", reason="all collected tests passed")
    elif returncode == 1 and result["failures"] > 0:
        result.update(status="failed", reason="test call failure")
    else:
        result["reason"] = "pytest exit code disagrees with test outcomes"
    return result


def run_suite(tree, artifact_dir, timeout=900):
    """Keep full evidence outside the disposable tree; never count infrastructure as a kill."""
    tree, artifact_dir = Path(tree).resolve(), Path(artifact_dir).resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    junit, log = artifact_dir / "pytest.xml", artifact_dir / "pytest.log"
    junit.unlink(missing_ok=True)  # a failed new invocation cannot reuse a stale report
    env = os.environ.copy()
    env.pop("PYTEST_ADDOPTS", None)
    env.update(DAA_REQUIRE_CPP="1", PYTHONPATH=str(tree), PYTHONDONTWRITEBYTECODE="1",
               PYTHONIOENCODING="utf-8")
    command = [sys.executable, "-B", "-m", "pytest", "-x", "-q", "-p", "no:cacheprovider",
               "--junitxml", str(junit), "-o", "junit_family=xunit2", "tests"]
    start, returncode, output, process_error = time.monotonic(), None, "", None
    timeout_process = None
    build_log, build_returncode = None, None
    if (tree / "cpp" / "build.py").is_file():
        # Some tests build C++ in their call body, which JUnit would label a failure.
        # Independently prove compilation first so that such errors cannot become kills.
        build_log = artifact_dir / "cpp-build.log"
        build_output, build_error = "", None
        try:
            built = _run_command(
                [sys.executable, "-B", str(tree / "cpp" / "build.py"), str(artifact_dir / "cpp-build")],
                cwd=tree, env=env, timeout=min(timeout, 120))
            build_returncode = built.returncode
            build_output = built.stdout + "\n--- stderr ---\n" + built.stderr
            if build_returncode != 0:
                build_error = f"C++ compile preflight failed (exit {build_returncode})"
        except (OSError, subprocess.TimeoutExpired) as exc:
            build_error = f"C++ compile preflight failed: {exc}"
            timeout_process = getattr(exc, "process_evidence", None)
            if isinstance(exc, subprocess.TimeoutExpired):
                for part in (exc.stdout, exc.stderr):
                    build_output += part.decode("utf-8", errors="replace") if isinstance(part, bytes) else part or ""
        build_log.write_text(build_output + f"\n{build_error or 'compile succeeded'}\n"
                             + json.dumps(timeout_process) + "\n", encoding="utf-8")
        if build_error:
            log.write_text(f"pytest not started: {build_error}\n", encoding="utf-8")
            result = _classify_junit(junit, None)
            result.update(reason=build_error, summary=build_error, returncode=None,
                          seconds=round(time.monotonic() - start, 2), junit=str(junit), log=str(log),
                          build_log=str(build_log), build_returncode=build_returncode,
                          timeout_process=timeout_process)
            return result
    try:
        process = _run_command(command, cwd=tree, env=env, timeout=timeout)
        returncode = process.returncode
        output = process.stdout + "\n--- stderr ---\n" + process.stderr
    except subprocess.TimeoutExpired as exc:
        def decoded(value):
            return value.decode("utf-8", errors="replace") if isinstance(value, bytes) else value or ""
        output = decoded(exc.stdout) + "\n--- stderr ---\n" + decoded(exc.stderr)
        process_error = f"pytest timeout after {timeout}s"
        timeout_process = getattr(exc, "process_evidence", None)
    except OSError as exc:
        process_error = f"cannot start pytest: {exc}"
    log.write_text(output + f"\nreturncode={returncode}; {process_error or ''}\n"
                   + json.dumps(timeout_process) + "\n", encoding="utf-8")
    result = _classify_junit(junit, returncode)
    if process_error:
        result.update(status="invalid", reason=process_error)
    summary = next((line for line in reversed(output.splitlines())
                    if " passed" in line or " failed" in line or " error" in line), result["reason"])
    result.update(returncode=returncode, summary=summary, seconds=round(time.monotonic() - start, 2),
                  junit=str(junit), log=str(log), build_log=str(build_log) if build_log else None,
                  build_returncode=build_returncode, timeout_process=timeout_process)
    return result


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, default=None)
    ap.add_argument("--out", default=None, help="report JSON; evidence is kept in a sibling *_artifacts directory")
    args = ap.parse_args(argv)
    if args.only is not None and not 0 <= args.only < len(MUTANTS):
        ap.error(f"--only must be between 0 and {len(MUTANTS) - 1}")
    sel = list(enumerate(MUTANTS)) if args.only is None else [(args.only, MUTANTS[args.only])]
    name = "mutation_report.json" if args.only is None else f"mutation_report.only-{args.only}.json"
    out = Path(args.out).resolve() if args.out else ROOT / "results" / name
    artifact_root = out.parent / (out.stem + "_artifacts")
    artifact_root.mkdir(parents=True, exist_ok=True)
    artifacts = Path(tempfile.mkdtemp(prefix="run_", dir=artifact_root))
    report = dict(mutants=[], baseline=None, selected_indices=[i for i, _ in sel],
                  artifacts_dir=str(artifacts), cpp_required=True)

    def ignore(directory, names):
        excluded = {out.resolve(), artifact_root.resolve()}
        return set(IGNORE(directory, names)) | {
            name for name in names if (Path(directory) / name).resolve() in excluded}

    def finish():
        report["total"] = len(report["mutants"])
        for key in ("caught", "survived", "invalid"):
            report[key] = sum(m["status"] == key for m in report["mutants"])
        complete = (report["baseline"] is not None and report["baseline"]["status"] == "passed"
                    and report["total"] == len(sel) and report["invalid"] == 0)
        report["status"] = "complete" if complete else "invalid"
        report["complete_full_matrix"] = complete and args.only is None
        out.write_text(json.dumps(report, indent=1), encoding="utf-8")
        if not complete:
            return 2
        return 0 if report["caught"] == len(sel) else 1

    with tempfile.TemporaryDirectory(prefix="daa_mut_") as tmp:
        base = Path(tmp) / "base"
        shutil.copytree(ROOT, base, ignore=ignore)
        baseline = run_suite(base, artifacts / "baseline")
        report["baseline"] = baseline
        print(f"baseline: {baseline['status']} {baseline['summary']}", flush=True)
        if baseline["status"] != "passed":
            print("unmutated suite fails; aborting", file=sys.stderr)
            return finish()
        for i, (rel, old, new, desc) in sel:
            tree = Path(tmp) / f"m{i}"
            evidence = artifacts / f"mutant_{i:03d}"
            evidence.mkdir()
            shutil.copytree(base, tree, ignore=ignore)
            f = tree / rel
            try:
                if not f.resolve().is_relative_to(tree.resolve()):
                    raise ValueError("mutant target is outside the disposable tree")
                src = f.read_text(encoding="utf-8")
                if src.count(old) != 1:
                    raise ValueError(f"pattern not found exactly once in {rel}")
                mutated = src.replace(old, new)
                if f.suffix == ".py":
                    compile(mutated, str(f), "exec")
                f.write_text(mutated, encoding="utf-8")
                result = run_suite(tree, evidence)
            except (OSError, ValueError, SyntaxError) as exc:
                log = evidence / "preflight.log"
                log.write_text(f"{type(exc).__name__}: {exc}\n", encoding="utf-8")
                result = dict(status="invalid", reason=str(exc), summary=str(exc),
                              seconds=0, returncode=None, log=str(log), junit=None)
            status = {"passed": "survived", "failed": "caught", "invalid": "invalid"}[result["status"]]
            report["mutants"].append(dict(result, index=i, file=rel, description=desc,
                                          status=status, caught=status == "caught",
                                          pytest_summary=result["summary"]))
            print(f"[{i:2d}] {status.upper()} {rel}: {desc} -> {result['summary']}", flush=True)
            target, temporary_root = tree.resolve(), Path(tmp).resolve()
            if target == temporary_root or not target.is_relative_to(temporary_root):
                raise ValueError("refusing to remove a path outside the temporary mutant directory")
            shutil.rmtree(target, ignore_errors=True)
            if status == "invalid":
                return finish()
    code = finish()
    print(f"caught {report['caught']}/{report['total']}; survived {report['survived']}; invalid {report['invalid']}")
    return code


if __name__ == "__main__":
    sys.exit(main())
