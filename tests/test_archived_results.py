"""Controlled Git/ZIP corruption cases; the real archive is audited separately in CI."""
import importlib.util
import json
from pathlib import Path
import subprocess
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("archive_checker", ROOT / "scripts/check_archived_results.py")
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


@pytest.fixture
def frozen(tmp_path):
    evaluation = ["config/default.json", *["daa/" + name + ".py" for name in
                  ("__init__", "alerting", "encounters", "geometry", "metrics", "sensors", "sim")],
                  "scripts/run_eval.py"]
    mutation = sorted(set(evaluation) | {"cpp/build.py", "cpp/daa_cli.cpp", "cpp/daa_core.hpp",
               "pytest.ini", "requirements.txt", "scripts/make_readme.py", "scripts/mutation_check.py"}
               | {"tests/" + name + ".py" for name in ("conftest", "test_alerting", "test_behavior_review",
                  "test_build", "test_cpp_agreement", "test_geometry", "test_geometry_review",
                  "test_manoeuvres", "test_metrics_sensors", "test_mutation_runner")})
    for name in mutation:
        target = tmp_path / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(b"FIXTURE_SOURCE = 1\n")
    subprocess.run(["git", "init", "--quiet", str(tmp_path)], check=True, capture_output=True)
    subprocess.run(["git", "-C", str(tmp_path), "-c", "core.autocrlf=false", "add", *mutation],
                   check=True, capture_output=True)
    subprocess.run(["git", "-C", str(tmp_path), "-c", "user.name=Audit fixture", "-c", "user.email=fixture@example.invalid",
                    "-c", "commit.gpgsign=false", "commit", "--quiet", "-m", "Controlled source snapshot"],
                   check=True, capture_output=True, timeout=20)
    commit = subprocess.check_output(["git", "-C", str(tmp_path), "rev-parse", "HEAD"], text=True).strip()
    policy = {label: {name: ("CRLF" if name == "daa/__init__.py" else "LF") for name in paths}
              for label, paths in (("evaluation", evaluation), ("mutation", mutation))}
    hashes = {label: {name: audit.digest(b"FIXTURE_SOURCE = 1\r\n" if ending == "CRLF" else b"FIXTURE_SOURCE = 1\n")
                      for name, ending in endings.items()} for label, endings in policy.items()}
    for name in audit.ARTIFACTS:
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"controlled historical artifact\n")
    members = {"mutation_evidence/log_" + str(i) + ".txt": b"controlled evidence " + str(i).encode() for i in range(90)}
    archive = tmp_path / "results/mutation_evidence.zip"
    with zipfile.ZipFile(archive, "w") as zipped:
        for name, content in members.items():
            zipped.writestr(name, content)
    rewrite(tmp_path, "results/summary.json", {"meta": {"n": 6000, "runtime_source_sha256": hashes["evaluation"]}})
    rewrite(tmp_path, "results/mutation_report.json", {"source_test_config_sha256": hashes["mutation"],
            "evidence_archive": archive.name, "evidence_archive_sha256": audit.digest(archive.read_bytes()),
            "evidence_member_sha256": {name: audit.digest(content) for name, content in members.items()}})
    rewrite(tmp_path, audit.DEFAULT_MANIFEST, {"schema": 1, "source_commit": commit, "encounters": 6000,
            "artifact_hash_policy": "SHA256; JSON/CSV CRLF normalized to LF; binary exact",
            "source_line_endings": policy,
            "artifacts_sha256": {name: audit.artifact_digest(tmp_path / name) for name in audit.ARTIFACTS}})
    # An intentional new version is detected, but not mistaken for a new evaluation.
    (tmp_path / "daa/sim.py").write_bytes(b"FIXTURE_SOURCE = 2\n")
    return tmp_path


def rewrite(root, name, value):
    (root / name).write_text(json.dumps(value), encoding="utf-8")


def reanchor_fixture_artifact(root, name):
    # Let a corruption reach the source/ZIP guard instead of the outer byte guard.
    manifest = json.loads((root / audit.DEFAULT_MANIFEST).read_text(encoding="utf-8"))
    manifest["artifacts_sha256"][name] = audit.artifact_digest(root / name)
    rewrite(root, audit.DEFAULT_MANIFEST, manifest)


def test_frozen_old_source_passes_while_reporting_current_controller_is_different(frozen):
    result = audit.verify(frozen)
    assert result["passed"] and not result["current_runtime_matches_archived_evaluation"]
    assert result["current_runtime_difference_ignoring_checkout_line_endings"] == ["daa/sim.py"]
    assert (result["artifact_count"], result["evaluation_source_files"], result["mutation_source_files"],
            result["mutation_evidence_members"]) == (8, 9, 26, 90)


def test_checkout_line_endings_do_not_change_archived_json_csv_identity(frozen):
    for name in audit.ARTIFACTS:
        path = frozen / name
        if path.suffix in audit.TEXT:
            path.write_bytes(path.read_bytes().replace(b"\r\n", b"\n").replace(b"\n", b"\r\n"))
    assert audit.verify(frozen)["passed"]


@pytest.mark.parametrize("name", ["results/sweep.csv", "results/mutation_evidence.zip"])
def test_altered_historical_data_or_evidence_is_rejected(frozen, name):
    with (frozen / name).open("ab") as stream:
        stream.write(b"changed")
    with pytest.raises(ValueError, match="historical artifact changed"):
        audit.verify(frozen)


def test_source_fingerprint_cannot_be_replaced_by_current_source_identity(frozen):
    name = "results/summary.json"
    summary = json.loads((frozen / name).read_text(encoding="utf-8"))
    summary["meta"]["runtime_source_sha256"]["daa/sim.py"] = audit.digest((frozen / "daa/sim.py").read_bytes())
    rewrite(frozen, name, summary)
    reanchor_fixture_artifact(frozen, name)
    with pytest.raises(ValueError, match="historical raw source fingerprint differs"):
        audit.verify(frozen)


def test_incomplete_identity_manifest_is_rejected(frozen):
    manifest = json.loads((frozen / audit.DEFAULT_MANIFEST).read_text(encoding="utf-8"))
    manifest["artifacts_sha256"].pop("results/fail_cases.csv")
    rewrite(frozen, audit.DEFAULT_MANIFEST, manifest)
    with pytest.raises(ValueError, match="artifact identity set is incomplete"):
        audit.verify(frozen)


def test_wrong_raw_checkout_line_ending_does_not_silently_normalize_original_run(frozen):
    manifest = json.loads((frozen / audit.DEFAULT_MANIFEST).read_text(encoding="utf-8"))
    manifest["source_line_endings"]["evaluation"]["daa/__init__.py"] = "LF"
    rewrite(frozen, audit.DEFAULT_MANIFEST, manifest)
    with pytest.raises(ValueError, match="historical raw source fingerprint differs"):
        audit.verify(frozen)


def test_missing_original_commit_is_not_certified_by_current_source(frozen):
    manifest = json.loads((frozen / audit.DEFAULT_MANIFEST).read_text(encoding="utf-8"))
    manifest["source_commit"] = "f" * 40
    rewrite(frozen, audit.DEFAULT_MANIFEST, manifest)
    with pytest.raises(ValueError, match="historical source unavailable"):
        audit.verify(frozen)


def test_mutation_zip_member_hash_is_independently_checked(frozen):
    name = "results/mutation_report.json"
    report = json.loads((frozen / name).read_text(encoding="utf-8"))
    key = next(iter(report["evidence_member_sha256"]))
    report["evidence_member_sha256"][key] = "0" * 64
    rewrite(frozen, name, report)
    reanchor_fixture_artifact(frozen, name)
    with pytest.raises(ValueError, match="evidence member changed"):
        audit.verify(frozen)
