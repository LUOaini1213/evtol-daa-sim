"""Audit frozen evaluations against their original Git source, never the new engine.

Original raw-checkout source hashes are reconstructed with the recorded per-file
LF/CRLF policy. This does not change any historical identity or rerun an encounter.
Git history must contain the fixed source commit (CI uses fetch-depth: 0).
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = "docs/historical_evaluation.json"
TEXT = {".json", ".csv"}
ARTIFACTS = {"results/summary.json", "results/sweep.csv", "results/fail_cases.csv",
             "results/mutation_report.json", "results/mutation_report_first_run.json",
             "results/mutation_evidence.zip", "docs/soc_curve.png", "docs/lead_time_hist.png"}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def inside(root, relative):
    path = (root / relative).resolve()
    if Path(relative).is_absolute() or not path.is_relative_to(root.resolve()):
        raise ValueError("archive reference must stay inside the repository")
    return path


def artifact_digest(path):
    data = path.read_bytes()
    return digest(data.replace(b"\r\n", b"\n") if path.suffix in TEXT else data)


def verify(root=ROOT, manifest_path=DEFAULT_MANIFEST):
    root = Path(root).resolve()
    manifest = json.loads(inside(root, manifest_path).read_text(encoding="utf-8"))
    commit = manifest["source_commit"]
    if manifest.get("schema") != 1 or not re.fullmatch(r"[a-f0-9]{40}", commit):
        raise ValueError("invalid fixed historical source commit")
    if manifest["artifact_hash_policy"] != "SHA256; JSON/CSV CRLF normalized to LF; binary exact":
        raise ValueError("unknown historical artifact hash policy")
    if set(manifest["artifacts_sha256"]) != ARTIFACTS:
        raise ValueError("historical artifact identity set is incomplete")
    for name, expected in manifest["artifacts_sha256"].items():
        if artifact_digest(inside(root, name)) != expected:
            raise ValueError("historical artifact changed: " + name)
    summary = json.loads(inside(root, "results/summary.json").read_text(encoding="utf-8"))
    mutation = json.loads(inside(root, "results/mutation_report.json").read_text(encoding="utf-8"))
    if summary["meta"]["n"] != manifest["encounters"]:
        raise ValueError("historical encounter count changed")
    profiles = {"evaluation": summary["meta"]["runtime_source_sha256"],
                "mutation": mutation["source_test_config_sha256"]}
    if len(profiles["evaluation"]) != 9 or len(profiles["mutation"]) != 26:
        raise ValueError("historical source identity set is incomplete")
    raw_files = {}
    for label, hashes in profiles.items():
        policy = manifest["source_line_endings"][label]
        if set(policy) != set(hashes):
            raise ValueError("historical source identity set differs: " + label)
        for name, expected in hashes.items():
            inside(root, name)  # Reject path escape without reading current source.
            key = name
            if key not in raw_files:
                result = subprocess.run(["git", "cat-file", "blob", commit + ":" + name], cwd=root,
                                        capture_output=True, timeout=20)
                if result.returncode:
                    raise ValueError("historical source unavailable: " + name + "; fetch original Git history")
                raw_files[key] = result.stdout
            original = raw_files[key].replace(b"\r\n", b"\n")
            if policy[name] == "CRLF":
                original = original.replace(b"\n", b"\r\n")
            elif policy[name] != "LF":
                raise ValueError("unknown recorded source line ending: " + name)
            if digest(original) != expected:
                raise ValueError("historical raw source fingerprint differs: " + label + " " + name)
    archive = inside(root, "results/" + mutation["evidence_archive"])
    if digest(archive.read_bytes()) != mutation["evidence_archive_sha256"]:
        raise ValueError("historical mutation evidence archive changed")
    members = mutation["evidence_member_sha256"]
    if len(members) != 90:
        raise ValueError("historical mutation evidence identity set is incomplete")
    with zipfile.ZipFile(archive) as zipped:
        names = [info.filename for info in zipped.infolist() if not info.is_dir()]
        if len(names) != len(set(names)) or set(names) != set(members):
            raise ValueError("historical mutation evidence member set changed")
        for name, expected in members.items():
            if digest(zipped.read(name)) != expected:
                raise ValueError("historical mutation evidence member changed: " + name)
    different = sorted(name for name in profiles["evaluation"]
                       if not inside(root, name).is_file()
                       or inside(root, name).read_bytes().replace(b"\r\n", b"\n")
                       != raw_files[name].replace(b"\r\n", b"\n"))
    return {"passed": True, "scope": "historical bytes/original source; no current engine evaluation",
            "source_commit": commit, "encounters": summary["meta"]["n"],
            "artifact_count": len(manifest["artifacts_sha256"]),
            "evaluation_source_files": len(profiles["evaluation"]),
            "mutation_source_files": len(profiles["mutation"]), "mutation_evidence_members": len(members),
            "current_runtime_matches_archived_evaluation": not different,
            "current_runtime_difference_ignoring_checkout_line_endings": different}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--manifest", default=DEFAULT_MANIFEST)
    args = parser.parse_args(argv)
    try:
        print(json.dumps(verify(args.root, args.manifest), indent=2))
        return 0
    except (ValueError, KeyError, OSError, zipfile.BadZipFile, subprocess.TimeoutExpired) as error:
        print("FAIL historical audit: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
