"""Exercise the mutation harness on isolated toy suites, never product mutants."""
import importlib.util
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

import pytest


@pytest.fixture
def runner():
    path = Path(__file__).resolve().parents[1] / "scripts" / "mutation_check.py"
    spec = importlib.util.spec_from_file_location("mutation_runner_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def toy_tree(tmp_path, source):
    tree = tmp_path / "toy"
    (tree / "tests").mkdir(parents=True)
    (tree / "tests" / "test_toy.py").write_text(source, encoding="utf-8")
    return tree


@pytest.mark.parametrize("source,status", [
    ("def test_ok():\n    assert 2 + 2 == 4\n", "passed"),
    ("def test_wrong():\n    assert 2 + 2 == 5\n", "failed"),
    ("import missing_dependency_for_mutation_test\n", "invalid"),
    ("import pytest\n@pytest.fixture\ndef built():\n    pytest.fail('compiler failed')\n"
     "def test_compiler(built):\n    pass\n", "invalid"),
    ("import pytest\n@pytest.fixture\ndef resource():\n    yield\n    assert False\n"
     "def test_cleanup(resource):\n    pass\n", "invalid"),
    ("import pytest\n@pytest.mark.skip(reason='compiler unavailable')\n"
     "def test_skipped():\n    pass\n", "invalid"),
    ("import os\ndef test_crash():\n    os._exit(3)\n", "invalid"),
    ("# no tests collected\n", "invalid"),
])
def test_real_pytest_outcome_classification(runner, tmp_path, source, status):
    tree = toy_tree(tmp_path, source)
    result = runner.run_suite(tree, artifact_dir=tmp_path / "evidence", timeout=30)
    assert result["status"] == status
    assert Path(result["log"]).is_file()
    assert Path(result["log"]).read_text(encoding="utf-8")
    if status == "failed":
        assert result["failures"] == 1
        assert result["errors"] == 0
        assert result["failure_cases"][0]["name"].endswith("test_wrong")
        assert Path(result["junit"]).is_file()
    if status == "passed":
        assert result["tests"] == 1
        assert result["skipped"] == 0


def test_timeout_is_invalid_and_keeps_log(runner, tmp_path):
    tree = toy_tree(tmp_path, "import time\ndef test_wait():\n    time.sleep(10)\n")
    result = runner.run_suite(tree, artifact_dir=tmp_path / "timeout", timeout=0.3)
    assert result["status"] == "invalid"
    assert "timeout" in result["reason"].lower()
    assert Path(result["log"]).is_file()


def test_forces_cpp_and_removes_inherited_selection(runner, tmp_path, monkeypatch):
    monkeypatch.setenv("DAA_REQUIRE_CPP", "0")
    monkeypatch.setenv("PYTEST_ADDOPTS", "-k missing_test_name")
    tree = toy_tree(tmp_path, "import os\ndef test_environment():\n"
                    "    assert os.environ['DAA_REQUIRE_CPP'] == '1'\n"
                    "    assert 'PYTEST_ADDOPTS' not in os.environ\n")
    result = runner.run_suite(tree, artifact_dir=tmp_path / "env")
    assert result["status"] == "passed"
    assert result["tests"] == 1


def test_missing_fresh_junit_cannot_reuse_previous_success(runner, tmp_path, monkeypatch):
    tree = toy_tree(tmp_path, "def test_ok():\n    pass\n")
    evidence = tmp_path / "evidence"
    first = runner.run_suite(tree, artifact_dir=evidence)
    assert first["status"] == "passed"
    monkeypatch.setattr(runner, "_run_command", lambda *a, **k:
                        subprocess.CompletedProcess(a[0], 0, "fake success", ""))
    second = runner.run_suite(tree, artifact_dir=evidence)
    assert second["status"] == "invalid"
    assert "junit" in second["reason"].lower()


def test_negative_only_rejected_before_any_suite(runner, tmp_path, monkeypatch):
    monkeypatch.setattr(runner, "ROOT", toy_tree(tmp_path, "def test_ok():\n    pass\n"))
    monkeypatch.setattr(runner, "run_suite", lambda *a, **k: pytest.fail("suite started"))
    with pytest.raises(SystemExit) as exc:
        runner.main(["--only", "-1"])
    assert exc.value.code == 2


@pytest.mark.parametrize("replacement,expected,exitcode", [
    ("VALUE = 2", "caught", 0),
    ("VALUE = 1 + 0", "survived", 1),
    ("VALUE = (", "invalid", 2),
    ("raise ImportError('broken import')", "invalid", 2),
])
def test_main_toy_mutants_exit_codes_and_persistent_evidence(
        runner, tmp_path, monkeypatch, replacement, expected, exitcode):
    tree = toy_tree(tmp_path, "from toy_value import VALUE\ndef test_value():\n    assert VALUE == 1\n")
    original = b"VALUE = 1\r\n"
    (tree / "toy_value.py").write_bytes(original)
    monkeypatch.setattr(runner, "ROOT", tree)
    monkeypatch.setattr(runner, "MUTANTS", [("toy_value.py", "VALUE = 1", replacement, "toy")])
    out = tmp_path / "reports" / "report.json"
    out.parent.mkdir()
    assert runner.main(["--out", str(out)]) == exitcode
    report = json.loads(out.read_text(encoding="utf-8"))
    assert report["baseline"]["status"] == "passed"
    assert report["mutants"][0]["status"] == expected
    assert report["caught"] == int(expected == "caught")
    assert report["invalid"] == int(expected == "invalid")
    assert Path(report["artifacts_dir"]).parent.name == "report_artifacts"
    assert Path(report["baseline"]["junit"]).is_file()
    assert Path(report["mutants"][0]["log"]).is_file()
    assert (tree / "toy_value.py").read_bytes() == original


def test_failed_baseline_aborts_without_mutant_claim(runner, tmp_path, monkeypatch):
    tree = toy_tree(tmp_path, "def test_bad():\n    assert False\n")
    (tree / "toy_value.py").write_text("VALUE = 1\n", encoding="utf-8")
    monkeypatch.setattr(runner, "ROOT", tree)
    monkeypatch.setattr(runner, "MUTANTS", [("toy_value.py", "VALUE = 1", "VALUE = 2", "toy")])
    out = tmp_path / "report.json"
    assert runner.main(["--out", str(out)]) == 2
    report = json.loads(out.read_text(encoding="utf-8"))
    assert report["baseline"]["status"] == "failed"
    assert report["mutants"] == []
    assert report["caught"] == 0
    assert report["status"] == "invalid"


@pytest.mark.parametrize("broken", [True, False])
def test_real_cpp_preflight_precedes_call_phase_failure(runner, tmp_path, broken):
    tree = toy_tree(tmp_path, "def test_behavior():\n    assert False\n")
    cpp = tree / "cpp"
    cpp.mkdir()
    shutil.copyfile(Path(runner.__file__).resolve().parents[1] / "cpp" / "build.py", cpp / "build.py")
    (cpp / "daa_cli.cpp").write_text(
        "int main() { this is invalid C++; }" if broken else "int main() { return 0; }",
        encoding="utf-8")
    result = runner.run_suite(tree, artifact_dir=tmp_path / "build-evidence")
    assert result["status"] == ("invalid" if broken else "failed")
    assert Path(result["build_log"]).is_file()
    if broken:
        assert result["build_returncode"] != 0
        assert not Path(result["junit"]).exists()
        assert "compile" in result["reason"].lower()
    else:
        assert result["build_returncode"] == 0
        assert result["failure_cases"][0]["name"].endswith("test_behavior")


def test_only_preserves_default_full_report(runner, tmp_path, monkeypatch):
    tree = toy_tree(tmp_path, "from toy_value import VALUE\ndef test_value():\n    assert VALUE == 1\n")
    (tree / "toy_value.py").write_text("VALUE = 1\n", encoding="utf-8")
    monkeypatch.setattr(runner, "ROOT", tree)
    monkeypatch.setattr(runner, "MUTANTS", [("toy_value.py", "VALUE = 1", "VALUE = 2", "toy")])
    results = tree / "results"
    results.mkdir()
    full_report = results / "mutation_report.json"
    full_report.write_text("preserve full report", encoding="utf-8")
    assert runner.main(["--only", "0"]) == 0
    assert full_report.read_text(encoding="utf-8") == "preserve full report"
    partial = json.loads((results / "mutation_report.only-0.json").read_text(encoding="utf-8"))
    assert partial["complete_full_matrix"] is False
    assert partial["selected_indices"] == [0]


def _process_running(pid):
    if os.name == "nt":
        import ctypes
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.OpenProcess.restype = ctypes.c_void_p
        kernel.GetExitCodeProcess.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong)]
        kernel.CloseHandle.argtypes = [ctypes.c_void_p]
        handle = kernel.OpenProcess(0x1000, False, pid)
        if not handle:
            return False
        try:
            code = ctypes.c_ulong()
            assert kernel.GetExitCodeProcess(handle, ctypes.byref(code))
            return code.value == 259  # STILL_ACTIVE
        finally:
            kernel.CloseHandle(handle)
    stat = Path(f"/proc/{pid}/stat")
    if stat.exists() and stat.read_text().split(")", 1)[1].split()[0] == "Z":
        return False
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def test_timeout_reaps_real_descendants(runner, tmp_path, record_testsuite_property):
    marker, child_pid, grandchild_pid = [tmp_path / name for name in ("escaped.txt", "child.pid", "grandchild.pid")]
    grandchild = ("import os,time\nfrom pathlib import Path\n"
                  f"Path({str(grandchild_pid)!r}).write_text(str(os.getpid()))\n"
                  f"time.sleep(6)\nPath({str(marker)!r}).write_text('escaped timeout')\n")
    child = ("import os,subprocess,sys,time\nfrom pathlib import Path\n"
             f"Path({str(child_pid)!r}).write_text(str(os.getpid()))\n"
             f"subprocess.Popen([sys.executable, '-c', {grandchild!r}])\ntime.sleep(10)\n")
    source = ("import subprocess,sys,time\ndef test_process_tree():\n"
              f"    subprocess.Popen([sys.executable, '-c', {child!r}])\n    time.sleep(10)\n")
    tree = toy_tree(tmp_path, source)
    result = runner.run_suite(tree, artifact_dir=tmp_path / "tree-timeout", timeout=2)
    assert result["status"] == "invalid"
    assert child_pid.exists() and grandchild_pid.exists(), "toy descendants did not start"
    assert not marker.exists(), "descendant outlived the pytest timeout"
    pids = [int(child_pid.read_text()), int(grandchild_pid.read_text())]
    for _ in range(50):
        if not any(_process_running(pid) for pid in pids):
            break
        time.sleep(0.05)
    assert not any(_process_running(pid) for pid in pids)
    assert result["timeout_process"]["returncode"] is not None
    record_testsuite_property("timeout_descendant_pids", json.dumps(pids))
    record_testsuite_property("timeout_cleanup", json.dumps(result["timeout_process"]))
