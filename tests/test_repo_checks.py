"""The local-path gate must fail on the defect it guards (make_readme --check is
exercised against results/ in CI; the mutation gate copies no results/)."""
import importlib.util
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name + "_under_test", ROOT / "scripts" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_local_path_gate_flags_home_paths_in_files_and_zip_members(tmp_path):
    gate = load("check_no_local_paths")
    home = "\\".join(["C:", "Users", "someone", "Desktop", "x.log"])
    clean = tmp_path / "clean.log"
    clean.write_text("<tmp>/m3/tests/test_x.py and pytest-of-<user> and <home>/x\n", encoding="utf-8")
    dirty = tmp_path / "dirty.log"
    dirty.write_text("ok\n" + home + "\n", encoding="utf-8")
    archive = tmp_path / "evidence.zip"
    with zipfile.ZipFile(archive, "w") as zipped:
        zipped.writestr("e/pytest.log", "/home/" + "someone/run/tests\n")
    assert gate.main([str(clean)]) == 0
    assert gate.main([str(dirty)]) == 1
    assert gate.main([str(archive)]) == 1
