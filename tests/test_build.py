"""Build the actual C++ source from non-ASCII paths, including shell characters."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.fixture
def unicode_build(tmp_path, monkeypatch):
    source = tmp_path / "source 空间 & 100% !" / "cpp"
    shutil.copytree(Path(__file__).resolve().parents[1] / "cpp", source)
    spec = importlib.util.spec_from_file_location("unicode_cpp_build", source / "build.py")
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    output = tmp_path / "output 结果 & 100% !"
    monkeypatch.delenv("CXX", raising=False)
    if os.name == "nt":
        # Exercise MSVC even if a GNU compiler is also on this machine's PATH.
        original_which = builder.shutil.which
        monkeypatch.setattr(builder.shutil, "which", lambda name, **kwargs:
                            None if name in ("g++", "clang++") else original_which(name, **kwargs))
    try:
        executable = builder.build(output)
    except FileNotFoundError:
        if os.environ.get("DAA_REQUIRE_CPP") == "1":
            raise
        pytest.skip("C++ compiler unavailable")
    return executable, output


def test_build_and_run_from_unicode_paths(unicode_build):
    executable, output = unicode_build
    result = subprocess.run([str(executable), "cpa"], input="1000 0 0 -50 0 0\n",
                            text=True, capture_output=True, check=True)
    assert list(map(float, result.stdout.split())) == [20.0, 0.0, 0.0]
    assert not list(output.glob("*.bat"))
