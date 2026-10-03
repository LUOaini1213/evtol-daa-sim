"""Build cpp/daa_cli with whatever C++17 compiler is available.

Order: $CXX, g++, clang++, then (Windows only) MSVC located via vswhere.
Usage: python cpp/build.py [output_dir]   -> prints the path of the executable.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "daa_cli.cpp"


def _gcc_like(cxx, out):
    cmd = [cxx, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off",
           "-o", str(out), str(SRC)]
    subprocess.run(cmd, check=True, capture_output=True, text=True)


def _msvc(out):
    vswhere = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / \
        "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
    if not vswhere.exists():
        raise FileNotFoundError("no C++ compiler found (g++, clang++ or MSVC)")
    inst = subprocess.run([str(vswhere), "-latest", "-products", "*", "-requires",
                           "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                           "-property", "installationPath", "-utf8"],
                          capture_output=True, text=True, encoding="utf-8", check=True).stdout.strip()
    vcvars = Path(inst) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
    if not vcvars.exists():
        raise FileNotFoundError("MSVC found but vcvars64.bat missing")
    # Bootstrap the toolchain only in cmd. /u makes SET's redirected output
    # UTF-16LE; source/output paths never enter shell syntax or an ASCII .bat.
    bootstrap_env = os.environ.copy()
    bootstrap_env["DAA_VCVARS"] = str(vcvars)
    bootstrap = subprocess.run(
        'cmd /d /u /v:off /c ""%DAA_VCVARS%" >nul && set"',
        env=bootstrap_env, capture_output=True, check=True,
    )
    compiler_env = {}
    for line in bootstrap.stdout.decode("utf-16-le").splitlines():
        key, separator, value = line.partition("=")
        if key and separator:
            compiler_env[key.upper()] = value
    compiler = shutil.which("cl.exe", path=compiler_env.get("PATH", ""))
    if compiler is None:
        raise RuntimeError("MSVC environment did not provide cl.exe")
    out = out.resolve()
    cmd = [compiler, "/nologo", "/std:c++17", "/O2", "/EHsc", "/W4", "/WX",
           "/fp:precise", "/D_CRT_SECURE_NO_WARNINGS", "/Fo" + str(out.with_suffix(".obj")),
           "/Fe" + str(out), str(SRC)]
    result = subprocess.run(cmd, env=compiler_env, capture_output=True)
    if result.returncode != 0:
        raise RuntimeError("MSVC build failed:\n" +
                           (result.stdout + result.stderr).decode(errors="replace"))


def build(out_dir) -> Path:
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    exe = out_dir / ("daa_cli.exe" if os.name == "nt" else "daa_cli")
    cxx = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
    if cxx:
        _gcc_like(cxx, exe)
    elif os.name == "nt":
        _msvc(exe)
    else:
        raise FileNotFoundError("no C++ compiler found")
    return exe


if __name__ == "__main__":
    print(build(sys.argv[1] if len(sys.argv) > 1 else HERE.parent / "build"))
