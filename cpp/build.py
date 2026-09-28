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
                           "-property", "installationPath"],
                          capture_output=True, text=True, check=True).stdout.strip()
    vcvars = Path(inst) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
    if not vcvars.exists():
        raise FileNotFoundError("MSVC found but vcvars64.bat missing")
    obj_dir = out.parent
    bat = obj_dir / "_build_msvc.bat"
    lines = [
        "@echo off",
        f'call "{vcvars}" >nul',
        f'cl /nologo /std:c++17 /O2 /EHsc /W4 /WX /fp:precise /D_CRT_SECURE_NO_WARNINGS /Fo"{obj_dir}\\\\" /Fe"{out}" "{SRC}"',
    ]
    bat.write_text("\r\n".join(lines) + "\r\n", encoding="ascii")
    r = subprocess.run(["cmd", "/d", "/c", str(bat)], capture_output=True, text=True)
    bat.unlink()
    if r.returncode != 0:
        raise RuntimeError("MSVC build failed:\n" + r.stdout + r.stderr)


def build(out_dir) -> Path:
    out_dir = Path(out_dir)
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
