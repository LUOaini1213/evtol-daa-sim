"""Fail if tracked files (or members of tracked ZIP archives) contain local machine paths.

    python scripts/check_no_local_paths.py           # scan the files tracked by Git
    python scripts/check_no_local_paths.py FILE ...  # scan the given files instead

A home-directory path (C:\\Users\\<name>\\, /Users/<name>/, /home/<name>/) or a
pytest-of-<name> temp folder identifies the machine that produced a file.
Placeholders written by scripts/mutation_check.py (<home>, <tmp>, <user>) pass.
"""
from __future__ import annotations

import io
import re
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PATTERN = re.compile(
    r"(?i)(?<![A-Za-z])[a-z]:(?:\\\\|\\|/)+users(?:\\\\|\\|/)+\w"  # Windows home folder
    r"|(?<![\w.<])/(?:Users|home)/(?!runner/)\w[\w.-]*/"           # macOS / Linux home folder
    r"|pytest-of-\w"                                                # pytest per-user temp folder
    r"|AppData(?:\\\\|\\|/)+Local(?:\\\\|\\|/)+Temp"                # Windows temp
)


def hits_in(name, data):
    text = data.decode("utf-8", errors="replace")
    return [f"{name}:{text.count(chr(10), 0, m.start()) + 1}: {m.group(0)}" for m in PATTERN.finditer(text)]


def scan(path, label=None):
    label = label or str(path)
    data = Path(path).read_bytes()
    if path.suffix.lower() == ".zip":
        found = []
        with zipfile.ZipFile(io.BytesIO(data)) as zipped:
            for name in zipped.namelist():
                found += hits_in(label + "!" + name, zipped.read(name))
        return found
    if b"\0" in data[:4096]:
        return []  # other binaries (PNG) carry no text paths
    return hits_in(label, data)


def tracked_files():
    names = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, capture_output=True,
                           check=True).stdout.decode("utf-8").split("\0")
    return [ROOT / name for name in names if name and (ROOT / name).is_file()]


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    files = [Path(a) for a in argv] if argv else tracked_files()
    found = []
    for f in files:
        label = f.resolve().relative_to(ROOT).as_posix() if f.resolve().is_relative_to(ROOT) else str(f)
        found += scan(f, label)
    if found:
        print("local machine paths found:", file=sys.stderr)
        for line in found:
            print("  " + line, file=sys.stderr)
        return 1
    print(f"no local machine paths in {len(files)} files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
