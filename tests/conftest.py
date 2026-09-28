import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from daa import load_config  # noqa: E402


@pytest.fixture(scope="session")
def cfg():
    return load_config()


@pytest.fixture(scope="session")
def cpp_cli(tmp_path_factory):
    """Build cpp/daa_cli once per session. Skips when no compiler is found,
    unless DAA_REQUIRE_CPP=1 (set in CI), in which case it fails."""
    sys.path.insert(0, str(ROOT / "cpp"))
    import build as cpp_build

    try:
        return cpp_build.build(tmp_path_factory.mktemp("cpp"))
    except FileNotFoundError as e:
        if os.environ.get("DAA_REQUIRE_CPP") == "1":
            pytest.fail(f"C++ build required but no compiler: {e}")
        pytest.skip(f"no C++ compiler: {e}")
    except Exception as e:  # compile error etc. is a real failure
        pytest.fail(f"C++ build failed: {e}")
