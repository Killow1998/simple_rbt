"""CLI checker routing uses real entrypoints and preserves acceptance options."""
import importlib.util
from pathlib import Path
from simple_rbt_mujoco.paths import ROOT
from types import SimpleNamespace
from unittest.mock import patch

import pytest

SCRIPT = ROOT / "tools/check_ros2.py"
spec = importlib.util.spec_from_file_location("check_ros2_routing", SCRIPT)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def arguments(runtime):
    return SimpleNamespace(runtime=runtime, robot="ranger_mini", backend="warp", terrain="hills",
                           controller="auto", control_mode="native", camera_hz=10, duration=8.)


def test_runtime_selector_uses_real_cpp_and_python_entrypoints(tmp_path):
    output = tmp_path / "capture with spaces"
    native = checker.cli_program(arguments("cpp"), output)
    oracle = checker.cli_program(arguments("python"), output)
    assert native[0] == str(checker.ROOT / "run.sh")
    assert oracle[0] == str(checker.ROOT / "reference/python/run.sh")
    assert native[1:] == oracle[1:] == [
        "--robot", "ranger_mini", "--backend", "warp", "--terrain", "hills", "--controller", "auto",
        "--control-mode", "native", "--camera-hz", "10", "--duration", "8.0", "--ros2", "--settle", "1",
        "--cameras", "--output", str(output)]
    assert all(Path(program[0]).is_file() for program in (native, oracle))


def test_unknown_runtime_cannot_silently_fall_back_to_python(tmp_path):
    with pytest.raises(ValueError, match="runtime must be cpp or python"):
        checker.cli_program(arguments("unknown"), tmp_path)


def test_no_runtime_argument_selects_cpp_before_any_ros_setup():
    # Stop at parsing: this tests the real CLI default without creating a DDS
    # node, spawning a simulator or initializing GPU/rendering resources.
    class Parsed(Exception):
        pass

    def inspect(parser, *unused_args, **unused_kwargs):
        parsed = original(parser, [])
        assert parsed.runtime == "cpp"
        raise Parsed

    original = checker.argparse.ArgumentParser.parse_args
    with patch.object(checker.argparse.ArgumentParser, "parse_args", inspect):
        with pytest.raises(Parsed):
            checker.main()
