"""Static contracts for the repository-owned, no-motion startup entry."""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path

import pytest


NAV = Path(__file__).resolve().parents[3]
ENTRY = NAV / "tools/startup_smoke"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source(relative: str) -> str:
    return (NAV / relative).read_text(encoding="utf-8")


def test_startup_manifest_has_safe_repo_relative_paths_and_matches_sources():
    manifest = json.loads((NAV / "STARTUP_MANIFEST.json").read_text())
    assert manifest["provenance"]["git_parent_commit"] == (
        "403639cafbc5b19d8a7ffc0699c0677831ba0fc6"
    )
    assert manifest["provenance"]["verified_reference_entry_sha256"] == (
        "bd07e9a48940e92660a9a349fca95c951d30418eb92dba1c5d35509356f73da8"
    )
    for relative, digest in manifest["files"].items():
        path = Path(relative)
        assert not path.is_absolute()
        assert ".." not in path.parts
        assert _sha256(NAV / path) == digest, relative


def test_shell_bootstrap_keeps_required_ros_initialization_order():
    shell = _source("tools/startup_smoke/startup_smoke.sh")
    assert "set -eo pipefail" in shell
    assert shell.index('export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"') < shell.index(
        "source /opt/ros/humble/setup.bash"
    )
    assert shell.index("source /opt/ros/humble/setup.bash") < shell.index("command -v ros2")
    assert shell.index("command -v xacro") < shell.index(
        'source "$CLEANNAV_STARTUP_OVERLAY_SETUP"'
    )
    assert "exec " not in shell
    assert "candidate-run/env.sh" not in shell
    assert "/home/d1zzy/" not in shell


def test_supervisor_mode_run_id_and_launch_arguments_match_repository_launch():
    supervisor = _source("tools/startup_smoke/startup_supervisor.py")
    module = ast.parse(supervisor)
    function = next(node for node in module.body if isinstance(node, ast.FunctionDef)
                    and node.name == "launch_argv")
    return_node = next(node for node in ast.walk(function) if isinstance(node, ast.Return))
    def literal_prefix(node):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            return node.value
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
            return literal_prefix(node.left) + literal_prefix(node.right)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "str":
            return ""
        return ""

    actual = {prefix.split(":=", 1)[0] for item in return_node.value.elts
              if ":=" in (prefix := literal_prefix(item))}
    launch_source = _source("tools/startup_smoke/startup_smoke.launch.py")
    launch_module = ast.parse(launch_source)
    defaults_node = next(node.value for node in launch_module.body if isinstance(node, ast.Assign)
                         and any(isinstance(target, ast.Name) and target.id == "DEFAULTS"
                                 for target in node.targets))
    declared = {keyword.arg for keyword in defaults_node.keywords if keyword.arg}
    declared |= {"world", "model_path", "maps_root"}
    assert actual == {"mode", "maps_root", "world", "model_path"}
    assert actual <= declared
    assert "choices=['startup_smoke']" in supervisor
    assert "p.add_argument('--run-id',required=True)" in supervisor
    assert "must be a unique startup-smoke-* identifier" in supervisor
    assert "/home/d1zzy/" not in supervisor
    assert "cleannav-deliverables/startup-and-git-recovery-20261021" not in supervisor


def test_startup_adapter_forwards_only_no_motion_profile():
    source = _source("tools/startup_smoke/startup_smoke.launch.py")
    module = ast.parse(source)
    defaults = next(node.value for node in module.body if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == "DEFAULTS"
                            for target in node.targets))
    values = {keyword.arg: keyword.value.value for keyword in defaults.keywords
              if keyword.arg and isinstance(keyword.value, ast.Constant)}
    assert values["mode"] == "startup_smoke"
    assert values["gui"] == values["start_rviz"] == "false"
    assert values["enable_rviz_goals"] == values["auto_start_task"] == "false"
    assert values["enable_motion"] == "false"
    assert "world" in source and "model_path" in source and "maps_root" in source


def test_mapping_composition_is_fail_closed_and_has_no_route_or_lease_node():
    source = _source("tools/startup_smoke/mapping_mode_retry.py")
    assert "if arg('mode') != 'startup_smoke':" in source
    assert "simulation_motion_authorization=False" in source
    assert "SetParameter(name='simulation_motion_authorization', value=False)" in source
    assert "use_sim_time='true', block_all='true'" in source
    assert "no broker/route/Lease" in source
    assert "lease_acquire" not in source
    assert "Mission" not in source
    assert "NavigateToPose" not in source
    assert "mapping_route_node" not in source
    assert "enable_motion" in source and "start_rviz" in source
    assert "RECOVERY_ROOF_XACRO = (\n    SOURCE_ROOT / 'cleannav_simulation' / 'urdf' / 'cleannav_ackermann_startup_smoke.urdf.xacro')" in source
    assert "RECOVERY_SIMULATION_LAUNCH = (\n    SOURCE_ROOT / 'cleannav_simulation' / 'launch' / 'startup_smoke_simulation.launch.py')" in source


def test_simulation_launch_requires_explicit_workspace_world_and_models():
    source = _source("cleannav_simulation/launch/startup_smoke_simulation.launch.py")
    module = ast.parse(source)
    calls = [node for node in ast.walk(module) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == "DeclareLaunchArgument"]
    explicit = {call.args[0].value: call for call in calls
                if call.args and isinstance(call.args[0], ast.Constant)
                and call.args[0].value in {"world", "model_path"}}
    assert set(explicit) == {"world", "model_path"}
    assert all(not any(keyword.arg == "default_value" for keyword in call.keywords)
               for call in explicit.values())
    assert "DEFAULT_WORLD" not in source and "DEFAULT_MODEL_PATH" not in source


def test_roof_and_low_lidar_source_contracts_are_both_preserved():
    xacro = _source("cleannav_simulation/urdf/cleannav_ackermann_startup_smoke.urdf.xacro")
    assert '<link name="base_scan">' in xacro
    assert '<link name="mapping_lidar_link">' in xacro
    assert "~/out:=scan_raw" in xacro
    assert "~/out:=/mapping/scan_raw" in xacro
    assert "~/out:=/mapping/scan" in xacro
    assert "<samples>360</samples>" in xacro
    assert "<update_rate>8</update_rate>" in xacro
    assert "frame_name>mapping_lidar_link" in xacro


def test_runtime_helpers_discover_workspace_and_keep_cgroup_exception_explicit():
    runtime = _source("tools/startup_smoke/roof_lidar_runtime.py")
    assert "def discover_workspace(anchor: Path)" in runtime
    assert "cleannav-build/locks/build.lock" in runtime
    assert "cleannav-build/locks/runtime.lock" in runtime
    assert '"status": "UNKNOWN"' in runtime
    assert "cgroup_memory_snapshot" in runtime


def test_package_prefix_gate_requires_operator_bound_exact_sim_and_rtab_prefixes(
    monkeypatch, tmp_path
):
    from ament_index_python import packages
    from roof_lidar_runtime import check_package_prefixes

    prefixes = {name: tmp_path / name for name in (
        "cleannav_simulation", "cleannav_rtabmap", "cleannav_safety_supervisor"
    )}
    for prefix in prefixes.values():
        (prefix / "share" / prefix.name).mkdir(parents=True)
    monkeypatch.setattr(packages, "get_package_prefix", lambda name: str(prefixes[name]))
    monkeypatch.setattr(packages, "get_package_share_directory",
                        lambda name: str(prefixes[name] / "share" / name))
    monkeypatch.setenv("CLEANNAV_EXPECTED_SIMULATION_PREFIX",
                       str(prefixes["cleannav_simulation"]))
    monkeypatch.setenv("CLEANNAV_EXPECTED_RTABMAP_PREFIX",
                       str(prefixes["cleannav_rtabmap"]))
    assert check_package_prefixes()["ok"] is True
    monkeypatch.setenv("CLEANNAV_EXPECTED_RTABMAP_PREFIX", str(tmp_path / "wrong"))
    result = check_package_prefixes()
    assert result["ok"] is False
    assert result["cleannav_rtabmap"]["prefix_matches_expected"] is False
    monkeypatch.delenv("CLEANNAV_EXPECTED_SIMULATION_PREFIX")
    assert check_package_prefixes()["ok"] is False


def test_documented_entry_and_required_static_checks_exist():
    guide = NAV / "docs/operations/STARTUP_SMOKE.md"
    assert guide.is_file()
    assert "tools/startup_smoke/startup_smoke.sh" in guide.read_text()
    assert (ENTRY / "run_static_checks.sh").is_file()


def test_supervisor_parser_refuses_other_modes_without_creating_output():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "startup_supervisor_static_test", ENTRY / "startup_supervisor.py"
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    assert module.parse_args(["--run-id", "startup-smoke-static-contract"]).mode == "startup_smoke"
    with pytest.raises(SystemExit):
        module.parse_args(["--mode", "mapping", "--run-id", "startup-smoke-static-contract"])
