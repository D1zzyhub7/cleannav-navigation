import importlib.util
import ast
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "amcl_diagnostic_profile.py"
SPEC = importlib.util.spec_from_file_location("amcl_diagnostic_profile_test", MODULE_PATH)
PROFILE_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROFILE_MODULE)


def test_default_profile_selection_preserves_existing_modes():
    assert PROFILE_MODULE.select_profile("none", {}) is None
    assert PROFILE_MODULE.select_profile("amcl", {}) is None


def test_diagnostic_profile_requires_explicit_amcl_selection():
    env = {
        "CLEANNAV_AMCL_DIAGNOSTIC_PROFILE": str(PROFILE_MODULE.PROFILE),
        "CLEANNAV_TF_PRIVATE_DIAGNOSTICS": "1",
    }
    assert PROFILE_MODULE.select_profile("amcl", env) == PROFILE_MODULE.PROFILE
    with pytest.raises(ValueError, match="only valid with localization_mode=amcl"):
        PROFILE_MODULE.select_profile("none", env)
    with pytest.raises(ValueError, match="unrecognized"):
        PROFILE_MODULE.select_profile("amcl", {**env, "CLEANNAV_AMCL_DIAGNOSTIC_PROFILE": "/tmp/other.json"})
    with pytest.raises(ValueError, match="require CLEANNAV_TF_PRIVATE_DIAGNOSTICS=1"):
        PROFILE_MODULE.select_profile("amcl", {key: value for key, value in env.items()
                                                 if key != "CLEANNAV_TF_PRIVATE_DIAGNOSTICS"})


def test_ldd_parser_records_resolved_library_identity(tmp_path):
    library = tmp_path / "libexample.so"
    library.write_bytes(b"static identity fixture")
    rows = PROFILE_MODULE.parse_ldd_output(f"\tlibexample.so => {library} (0x1234)\n")
    assert rows == [{"needed": "libexample.so", "path": str(library.resolve()),
                     "sha256": PROFILE_MODULE.digest(library)}]


@pytest.mark.parametrize("line", [
    "\tlibmissing.so => not found",
    "undefined symbol: missing_symbol (./libcandidate.so)",
])
def test_ldd_parser_rejects_unresolved_or_undefined_symbols(line):
    with pytest.raises(ValueError):
        PROFILE_MODULE.parse_ldd_output(line)


@pytest.mark.parametrize("selected", [False, True])
def test_launch_materializes_the_verified_amcl_environment_without_nesting(selected):
    from launch import LaunchContext
    from launch.actions import UnsetEnvironmentVariable
    from launch.descriptions import Executable

    source = ast.parse(MODULE_PATH.with_name("mapping_mode_retry.py").read_text())
    assignment = next(node for node in ast.walk(source) if isinstance(node, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == "amcl_node_options"
                              for t in node.targets))
    selection = next(node for node in ast.walk(source) if isinstance(node, ast.If)
                     and ast.unparse(node.test) == "amcl_additional_env is not None")
    verified = {"LD_LIBRARY_PATH": "/isolated/amcl/lib:/isolated/tf2/lib:/opt/ros/humble/lib",
                "CLEANNAV_TF_PRIVATE_DIAGNOSTICS": "1"}
    scope = {"amcl_additional_env": verified if selected else None}
    exec(compile(ast.Module(body=[assignment, selection], type_ignores=[]),
                 "amcl_launch_environment", "exec"), scope)
    context = LaunchContext()
    context.environment["LD_LIBRARY_PATH"] = "/opt/ros/humble/lib"
    context.environment["CLEANNAV_TF_PRIVATE_DIAGNOSTICS"] = "1"
    UnsetEnvironmentVariable("CLEANNAV_TF_PRIVATE_DIAGNOSTICS").execute(context)
    process = Executable(cmd=["/bin/true"], **scope["amcl_node_options"])
    process.prepare(context, None)  # Substitution only; no process is started.
    assert process.final_env["LD_LIBRARY_PATH"] == (
        verified["LD_LIBRARY_PATH"] if selected else "/opt/ros/humble/lib")
    assert process.final_env.get("CLEANNAV_TF_PRIVATE_DIAGNOSTICS") == (
        "1" if selected else None)
    assert "CLEANNAV_TF_PRIVATE_DIAGNOSTICS" not in context.environment


@pytest.mark.parametrize("selected", [False, True])
def test_scan_wait_override_is_scoped_to_the_diagnostic_amcl(selected, tmp_path, monkeypatch):
    import tempfile
    import yaml
    from launch import LaunchContext
    from launch_ros.actions import Node

    source = ast.parse(MODULE_PATH.with_name("mapping_mode_retry.py").read_text())
    selection = next(node for node in ast.walk(source) if isinstance(node, ast.If)
                     and ast.unparse(node.test) == "diagnostic_identity is not None")
    scope = {"diagnostic_identity": {"amcl_parameter_overrides": {
        "scan_tf_wait_timeout": 0.30}} if selected else None,
        "amcl_overrides": {"transform_tolerance": 0.0}}
    exec(compile(ast.Module(body=[selection], type_ignores=[]), "amcl_scan_wait_override", "exec"), scope)
    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path))
    node = Node(package="nav2_amcl", executable="amcl", name="amcl", namespace="/",
                parameters=[scope["amcl_overrides"]])
    node._perform_substitutions(LaunchContext())
    path = Path(node._Node__expanded_parameter_arguments[-1][0])
    values = yaml.safe_load(path.read_text())["/amcl"]["ros__parameters"]
    assert values["transform_tolerance"] == 0.0
    assert values.get("scan_tf_wait_timeout") == (0.30 if selected else None)
