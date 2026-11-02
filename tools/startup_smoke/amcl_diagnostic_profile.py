"""Static identity checks for the opt-in AMCL private-Buffer diagnostic build."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


NAV = Path(__file__).resolve().parents[2]
PROFILE = Path(__file__).with_name("amcl_private_buffer_diagnostic_profile.json").resolve()
MANIFEST = NAV / "STARTUP_MANIFEST.json"


def digest(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_config_sha256(path: Path) -> str:
    data = json.loads(path.read_text(encoding="utf-8"))
    data.pop("source_manifest_sha256", None)
    payload = json.dumps(data, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def select_profile(localization_mode: str, environment: dict[str, str]) -> Path | None:
    selected = environment.get("CLEANNAV_AMCL_DIAGNOSTIC_PROFILE")
    enabled = environment.get("CLEANNAV_TF_PRIVATE_DIAGNOSTICS")
    if selected is None and enabled is None:
        return None
    if localization_mode != "amcl":
        raise ValueError("AMCL diagnostic profile is only valid with localization_mode=amcl")
    if Path(selected or "").resolve() != PROFILE:
        raise ValueError("unrecognized AMCL diagnostic profile path")
    if enabled != "1":
        raise ValueError("AMCL private Buffer diagnostics require CLEANNAV_TF_PRIVATE_DIAGNOSTICS=1")
    return PROFILE


def _default_prefix_resolver(package: str) -> str:
    from ament_index_python.packages import get_package_prefix
    return get_package_prefix(package)


def parse_ldd_output(output: str) -> list[dict[str, str]]:
    rows = []
    for line in output.splitlines():
        match = re.match(r"\s*(\S+)\s+=>\s+(\S+)", line)
        if match:
            needed, raw_path = match.groups()
            if raw_path.startswith("not"):
                raise ValueError("unresolved shared-library dependency: " + line.strip())
            path = Path(raw_path).resolve(strict=True)
            rows.append({"needed": needed, "path": str(path), "sha256": digest(path)})
            continue
        match = re.match(r"\s*(/\S+)\s+\(0x[0-9a-fA-F]+\)", line)
        if match:
            path = Path(match.group(1)).resolve(strict=True)
            rows.append({"needed": path.name, "path": str(path), "sha256": digest(path)})
        elif "not found" in line or "undefined symbol:" in line:
            raise ValueError("dynamic dependency check failed: " + line.strip())
    return sorted(rows, key=lambda row: (row["needed"], row["path"]))


def _run_ldd(target: Path, environment: dict[str, str], runner) -> list[dict[str, str]]:
    if target.read_bytes()[:4] != b"\x7fELF":
        raise ValueError("runtime dependency target is not an ELF file: " + str(target))
    result = runner(["ldd", "-r", str(target)], env=environment,
                    capture_output=True, text=True, timeout=10)
    if result.returncode != 0:
        raise ValueError(f"ldd -r failed for {target}: {result.stderr.strip()}")
    return parse_ldd_output(result.stdout)


def _package_markers(package: str, ament_prefix_path: str) -> list[str]:
    marker = Path("share/ament_index/resource_index/packages") / package
    return [str(Path(prefix).resolve()) for prefix in ament_prefix_path.split(":")
            if prefix and (Path(prefix) / marker).is_file()]


def _check_source_manifest(package: str, record: dict) -> None:
    manifest_path = Path(record["source_manifest"]["path"]).resolve(strict=True)
    if digest(manifest_path) != record["source_manifest"]["sha256"]:
        raise ValueError(package + " installed source manifest SHA-256 mismatch")
    source_copy = Path(record["source_manifest_source_path"]).resolve(strict=True)
    if digest(source_copy) != digest(manifest_path):
        raise ValueError(package + " installed/source manifest copies differ")
    source_root = Path(record["source_root"]).resolve(strict=True)
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    if data.get("package") != package or not data.get("files"):
        raise ValueError(package + " source manifest package/files are invalid")
    for relative, expected in data["files"].items():
        rel = Path(relative)
        if rel.is_absolute() or ".." in rel.parts:
            raise ValueError(package + " source manifest path escapes package source")
        path = (source_root / rel).resolve(strict=True)
        if source_root not in path.parents or digest(path) != expected:
            raise ValueError(package + " source file identity mismatch: " + relative)


def _check_build_receipt(package: str, record: dict) -> None:
    build = record["build_receipt"]
    for item in build["files"]:
        path = Path(item["path"]).resolve(strict=True)
        if digest(path) != item["sha256"]:
            raise ValueError(package + " build receipt changed: " + str(path))
    result_path = Path(build["result_path"]).resolve(strict=True)
    receipt = json.loads(result_path.read_text(encoding="utf-8"))
    if (receipt.get("target") != build["target"] or receipt.get("exit_code") != 0
            or receipt.get("build_exit_code") != 0):
        raise ValueError(package + " build receipt does not record a successful target")
    commands = [json.loads(line) for line in Path(build["commands_path"]).read_text(
        encoding="utf-8").splitlines() if line.strip()]
    source_root = str(Path(record["source_root"]).resolve())
    prefix = str(Path(record["prefix"]).resolve())
    configured = any(
        row.get("kind") == "cmake" and row.get("argv", [None])[0] == source_root
        and any(value == "-DCMAKE_INSTALL_PREFIX=" + prefix for value in row.get("argv", []))
        for row in commands)
    if not configured:
        raise ValueError(package + " build command does not bind selected source and install prefix")
    artifacts_path = Path(build["artifacts_path"]).resolve(strict=True)
    artifacts = json.loads(artifacts_path.read_text(encoding="utf-8"))
    if (artifacts.get("package_prefix") != prefix or artifacts.get("version") != record["version"]
            or not artifacts.get("checks") or not all(artifacts["checks"].values())):
        raise ValueError(package + " safe-build artifact checks are incomplete or mismatched")


def _check_release_provenance(package: str, record: dict, provenance_path: Path) -> None:
    if digest(provenance_path) != record["release_provenance"]["evidence_sha256"]:
        raise ValueError(package + " release provenance evidence SHA-256 mismatch")
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    matches = [row for row in provenance["packages"] if row.get("package") == package]
    release = record["release_provenance"]
    if len(matches) != 1:
        raise ValueError(package + " release provenance record is missing or duplicated")
    source = matches[0]
    for key in ("url", "release_tag", "archive_sha256"):
        if source.get(key) != release.get(key):
            raise ValueError(package + " release provenance differs from the recorded evidence")
    if Path(source["source"]).resolve() != Path(record["source_root"]).resolve():
        raise ValueError(package + " release provenance source path differs from build source")


def _check_installed_record(package: str, record: dict) -> None:
    prefix = Path(record["prefix"]).resolve(strict=True)
    for relative, expected in record["installed_files"].items():
        rel = Path(relative)
        if rel.is_absolute() or ".." in rel.parts:
            raise ValueError(package + " installed-file path escapes prefix")
        path = (prefix / rel).resolve(strict=True)
        if prefix not in path.parents or digest(path) != expected:
            raise ValueError(package + " installed file identity mismatch: " + relative)
    package_xml = Path(record["package_xml"]).resolve(strict=True)
    if digest(package_xml) != record["package_xml_sha256"]:
        raise ValueError(package + " package.xml SHA-256 mismatch")
    if ET.parse(package_xml).getroot().findtext("version") != record["version"]:
        raise ValueError(package + " package version mismatch")
    for item in record["executables"]:
        executable = Path(item["path"]).resolve(strict=True)
        if prefix not in executable.parents or digest(executable) != item["sha256"]:
            raise ValueError(package + " executable identity mismatch: " + item["path"])
    for path, expected in record["libraries"].items():
        library = Path(path).resolve(strict=True)
        if prefix not in library.parents or digest(library) != expected:
            raise ValueError(package + " shared library identity mismatch: " + path)
    _check_source_manifest(package, record)
    _check_build_receipt(package, record)


def verify_profile(profile_path: str | Path = PROFILE, *, environment=None,
                   prefix_resolver=None, runner=subprocess.run) -> dict:
    env = dict(os.environ if environment is None else environment)
    selected_path = Path(profile_path).resolve(strict=True)
    if selected_path != PROFILE:
        raise ValueError("diagnostic profile must be the manifest-bound startup profile")
    profile = json.loads(selected_path.read_text(encoding="utf-8"))
    if profile.get("schema_version") != 1 or profile.get("profile_id") != "amcl-private-buffer-diagnostics-20261030":
        raise ValueError("unsupported AMCL diagnostic profile")

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    relative = selected_path.relative_to(NAV).as_posix()
    if manifest.get("files", {}).get(relative) != digest(selected_path):
        raise ValueError("AMCL diagnostic profile is not bound by STARTUP_MANIFEST.json")

    selection = profile["selection"]
    required_environment = {
        "CLEANNAV_AMCL_DIAGNOSTIC_PROFILE": str(selected_path),
        "CLEANNAV_TF_PRIVATE_DIAGNOSTICS": "1",
        "CLEANNAV_STARTUP_OVERLAY_SETUP": selection["startup_overlay_setup"],
        "CLEANNAV_RVIZ_OVERLAY_SETUP": selection["base_overlay_setup"],
        "CLEANNAV_RVIZ_MACHINE_CONFIG": selection["base_machine_config"],
        "ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "204", "ROS_LOCALHOST_ONLY": "1",
    }
    for key, expected in required_environment.items():
        if env.get(key) != expected:
            raise ValueError(f"{key} differs from reviewed AMCL diagnostic profile")
    for key in ("LD_PRELOAD", "LD_AUDIT"):
        if env.get(key):
            raise ValueError(key + " is not permitted for isolated AMCL diagnostics")
    for raw_path, expected in selection["setup_file_hashes"].items():
        path = Path(raw_path).resolve(strict=True)
        if digest(path) != expected:
            raise ValueError("reviewed setup input changed: " + str(path))
    config_path = Path(selection["base_machine_config"]).resolve(strict=True)
    if _canonical_config_sha256(config_path) != selection["base_machine_config_sha256_without_source_manifest"]:
        raise ValueError("reviewed base machine configuration changed outside its manifest binding")
    if env.get("CLEANNAV_AMCL_DIAG_BASE_LD_LIBRARY_PATH", "") != env.get("LD_LIBRARY_PATH", ""):
        raise ValueError("diagnostic overlay did not restore the approved base LD_LIBRARY_PATH")

    prefixes = {name: str(Path(value).resolve(strict=True))
                for name, value in profile["package_prefixes"].items()}
    expected_packages = {"nav2_amcl", "tf2_ros", "nav2_lifecycle_manager"}
    if set(prefixes) != expected_packages:
        raise ValueError("profile must bind exactly AMCL, tf2_ros, and Lifecycle Manager prefixes")
    for package, expected in prefixes.items():
        actual = str(Path((prefix_resolver or _default_prefix_resolver)(package)).resolve(strict=True))
        if actual != expected:
            raise ValueError(f"{package} resolved to {actual!r}, expected {expected!r}")
        markers = _package_markers(package, env.get("AMENT_PREFIX_PATH", ""))
        if markers != profile["allowed_package_markers"][package]:
            raise ValueError(f"{package} has unexpected same-name package markers: {markers!r}")

    for package, record in profile["packages"].items():
        if package not in ("nav2_amcl", "tf2_ros"):
            raise ValueError("only the diagnostic AMCL and tf2_ros packages may use isolated provenance")
        if prefixes[package] != str(Path(record["prefix"]).resolve(strict=True)):
            raise ValueError(package + " prefix differs inside package record")
        _check_installed_record(package, record)
        _check_release_provenance(package, record,
            Path(profile["source_provenance"]["path"]).resolve(strict=True))
    if set(profile["packages"]) != {"nav2_amcl", "tf2_ros"}:
        raise ValueError("diagnostic package provenance must bind AMCL and tf2_ros only")

    manager = profile["lifecycle_manager"]
    if prefixes["nav2_lifecycle_manager"] != str(Path(manager["prefix"]).resolve(strict=True)):
        raise ValueError("Lifecycle Manager prefix differs from its approved record")
    for name in ("package_xml", "executable"):
        path = Path(manager[name]).resolve(strict=True)
        if digest(path) != manager[name + "_sha256"]:
            raise ValueError("Lifecycle Manager " + name + " identity mismatch")
    if ET.parse(manager["package_xml"]).getroot().findtext("version") != manager["version"]:
        raise ValueError("Lifecycle Manager package version mismatch")
    manager_prefix = Path(manager["prefix"]).resolve(strict=True)
    if (Path(manager["package_xml"]).resolve() != manager_prefix / "share/nav2_lifecycle_manager/package.xml"
            or Path(manager["executable"]).resolve() != manager_prefix / "lib/nav2_lifecycle_manager/lifecycle_manager"):
        raise ValueError("Lifecycle Manager files are outside the approved package prefix")

    candidate_lib_dirs = [str(Path(profile["packages"][name]["prefix"]).resolve() / "lib")
                          for name in ("nav2_amcl", "tf2_ros")]
    base_library_path = env.get("LD_LIBRARY_PATH", "")
    if any(path in base_library_path.split(":") for path in candidate_lib_dirs):
        raise ValueError("diagnostic libraries must be private to the AMCL process")
    amcl_library_path = ":".join(candidate_lib_dirs + ([base_library_path] if base_library_path else []))
    amcl_env = dict(env, LD_LIBRARY_PATH=amcl_library_path,
                    CLEANNAV_TF_PRIVATE_DIAGNOSTICS="1")
    if profile["amcl_process"]["ld_library_path_prefix"] != candidate_lib_dirs:
        raise ValueError("AMCL private library path differs from package prefixes")

    nodes = profile["nodes"]
    if (nodes.get("amcl", {}).get("package") != "nav2_amcl"
            or nodes.get("amcl", {}).get("executable") not in {
                item["path"] for item in profile["packages"]["nav2_amcl"]["executables"]}
            or nodes.get("lifecycle_manager", {}).get("package") != "nav2_lifecycle_manager"
            or nodes.get("lifecycle_manager", {}).get("executable") != manager["executable"]):
        raise ValueError("node executable paths differ from their package identity records")
    resolved = {}
    for node_name, node in nodes.items():
        if node_name not in ("amcl", "lifecycle_manager"):
            raise ValueError("unexpected node in diagnostic runtime profile")
        exe = Path(node["executable"]).resolve(strict=True)
        if digest(exe) != node["executable_sha256"]:
            raise ValueError(node_name + " executable SHA-256 mismatch")
        node_env = amcl_env if node_name == "amcl" else env
        actual = _run_ldd(exe, node_env, runner)
        if actual != node["runtime_dependencies"]:
            raise ValueError(node_name + " runtime dependency closure differs from profile")
        resolved[node_name] = actual
    tf2_library = Path(profile["packages"]["tf2_ros"]["primary_library"]).resolve(strict=True)
    tf2_matches = [row for row in resolved["amcl"] if row["needed"] == "libtf2_ros.so"]
    if len(tf2_matches) != 1 or tf2_matches[0]["path"] != str(tf2_library):
        raise ValueError("AMCL does not resolve exactly one isolated libtf2_ros.so")
    for soname, expected_path in profile["amcl_process"]["required_libraries"].items():
        matches = [row for row in resolved["amcl"] if row["needed"] == soname]
        if len(matches) != 1 or matches[0]["path"] != str(Path(expected_path).resolve(strict=True)):
            raise ValueError("AMCL shared library resolved outside diagnostic profile: " + soname)
    tf2_deps = _run_ldd(tf2_library, amcl_env, runner)
    if tf2_deps != profile["tf2_ros_runtime_dependencies"]:
        raise ValueError("tf2_ros runtime dependency closure differs from profile")

    params = profile["amcl_parameters"]
    scan_wait = params["scan_tf_wait_timeout"]
    if not isinstance(scan_wait, (int, float)) or isinstance(scan_wait, bool) or not 0 < scan_wait <= 1:
        raise ValueError("scan_tf_wait_timeout must be finite and in (0, 1] seconds")
    params_path = Path(params["path"]).resolve(strict=True)
    contract_path = Path(params["map_contract"]).resolve(strict=True)
    if digest(params_path) != params["sha256"] or digest(contract_path) != params["map_contract_sha256"]:
        raise ValueError("profile-bound AMCL parameter or map contract SHA-256 mismatch")
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    binding = contract.get("source_bindings", {}).get("amcl_params_source", {})
    localization = contract.get("localization_runtime", {})
    expected_params = (Path(selection["expected_navigation_prefix"]).resolve()
                       / "share/cleannav_navigation/config/nav2_params_ackermann.yaml")
    if (binding.get("sha256") != params["sha256"] or Path(binding.get("path", "")).as_posix()
            != "cleannav_navigation/config/nav2_params_ackermann.yaml"
            or params_path != expected_params or localization.get("package") != "nav2_amcl"
            or localization.get("base_package") != "nav2_lifecycle_manager"
            or localization.get("global_frame_id") != "map"
            or localization.get("odom_frame_id") != "odom"
            or localization.get("base_frame_id") != "base_footprint"
            or localization.get("scan_topic") != "/mapping/scan"
            or localization.get("tf_broadcast") is not True):
        raise ValueError("AMCL parameter file is not bound by the selected localization contract")

    manager_path = str(Path(manager["executable"]).resolve())
    return {
        "status": "PASS", "profile_id": profile["profile_id"],
        "scope": "static identity and loader resolution; ROS/Gazebo dynamic NOT RUN",
        "profile_sha256": digest(selected_path), "package_prefixes": prefixes,
        "allowed_package_markers": profile["allowed_package_markers"],
        "amcl_executable": nodes["amcl"]["executable"],
        "amcl_executable_sha256": nodes["amcl"]["executable_sha256"],
        "tf2_ros_library": str(tf2_library),
        "tf2_ros_library_sha256": profile["packages"]["tf2_ros"]["libraries"][str(tf2_library)],
        "lifecycle_manager_executable": manager_path,
        "lifecycle_manager_executable_sha256": manager["executable_sha256"],
        "amcl_runtime_dependency_count": len(resolved["amcl"]),
        "lifecycle_manager_runtime_dependency_count": len(resolved["lifecycle_manager"]),
        "tf2_ros_runtime_dependency_count": len(tf2_deps), "runtime_dependencies": resolved,
        "localization_runtime_identity": {
            "package_prefix": prefixes["nav2_amcl"],
            "base_package_prefix": prefixes["nav2_lifecycle_manager"],
            "package_xml": profile["packages"]["nav2_amcl"]["package_xml"],
            "package_xml_sha256": profile["packages"]["nav2_amcl"]["package_xml_sha256"],
            "version": profile["packages"]["nav2_amcl"]["version"],
            "executable": nodes["amcl"]["executable"],
            "executable_sha256": nodes["amcl"]["executable_sha256"],
            "params": str(params_path), "params_sha256": params["sha256"],
        },
        "amcl_node_environment": {
            "LD_LIBRARY_PATH": amcl_library_path,
            "CLEANNAV_TF_PRIVATE_DIAGNOSTICS": "1",
        },
        "amcl_parameter_overrides": {"scan_tf_wait_timeout": scan_wait},
        "dynamic_entry_calls": 0,
    }


def verify_selected_profile(localization_mode: str, environment=None) -> dict | None:
    env = os.environ if environment is None else environment
    selected = select_profile(localization_mode, env)
    if selected is None:
        return None
    return verify_profile(selected, environment=env)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", help="write a PASS report to a new JSON path")
    args = parser.parse_args(argv)
    report = verify_profile()
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.report:
        with Path(args.report).open("x", encoding="utf-8") as stream:
            stream.write(text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
