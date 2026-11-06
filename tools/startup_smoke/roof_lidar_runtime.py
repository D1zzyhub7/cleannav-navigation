#!/usr/bin/env python3
"""Time-bounded stationary lidar/RTAB-Map probe; owns and cleans one launch group."""
from __future__ import annotations

import fcntl
import json
import math
import os
import re
import signal
import socket
import sqlite3
import subprocess
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath


def discover_workspace(anchor: Path) -> Path:
    configured=os.environ.get('CLEANNAV_WORKSPACE_ROOT')
    candidates=[Path(configured).expanduser().resolve()] if configured else list(Path(anchor).resolve().parents)
    for candidate in candidates:
        if ((candidate/'cleannav-build/locks/build.lock').is_file()
                and (candidate/'cleannav-build/locks/runtime.lock').is_file()):
            return candidate
    raise RuntimeError('CleanNav workspace root not found from repository path; set CLEANNAV_WORKSPACE_ROOT')

ROOT = discover_workspace(Path(__file__))
LOCK = ROOT / 'cleannav-build/locks/build.lock'
RUNTIME_LOCK = ROOT / 'cleannav-build/locks/runtime.lock'
PORT = 11704
DOMAIN = "204"
MAX_ACTIVE_FROM_T0 = 120.0
HARD_STOP_FROM_T0 = 150.0
TOTAL_LIMIT_FROM_T0 = 180.0
MIN_EFFECTIVE_OBSERVATION = 30.0
MEM_AVAILABLE_MIN_KIB = 4 * 1024 * 1024
SWAP_USED_MAX_KIB = 1024 * 1024
PSI_FULL_AVG10_REJECT_AT = 1.0
CGROUP_STARTUP_MIN_HEADROOM_BYTES = 5 * 1024 * 1024 * 1024
CGROUP_RUNTIME_MIN_HEADROOM_BYTES = 4 * 1024 * 1024 * 1024
WAIVER_STARTUP_MIN_AVAILABLE_KIB = 5 * 1024 * 1024
WAIVER_STABILITY_SECONDS = 30.0
WAIVER_STABILITY_INTERVAL_SECONDS = 1.0
CLEANUP_SIGINT_GRACE_SECONDS = 14.0
CLEANUP_SIGTERM_GRACE_SECONDS = 8.0
CLEANUP_SIGKILL_GRACE_SECONDS = 5.0
CLEANUP_POLL_SECONDS = 0.1
FAILURES: list[str] = []


class PreflightBlocked(Exception):
    """A fail-closed preflight result before any simulation process starts."""


def sha256(path: Path) -> str:
    import hashlib
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def read_meminfo() -> dict[str, int]:
    values = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        if ":" in line:
            key, rest = line.split(":", 1)
            values[key] = int(rest.split()[0])
    return values


def read_psi_avg10() -> float | None:
    path = Path("/proc/pressure/memory")
    if not path.is_file():
        return None
    for line in path.read_text().splitlines():
        if line.startswith("full "):
            fields = dict(item.split("=", 1) for item in line.split()[1:])
            return float(fields["avg10"])
    return None


def _unescape_mount_path(value: str) -> str:
    return re.sub(r"\\([0-7]{3})", lambda match: chr(int(match.group(1), 8)), value)


def _cgroup2_location(membership_file=Path("/proc/self/cgroup"),
                      mountinfo_file=Path("/proc/self/mountinfo")) -> dict:
    """Map this process's unified membership to the visible cgroup2 mount."""
    try:
        membership = next(
            line.split(":", 2)[2]
            for line in Path(membership_file).read_text().splitlines()
            if line.startswith("0::"))
        if ".." in PurePosixPath(membership).parts:
            raise ValueError("cgroup membership escapes its hierarchy")
        member_path = PurePosixPath(membership)
        mounts = []
        for line in Path(mountinfo_file).read_text().splitlines():
            left, separator, right = line.partition(" - ")
            if not separator:
                continue
            left_fields = left.split()
            right_fields = right.split()
            if len(left_fields) < 6 or not right_fields or right_fields[0] != "cgroup2":
                continue
            mount_root = PurePosixPath(_unescape_mount_path(left_fields[3]))
            if member_path != mount_root and mount_root not in member_path.parents:
                continue
            mountpoint = Path(_unescape_mount_path(left_fields[4]))
            relative = member_path.relative_to(mount_root)
            leaf = mountpoint.joinpath(*relative.parts)
            mount_options = set(left_fields[5].split(","))
            super_options = set(right_fields[2].split(",")) if len(right_fields) > 2 else set()
            mounts.append({
                "membership": membership,
                "mount_root": str(mount_root),
                "mountpoint": mountpoint,
                "leaf": leaf,
                "read_write": "rw" in mount_options or "rw" in super_options,
            })
        if not mounts:
            raise FileNotFoundError("no cgroup2 mount covers this process membership")
        return max(mounts, key=lambda item: len(PurePosixPath(item["mount_root"]).parts))
    except (OSError, StopIteration, ValueError) as error:
        return {"error": f"{type(error).__name__}: {error}"}


def cgroup_memory_snapshot(root=Path("/sys/fs/cgroup"),
                           membership_file=Path("/proc/self/cgroup"),
                           mountinfo_file=Path("/proc/self/mountinfo")) -> dict:
    """Return effective cgroup-v2 headroom, preserving missing data as UNKNOWN."""
    location = _cgroup2_location(membership_file, mountinfo_file)
    if "error" in location:
        return {"status": "UNKNOWN", "headroom_bytes": None,
                "source": None, "hierarchy": [], "reason": location["error"]}
    mountpoint = Path(location["mountpoint"])
    if mountpoint != Path(root):
        return {"status": "UNKNOWN", "headroom_bytes": None,
                "source": str(location["leaf"]), "hierarchy": [],
                "reason": f"cgroup2 mountpoint differs from expected root: {mountpoint}"}
    leaf = Path(location["leaf"])
    try:
        leaf.relative_to(mountpoint)
    except ValueError:
        return {"status": "UNKNOWN", "headroom_bytes": None,
                "source": str(leaf), "hierarchy": [],
                "reason": "mapped cgroup membership escapes the mounted hierarchy"}

    hierarchy = []
    directory = leaf
    while True:
        try:
            limit_text = (directory / "memory.max").read_text().strip()
            current = int((directory / "memory.current").read_text().strip())
            limit = None if limit_text == "max" else int(limit_text)
            if limit is not None and limit < 0:
                raise ValueError("negative memory.max")
        except (OSError, ValueError) as error:
            return {
                "status": "UNKNOWN", "headroom_bytes": None,
                "source": str(directory), "hierarchy": hierarchy,
                "reason": f"{directory}: {type(error).__name__}: {error}",
                "membership": location["membership"],
                "mapped_path": str(location["leaf"]),
                "mount_root": location["mount_root"],
                "mountpoint": str(mountpoint),
                "mount_read_write": location["read_write"],
            }
        hierarchy.append({
            "path": str(directory), "max_bytes": limit, "current_bytes": current,
            "headroom_bytes": None if limit is None else limit - current,
            "status": "UNLIMITED" if limit is None else "FINITE",
        })
        if directory == mountpoint:
            break
        if mountpoint not in directory.parents:
            return {"status": "UNKNOWN", "headroom_bytes": None,
                    "source": str(directory), "hierarchy": hierarchy,
                    "reason": "cgroup membership escaped the mounted hierarchy"}
        directory = directory.parent

    finite = [entry for entry in hierarchy if entry["status"] == "FINITE"]
    base = {"hierarchy": hierarchy, "membership": location["membership"],
            "mapped_path": str(leaf),
            "mount_root": location["mount_root"], "mountpoint": str(mountpoint),
            "mount_read_write": location["read_write"]}
    if finite:
        limiting = min(finite, key=lambda entry: entry["headroom_bytes"])
        return {**base, "status": "FINITE", "max_bytes": limiting["max_bytes"],
                "current_bytes": limiting["current_bytes"],
                "headroom_bytes": limiting["headroom_bytes"], "source": limiting["path"]}
    return {**base, "status": "UNLIMITED", "max_bytes": None,
            "current_bytes": hierarchy[0]["current_bytes"],
            "headroom_bytes": None, "source": str(leaf)}


def _systemd_slice_properties(unit, *, runner=subprocess.run):
    """Read the unit's authoritative cgroup path and configured memory limits."""
    result = runner(
        ["systemctl", "show", unit, "--no-pager", "-p", "LoadState", "-p",
         "ActiveState", "-p", "ControlGroup", "-p", "MemoryMax", "-p", "MemoryHigh"],
        capture_output=True, text=True, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"systemctl show {unit} exited {result.returncode}: "
                           f"{result.stderr.strip()[:200]}")
    properties = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            properties[key] = value
    required = ("LoadState", "ActiveState", "ControlGroup", "MemoryMax", "MemoryHigh")
    missing = [key for key in required if not properties.get(key)]
    if missing:
        raise ValueError(f"systemd properties missing for {unit}: {', '.join(missing)}")
    return properties


def wsl_cgroup_waiver_preflight(
        *, cgroup_root=Path("/sys/fs/cgroup"),
        membership_file=Path("/proc/self/cgroup"),
        mountinfo_file=Path("/proc/self/mountinfo"),
        kernel_release_file=Path("/proc/sys/kernel/osrelease"),
        pid1_comm_file=Path("/proc/1/comm"), uid=None,
        systemd_properties_fn=_systemd_slice_properties):
    """Qualify the prior narrow WSL2 exception without claiming headroom."""
    root = Path(cgroup_root)
    evidence = {"status": "NOT_ELIGIBLE", "eligible": False,
                "cgroup_memory_status": "UNKNOWN",
                "cgroup_memory_disposition": "NOT_WAIVED", "reason": None,
                "kernel_release": None, "pid1_comm": None, "membership": None,
                "cgroup2_mount": None, "root_memory_controls": {}, "visible_slices": {}}

    def reject(status, reason):
        evidence["status"] = status
        evidence["reason"] = reason
        return evidence

    try:
        release = Path(kernel_release_file).read_text().strip()
        pid1_comm = Path(pid1_comm_file).read_text().strip()
        membership_lines = Path(membership_file).read_text().splitlines()
        mount_lines = Path(mountinfo_file).read_text().splitlines()
    except OSError as error:
        return reject("UNKNOWN", f"WSL/cgroup identity evidence unavailable: {error}")
    evidence["kernel_release"] = release
    evidence["pid1_comm"] = pid1_comm
    if "microsoft-standard-wsl2" not in release.lower() or pid1_comm != "systemd":
        return reject("NOT_ELIGIBLE", "environment is not identified as systemd WSL2")

    try:
        membership = next(line.split(":", 2)[2] for line in membership_lines
                         if line.startswith("0::"))
    except StopIteration:
        return reject("UNKNOWN", "unified cgroup membership is unavailable")
    evidence["membership"] = membership
    if membership != "/":
        return reject("NOT_ELIGIBLE", "current shell is not at the visible cgroup2 root")

    cgroup_mount = None
    for line in mount_lines:
        left, separator, right = line.partition(" - ")
        if not separator:
            continue
        left_fields, right_fields = left.split(), right.split()
        if len(left_fields) < 6 or not right_fields or right_fields[0] != "cgroup2":
            continue
        mount_root = _unescape_mount_path(left_fields[3])
        mountpoint = _unescape_mount_path(left_fields[4])
        mount_options = set(left_fields[5].split(","))
        super_options = set(right_fields[2].split(",")) if len(right_fields) > 2 else set()
        if Path(mountpoint) == root:
            cgroup_mount = {"root": mount_root, "mountpoint": mountpoint,
                            "read_write": "rw" in mount_options or "rw" in super_options}
            break
    evidence["cgroup2_mount"] = cgroup_mount
    if not cgroup_mount or cgroup_mount["root"] != "/" or not cgroup_mount["read_write"]:
        return reject("NOT_ELIGIBLE", "cgroup2 root is not visibly mounted read-write")

    controls = ("memory.max", "memory.current", "memory.high", "memory.events")
    missing = []
    for name in controls:
        try:
            (root / name).read_text()
        except FileNotFoundError:
            missing.append(name)
            evidence["root_memory_controls"][name] = "MISSING"
        except OSError as error:
            evidence["root_memory_controls"][name] = f"UNKNOWN:{type(error).__name__}"
            return reject("UNKNOWN", f"cannot classify root cgroup control {name}: {error}")
        else:
            evidence["root_memory_controls"][name] = "PRESENT"
    if set(missing) != set(controls):
        return reject("NOT_ELIGIBLE", "root cgroup memory controls are not all absent")

    current_uid = os.getuid() if uid is None else int(uid)
    unknown_reasons = []
    ineligible_reasons = []
    for name in ("user.slice", f"user-{current_uid}.slice", "system.slice"):
        try:
            properties = systemd_properties_fn(name)
            if properties["LoadState"] != "loaded" or properties["ActiveState"] != "active":
                raise ValueError(f"systemd unit is not loaded and active: "
                                 f"{properties['LoadState']}/{properties['ActiveState']}")
            control_group = PurePosixPath(properties["ControlGroup"])
            if (not control_group.is_absolute() or ".." in control_group.parts
                    or len(control_group.parts) < 2):
                raise ValueError(f"invalid systemd ControlGroup path: {properties['ControlGroup']!r}")
            directory = root.joinpath(*control_group.parts[1:])
            directory.relative_to(root)
            max_value = (directory / "memory.max").read_text().strip()
            high_value = (directory / "memory.high").read_text().strip()
            events = {}
            for line in (directory / "memory.events").read_text().splitlines():
                key, value = line.split(maxsplit=1)
                events[key] = int(value)
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            evidence["visible_slices"][name] = {
                "status": "UNKNOWN", "reason": f"{type(error).__name__}: {error}"}
            unknown_reasons.append(f"visible slice evidence unavailable: {name}")
            continue
        evidence["visible_slices"][name] = {
            "systemd_properties": properties, "control_group_path": str(control_group),
            "filesystem_path": str(directory), "memory.max": max_value,
            "memory.high": high_value,
            "oom_events": {key: value for key, value in events.items()
                           if key.startswith("oom")}}
        if properties["MemoryMax"] != "infinity" or properties["MemoryHigh"] != "infinity":
            ineligible_reasons.append(f"systemd memory cap found in {name}")
        if (properties["MemoryMax"] == "infinity") != (max_value == "max"):
            unknown_reasons.append(f"systemd and memory.max disagree in {name}")
        if (properties["MemoryHigh"] == "infinity") != (high_value == "max"):
            unknown_reasons.append(f"systemd and memory.high disagree in {name}")
        oom_events = {key: value for key, value in events.items() if key.startswith("oom")}
        if max_value != "max" or high_value != "max":
            ineligible_reasons.append(f"numeric memory cap found in {name}")
        if not {"oom", "oom_kill"}.issubset(oom_events):
            unknown_reasons.append(f"OOM counters are incomplete in {name}")
        if any(value != 0 for value in oom_events.values()):
            ineligible_reasons.append(f"non-zero OOM event found in {name}")

    if ineligible_reasons:
        return reject("NOT_ELIGIBLE", "; ".join(ineligible_reasons))
    if unknown_reasons:
        return reject("UNKNOWN", "; ".join(unknown_reasons))

    evidence.update({
        "status": "WAIVER_ELIGIBLE", "eligible": True,
        "cgroup_memory_status": "UNKNOWN", "cgroup_memory_disposition": "WAIVED",
        "reason": "WSL2 root controls are absent; visible slices are uncapped with zero OOM counters",
    })
    return evidence


def cgroup_startup_gate(cgroup, waiver, *, stable_window=None, auxiliary_gates_ok=False):
    """Apply existing startup limits; UNKNOWN passes only with complete exception evidence."""
    status = cgroup.get("status", "UNKNOWN")
    if status == "UNLIMITED":
        return {"ok": True, "status": "VERIFIED", "cgroup_memory_status": status,
                "cgroup_memory_disposition": "NOT_WAIVED", "reason": None}
    if status == "FINITE":
        headroom = cgroup.get("headroom_bytes")
        ok = headroom is not None and headroom >= CGROUP_STARTUP_MIN_HEADROOM_BYTES
        return {"ok": ok, "status": "VERIFIED" if ok else "FAILED",
                "cgroup_memory_status": status, "cgroup_memory_disposition": "NOT_WAIVED",
                "reason": None if ok else "cgroup startup headroom is below 5 GiB or unavailable"}
    exception_ready = (
        waiver.get("eligible") is True and waiver.get("status") == "WAIVER_ELIGIBLE"
        and waiver.get("cgroup_memory_status") == "UNKNOWN"
        and waiver.get("cgroup_memory_disposition") == "WAIVED")
    window_ok = bool(stable_window and stable_window.get("status") == "VERIFIED"
                     and stable_window.get("actual_duration_seconds", 0) >= WAIVER_STABILITY_SECONDS
                     and stable_window.get("sample_count", 0) >= 31)
    allowed = exception_ready and window_ok and auxiliary_gates_ok
    reason = None
    if not exception_ready:
        reason = waiver.get("reason") or cgroup.get("reason") or "cgroup headroom is UNKNOWN"
    elif not window_ok:
        reason = "required 30-second resource stability window is not verified"
    elif not auxiliary_gates_ok:
        reason = "process, locks, port, or ROS Domain checks are not all verified"
    return {"ok": allowed, "status": "PASS_WITH_EXCEPTION" if allowed else "UNKNOWN",
            "cgroup_memory_status": "UNKNOWN",
            "cgroup_memory_disposition": "WAIVED" if allowed else "NOT_WAIVED",
            "reason": reason, "exception_evidence": waiver}


def collect_resource_stability_window(sample_fn, *, duration=WAIVER_STABILITY_SECONDS,
                                      interval=WAIVER_STABILITY_INTERVAL_SECONDS,
                                      clock=time.monotonic, sleeper=time.sleep):
    started = clock()
    deadline = started + duration
    samples = []
    while True:
        sample = sample_fn()
        samples.append(sample)
        if not sample.get("ok"):
            return {"status": "FAILED", "sample_count": len(samples),
                    "actual_duration_seconds": round(clock() - started, 3),
                    "failed_sample": sample, "samples": samples}
        elapsed = clock() - started
        if elapsed >= duration and len(samples) >= int(duration / interval) + 1:
            return {"status": "VERIFIED", "sample_count": len(samples),
                    "actual_duration_seconds": round(elapsed, 3),
                    "min_mem_available_kib": min(
                        item.get("MemAvailable_kib", 0) for item in samples),
                    "max_swap_used_kib": max(item.get("SwapUsed_kib", 0) for item in samples),
                    "max_memory_full_psi_avg10": max(
                        item.get("memory_full_psi_avg10", 0.0) for item in samples),
                    "samples": samples}
        remaining = deadline - clock()
        if remaining <= 0:
            return {"status": "UNKNOWN", "sample_count": len(samples),
                    "actual_duration_seconds": round(clock() - started, 3),
                    "reason": "stability window ended without the required sample count",
                    "samples": samples}
        sleeper(min(interval, remaining))


def memory_snapshot(*, cgroup_waiver=None, min_available_kib=MEM_AVAILABLE_MIN_KIB,
                    min_cgroup_headroom_bytes=CGROUP_RUNTIME_MIN_HEADROOM_BYTES,
                    memavailable_warning_only=False) -> dict:
    values = read_meminfo()
    available = values.get("MemAvailable")
    swap_total = values.get("SwapTotal")
    swap_free = values.get("SwapFree")
    swap_used = None if swap_total is None or swap_free is None else swap_total - swap_free
    psi = read_psi_avg10()
    cgroup = cgroup_memory_snapshot()
    waiver_active = bool(cgroup_waiver and cgroup_waiver.get("eligible") is True
                         and cgroup_waiver.get("status") == "WAIVER_ELIGIBLE"
                         and cgroup_waiver.get("cgroup_memory_disposition") == "WAIVED")
    cgroup_ok = cgroup["status"] == "UNLIMITED"
    if cgroup["status"] == "FINITE":
        cgroup_ok = cgroup.get("headroom_bytes") is not None and (
            cgroup["headroom_bytes"] >= min_cgroup_headroom_bytes)
    elif cgroup["status"] == "UNKNOWN":
        cgroup_ok = waiver_active
    reasons = []
    if available is None or swap_used is None or psi is None:
        reasons.append("memory_or_psi_unavailable")
    if available is not None and available < min_available_kib and not memavailable_warning_only:
        reasons.append(f"MemAvailable_below_{min_available_kib // (1024 * 1024)}GiB")
    if swap_used is not None and swap_used > SWAP_USED_MAX_KIB:
        reasons.append("swap_used_above_1GiB")
    if psi is not None and psi >= PSI_FULL_AVG10_REJECT_AT:
        reasons.append("memory_full_PSI_avg10_at_least_1")
    if not cgroup_ok:
        if cgroup["status"] == "FINITE":
            reasons.append("cgroup_headroom_below_runtime_threshold")
        else:
            reasons.append("cgroup_memory_headroom_unknown_or_unwaived")
    return {
        "ok": not reasons,
        "MemAvailable_kib": available,
        "MemAvailable_warning": available is not None and available < min_available_kib,
        "MemAvailable_warning_only": memavailable_warning_only,
        "SwapTotal_kib": swap_total,
        "SwapFree_kib": swap_free,
        "SwapUsed_kib": swap_used,
        "SwapCached_kib": values.get("SwapCached"),
        "memory_full_psi_avg10": psi,
        "cgroup_memory_status": cgroup["status"],
        "cgroup_memory_disposition": "WAIVED" if cgroup["status"] == "UNKNOWN" and waiver_active else "NOT_WAIVED",
        "cgroup_memory_headroom_bytes": cgroup.get("headroom_bytes"),
        "cgroup_memory_source": cgroup.get("source"),
        "cgroup_memory_reason": cgroup.get("reason"),
        "cgroup_memory_hierarchy": cgroup.get("hierarchy", []),
        "reasons": reasons,
    }


def process_visibility_and_conflicts() -> dict:
    pid1_comm = Path("/proc/1/comm").read_text().strip()
    pid1_cgroup = Path("/proc/1/cgroup").read_text()
    visible = not pid1_comm.lower().startswith("codex") and "/init.scope" in pid1_cgroup
    blockers = []
    blocked_names = {
        "gzserver", "gzclient", "gazebo", "rviz2", "rtabmap", "colcon", "cmake",
        "make", "gmake", "ninja", "cc1plus", "cc1", "ros2",
    }
    self_pid = os.getpid()
    parents = {self_pid, os.getppid()}
    cursor = os.getppid()
    for _ in range(16):
        try:
            stat = Path(f"/proc/{cursor}/stat").read_text()
            end = stat.rfind(")")
            ppid = int(stat[end + 2:].split()[1])
        except (OSError, ValueError, IndexError):
            break
        if cursor <= 1 or cursor in parents:
            break
        parents.add(cursor)
        cursor = ppid
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) in parents:
            continue
        try:
            comm = (entry / "comm").read_text().strip()
            cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if comm in blocked_names:
            blockers.append({"pid": int(entry.name), "comm": comm, "cmdline": cmdline[:240]})
        elif any(token in cmdline for token in ("ros2 launch", "run_mapping.sh", "simulation_runner", "colcon build")):
            blockers.append({"pid": int(entry.name), "comm": comm, "cmdline": cmdline[:240]})
    return {"pid1_comm": pid1_comm, "pid1_cgroup": pid1_cgroup.strip(),
            "full_process_visibility": visible, "conflicting_processes": blockers,
            "ok": visible and not blockers}


def port_free(port: int) -> dict:
    results = {}
    for kind, socktype in (("tcp", socket.SOCK_STREAM), ("udp", socket.SOCK_DGRAM)):
        try:
            sock = socket.socket(socket.AF_INET, socktype)
            try:
                sock.bind(("127.0.0.1", port))
                results[kind] = True
            finally:
                sock.close()
        except OSError as error:
            results[kind] = False
            results[f"{kind}_error"] = f"{type(error).__name__}: {error}"
    results["ok"] = results["tcp"] and results["udp"]
    return results


def check_package_prefixes() -> dict:
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        expected_environment = {
            "cleannav_simulation": "CLEANNAV_EXPECTED_SIMULATION_PREFIX",
            "cleannav_rtabmap": "CLEANNAV_EXPECTED_RTABMAP_PREFIX",
        }
        values = {}
        for package in ("cleannav_simulation", "cleannav_rtabmap", "cleannav_safety_supervisor"):
            prefix=Path(get_package_prefix(package)).resolve()
            share=Path(get_package_share_directory(package)).resolve()
            expected_text = os.environ.get(expected_environment.get(package, ""))
            expected = Path(expected_text).expanduser().resolve() if expected_text else None
            values[package]={"prefix":str(prefix),"share":str(share),
                "share_exists":share.is_dir(),
                "expected_prefix":str(expected) if expected else None,
                "prefix_matches_expected":prefix == expected if expected else False}
        values["simulation"]=values["cleannav_simulation"]["prefix"]
        values["rtabmap"]=values["cleannav_rtabmap"]["prefix"]
        required = ("cleannav_simulation", "cleannav_rtabmap", "cleannav_safety_supervisor")
        values["ok"] = all(values[package]["share_exists"] for package in required) and all(
            values[package]["prefix_matches_expected"] for package in expected_environment
        )
        return values
    except Exception as error:
        return {"ok": False, "error": repr(error)}


def acquire_locks():
    """Hold the established build and runtime locks for the full test lifetime."""
    descriptors = []
    records = []
    try:
        for path in (LOCK, RUNTIME_LOCK):
            if not path.is_file():
                raise RuntimeError(f"required exclusive lock file missing: {path}")
            fd = os.open(path, os.O_RDWR)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except Exception:
                os.close(fd)
                raise RuntimeError(f"required build/runtime lock is held: {path}")
            descriptors.append(fd)
            records.append({"path": str(path), "exclusive": True})
        return descriptors, records
    except Exception:
        for fd in descriptors:
            os.close(fd)
        raise


def check_domain_isolated(rclpy, node) -> dict:
    deadline = time.monotonic() + 4.0
    observed = set()
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        observed.update(node.get_node_names_and_namespaces())
        time.sleep(0.1)
    peers = sorted(f"{name}@{namespace}" for name, namespace in observed
                   if name != node.get_name())
    return {"ok": not peers, "domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY"), "peer_nodes": peers}


def process_group_snapshot(pgid: int, proc_root=Path("/proc")) -> dict:
    """Inspect the owned process group and distinguish zombies from live work."""
    members = []
    errors = []
    try:
        entries = list(Path(proc_root).iterdir())
    except OSError as error:
        return {"status": "UNKNOWN", "scan_complete": False, "members": [],
                "running_members": [], "zombie_members": [],
                "errors": [f"{type(error).__name__}: {error}"]}
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            stat_text = (entry / "stat").read_text()
        except FileNotFoundError:
            continue
        except OSError as error:
            errors.append({"pid": int(entry.name), "error": f"{type(error).__name__}: {error}"})
            continue
        end = stat_text.rfind(")")
        if end < 0:
            errors.append({"pid": int(entry.name), "error": "malformed /proc stat comm field"})
            continue
        fields = stat_text[end + 2:].split()
        try:
            state = fields[0]
            process_group = int(fields[2])
        except (IndexError, ValueError):
            errors.append({"pid": int(entry.name), "error": "malformed /proc stat fields"})
            continue
        if process_group == pgid:
            members.append({"pid": int(entry.name), "state": state, "process_group": process_group})
    running = [item for item in members if item["state"] not in ("Z", "X")]
    zombies = [item for item in members if item["state"] == "Z"]
    complete = not errors
    if not complete:
        status = "UNKNOWN"
    elif running:
        status = "ALIVE"
    elif zombies:
        status = "ZOMBIES_ONLY"
    else:
        status = "EMPTY"
    return {"status": status, "scan_complete": complete, "members": members,
            "running_members": running, "zombie_members": zombies, "errors": errors}


def _wait_for_group_and_leader(proc, pgid, deadline, *, snapshot_fn, clock, sleeper,
                               wait_fn):
    snapshot = {"status": "UNKNOWN", "scan_complete": False,
                "members": [], "running_members": [], "zombie_members": [],
                "errors": ["process group has not been inspected"]}
    while True:
        code = proc.poll()
        snapshot = snapshot_fn(pgid)
        if code is not None and snapshot.get("scan_complete") is True \
                and not snapshot.get("running_members"):
            return snapshot, True
        remaining = deadline - clock()
        if remaining <= 0:
            return snapshot, False
        interval = min(CLEANUP_POLL_SECONDS, remaining)
        if code is None:
            try:
                wait_fn(proc, interval)
            except subprocess.TimeoutExpired:
                pass
        else:
            sleeper(interval)


def signal_and_reap(proc: subprocess.Popen | None, run_started: bool, t0: float, *,
                    snapshot_fn=process_group_snapshot, killpg_fn=os.killpg,
                    clock=time.monotonic, sleeper=time.sleep, wait_fn=None,
                    sigint_grace=CLEANUP_SIGINT_GRACE_SECONDS,
                    sigterm_grace=CLEANUP_SIGTERM_GRACE_SECONDS,
                    sigkill_grace=CLEANUP_SIGKILL_GRACE_SECONDS) -> dict:
    if proc is None:
        return {"started": False, "cleanup_ok": True, "launch_exit_code": None}
    cleanup_started = clock()
    events = []
    signal_records = []
    cleanup_phases = []
    wait_fn = wait_fn or (lambda child, timeout: child.wait(timeout=timeout))

    def timestamp():
        return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")

    def record_signal(sig, target, label, before):
        events.append(label)
        signal_records.append({
            "signal": signal.Signals(sig).name, "target": target,
            "wall_time_utc": timestamp(), "t0_elapsed_s": round(clock() - t0, 3),
            "launch_returncode_before": proc.poll(),
            "process_group_snapshot_before": before,
        })

    def wait_phase(name, grace, before):
        phase_started = clock()
        started_utc = timestamp()
        snapshot, settled = _wait_for_group_and_leader(
            proc, proc.pid, phase_started + grace, snapshot_fn=snapshot_fn,
            clock=clock, sleeper=sleeper, wait_fn=wait_fn)
        cleanup_phases.append({
            "phase": name, "started_at_utc": started_utc,
            "ended_at_utc": timestamp(),
            "t0_elapsed_start_s": round(phase_started - t0, 3),
            "t0_elapsed_end_s": round(clock() - t0, 3),
            "grace_seconds": grace, "settled": settled,
            "launch_returncode_after": proc.poll(),
            "process_group_snapshot_before": before,
            "process_group_snapshot_after": snapshot,
        })
        return snapshot, settled

    if proc.poll() is None:
        before = snapshot_fn(proc.pid)
        try:
            proc.send_signal(signal.SIGINT)
            record_signal(signal.SIGINT, "launch_supervisor", "SIGINT_launch_supervisor", before)
        except ProcessLookupError:
            events.append("SIGINT_launch_supervisor_process_missing")
        snapshot, settled = wait_phase("wait_after_SIGINT", sigint_grace, before)
    else:
        snapshot = snapshot_fn(proc.pid)
        settled = snapshot.get("scan_complete") is True and not snapshot.get("running_members")

    def send_group(sig, label):
        current = snapshot_fn(proc.pid)
        if current.get("scan_complete") is True and not current.get("running_members"):
            return False
        if current.get("scan_complete") is not True:
            events.append(f"{label}_not_sent_process_group_state_unknown")
            return False
        try:
            killpg_fn(proc.pid, sig)
            record_signal(sig, "owned_process_group", label, current)
            return True
        except ProcessLookupError:
            return False
        except PermissionError as error:
            events.append(f"{label}_permission_error:{error.errno}")
            return False

    if not settled:
        if "SIGINT_launch_supervisor" in events:
            events.append("launch_or_group_did_not_exit_after_SIGINT_grace")
        else:
            events.append("owned_group_outlived_launch_supervisor")
        if send_group(signal.SIGTERM, "SIGTERM_owned_process_group"):
            before = signal_records[-1]["process_group_snapshot_before"]
            snapshot, settled = wait_phase("wait_after_SIGTERM", sigterm_grace, before)
        else:
            snapshot = snapshot_fn(proc.pid)
            settled = (proc.poll() is not None and snapshot.get("scan_complete") is True
                       and not snapshot.get("running_members"))

    if not settled:
        events.append("owned_group_did_not_exit_after_SIGTERM_grace")
        if send_group(signal.SIGKILL, "SIGKILL_owned_process_group"):
            before = signal_records[-1]["process_group_snapshot_before"]
            snapshot, settled = wait_phase("wait_after_SIGKILL", sigkill_grace, before)
        else:
            snapshot = snapshot_fn(proc.pid)
            settled = (proc.poll() is not None and snapshot.get("scan_complete") is True
                       and not snapshot.get("running_members"))

    # Reap the direct child even if the process-group snapshot has already settled.
    code = proc.poll()
    reap_started = clock()
    reap_started_utc = timestamp()
    reap_wait_attempted = False
    if code is None:
        try:
            reap_wait_attempted = True
            code = wait_fn(proc, 0.5)
        except subprocess.TimeoutExpired:
            events.append("launch_process_did_not_reap")
            code = proc.poll()
    snapshot = snapshot_fn(proc.pid)
    if snapshot.get("scan_complete") is not True:
        group_alive = None
        group_state = "UNKNOWN"
    else:
        group_alive = bool(snapshot.get("running_members"))
        group_state = snapshot.get("status", "UNKNOWN")
    code = proc.poll()
    return {"started": run_started, "signals": events, "launch_exit_code": code,
            "signal_records": signal_records, "cleanup_phases": cleanup_phases,
            "launch_main_reaped": code is not None,
            "launch_main_reap_check": {
                "started_at_utc": reap_started_utc, "ended_at_utc": timestamp(),
                "t0_elapsed_start_s": round(reap_started - t0, 3),
                "t0_elapsed_end_s": round(clock() - t0, 3),
                "wait_attempted": reap_wait_attempted, "returncode": code},
            "process_group_state_after_cleanup": group_state,
            "process_group_snapshot_after_cleanup": snapshot,
            "process_group_snapshot_after_cleanup_at_utc": timestamp(),
            "process_group_alive_after_cleanup": group_alive,
            "cleanup_ok": code is not None and group_alive is False
                          and snapshot.get("scan_complete") is True,
            "elapsed_from_T0_s": round(clock() - t0, 2),
            "cleanup_elapsed_s": round(clock() - cleanup_started, 2)}



# This module intentionally exposes only shared preflight and cleanup primitives.
