#!/usr/bin/env python3
"""Fail-closed supervisor for no-motion and explicit one-goal RViz sessions.

The supported public caller is run_rviz_navigation.sh. Motion remains opt-in
through the run-bound short-goal profile and is admitted only after the
existing Safety, Collision Monitor, Mission, and command-chain checks pass.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import traceback

HERE = Path(__file__).resolve().parent
NAV = HERE.parents[1]
sys.path.insert(0, str(NAV / "tools/startup_smoke"))
import roof_lidar_runtime as runtime
from publisher_graph_gate import (
    PublisherGraphConvergence,
    endpoint_record,
)
from supervisor_policy import (
    LOCALIZATION_DIAGNOSTIC, SHORT_GOAL_MOTION, HISTORICAL_MOTION, TIMING_PROFILE, MIN_LAUNCH_OBSERVATION_S,
    MAX_LAUNCH_RUNTIME_S, HISTORICAL_GUI_GOAL_WAIT_S, HISTORICAL_TASK_TIMEOUT_S,
    PREFLIGHT_LATEST_LAUNCH_S, timing_identity,
    ACCOUNTING_LIMIT_FROM_T0_S,
    CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S,
    HARD_PROCESS_CUTOFF_FROM_T0_S,
    MIN_CLEANUP_RESERVE_S,
    MOTION_ADMISSION_STABLE_S,
    MAX_MOTION_SESSION_DISPLACEMENT_M,
    MAX_HISTORICAL_ODOMETRY_M, MAX_HISTORICAL_DISPLACEMENT_M,
    NO_MOTION_OBSERVATION_S,
    MAX_NO_MOTION_DISPLACEMENT_M,
    PHASES,
    READINESS_DEADLINE_FROM_T0_S,
    SOFT_STOP_FROM_T0_S,
    ContractError,
    admitted_safety_receipt_decision,
    observe_safety_sample,
    PhaseLedger,
    cleanup_is_verified,
    cpu_usage_percent,
    diagnostic_readiness_loss_decision,
    deadline_stop_reason,
    final_status,
    gazebo_entity_state_from_response,
    guard_receipt_snapshot_status,
    linux_cpu_counters,
    no_motion_startup_qualifies,
    idle_command_baseline_ok,
    motion_admission_decision,
    motion_completion_decision,
    phase_readiness,
    format_navigation_ready,
    format_navigation_not_ready,
    raw_publisher_phase_contract,
    readiness_loss_record,
    runtime_memory_decision,
    validate_environment,
    validate_guard_state,
    validate_motion_guard_state,
    validate_motion_guard_runtime_state,
    verify_no_motion_launch_argv,
    verify_short_goal_launch_argv,
    validate_zero_command,
    zero_command_observation_ok,
)
from short_goal_policy import (
    LEASE_RELEASE_CONFIRMATION_S,
    MAX_AUTHORITY_TTL_S,
    MAX_GOAL_RUNTIME_S,
    MAX_GUI_GOAL_WAIT_S,
)

sys.path.insert(0, str(HERE))
import integration
from process_memory_telemetry import ProcessMemoryTelemetry

OUTPUT_ROOT = Path(os.environ.get(
    "CLEANNAV_RVIZ_OUTPUT_ROOT", runtime.ROOT / "cleannav-deliverables/rviz-navigation-runs"
)).expanduser().resolve()
MODEL_ASSETS = runtime.ROOT.parent / "cleannav_demo_assets/osrf_car_demo/car_demo"
ROS_SETUP = Path("/opt/ros/humble/setup.bash")
ENTRY = HERE / "rviz_navigation.launch.py"
DEPENDENCIES = json.loads((HERE / "RUNTIME_DEPENDENCIES.json").read_text())
PREFIX_ENVIRONMENT_KEYS = tuple(
    binding["prefix_env"] for binding in DEPENDENCIES["cleannav_packages"].values()
)
ENVIRONMENT_VARIABLE_KEYS = (
    "ROS_DISTRO", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "AMENT_PREFIX_PATH",
    "COLCON_PREFIX_PATH", "GAZEBO_MASTER_URI", "GAZEBO_IP",
    *PREFIX_ENVIRONMENT_KEYS,
)
STOP_REQUESTED: str | None = None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def environment_variable_snapshot() -> dict[str, str | None]:
    """Capture the exact custom prefixes declared by the runtime dependency contract."""
    return {key: os.environ.get(key) for key in ENVIRONMENT_VARIABLE_KEYS}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def atomic_json(path: Path, value: dict) -> None:
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True, default=str) + "\n")
    temporary.replace(path)


def pressure_snapshot(proc_root: Path = Path('/proc')) -> dict:
    """Read CPU, memory, and I/O PSI with missing interfaces left UNKNOWN."""
    result = {}
    for resource in ('cpu', 'memory', 'io'):
        path = Path(proc_root) / 'pressure' / resource
        try:
            lines = path.read_text().splitlines()
            values = {}
            for line in lines:
                fields = line.split()
                if not fields or fields[0] not in ('some', 'full'):
                    continue
                row = {}
                for field in fields[1:]:
                    key, separator, value = field.partition('=')
                    if separator:
                        row[key] = float(value) if key.startswith('avg') else int(value)
                values[fields[0]] = row
            result[resource] = {'status': 'AVAILABLE', 'values': values}
        except (OSError, ValueError) as error:
            result[resource] = {'status': 'UNKNOWN',
                                'reason': f'{type(error).__name__}: {error}'}
    return result


def write_operator_notice(text: str, streams=None) -> None:
    """Write an operator line to the launch terminal, then flush it."""
    owned = []
    if streams is None:
        streams = [sys.stdout]
        if not sys.stdout.isatty():
            try:
                terminal = open("/dev/tty", "w", buffering=1)
            except OSError:
                terminal = None
            if terminal is not None:
                streams = [terminal, sys.stdout]
                owned.append(terminal)
    try:
        for stream in streams:
            stream.write(text)
            stream.flush()
    finally:
        for stream in owned:
            stream.close()


def write_checksums(directory: Path) -> list[dict]:
    rows = []
    for path in sorted(Path(directory).rglob("*")):
        if not path.is_file() or path.name == "SHA256SUMS" or path.name.endswith(".tmp"):
            continue
        rows.append({"path": path.relative_to(directory).as_posix(), "sha256": sha256(path)})
    (Path(directory) / "SHA256SUMS").write_text(
        "".join(f"{row['sha256']}  {row['path']}\n" for row in rows))
    return rows


def wrapper_failure(args: argparse.Namespace) -> int:
    """Write a non-success receipt when environment setup fails before Python run."""
    run_dir = Path(args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    try:
        wrapper = json.loads((run_dir / "wrapper.json").read_text())
    except (OSError, ValueError):
        wrapper = {"run_id": args.run_id, "run_dir": str(run_dir), "t0_monotonic_ns": args.t0_ns}
    result = {
        **wrapper,
        "status": "BLOCKED",
        "launch_started": False,
        "launch_exit_code": None,
        "failure_stage": "environment_setup",
        "failure": args.reason or "ROS/overlay setup failed before supervisor admission",
        "phases": {name: "NOT RUN" for name in PHASES},
        "cleanup": {"status": "NOT_STARTED", "cleanup_ok": None,
                    "reason": "no launch process was created"},
        "hard_deadline_enforced": False,
    }
    atomic_json(run_dir / "cleanup.json", result["cleanup"])
    atomic_json(run_dir / "environment.json", {
        "status": "FAILED_BEFORE_SUPERVISOR",
        "failure": result["failure"],
        "ros_setup": str(ROS_SETUP),
        "overlay_setup_argument": wrapper.get("overlay_setup_argument"),
    })
    atomic_json(run_dir / "source_identity.json", {
        "status": "PARTIAL", "navigation_worktree": wrapper.get("navigation_worktree"),
        "workspace_root": wrapper.get("workspace_root"),
        "map_directory_argument": wrapper.get("map_directory_argument"),
        "world_argument": wrapper.get("world_argument"),
        "map_profile": wrapper.get("map_profile"),
        "odometry_mode": wrapper.get("odometry_mode", "wheel"),
    })
    atomic_json(run_dir / "events.json", {"events": [{
        "event": "wrapper_setup_failed", "utc": utc_now(), "reason": result["failure"]}]})
    atomic_json(run_dir / "result.json", result)
    write_checksums(run_dir)
    return 2


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--t0-ns", required=True, type=int)
    parser.add_argument("--map-profile", default="")
    parser.add_argument("--map-directory", default="")
    parser.add_argument("--world", default="")
    parser.add_argument("--overlay-setup", default="")
    parser.add_argument("--overlay-before-sha256", default="")
    parser.add_argument("--overlay-after-sha256", default="")
    parser.add_argument("--motion-profile", choices=("none", "short-goal", "historical", "patrol"), default="none")
    parser.add_argument("--odometry-mode", choices=("wheel", "oracle"), default="wheel")
    parser.add_argument("--record-wrapper-failure", action="store_true")
    parser.add_argument("--record-watchdog-failure", action="store_true")
    parser.add_argument("--record-watchdog-exit", action="store_true")
    parser.add_argument("--watchdog-pid", type=int, default=0)
    parser.add_argument("--watchdog-exit-code", type=int, default=0)
    parser.add_argument("--supervisor-exit-code", type=int, default=0)
    parser.add_argument("--reason", default="")
    args = parser.parse_args(argv)
    if not re.fullmatch(r"rviz-nav-[0-9]{8}T[0-9]{6}Z-[0-9a-f]{8}", args.run_id):
        parser.error("run-id must be a generated rviz-nav-<UTC>-<uuid8> identifier")
    if args.record_wrapper_failure or args.record_watchdog_failure or args.record_watchdog_exit:
        return args
    if args.map_profile not in integration.PROFILE_CONTRACTS:
        parser.error("map-profile must select an allow-listed map profile")
    if not args.map_directory or not args.world or not args.overlay_setup:
        parser.error("map-profile, map-directory, world, and overlay-setup are required for a run")
    if args.t0_ns <= 0:
        parser.error("t0-ns must be a positive monotonic timestamp")
    if (args.motion_profile in ("historical", "patrol")) != HISTORICAL_MOTION:
        parser.error("historical motion selection disagrees with timing identity")
    if (args.motion_profile == "short-goal") != SHORT_GOAL_MOTION:
        parser.error("supervisor motion profile does not match its outer timing profile identity")
    return args


def proc_start_ticks(pid: int) -> int:
    text = (Path("/proc") / str(pid) / "stat").read_text()
    end = text.rfind(")")
    return int(text[end + 2:].split()[19])


RUN_DIRECTORY_INITIAL_FILES = frozenset({
    "wrapper.json", "environment-setup.log", "watchdog_ready.json",
    "supervisor_state.json", "watchdog.log", "supervisor.log",
})
RUN_DIRECTORY_TERMINAL_FILES = frozenset({
    "cleanup.json", "result.json", "supervisor_complete.json", "watchdog_exit.json",
    "hard_deadline.json", "watchdog_failure.json", "supervisor_failure.json",
})


def process_identity(pid: int) -> dict | None:
    """Return the live PID identity fields used by the startup receipt contract."""
    try:
        text = (Path("/proc") / str(pid) / "stat").read_text()
        end = text.rfind(")")
        fields = text[end + 2:].split()
        return {
            "pid": int(pid), "state": fields[0], "pgid": int(fields[2]),
            "session_id": os.getsid(pid), "start_ticks": int(fields[19]),
        }
    except (OSError, ValueError, IndexError, ProcessLookupError):
        return None


def _contract_json(path: Path, name: str) -> dict:
    try:
        value = json.loads(path.read_text())
    except (OSError, ValueError) as error:
        raise ContractError(f"run-directory credential {name} is unreadable or invalid JSON") from error
    if not isinstance(value, dict):
        raise ContractError(f"run-directory credential {name} must be a JSON object")
    return value


def validate_run_directory(run_dir: Path, args: argparse.Namespace, *,
                           output_root: Path | None = None,
                           identity_reader=process_identity,
                           current_pid: int | None = None,
                           parent_pid: int | None = None) -> dict:
    """Validate the exact, same-run initialization files written by the public wrapper.

    This guard intentionally rejects terminal receipts and every unrecognized file.
    The watchdog and supervisor files are admissible only while they are still
    startup credentials for this Run ID and their PID/start-time identities match
    the live processes participating in this invocation.
    """
    raw_dir = Path(run_dir)
    if raw_dir.is_symlink():
        raise ContractError("run directory must not be a symlink")
    try:
        resolved_dir = raw_dir.resolve(strict=True)
    except OSError as error:
        raise ContractError("run directory is unavailable") from error
    if not resolved_dir.is_dir() or str(raw_dir.absolute()) != str(resolved_dir):
        raise ContractError("run directory path is not a canonical directory")
    root = Path(output_root or OUTPUT_ROOT).resolve(strict=True)
    try:
        resolved_dir.relative_to(root)
    except ValueError as error:
        raise ContractError("run directory must be inside the approved output root") from error
    if resolved_dir.name != args.run_id:
        raise ContractError("run directory leaf must equal its unique Run ID")

    entries = list(resolved_dir.iterdir())
    non_regular = [p.name for p in entries if p.is_symlink() or not p.is_file()]
    if non_regular:
        raise ContractError("run directory contains non-regular entries: " + ", ".join(sorted(non_regular)))
    names = {p.name for p in entries}
    unexpected = names - RUN_DIRECTORY_INITIAL_FILES
    if unexpected:
        old_receipts = sorted(unexpected & RUN_DIRECTORY_TERMINAL_FILES)
        if old_receipts:
            raise ContractError("run directory contains existing terminal receipt(s): "
                                + ", ".join(old_receipts))
        raise ContractError("run directory contains unexpected file(s): "
                            + ", ".join(sorted(unexpected)))
    missing = RUN_DIRECTORY_INITIAL_FILES - names
    if missing:
        raise ContractError("run directory is missing initialization file(s): "
                            + ", ".join(sorted(missing)))

    wrapper = _contract_json(resolved_dir / "wrapper.json", "wrapper.json")
    ready = _contract_json(resolved_dir / "watchdog_ready.json", "watchdog_ready.json")
    state = _contract_json(resolved_dir / "supervisor_state.json", "supervisor_state.json")
    t0_ns = int(args.t0_ns)
    if LOCALIZATION_DIAGNOSTIC and any(item.get('timing_profile') != TIMING_PROFILE
                                       for item in (wrapper, ready, state)):
        raise ContractError('run-directory timing profile identity mismatch')
    if wrapper.get("run_id") != args.run_id or wrapper.get("t0_monotonic_ns") != t0_ns:
        raise ContractError("wrapper.json Run ID or T0 does not match this invocation")
    if state.get("run_id") != args.run_id or state.get("t0_monotonic_ns") != t0_ns:
        raise ContractError("supervisor_state.json Run ID or T0 does not match this invocation")
    if ready.get("run_id") != args.run_id or ready.get("t0_monotonic_ns") != t0_ns:
        raise ContractError("watchdog_ready.json Run ID or T0 does not match this invocation")

    if wrapper.get("navigation_worktree") != str(NAV.resolve()) \
            or wrapper.get("workspace_root") != str(runtime.ROOT.resolve()):
        raise ContractError("wrapper.json Navigation/workspace path identity mismatch")
    if wrapper.get("map_directory_argument") != args.map_directory \
            or wrapper.get("world_argument") != args.world \
            or wrapper.get("map_profile") != args.map_profile:
        raise ContractError("wrapper.json map profile/path does not match supervisor arguments")
    if wrapper.get("odometry_mode", "wheel") != args.odometry_mode:
        raise ContractError("wrapper.json odometry mode does not match supervisor arguments")
    if not Path(args.map_directory).is_absolute() or not Path(args.world).is_absolute():
        raise ContractError("map and world arguments must be absolute paths")
    recorded_overlay = wrapper.get("overlay_setup_argument")
    try:
        if not isinstance(recorded_overlay, str) or not Path(recorded_overlay).is_absolute() \
                or Path(recorded_overlay).resolve(strict=True) != Path(args.overlay_setup).resolve(strict=True):
            raise ContractError("wrapper.json overlay setup path does not match supervisor arguments")
    except OSError as error:
        raise ContractError("wrapper.json overlay setup path is unavailable") from error
    try:
        t0_utc = wrapper.get("t0_utc")
        if not isinstance(t0_utc, str) or not t0_utc.endswith("Z"):
            raise ValueError("missing UTC suffix")
        datetime.fromisoformat(t0_utc[:-1] + "+00:00")
    except ValueError as error:
        raise ContractError("wrapper.json T0 UTC identity is missing or invalid")

    expected_state = {
        "launch_pid": None, "launch_pgid": None, "launch_start_ticks": None,
        "readiness_all_passed": False, "readiness_phases": {},
    }
    if any(state.get(key) != value for key, value in expected_state.items()):
        raise ContractError("supervisor_state.json is not in the initial pre-launch state")
    if ready.get("status") != "READY" or any(ready.get(key) is not None for key in (
            "supervisor_pid", "supervisor_start_ticks", "launch_pid", "launch_start_ticks")):
        raise ContractError("watchdog_ready.json is not in the initial pre-launch state")

    supervisor_pid = int(current_pid if current_pid is not None else os.getpid())
    wrapper_pid = int(parent_pid if parent_pid is not None else os.getppid())
    supervisor_identity = identity_reader(supervisor_pid)
    wrapper_identity = identity_reader(wrapper_pid)
    try:
        watchdog_pid = int(ready["pid"])
        watchdog_identity = identity_reader(watchdog_pid)
    except (KeyError, TypeError, ValueError):
        watchdog_identity = None
        watchdog_pid = -1
    if not supervisor_identity or supervisor_identity.get("state") == "Z" \
            or state.get("supervisor_pid") != supervisor_pid \
            or state.get("supervisor_start_ticks") != supervisor_identity.get("start_ticks"):
        raise ContractError("supervisor_state.json PID/start-time identity mismatch")
    if not wrapper_identity or wrapper_identity.get("state") == "Z" \
            or ready.get("wrapper_pid") != wrapper_pid \
            or ready.get("wrapper_start_ticks") != wrapper_identity.get("start_ticks"):
        raise ContractError("watchdog_ready.json wrapper PID/start-time identity mismatch")
    if not watchdog_identity or watchdog_identity.get("state") == "Z" \
            or ready.get("start_ticks") != watchdog_identity.get("start_ticks") \
            or ready.get("process_group_id") != watchdog_pid \
            or ready.get("session_id") != watchdog_pid \
            or watchdog_identity.get("pgid") != watchdog_pid \
            or watchdog_identity.get("session_id") != watchdog_pid:
        raise ContractError("watchdog_ready.json watchdog PID/session/process-group identity mismatch")
    if len({supervisor_pid, wrapper_pid, watchdog_pid}) != 3:
        raise ContractError("startup process identities are not distinct")
    return {"run_id": args.run_id, "run_dir": str(resolved_dir), "t0_monotonic_ns": t0_ns,
            "wrapper_pid": wrapper_pid, "supervisor_pid": supervisor_pid,
            "watchdog_pid": watchdog_pid}


def _write_json_exclusive(path: Path, value: dict) -> bool:
    """Write a diagnostic receipt without replacing any existing run evidence."""
    payload = (json.dumps(value, indent=2, sort_keys=True, default=str) + "\n").encode()
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        return False
    with os.fdopen(fd, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    return True


def record_run_directory_rejection(run_dir: Path, args: argparse.Namespace,
                                   error: ContractError) -> None:
    """Leave a same-run, pre-launch failure reason for the independent watchdog."""
    run_dir = Path(run_dir)
    try:
        resolved = run_dir.resolve(strict=True)
        resolved.relative_to(OUTPUT_ROOT.resolve(strict=True))
        if resolved.name != args.run_id or run_dir.is_symlink():
            return
        if any((resolved / name).exists() for name in RUN_DIRECTORY_TERMINAL_FILES):
            return
        state = _contract_json(resolved / "supervisor_state.json", "supervisor_state.json")
        current = process_identity(os.getpid())
        if (state.get("run_id") != args.run_id or state.get("t0_monotonic_ns") != args.t0_ns
                or state.get("supervisor_pid") != os.getpid() or not current
                or state.get("supervisor_start_ticks") != current.get("start_ticks")
                or state.get("launch_pid") is not None):
            return
        ready = _contract_json(resolved / "watchdog_ready.json", "watchdog_ready.json")
        receipt = {
            "schema_version": 1, "run_id": args.run_id, "run_dir": str(resolved),
            "t0_monotonic_ns": args.t0_ns, "status": "REJECTED_BEFORE_ADMISSION",
            "failure_stage": "run_directory_admission", "reason": str(error),
            "launch_started": False, "launch_pid": None, "launch_pgid": None,
            "supervisor_pid": os.getpid(), "supervisor_start_ticks": current["start_ticks"],
            "wrapper_pid": os.getppid(),
            "wrapper_start_ticks": ready.get("wrapper_start_ticks"),
            "watchdog_pid": ready.get("pid"),
            "watchdog_start_ticks": ready.get("start_ticks"),
            "recorded_at_utc": utc_now(), "traceback": traceback.format_exc(),
        }
        _write_json_exclusive(resolved / "supervisor_failure.json", receipt)
    except (OSError, ContractError, ValueError):
        # If the run directory itself cannot be safely identified, the watchdog
        # will retain its generic fail-closed receipt instead of writing here.
        return


def record_watchdog_failure(args: argparse.Namespace) -> int:
    """Finalize a fail-closed run after the independent watchdog cut off Python."""
    run_dir = Path(args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    watchdog_path = run_dir / "hard_deadline.json"
    deadline = time.monotonic() + 4.5
    watchdog_record = read_json(watchdog_path)
    while (watchdog_record and watchdog_record.get("status") == "CUT_OFF_IN_PROGRESS"
           and time.monotonic() < deadline):
        time.sleep(0.05)
        watchdog_record = read_json(watchdog_path)
    if watchdog_record is None:
        watchdog_record = read_json(run_dir / "watchdog_failure.json") or {
            "status": "WATCHDOG_RECEIPT_MISSING", "reason": "watchdog exited without a terminal receipt",
            "hard_deadline_enforced": False,
        }
    admission_failure = watchdog_record.get("first_failure")
    admission_failure_valid = bool(
        isinstance(admission_failure, dict)
        and admission_failure.get("run_id") == args.run_id
        and admission_failure.get("failure_stage") == "run_directory_admission"
        and admission_failure.get("launch_started") is False
        and isinstance(admission_failure.get("reason"), str)
        and watchdog_record.get("launch_started") is False
        and watchdog_record.get("cleanup_status") == "NOT_STARTED"
    )
    try:
        result = json.loads((run_dir / "result.json").read_text())
        if not isinstance(result, dict):
            result = {}
    except (OSError, ValueError):
        result = {}
    if not result:
        wrapper = read_json(run_dir / "wrapper.json") or {}
        prior_cleanup = read_json(run_dir / "cleanup.json")
        prior_cleanup_verified = cleanup_is_verified(prior_cleanup)
        result = {
            "schema_version": 1, "run_id": args.run_id, "status": "FAIL",
            "t0_monotonic_ns": args.t0_ns,
            "started_at_utc": wrapper.get("t0_utc"),
            "launch_started": (read_json(run_dir / "supervisor_state.json") or {}).get("launch_pid") is not None,
            "launch_exit_code": None, "dynamic_entry_calls": 1,
            "motion_enabled": False, "automatic_task_enabled": False,
            "authorization_or_lease_created": False,
            "phases": {name: "NOT RUN" for name in PHASES},
            "failures": [],
        }
        if prior_cleanup_verified:
            result["cleanup"] = prior_cleanup
            result["cleanup_verified"] = True
    result["status"] = "FAIL"
    result["failure_stage"] = (
        "hard_deadline" if watchdog_record.get("hard_deadline_enforced") else
        "run_directory_admission" if admission_failure_valid else "watchdog")
    result["hard_deadline_enforced"] = watchdog_record.get("hard_deadline_enforced") is True
    result["hard_deadline"] = watchdog_record
    result["hard_process_cutoff_t0_s"] = HARD_PROCESS_CUTOFF_FROM_T0_S
    result["cleanup_evidence_cutoff_t0_s"] = CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S
    failures = result.setdefault("failures", [])
    if admission_failure_valid:
        result["first_failure"] = admission_failure
        result["launch_started"] = False
        result["launch_pid"] = None
        failures.append("supervisor rejected run-directory admission: " + admission_failure["reason"])
    failures.append("independent watchdog terminated or could not verify the supervisor before finalization")
    if not result.get("cleanup_verified"):
        group = watchdog_record.get("process_group_snapshot_after", {})
        if admission_failure_valid:
            result["cleanup"] = {
                "status": "NOT_STARTED", "cleanup_ok": None,
                "reason": "supervisor rejected the run directory before launch creation",
                "launch_main_reaped": "NOT_APPLICABLE",
                "process_group_alive_after_cleanup": None,
                "process_group_state_after_cleanup": "NOT_STARTED",
                "process_group_snapshot_after_cleanup": group,
                "watchdog_signals": watchdog_record.get("signals", []),
            }
        else:
            result["cleanup"] = {
                "status": "FAIL_CLOSED_WATCHDOG", "cleanup_ok": False,
                "reason": watchdog_record.get("reason", "watchdog terminal evidence unavailable"),
                "launch_main_reaped": watchdog_record.get("launch_main_reaped", "UNKNOWN"),
                "process_group_alive_after_cleanup": (
                    bool(group.get("running_members")) if group.get("scan_complete") is True else None),
                "process_group_state_after_cleanup": group.get("status", "UNKNOWN"),
                "process_group_snapshot_after_cleanup": group,
                "watchdog_signals": watchdog_record.get("signals", []),
            }
        result["cleanup_verified"] = False
    cleanup = result.get("cleanup") or {
        "status": "UNKNOWN", "cleanup_ok": False, "launch_main_reaped": "UNKNOWN",
        "process_group_state_after_cleanup": "UNKNOWN",
        "process_group_alive_after_cleanup": None,
        "process_group_snapshot_after_cleanup": {
            "status": "UNKNOWN", "scan_complete": False,
            "running_members": [], "zombie_members": [],
        },
    }
    result["completed_at_utc"] = utc_now()
    result["total_elapsed_s"] = round(time.monotonic() - args.t0_ns / 1_000_000_000, 3)
    if result["total_elapsed_s"] > ACCOUNTING_LIMIT_FROM_T0_S:
        failures.append("terminal failure receipt was finalized after the T0+180s evidence cutoff")
    result["failures"] = failures
    events_path = run_dir / "events.json"
    events = read_json(events_path)
    event_rows = events.get("events", []) if events else []
    event_rows.append({"event": "watchdog_finalized_failure", "utc": utc_now(),
                       "monotonic_ns": time.monotonic_ns(),
                       "hard_deadline_enforced": result["hard_deadline_enforced"]})
    write_run_receipts(run_dir, result, cleanup=cleanup,
                       environment=read_json(run_dir / "environment.json") or {
                           "status": "PARTIAL", "variables": environment_variable_snapshot()},
                       identity=read_json(run_dir / "source_identity.json") or {"status": "PARTIAL"},
                       events=event_rows)
    return 124


def record_watchdog_exit(args: argparse.Namespace) -> int:
    """Bind the shell parent's wait/reap results into the normal run receipt."""
    run_dir = Path(args.run_dir).resolve()
    result = read_json(run_dir / "result.json")
    if result is None:
        return record_watchdog_failure(args)
    if read_json(run_dir / "hard_deadline.json") or read_json(run_dir / "watchdog_failure.json"):
        return record_watchdog_failure(args)
    receipt = read_json(run_dir / "watchdog_exit.json")
    result["deadline_watchdog"] = {
        "pid": args.watchdog_pid, "entry": str(HERE / "deadline_watchdog.py"),
        "entry_sha256": sha256(HERE / "deadline_watchdog.py"),
        "exit_code": args.watchdog_exit_code, "reaped_by_wrapper": True,
        "terminal_receipt": receipt,
    }
    result["wrapper_wait_status"] = {
        "supervisor_pid_reaped": True, "supervisor_exit_code": args.supervisor_exit_code,
        "watchdog_pid_reaped": True, "watchdog_exit_code": args.watchdog_exit_code,
    }
    result["total_elapsed_s"] = round(time.monotonic() - args.t0_ns / 1_000_000_000, 3)
    result["wrapper_finalized_at_utc"] = utc_now()
    if result["total_elapsed_s"] > ACCOUNTING_LIMIT_FROM_T0_S:
        result["status"] = "FAIL"
        result.setdefault("failures", []).append("wrapper finalization exceeded the T0+180s evidence cutoff")
    if args.watchdog_exit_code != 0 or not isinstance(receipt, dict) \
            or receipt.get("status") != "NORMAL_COMPLETION":
        result["status"] = "FAIL"
        result.setdefault("failures", []).append("watchdog did not complete normally or was not reaped")
    if args.supervisor_exit_code != 0 and result.get("status") in ("PASS", "PASS_WITH_EXCEPTION"):
        result["status"] = "FAIL"
        result.setdefault("failures", []).append("supervisor process exited nonzero")
    events = read_json(run_dir / "events.json") or {"events": []}
    rows = events.get("events", [])
    rows.append({"event": "wrapper_reaped_supervisor_and_watchdog", "utc": utc_now(),
                 "monotonic_ns": time.monotonic_ns(),
                 "supervisor_exit_code": args.supervisor_exit_code,
                 "watchdog_exit_code": args.watchdog_exit_code})
    write_run_receipts(
        run_dir, result, cleanup=read_json(run_dir / "cleanup.json"),
        environment=read_json(run_dir / "environment.json"),
        identity=read_json(run_dir / "source_identity.json"), events=rows)
    return 0


def source_identity():
    try:
        head = subprocess.run(["git", "-C", str(NAV), "rev-parse", "HEAD"],
                              capture_output=True, text=True, timeout=3, check=True).stdout.strip()
        branch = subprocess.run(["git", "-C", str(NAV), "branch", "--show-current"],
                                capture_output=True, text=True, timeout=3, check=True).stdout.strip()
        status = subprocess.run(["git", "-C", str(NAV), "status", "--short", "--branch"],
                                capture_output=True, text=True, timeout=3, check=True).stdout.splitlines()
    except Exception as error:
        raise ContractError(f"Git source identity unavailable: {type(error).__name__}: {error}") from error
    manifest = integration.verify_source_manifest()
    return {
        "repository": str(NAV), "branch": branch, "head": head,
        "status_short_branch": status,
        "source_manifest": str(HERE / "SOURCE_MANIFEST.json"),
        "source_manifest_sha256": sha256(HERE / "SOURCE_MANIFEST.json"),
        "manifest_file_count": len(manifest["files"]),
        "manifest_files": manifest["files"],
        "actual_launch_source": str(ENTRY), "actual_launch_source_sha256": sha256(ENTRY),
    }


def installed_file_identity(prefixes: dict[str, str], environment=None) -> dict:
    records = {}
    for package, binding in DEPENDENCIES["cleannav_packages"].items():
        prefix = Path(prefixes[package]).resolve(strict=True)
        try:
            profile_binding, selected = integration.profile_install_binding(
                package, binding, str(prefix), environment)
        except ValueError as error:
            raise ContractError(str(error)) from error
        files = []
        for relative, expected in selected["files"].items():
            path = prefix / relative
            if not path.is_file():
                raise ContractError(f"installed selected file missing: {package}/{relative}")
            actual = sha256(path)
            if actual != expected:
                raise ContractError(f"installed selected file hash mismatch: {package}/{relative}")
            files.append({"path": str(path.resolve()), "sha256": actual})
        linked = []
        for link, source_hashes in selected["linked_python_sources"].items():
            marker = prefix / link
            source_root = Path(marker.read_text().splitlines()[0]).resolve(strict=True)
            for relative, expected in source_hashes.items():
                path = source_root / relative
                actual = sha256(path)
                if actual != expected:
                    raise ContractError(f"linked installed source hash mismatch: {package}/{relative}")
                linked.append({"path": str(path), "sha256": actual, "marker": str(marker)})
        records[package] = {"prefix": str(prefix), "profile_binding": profile_binding,
                            "selected_files": files, "linked_sources": linked}
    return records


def bounded_launch_argv(*, world: Path, map_directory: Path, map_profile: str, output_dir: Path,
                        runtime_timeout_s: float, motion_profile: str = "none",
                        odometry_mode: str = "wheel") -> list[str]:
    if motion_profile not in ("none", "short-goal", "historical", "patrol"):
        raise ContractError("unsupported bounded motion profile")
    if odometry_mode not in ("wheel", "oracle"):
        raise ContractError("unsupported odometry mode")
    motion_enabled = motion_profile != "none"
    if not (0 < runtime_timeout_s <= MAX_LAUNCH_RUNTIME_S):
        raise ContractError("launch timeout exceeds the selected timing profile bound")
    # Deferred included launch actions can resolve substitutions after their
    # include scope is restored. Bind the outer argv to the same hashed profile,
    # rather than leaving outer -8m/10s defaults that override -5m/60s later.
    profile_contract = json.loads(integration.contract_path_for_profile(map_profile).read_text())
    spawn = profile_contract.get('spawn', {})
    spawn_args = [f'spawn_{axis}:={spawn[axis]}' for axis in ('x','y','z','yaw') if axis in spawn]
    spawn_delay = profile_contract.get('spawn_delay_sec', '10.0')
    # The source launch is passed explicitly. No installed launch with the same
    # name can shadow the code bound by SOURCE_MANIFEST.json.
    return [
        "ros2", "launch", str(ENTRY),
        f"world:={world}", f"map_directory:={map_directory}", f"map_profile:={map_profile}",
        f"odometry_mode:={odometry_mode}",
        f"model_path:={MODEL_ASSETS / 'models'}",
        f"enable_motion:={'true' if motion_enabled else 'false'}",
        *( [f"motion_profile:={motion_profile}"] if motion_enabled else [] ),
        "auto_start_task:=false", f"enable_rviz_goals:={'false' if motion_profile == 'patrol' else 'true'}",
        "start_rviz:=true", "gui:=false",
        f"enable_diagnostics:={'true' if motion_enabled else 'false'}",
        "ros_domain_id:=204", "gazebo_port:=11704", f"spawn_delay_sec:={spawn_delay}", *spawn_args,
        "runtime_timeout_sec:=" + str(runtime_timeout_s),
    ]


def all_command_streams_zero(command_counts: dict, nonzero_counts: dict) -> bool:
    """Stopped baseline: observed zero /cmd_vel, and no nonzero upstream sample.

    Idle controller_server and collision_monitor publishers emit nothing until
    a goal produces a command. That silence is not a nonzero command.
    """
    topics = ("/cleannav/navigation/cmd_vel_raw", "/cleannav/cmd_vel_candidate", "/cmd_vel")
    zeros = {}
    for topic in topics:
        count = command_counts.get(topic, 0)
        nonzero = nonzero_counts.get(topic, 0)
        if count < 0 or nonzero < 0 or nonzero > count:
            return False
        zeros[topic] = count - nonzero
    return idle_command_baseline_ok(command_counts, zeros)[0]


def launch_after_preflight(preflight: dict, popen):
    """Single audited process-creation edge; blocked gates cannot call Popen."""
    if not isinstance(preflight, dict) or preflight.get("ok") is not True:
        raise ContractError("launch process creation denied by failed preflight")
    return popen()


def first_admitted_phase_loss(ledger, readiness, guard):
    """The real loop's loss branch, callable without launching a ROS process."""
    for phase in PHASES:
        if ledger.states[phase] == 'PASS' and not readiness[phase][0]:
            return readiness_loss_record(phase, readiness, guard, ledger.evidence.get(phase))
    return None


def process_group_processes(pgid: int, proc_root: Path = Path("/proc")) -> list[dict]:
    """Return identity metadata for processes in the owned launch group only."""
    found = []
    for entry in Path(proc_root).iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat_text = (entry / "stat").read_text()
            close = stat_text.rfind(")")
            fields = stat_text[close + 2:].split()
            state, group = fields[0], int(fields[2])
            start_ticks = int(fields[19])
            if group != pgid:
                continue
            comm = (entry / "comm").read_text().strip()
            cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            found.append({"pid": int(entry.name), "comm": comm, "state": state,
                          "pgid": group, "start_ticks": start_ticks, "cmdline": cmdline[:500]})
        except (OSError, ValueError, IndexError):
            continue
    return found


def one_expected_publisher(endpoints, expected_node: str, *, expected_type="geometry_msgs/msg/Twist") -> tuple[bool, list[dict]]:
    records = [endpoint_record(item) for item in endpoints]
    if len(records) != 1:
        return False, records
    item = records[0]
    gid = item.get("endpoint_gid")
    ok = (item.get("node_name") == expected_node and item.get("node_namespace") == "/"
          and item.get("topic_type") == expected_type
          and gid not in (None, "", [], {}))
    return ok, records


def expected_rviz_graph_node(nodes) -> bool:
    """Match the explicit historical ROS graph name, not executable identity."""
    return ("rviz", "/") in {(name, namespace) for name, namespace in nodes}


def discover_artifact(ros_log_dir: Path, filename: str) -> Path | None:
    matches = sorted(ros_log_dir.rglob(filename)) if ros_log_dir.exists() else []
    if len(matches) > 1:
        raise ContractError(f"multiple per-run {filename} receipts found")
    return matches[0] if matches else None


def read_json(path: Path | None) -> dict | None:
    if path is None:
        return None
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def refresh_guard_receipt(path: Path | None, previous: dict | None, *,
                          now_monotonic: float | None = None,
                          read_json_fn=read_json) -> tuple[dict | None, dict]:
    """Read after blocking graph work; compute freshness after the read completes."""
    started = time.monotonic_ns()
    latest = read_json_fn(path)
    finished = time.monotonic_ns()
    status = guard_receipt_snapshot_status(
        previous, latest, now_monotonic=(finished / 1e9
            if now_monotonic is None else now_monotonic))
    status.update(read_started_monotonic_ns=started, read_finished_monotonic_ns=finished,
                  previous_sequence=(previous or {}).get('receipt_sequence'),
                  latest_sequence=(latest or {}).get('receipt_sequence'),
                  latest_writer_monotonic=(latest or {}).get('monotonic'),
                  cached_object_reused=False)
    return latest, status


def startup_verification_record(*, run_id, launch_elapsed_s, observation_elapsed,
                                phases, launch_mode, command_samples,
                                nonzero_samples, distance_m):
    """Serialize the live success branch; exercised by filesystem regression."""
    return {
        "status": "PASS", "run_id": run_id,
        "completed_at_utc": utc_now(),
        "completed_monotonic_ns": time.monotonic_ns(),
        "launch_elapsed_s": round(launch_elapsed_s, 6),
        "continuous_all_ready_observation_s": round(observation_elapsed, 6),
        "required_stable_s": NO_MOTION_OBSERVATION_S,
        "phases": dict(phases),
        "launch_mode_contract": launch_mode,
        "command_samples": command_samples,
        "nonzero_command_samples": nonzero_samples,
        "guard_distance_m": distance_m,
        "max_no_motion_displacement_m": MAX_NO_MOTION_DISPLACEMENT_M,
        "goal_submission_path": "NOT PRESENT IN PUBLIC NO-MOTION WRAPPER",
        "authorization_scope_lease_created": False,
        "rviz_ros_process_ready": phases.get("rviz") == "PASS",
        "rviz_rendering": "UNKNOWN; prior GLSL issue is not assessed by this level",
    }


def release_locks(descriptors: list[int]) -> list[dict]:
    results = []
    for fd in reversed(descriptors):
        try:
            os.close(fd)
            results.append({"fd": fd, "closed": True})
        except OSError as error:
            results.append({"fd": fd, "closed": False, "error": str(error)})
    return results


def write_run_receipts(run_dir: Path, result: dict, *, cleanup: dict | None,
                       environment: dict | None, identity: dict | None,
                       events: list[dict]) -> None:
    Path(run_dir).mkdir(parents=True, exist_ok=True)
    if cleanup is not None:
        atomic_json(run_dir / "cleanup.json", cleanup)
    if environment is not None:
        atomic_json(run_dir / "environment.json", environment)
    if identity is not None:
        atomic_json(run_dir / "source_identity.json", identity)
    atomic_json(run_dir / "events.json", {"events": events})
    atomic_json(run_dir / "result.json", result)
    write_checksums(run_dir)


def run(args: argparse.Namespace) -> int:
    global STOP_REQUESTED
    run_dir = Path(args.run_dir)
    try:
        directory_identity = validate_run_directory(run_dir, args)
        run_dir = Path(directory_identity["run_dir"])
    except ContractError as error:
        record_run_directory_rejection(run_dir, args, error)
        raise
    t0 = args.t0_ns / 1_000_000_000
    ros_home, ros_log_dir = run_dir / "ros-home", run_dir / "ros-log"
    ros_home.mkdir(exist_ok=True)
    ros_log_dir.mkdir(exist_ok=True)
    os.environ["ROS_HOME"] = str(ros_home)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)
    events: list[dict] = []
    descriptors: list[int] = []
    process = None
    launch_log = None
    ros = None
    observer = None
    diagnostic = None
    memory_telemetry = None
    cleanup = None
    source = None
    environment = None
    failures: list[str] = []
    stop_reason = None
    ledger = PhaseLedger()
    launched = False
    cgroup_disposition = None
    observation_started = None
    launch_monotonic_ns = None
    verified_startup = None
    startup_verification_complete = False
    diagnostic_memory_policy = False
    guard = None
    command_nonzero_count = 0
    command_count = 0
    launch_log_offset = 0
    log_text = ""
    graph_gate = None
    graph_records = []
    motion_mode = args.motion_profile == "short-goal"
    patrol_spec = None
    patrol_motion = args.motion_profile == "patrol"
    historical_motion = args.motion_profile in ("historical", "patrol")
    any_motion = motion_mode or historical_motion
    historical_gui_ready_at = None
    historical_lease_seen = False
    historical_initial_truth = None
    motion_admitted = False
    guard_writer_pid = None
    guard_receipt_confirmed_stamp_ns = None
    motion_authority_issued = False
    motion_interlock_open = False
    motion_stable_since = None
    motion_initial_ground_truth = None
    motion_terminal_monotonic = None
    motion_close_confirmed = False
    motion_completion = None
    motion_goal_wait_started = None
    motion_goal_seen_at = None
    motion_click_command_id = None
    motion_task_id = None
    motion_task_deadline = None
    motion_post_terminal_zero_counts = {
        "/cleannav/navigation/cmd_vel_raw": 0,
        "/cleannav/cmd_vel_candidate": 0,
        "/cmd_vel": 0,
    }
    motion_terminal_payload = None
    motion_event_index = 0
    motion_admission_wait_started = None
    motion_last_graph_signature = None
    motion_interlock_future = None
    motion_interlock_request_started = None
    motion_interlock_stage = None
    waiver = None
    resource_samples = 0
    runtime_memory_soft_warning_samples = 0
    result = {
        "schema_version": 1,
        "run_id": args.run_id,
        "status": "BLOCKED",
        "t0_monotonic_ns": args.t0_ns,
        "started_at_utc": utc_now(),
        "timing_contract": timing_identity(),
        "total_accounting_limit_s": ACCOUNTING_LIMIT_FROM_T0_S,
        "soft_stop_deadline_t0_s": SOFT_STOP_FROM_T0_S,
        "readiness_deadline_t0_s": READINESS_DEADLINE_FROM_T0_S,
        "hard_process_cutoff_t0_s": HARD_PROCESS_CUTOFF_FROM_T0_S,
        "cleanup_evidence_cutoff_t0_s": CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S,
        "cleanup_reserve_s": MIN_CLEANUP_RESERVE_S,
        "hard_deadline_enforced": False,
        "launch_started": False,
        "launch_exit_code": None,
        "process_group_id": None,
        "phases": dict(ledger.states),
        "failures": failures,
        "dynamic_entry_calls": 1,
        "motion_profile": args.motion_profile,
        "odometry_mode": args.odometry_mode,
        "motion_enabled": any_motion,
        "automatic_task_enabled": False,
        "authorization_or_lease_created": False,
        "motion_admission": {"status": "NOT_APPLICABLE" if historical_motion else "NOT STARTED"},
        "motion_execution": {"status": "NOT RUN"},
        "verified_no_motion_startup": {"status": "NOT ACHIEVED"},
        "startup_soak": {"status": "NOT COMPLETED"},
        "memory_attribution": {"status": "NOT STARTED"},
    }
    wrapper_record = read_json(run_dir / "wrapper.json") or {}
    result["t0_utc"] = wrapper_record.get("t0_utc")
    input_identity = {
        "navigation_worktree": str(NAV), "source_entry": str(ENTRY),
        "map_directory_argument": args.map_directory, "world_argument": args.world,
        "map_profile": args.map_profile,
        "odometry_mode": args.odometry_mode,
        "overlay_setup_argument": args.overlay_setup,
    }
    try:
        input_identity["world_sha256"] = sha256(Path(args.world).resolve(strict=True))
    except (OSError, ValueError):
        input_identity["world_sha256"] = None
    try:
        map_arg = Path(args.map_directory).resolve(strict=True)
        input_identity["map_directory_resolved"] = str(map_arg)
        selected_contract_path = integration.contract_path_for_profile(args.map_profile)
        input_identity["map_contract_path"] = str(selected_contract_path)
        input_identity["map_contract_sha256"] = sha256(selected_contract_path)
        input_identity["map_contract_data"] = json.loads(selected_contract_path.read_text())
        input_identity["map_asset_hashes"] = {
            name: sha256(map_arg / name) if (map_arg / name).is_file() else None
            for name in input_identity["map_contract_data"]["files"]
        }
    except (OSError, ValueError):
        input_identity["map_directory_resolved"] = None

    def elapsed() -> float:
        return time.monotonic() - t0

    operator_ready_announced = False

    def announce_navigation_ready() -> None:
        nonlocal operator_ready_announced
        if operator_ready_announced:
            return
        operator_ready_announced = True
        write_operator_notice(format_navigation_ready(
            run_id=args.run_id, odometry_mode=args.odometry_mode,
            motion_profile=args.motion_profile, map_profile=args.map_profile))

    def announce_navigation_not_ready(stage: str, reason: str) -> None:
        if operator_ready_announced:
            return
        write_operator_notice(format_navigation_not_ready(stage=stage, reason=reason))

    def event(kind: str, **fields):
        record = {"event": kind, "run_id": args.run_id, "utc": utc_now(),
                  "monotonic_ns": time.monotonic_ns(), "t0_elapsed_s": round(elapsed(), 6), **fields}
        events.append(record)
        with (run_dir / "events.jsonl").open("a") as stream:
            stream.write(json.dumps(record, sort_keys=True, default=str) + "\n")

    def fail(why: str, stage: str = "supervision"):
        failures.append(why)
        result["failure_stage"] = stage
        if (any_motion and (historical_motion or motion_interlock_open or motion_interlock_stage == "OPENING")
                and observer is not None):
            try:
                if observer.stop_publisher is not None:
                    observer.stop_publisher.publish(Bool(data=True))
                if observer.cancel_client is not None and observer.cancel_client.service_is_ready():
                    observer.cancel_client.call_async(CancelGoal.Request())
                event("motion_fail_closed_stop_requested", reason=why,
                      estop_published=observer.stop_publisher is not None,
                      nav2_cancel_requested=(observer.cancel_client is not None
                          and observer.cancel_client.service_is_ready()),
                      safety_owner=(observer.safety_state or {}).get("owner"))
            except Exception as stop_error:
                event("motion_fail_closed_stop_error",
                      error=f"{type(stop_error).__name__}: {stop_error}")
        if stage in PHASES and ledger.failed_at is None:
            ledger.fail(stage, why, monotonic_ns=time.monotonic_ns())
        event("failure", stage=stage, reason=why)
        if historical_motion:
            announce_navigation_not_ready(stage, why)
        raise ContractError(why)

    def phase_from_failure_text(text: str) -> str:
        folded = text.lower()
        for tokens, phase in ((('simulation', 'spawn', 'gazebo'), 'gazebo_vehicle'),
                              (('collision',), 'collision_monitor'),
                              (('slam', 'localization', 'map'), 'map_localization'),
                              (('safety', 'guard', 'cmd_vel'), 'safety'),
                              (('navigation', 'planner', 'controller'), 'nav2'),
                              (('mission', 'task'), 'mission_manager'),
                              (('rviz',), 'rviz')):
            if any(token in folded for token in tokens):
                return phase
        return "supervision"

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def request_stop(signum, _frame):
        global STOP_REQUESTED
        STOP_REQUESTED = signal.Signals(signum).name

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        atomic_json(run_dir / "supervisor_state.json", {
            "run_id": args.run_id, "supervisor_pid": os.getpid(),
            "supervisor_start_ticks": proc_start_ticks(os.getpid()),
            "launch_pid": None, "launch_pgid": None, "launch_start_ticks": None,
            "timing_profile": TIMING_PROFILE,
            "readiness_all_passed": False,
            "readiness_phases": dict(ledger.states),
            "t0_monotonic_ns": args.t0_ns,
        })
        event("preflight_started")
        if args.overlay_before_sha256 != args.overlay_after_sha256:
            fail("overlay setup changed while being sourced", "environment_identity")
        overlay = Path(args.overlay_setup).resolve(strict=True)
        if sha256(overlay) != args.overlay_after_sha256:
            fail("overlay setup SHA-256 differs from the wrapper record", "environment_identity")
        environment = {
            "ros_setup": str(ROS_SETUP), "ros_setup_sha256": sha256(ROS_SETUP),
            "overlay_setup": str(overlay), "overlay_setup_sha256": sha256(overlay),
            "ros2_path": shutil.which("ros2"), "xacro_path": shutil.which("xacro"),
            "rviz2_path": shutil.which("rviz2"),
            "gui_display_available": bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")),
            "python_executable": sys.executable,
            "variables": environment_variable_snapshot(),
            "ament_prefix_path_entries": [x for x in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if x],
            "colcon_prefix_path_entries": [x for x in os.environ.get("COLCON_PREFIX_PATH", "").split(os.pathsep) if x],
            "cwd": str(Path.cwd()), "pid1_comm": Path("/proc/1/comm").read_text().strip(),
        }
        validate_environment(os.environ, DEPENDENCIES)
        if Path.cwd().resolve() != NAV:
            fail("working directory differs from the Navigation worktree", "source_identity")
        source = identity = source_identity()
        world = Path(args.world).resolve(strict=True)
        map_directory = Path(args.map_directory).resolve(strict=True)
        if not (MODEL_ASSETS / "models").is_dir():
            fail("audited Gazebo model assets are unavailable", "model_identity")
        map_contract_path = integration.contract_path_for_profile(args.map_profile)
        map_contract = integration.verify_map(map_directory, world, profile=args.map_profile,
                                              odometry_mode=args.odometry_mode)
        identity["map"] = {"profile_id": args.map_profile, "directory": str(map_directory),
                            "contract_path": str(map_contract_path),
                            "contract_sha256": sha256(map_contract_path),
                            "contract": map_contract,
                            "asset_hashes": map_contract["files"], "world": str(world),
                            "world_sha256": sha256(world)}
        identity["model_assets"] = {"directory": str(MODEL_ASSETS / "models"),
                                    "root": str(MODEL_ASSETS)}
        identity["overlay"] = {"setup_path": str(overlay), "sha256": args.overlay_after_sha256}
        if not environment["ros2_path"] or not Path(environment["ros2_path"]).resolve().is_relative_to(Path("/opt/ros/humble")):
            fail("ros2 executable is not resolved from /opt/ros/humble", "environment_identity")
        if not environment["xacro_path"] or not Path(environment["xacro_path"]).resolve().is_relative_to(Path("/opt/ros/humble")):
            fail("xacro executable is not resolved from /opt/ros/humble", "environment_identity")
        if not environment["rviz2_path"] or not Path(environment["rviz2_path"]).resolve().is_relative_to(Path("/opt/ros/humble")):
            fail("rviz2 executable is not resolved from /opt/ros/humble", "environment_identity")
        if not environment["gui_display_available"]:
            fail("RViz GUI display is unavailable; headless launch cannot satisfy the RViz phase",
                 "environment_identity")
        from ament_index_python.packages import get_package_prefix
        prefixes = integration.verify_packages(get_package_prefix)
        identity["amcl_runtime"] = integration.verify_amcl_navigation_runtime(map_contract)
        identity["resolved_packages"] = prefixes
        identity["installed_file_identity"] = installed_file_identity(prefixes)
        identity["installed_selected_file_count"] = sum(
            len(item["selected_files"]) + len(item["linked_sources"])
            for item in identity["installed_file_identity"].values())
        event("identity_verified", git_head=identity["head"],
              source_manifest_sha256=identity["source_manifest_sha256"],
              map_sha256=identity["map"]["asset_hashes"].get("map.yaml"),
              world_sha256=identity["map"]["world_sha256"], package_count=len(prefixes),
              selected_install_file_count=identity["installed_selected_file_count"])

        descriptors, lock_records = runtime.acquire_locks()
        result["locks"] = lock_records
        preflight = {"ok": False, "gates": {}}
        result["preflight"] = preflight

        def gate(name, value):
            preflight["gates"][name] = value
            if not isinstance(value, dict) or value.get("ok") is not True:
                fail(f"preflight gate rejected: {name}", "preflight")

        gate("simulation_rtabmap_safety_prefixes", runtime.check_package_prefixes())
        gate("process_conflicts", runtime.process_visibility_and_conflicts())
        gate("gazebo_port_11704", runtime.port_free(11704))
        waiver = runtime.wsl_cgroup_waiver_preflight()
        snapshot = runtime.memory_snapshot(
            cgroup_waiver=waiver,
            min_available_kib=runtime.WAIVER_STARTUP_MIN_AVAILABLE_KIB,
            min_cgroup_headroom_bytes=runtime.CGROUP_STARTUP_MIN_HEADROOM_BYTES,
            memavailable_warning_only=True,
        )
        gate("memory_swap_psi_cgroup", snapshot)
        cgroup_disposition = snapshot.get("cgroup_memory_disposition")
        result["resource_exception"] = {
            "cgroup_memory_status": snapshot.get("cgroup_memory_status"),
            "cgroup_memory_disposition": cgroup_disposition,
            "waiver_eligible": waiver.get("eligible"), "waiver_reason": waiver.get("reason"),
        }
        import rclpy
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
        from std_msgs.msg import Bool, String
        from cleannav_interfaces.msg import SafetyStatus
        from gazebo_msgs.srv import GetEntityState
        if patrol_motion:
            from cleannav_interfaces.srv import SubmitTaskSequence
            from cleannav_interfaces.msg import TaskSequenceStep, TaskSequenceStatus, TaskStatus
            patrol_spec = map_contract['patrol']
        from std_srvs.srv import SetBool
        from action_msgs.srv import CancelGoal
        ros = rclpy
        ros.init(args=[], signal_handler_options=SignalHandlerOptions.NO)

        class SupervisorObserver(Node):
            def __init__(self):
                super().__init__("cleannav_rviz_navigation_supervisor")
                if patrol_motion:
                    from rclpy.parameter import Parameter
                    self.set_parameters([Parameter('use_sim_time', value=True)])
                self.patrol_owner = ''
                self.patrol_owners = set()
                self.patrol_accepted = False
                self.patrol_future = None
                self.patrol_started = None
                self.patrol_status = None
                self.patrol_progress = 0
                self.patrol_statuses = []
                self.patrol_tasks = []
                if patrol_motion:
                    self.patrol_client = self.create_client(SubmitTaskSequence, '/cleannav/mission/submit_task_sequence')
                    self.create_subscription(TaskSequenceStatus, '/cleannav/task_sequence_status', self.on_sequence, 10)
                    self.create_subscription(TaskStatus, '/cleannav/task_status', self.on_task_status, 10)
                self.command_samples = 0
                self.nonzero_samples = 0
                self.last_command = None
                self.initial_pose_samples = []
                self.command_topics = (("/cleannav/navigation/cmd_vel_raw",
                    "/cleannav/cmd_vel_candidate", "/cmd_vel") if any_motion else ("/cmd_vel",))
                self.command_counts = {topic: 0 for topic in self.command_topics}
                self.command_zero_counts = {topic: 0 for topic in self.command_topics}
                self.command_nonzero_counts = {topic: 0 for topic in self.command_topics}
                self.command_last = {}
                self.safety_state = None
                self.safety_owner_history = []
                self.rviz_goal_count = 0
                self.rviz_goal_stamp_ns = None
                self.motion_events = []
                self.ground_truth_samples = []
                self.plan_points=[];self.static_grid_message=None
                self.ground_truth_future = None
                self.ground_truth_next_request = 0.0
                self.stop_publisher = (self.create_publisher(
                    Bool, "/cleannav/safety/emergency_stop", 10) if any_motion else None)
                self.cancel_client = (self.create_client(
                    CancelGoal, ("/cleannav/navigate_within_polygon/_action/cancel_goal" if patrol_motion
                                     else "/cleannav/navigate_to_pose/_action/cancel_goal"))
                    if any_motion else None)
                for topic in self.command_topics:
                    self.create_subscription(Twist, topic,
                        lambda msg, name=topic: self.on_command(msg, name), 10)
                if any_motion:
                    self.safety_client = self.create_client(
                        SetBool, "/cleannav/safety/set_simulation_motion_enabled")
                    self.ground_truth_client = self.create_client(
                        GetEntityState, "/gazebo/get_entity_state")
                    self.create_subscription(SafetyStatus, "/cleannav/safety_status",
                                             self.on_safety, 10)
                    self.create_subscription(String, "/cleannav/simulation/rviz_goal_events",
                                             self.on_motion_event, 10)
                if historical_motion or motion_mode:
                    from nav_msgs.msg import Path as NavigationPath, OccupancyGrid
                    from rclpy.qos import QoSProfile, DurabilityPolicy
                    self.create_subscription(NavigationPath, '/plan', self.on_plan, 10)
                    self.map_last_written = {}
                    for label, topic in [('static', '/cleannav/localization/map'),
                                         ('global_costmap', '/global_costmap/costmap'),('local_costmap','/local_costmap/costmap')]:
                        self.create_subscription(OccupancyGrid, topic,
                            lambda msg, name=label: self.on_grid(msg, name),
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                self.create_subscription(PoseStamped, '/cleannav/rviz/goal_pose' , self.on_goal, 10)
                self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/initialpose', self.on_initial_pose, 10)

            def on_sequence(self, msg):
                from rosidl_runtime_py.convert import message_to_ordereddict
                item = dict(message_to_ordereddict(msg))
                if msg.sequence_id != args.run_id or msg.command_id != args.run_id+'-patrol':
                    if msg.sequence_id:
                        fail('unexpected task sequence identity', 'mission_manager')
                    return
                if msg.current_execution_id:
                    self.patrol_owner = msg.current_execution_id
                    self.patrol_owners.add(msg.current_execution_id)
                self.patrol_progress = max(self.patrol_progress, msg.current_segment)
                self.patrol_status = item
                self.patrol_statuses.append(item)
                event('patrol_sequence_status', **item)

            def on_task_status(self, msg):
                from rosidl_runtime_py.convert import message_to_ordereddict
                item = dict(message_to_ordereddict(msg))
                self.patrol_tasks.append(item)
                event('patrol_task_status', **item)

            def submit_patrol(self):
                from rosidl_runtime_py.convert import message_to_ordereddict
                if self.patrol_started is not None:
                    raise ContractError('duplicate patrol submission')
                if not self.patrol_client.service_is_ready():
                    raise ContractError('SubmitTaskSequence service is not ready')
                ok, endpoints = one_expected_publisher(
                    self.get_publishers_info_by_topic('/cleannav/task_sequence_status'), 'mission_manager_node',
                    expected_type='cleannav_interfaces/msg/TaskSequenceStatus')
                if not ok:
                    raise ContractError('task sequence status publisher identity rejected')
                request = SubmitTaskSequence.Request()
                request.header.stamp = self.get_clock().now().to_msg()
                request.interface_version = '1.0'
                request.sequence_id = args.run_id
                request.command_id = args.run_id+'-patrol'
                request.source = 2
                request.valid_for.sec = 30
                request.user_confirmed = True
                request.map_id = patrol_spec['map_identity']['map_id']
                request.map_revision = patrol_spec['map_identity']['revision']
                request.map_sha256 = patrol_spec['map_identity']['sha256']
                goto = TaskSequenceStep()
                goto.step_id = 'goto-A-entry'; goto.operation = TaskSequenceStep.GOTO_REGION
                goto.region_id = patrol_spec['region_id']
                lap = TaskSequenceStep()
                lap.step_id = 'patrol-A-one-lap'; lap.operation = TaskSequenceStep.PATROL_REGION
                lap.region_id = patrol_spec['region_id']; lap.route_id = patrol_spec['route_id']; lap.laps = 1
                request.steps = [goto, lap]
                self.patrol_started = time.monotonic()
                event('patrol_sequence_request', request=message_to_ordereddict(request), publishers=endpoints)
                self.patrol_future = self.patrol_client.call_async(request)
                def received(future):
                    response = future.result()
                    event('patrol_sequence_response', response=message_to_ordereddict(response))
                    if not response.accepted or response.replayed or response.sequence_id != args.run_id:
                        fail('patrol sequence rejected/replayed: '+response.message, 'mission_manager')
                    self.patrol_accepted = True
                    if response.current_execution_id:
                        self.patrol_owner = response.current_execution_id
                        self.patrol_owners.add(response.current_execution_id)
                self.patrol_future.add_done_callback(received)

            def on_plan(self, msg):
                from rosidl_runtime_py.convert import message_to_ordereddict
                atomic_json(run_dir/'live-plan.json',dict(message_to_ordereddict(msg)))
                self.plan_points=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
                event('navigation_plan_observed', frame_id=msg.header.frame_id,
                      pose_count=len(msg.poses))

            def on_grid(self, msg, label):
                if label=='static':self.static_grid_message=msg
                if time.monotonic()-self.map_last_written.get(label, -1e9) < 1.0:
                    return
                self.map_last_written[label] = time.monotonic()
                origin=msg.info.origin
                atomic_json(run_dir/('live-'+label+'.json'), dict(
                    frame_id=msg.header.frame_id, monotonic_ns=time.monotonic_ns(),
                    width=msg.info.width, height=msg.info.height, resolution=msg.info.resolution,
                    origin=[origin.position.x, origin.position.y, origin.orientation.z, origin.orientation.w],
                    data=list(msg.data)))

            def on_goal(self, msg):
                self.rviz_goal_count += 1
                record = {'frame_id': msg.header.frame_id,
                          'stamp_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                          'monotonic_ns': time.monotonic_ns(), 'count': self.rviz_goal_count}
                self.rviz_goal_stamp_ns = record['stamp_ns']
                event('rviz_goal_observed', **record)
                if patrol_motion:
                    fail('GUI goal is forbidden during the bound patrol sequence', 'safety')
                if not any_motion:
                    fail('unexpected navigation goal in no-motion session', 'safety')
                elif historical_motion:
                    if historical_gui_ready_at is None or self.rviz_goal_count > 1:
                        fail('GUI goal before historical readiness or second goal', 'safety')
                elif not motion_admitted or self.rviz_goal_count > 1:
                    fail('RViz goal arrived before Admission or exceeded the one-goal Scope', 'safety')
                else:
                    confirmed = self.safety_state if isinstance(self.safety_state, dict) else None
                    owner = (confirmed or {}).get("owner")
                    decision = admitted_safety_receipt_decision(
                        guard, confirmed, now_monotonic=time.monotonic(),
                        deadline_monotonic=t0 + READINESS_DEADLINE_FROM_T0_S,
                        expected_run_id=args.run_id,
                        expected_writer_pid=guard_writer_pid,
                        motion_admitted=True,
                        mission_lease_active=owner == (
                            self.patrol_owner if patrol_motion else "ros-execution-1"),
                        expected_owner=(self.patrol_owner if patrol_motion else "ros-execution-1"),
                        generation_stamp_ns=None if confirmed is None else confirmed.get(
                            "generation_stamp_ns"),
                        minimum_receipt_stamp_ns=guard_receipt_confirmed_stamp_ns)
                    if decision["action"] != "accept":
                        fail(decision["reason"] if decision["action"] == "fail" else
                             "RViz goal arrived before the Safety guard receipt confirmed the current status",
                             "safety")

            def on_safety(self, msg):
                previous_owner = (self.safety_state.get("owner")
                                  if isinstance(self.safety_state, dict) else None)
                sample = {
                    'block_all': msg.block_all, 'estop': msg.emergency_stop_active,
                    'enabled': msg.autonomous_enabled,
                    'owner': msg.lease_owner_execution_id or '',
                    'monotonic_ns': time.monotonic_ns(),
                    'stamp_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                }
                try:
                    self.safety_state = observe_safety_sample(self.safety_state, sample)
                except ContractError as error:
                    fail(str(error), 'safety')
                if previous_owner != self.safety_state['owner']:
                    transition = {"run_id": args.run_id,
                        "source": "/cleannav/safety_status",
                        "previous_owner": previous_owner,
                        "owner": self.safety_state['owner'],
                        **self.safety_state}
                    self.safety_owner_history.append(transition)
                    event("safety_lease_owner_transition", **transition)

            def on_motion_event(self, msg):
                try:
                    payload = json.loads(msg.data)
                except (ValueError, TypeError):
                    fail('malformed RViz motion adapter event', 'safety')
                if not historical_motion and payload.get('run_id') != args.run_id:
                    fail('RViz motion adapter event Run ID mismatch', 'safety')
                if patrol_motion:
                    fail('unexpected RViz adapter event in patrol mode', 'safety')
                self.motion_events.append(payload)
                event('rviz_motion_adapter_event', **payload)

            def on_command(self, msg, topic='/cmd_vel'):
                values = [msg.linear.x, msg.linear.y, msg.linear.z,
                          msg.angular.x, msg.angular.y, msg.angular.z]
                moving = any(abs(value) > 1e-8 for value in values)
                now_ns = time.monotonic_ns()
                self.command_counts[topic] += 1
                self.command_zero_counts[topic] += int(not moving)
                self.command_nonzero_counts[topic] += int(moving)
                record = {'topic': topic, 'values': values, 'nonzero': moving,
                          'monotonic_ns': now_ns}
                pending = self.command_last.get(topic, {}).get('pending_safety_values')
                if pending is not None:
                    record['pending_safety_values'] = pending
                self.command_last[topic] = record
                if any_motion and motion_terminal_monotonic is not None:
                    if moving:
                        motion_post_terminal_zero_counts[topic] = 0
                    else:
                        motion_post_terminal_zero_counts[topic] += 1
                if topic == '/cmd_vel':
                    self.command_samples += 1
                    self.nonzero_samples += int(moving)
                    self.last_command = record
                if historical_motion:
                    from guard_policy import command_error
                    safety = self.safety_state
                    if safety and now_ns-safety.get('monotonic_ns', 0) > 1_000_000_000:
                        safety = None
                    error = command_error(values, True, safety,
                        expected_owner=(observer.patrol_owner if patrol_motion else 'ros-execution-1'), motion_profile=args.motion_profile)
                    if (error == 'command without valid existing Safety/Lease state'
                            and topic != '/cmd_vel' and safety
                            and safety.get('block_all') is False
                            and safety.get('estop') is False
                            and safety.get('enabled') is False and not safety.get('owner')):
                        record['pending_safety_values'] = values
                        event('command_safety_check_pending', **record)
                        error = None
                    if error and not failures:
                        fail(f'{topic} historical command contract: {error}', 'safety')
                elif motion_mode:
                    from short_goal_policy import command_policy, validate_lifecycle
                    safety = self.safety_state
                    safety_age = (now_ns-safety['monotonic_ns']
                                  if isinstance(safety, dict) else None)
                    if safety_age is None or not 0 <= safety_age <= 1_000_000_000:
                        safety = None
                    admitted = False
                    try:
                        run_path = Path(args.run_dir)
                        admission = json.loads((run_path/'motion-admission.json').read_text())
                        validate_lifecycle(run_id=args.run_id,
                            lifecycle_path=run_path/'motion-lifecycle.json',
                            authorization_path=run_path/'motion-authorization.json',
                            scope_path=run_path/'motion-scope.json', now_monotonic_ns=now_ns)
                        admitted = admission.get('run_id') == args.run_id and admission.get('status') == 'ADMITTED'
                    except (OSError, ValueError):
                        admitted = False
                    mission_accepted = any(item.get('event') == 'ACCEPTED'
                        and item.get('execution_id') == 'ros-execution-1'
                        and item.get('command_id') == motion_click_command_id
                        for item in self.motion_events)
                    error = command_policy(values, safety=safety,
                        admitted=admitted, goal_seen=mission_accepted)
                    if error:
                        fail(f'{topic} command contract: {error}', 'safety')
                elif topic == '/cmd_vel':
                    validate_zero_command(values)
                event('command_chain_sample' if any_motion else 'cmd_vel_observed', **record)

            def request_ground_truth(self, now):
                if not any_motion:
                    return
                if self.ground_truth_future is not None and self.ground_truth_future.done():
                    future, self.ground_truth_future = self.ground_truth_future, None
                    try:
                        response = future.result()
                        state = gazebo_entity_state_from_response(response)
                        sample = {
                            'run_id': args.run_id, 'source': '/gazebo/get_entity_state',
                            'entity_name': 'cleannav_ackermann', 'reference_frame': 'world',
                            'monotonic_ns': time.monotonic_ns(),
                            'position': [state.pose.position.x, state.pose.position.y,
                                         state.pose.position.z],
                            'orientation_xyzw': [state.pose.orientation.x,
                                state.pose.orientation.y, state.pose.orientation.z,
                                state.pose.orientation.w],
                            'linear_velocity': [state.twist.linear.x, state.twist.linear.y,
                                                state.twist.linear.z],
                            'angular_velocity': [state.twist.angular.x, state.twist.angular.y,
                                                 state.twist.angular.z],
                        }
                        if motion_mode and self.nonzero_samples:
                            from short_goal_rviz_adapter import _known_free_neighborhood
                            from guard_policy import point_path_distance
                            # Conservative circumscribed radius from the existing configured footprint.
                            import yaml
                            cfg=yaml.safe_load((Path(prefixes['cleannav_navigation'])/'share/cleannav_navigation/config/nav2_params_ackermann.yaml').read_text())
                            global_cfg=cfg['global_costmap']['global_costmap']['ros__parameters']
                            footprint=json.loads(global_cfg['footprint'])
                            radius=max(math.hypot(*p) for p in footprint)
                            x,y=sample['position'][:2]
                            if not _known_free_neighborhood(self.static_grid_message,x,y,radius):
                                fail('truth footprint entered occupied/unknown or map unavailable','safety')
                            if point_path_distance((x,y),self.plan_points)>global_cfg['inflation_layer']['inflation_radius']:
                                fail('truth deviated beyond existing path inflation corridor','safety')
                        self.ground_truth_samples.append(sample)
                        with (Path(args.run_dir)/'gazebo-ground-truth.jsonl').open('a') as stream:
                            stream.write(json.dumps(sample, sort_keys=True)+'\n')
                        event('gazebo_ground_truth_sample', **sample)
                    except Exception as error:
                        event('gazebo_ground_truth_error', error=f'{type(error).__name__}: {error}')
                if (self.ground_truth_future is None and now >= self.ground_truth_next_request
                        and self.ground_truth_client.service_is_ready()):
                    request = GetEntityState.Request()
                    request.name = 'cleannav_ackermann'
                    request.reference_frame = 'world'
                    self.ground_truth_future = self.ground_truth_client.call_async(request)
                    self.ground_truth_next_request = now + 0.5

            def on_initial_pose(self, msg):
                pose = msg.pose.pose
                sample = dict(frame_id=msg.header.frame_id,
                    stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                    position=[pose.position.x,pose.position.y,pose.position.z],
                    orientation=[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w],
                    monotonic_ns=time.monotonic_ns())
                self.initial_pose_samples.append(sample)
                event('initial_pose_observed', **sample)

        observer = SupervisorObserver()

        def set_simulation_interlock(enabled: bool, *, timeout_s: float = 3.0) -> dict:
            """Use the existing simulation interlock service; never acquire a Lease here."""
            if not observer.safety_client.service_is_ready():
                return {"success": False, "stage": "service_discovery",
                        "reason": "simulation interlock service is unavailable"}
            current = observer.safety_state
            if not enabled and (current is None
                    or time.monotonic_ns() - current.get("monotonic_ns", 0) > 1_000_000_000
                    or current.get("owner") not in ("", None)):
                return {"success": False, "stage": "close_precondition",
                        "reason": "cannot close interlock while Safety owner is active or unknown",
                        "safety": current}
            future = observer.safety_client.call_async(SetBool.Request(data=enabled))
            deadline = time.monotonic() + timeout_s
            response_record = None
            while time.monotonic() < deadline and ros.ok():
                ros.spin_once(observer, timeout_sec=0.05)
                if future.done():
                    try:
                        response = future.result()
                        response_record = {"success": bool(response.success),
                                           "message": str(response.message)}
                    except Exception as error:
                        response_record = {"success": False,
                            "reason": f"{type(error).__name__}: {error}"}
                    break
            if response_record is None:
                return {"success": False, "stage": "service_response",
                        "reason": "simulation interlock service response timed out"}
            if response_record.get("success") is not True:
                return {"success": False, "stage": "service_response",
                        "response": response_record}
            target_deadline = time.monotonic() + timeout_s
            while time.monotonic() < target_deadline and ros.ok():
                state = observer.safety_state
                fresh = (state is not None
                         and 0 <= time.monotonic_ns()-state.get("monotonic_ns", 0)
                         <= 1_000_000_000)
                if fresh:
                    if enabled:
                        observed = (state.get("block_all") is False
                            and state.get("estop") is False
                            and state.get("enabled") is False
                            and state.get("owner") in ("", None))
                    else:
                        observed = (state.get("block_all") is True
                            and state.get("enabled") is False
                            and state.get("owner") in ("", None))
                    if observed:
                        return {"success": True, "stage": "safety_status_confirmed",
                                "request_enable": enabled, "response": response_record,
                                "safety": state}
                ros.spin_once(observer, timeout_sec=0.05)
            return {"success": False, "stage": "safety_status_confirmation",
                    "response": response_record, "safety": observer.safety_state}

        def confirm_admitted_safety_receipt(*, admitted: bool) -> None:
            """Re-read until the guard receipt covers the current Safety sample.

            Admission stays unconfirmed, and a goal stays rejected, until this
            returns. A stale receipt, a deadline, or a conflicting sample fails
            closed. This does not sleep and then assume the states match.
            """
            nonlocal guard, guard_writer_pid, guard_receipt_confirmed_stamp_ns
            deadline = t0 + READINESS_DEADLINE_FROM_T0_S
            while True:
                owner = ((observer.safety_state or {}).get("owner")
                         if isinstance(observer.safety_state, dict) else None)
                confirmed = observer.safety_state if isinstance(observer.safety_state, dict) else None
                decision = admitted_safety_receipt_decision(
                    guard, confirmed, now_monotonic=time.monotonic(),
                    deadline_monotonic=deadline, expected_run_id=args.run_id,
                    expected_writer_pid=guard_writer_pid, motion_admitted=admitted,
                    mission_lease_active=owner == (
                        observer.patrol_owner if patrol_motion else "ros-execution-1"),
                    expected_owner=(observer.patrol_owner if patrol_motion else "ros-execution-1"),
                    generation_stamp_ns=None if confirmed is None else confirmed.get(
                        "generation_stamp_ns"),
                    minimum_receipt_stamp_ns=guard_receipt_confirmed_stamp_ns)
                if decision["action"] == "fail":
                    raise ContractError(decision["reason"])
                if decision["action"] == "accept":
                    guard_writer_pid = decision["writer_pid"]
                    guard_receipt_confirmed_stamp_ns = decision["stamp_ns"]
                    if motion_admitted:
                        path=run_dir/'motion-admission.json'
                        proof=json.loads(path.read_text())
                        if proof['safety_status'].get('generation_stamp_ns') != confirmed.get('generation_stamp_ns'):
                            proof.update(safety_status=confirmed)
                            atomic_json(path,proof)
                    return
                event("safety_receipt_confirmation_pending",
                      receipt_sequence=(guard or {}).get("receipt_sequence"),
                      receipt_stamp_ns=((guard or {}).get("safety") or {}).get("stamp_ns"),
                      confirmed_stamp_ns=(observer.safety_state or {}).get("stamp_ns")
                      if isinstance(observer.safety_state, dict) else None)
                ros.spin_once(observer, timeout_sec=0.01)
                guard, _ = refresh_guard_receipt(guard_path, guard)

        def revoke_motion_before_shutdown(reason: str) -> dict:
            """Revoke this run's task receipts/interlock before stopping owned nodes.

            Mission Manager remains the only Safety Lease owner. This routine
            requests ESTOP/cancel, waits for an actually observed Lease release,
            closes the simulation interlock, and only then records local Auth/
            Scope revocation. It never fabricates a Lease release.
            """
            nonlocal motion_interlock_open, motion_close_confirmed
            # Revoke the consumable proof before waiting for any ROS service.
            admission_path=run_dir/'motion-admission.json'
            if motion_mode and admission_path.exists():
                proof=json.loads(admission_path.read_text())
                proof.update(status='REVOKED',reason=reason)
                atomic_json(admission_path,proof)
            if historical_motion:
                if observer is None or not ros.ok():
                    return {"status": "UNKNOWN", "verified": False,
                            "reason": "observer unavailable for historical stop"}
                state = observer.safety_state
                if not state:
                    return {"status": "UNKNOWN", "verified": False,
                            "reason": "Safety state unavailable"}
                if state.get('owner') or failures:
                    observer.stop_publisher.publish(Bool(data=True))
                    if observer.cancel_client.service_is_ready():
                        observer.cancel_client.call_async(CancelGoal.Request())
                deadline = min(time.monotonic()+5.0, t0+SOFT_STOP_FROM_T0_S)
                while time.monotonic() < deadline:
                    ros.spin_once(observer, timeout_sec=.05)
                    state = observer.safety_state or {}
                    if (state and not state.get('owner') and observer.last_command
                            and not observer.last_command.get('nonzero')):
                        break
                state=observer.safety_state or {}
                if state.get('owner') or not state:
                    return {"status": "UNKNOWN", "verified": False,
                            "reason": "Mission Lease release not observed", "safety": state}
                closed=set_simulation_interlock(False)
                receipt=dict(status='PASS' if closed.get('success') else 'FAIL',
                    verified=closed.get('success') is True, reason=reason,
                    existing_service_only=True, created_auth_scope=False, interlock=closed)
                atomic_json(run_dir/'historical-motion-stop.json', receipt)
                return receipt
            if not motion_mode:
                return {"status": "NOT_REQUIRED", "verified": True}
            auth_path = run_dir / "motion-authorization.json"
            scope_path = run_dir / "motion-scope.json"
            lifecycle_path = run_dir / "motion-lifecycle.json"
            has_receipts = all(path.is_file() for path in
                               (auth_path, scope_path, lifecycle_path))
            prior_revocation = read_json(run_dir / "motion-revocation.json")
            lifecycle_before = read_json(lifecycle_path) or {}
            if (motion_close_confirmed and prior_revocation
                    and lifecycle_before.get("status") == "CLOSED"):
                return {"status": "PASS", "verified": True,
                        "already_revoked": True, "revocation_receipt": prior_revocation}
            opening_or_open = (motion_interlock_open or motion_interlock_stage in
                               {"OPENING", "ADMITTED"})
            if not motion_authority_issued and not opening_or_open:
                return {"status": "NOT_REQUIRED", "verified": True}
            if not has_receipts:
                return {"status": "UNKNOWN", "verified": False,
                        "reason": "run-bound authority receipts are incomplete"}
            if ros is None or observer is None or not ros.ok():
                return {"status": "UNKNOWN", "verified": False,
                        "reason": "ROS observer is unavailable for motion revocation"}

            try:
                if observer.stop_publisher is not None:
                    observer.stop_publisher.publish(Bool(data=True))
                cancel_sent = False
                if observer.cancel_client is not None and observer.cancel_client.service_is_ready():
                    observer.cancel_client.call_async(CancelGoal.Request())
                    cancel_sent = True
                event("motion_shutdown_revoke_requested", reason=reason,
                      estop_published=observer.stop_publisher is not None,
                      nav2_cancel_requested=cancel_sent)

                from short_goal_policy import (
                    PROFILE_ID, close_lifecycle, update_lifecycle, update_scope,
                    write_revocation_receipt, sha256 as receipt_sha256,
                )

                deadline = time.monotonic() + LEASE_RELEASE_CONFIRMATION_S
                safety = observer.safety_state
                lease_acquired = any(item.get("event") == "LEASE_ACQUIRED"
                    and item.get("execution_id") == "ros-execution-1"
                    for item in observer.motion_events)
                lease_acquired = lease_acquired or any(
                    transition.get("owner") == "ros-execution-1"
                    for transition in observer.safety_owner_history)
                lease_released = any(item.get("event") == "LEASE_RELEASED"
                    and item.get("execution_id") == "ros-execution-1"
                    for item in observer.motion_events)
                lease_released = lease_released or any(
                    transition.get("previous_owner") == "ros-execution-1"
                    and transition.get("owner") in ("", None)
                    for transition in observer.safety_owner_history)
                while time.monotonic() < deadline:
                    try:
                        ros.spin_once(observer, timeout_sec=0.05)
                    except Exception as spin_error:
                        event("motion_shutdown_observer_error",
                              error=f"{type(spin_error).__name__}: {spin_error}")
                    safety = observer.safety_state
                    fresh = (isinstance(safety, dict)
                        and 0 <= time.monotonic_ns()-safety.get("monotonic_ns", 0)
                            <= 1_000_000_000)
                    lease_acquired = lease_acquired or any(
                        item.get("event") == "LEASE_ACQUIRED"
                        and item.get("execution_id") == "ros-execution-1"
                        for item in observer.motion_events)
                    lease_acquired = lease_acquired or any(
                        transition.get("owner") == "ros-execution-1"
                        for transition in observer.safety_owner_history)
                    lease_released = lease_released or any(
                        item.get("event") == "LEASE_RELEASED"
                        and item.get("execution_id") == "ros-execution-1"
                        for item in observer.motion_events)
                    lease_released = lease_released or any(
                        transition.get("previous_owner") == "ros-execution-1"
                        and transition.get("owner") in ("", None)
                        for transition in observer.safety_owner_history)
                    owner = safety.get("owner") if fresh else None
                    if owner == "ros-execution-1":
                        lease_acquired = True
                    elif fresh and owner in ("", None):
                        lifecycle_now = read_json(lifecycle_path) or {}
                        receipt_release = (lifecycle_now.get("status") == "CLOSED"
                            and lifecycle_now.get("lease_release_confirmed") is True
                            and isinstance(lifecycle_now.get("lease_released_monotonic_ns"), int))
                        if not lease_acquired or lease_released or receipt_release:
                            break
                    time.sleep(0.05)

                safety = observer.safety_state
                safety_fresh = (isinstance(safety, dict)
                    and 0 <= time.monotonic_ns()-safety.get("monotonic_ns", 0)
                        <= 1_000_000_000)
                owner_empty = safety_fresh and safety.get("owner") in ("", None)
                lifecycle_now = read_json(lifecycle_path) or {}
                lease_released = lease_released or (
                    lifecycle_now.get("status") == "CLOSED"
                    and lifecycle_now.get("lease_release_confirmed") is True
                    and isinstance(lifecycle_now.get("lease_released_monotonic_ns"), int))
                if not owner_empty or (lease_acquired and not lease_released):
                    failure = {"status": "FAIL", "verified": False,
                        "reason": "Safety owner cleared or Lease release could not be confirmed",
                        "lease_acquired": lease_acquired,
                        "lease_released": lease_released,
                        "safety": safety, "lifecycle": lifecycle_now,
                        "wait_budget_s": 5.0}
                    event("motion_shutdown_revocation_failed", **failure)
                    return failure

                close_result = None
                if opening_or_open:
                    close_result = set_simulation_interlock(False)
                    if close_result.get("success") is not True:
                        failure = {"status": "FAIL", "verified": False,
                            "reason": "Safety block-all interlock close was not confirmed",
                            "lease_acquired": lease_acquired,
                            "lease_released": lease_released,
                            "safety": observer.safety_state,
                            "interlock_close": close_result}
                        event("motion_shutdown_revocation_failed", **failure)
                        return failure
                    motion_interlock_open = False
                    motion_close_confirmed = True

                safety_final = observer.safety_state
                safety_final_fresh = (isinstance(safety_final, dict)
                    and 0 <= time.monotonic_ns()-safety_final.get("monotonic_ns", 0)
                        <= 1_000_000_000)
                interlock_closed = (safety_final_fresh
                    and safety_final.get("block_all") is True
                    and safety_final.get("enabled") is False
                    and safety_final.get("owner") in ("", None))
                if not interlock_closed:
                    failure = {"status": "FAIL", "verified": False,
                        "reason": "Safety did not return to fresh block-all idle before shutdown",
                        "safety": safety_final, "interlock_close": close_result}
                    event("motion_shutdown_revocation_failed", **failure)
                    return failure

                if prior_revocation and lifecycle_now.get("status") == "CLOSED":
                    from short_goal_policy import validate_revocation_receipt
                    verified_receipt = validate_revocation_receipt(
                        run_id=args.run_id, authorization_path=auth_path,
                        scope_path=scope_path, lifecycle_path=lifecycle_path,
                        receipt_path=run_dir / "motion-revocation.json",
                        lease_acquired=lease_acquired, lease_released=lease_released)
                    return {"status": "PASS", "verified": True,
                        "already_revoked": True, "lease_acquired": lease_acquired,
                        "lease_released": lease_released,
                        "safety_block_all_and_disabled_fresh": True,
                        "interlock_close": close_result,
                        "revocation_receipt": verified_receipt}

                scope_now = read_json(scope_path) or {}
                if scope_now.get("status") != "CLOSED":
                    update_scope(scope_path, expected_run_id=args.run_id,
                        expected_profile_id=PROFILE_ID, updates={
                            "status": "CLOSED", "lease_owner_execution_id": None,
                            "lease_released_monotonic_ns": (time.monotonic_ns()
                                if lease_acquired and lease_released else None),
                            "closed_monotonic_ns": time.monotonic_ns(),
                            "release_reason": reason,
                        })
                lifecycle_now = read_json(lifecycle_path) or {}
                if lifecycle_now.get("status") != "CLOSED":
                    update_lifecycle(lifecycle_path, expected_run_id=args.run_id,
                        expected_profile_id=PROFILE_ID,
                        updates={"scope_sha256": receipt_sha256(scope_path)})
                    lifecycle_now = close_lifecycle(lifecycle_path,
                        expected_run_id=args.run_id, expected_profile_id=PROFILE_ID,
                        monotonic_ns=time.monotonic_ns(), release_reason=reason,
                        lease_released=lease_released)
                revocation = write_revocation_receipt(run_id=args.run_id,
                    profile_id=PROFILE_ID, run_dir=run_dir,
                    authorization_path=auth_path, scope_path=scope_path,
                    lifecycle_path=lifecycle_path, monotonic_ns=time.monotonic_ns(),
                    release_reason=reason, lease_acquired=lease_acquired,
                    lease_released=lease_released)
                return {"status": "PASS", "verified": True,
                    "lease_acquired": lease_acquired, "lease_released": lease_released,
                    "safety_owner_empty_fresh": True,
                    "safety_block_all_and_disabled_fresh": True,
                    "interlock_close": close_result,
                    "lifecycle": lifecycle_now, "revocation_receipt": revocation}
            except Exception as error:
                failure = {"status": "UNKNOWN", "verified": False,
                    "error": f"{type(error).__name__}: {error}",
                    "traceback": traceback.format_exc(),
                    "safety": observer.safety_state,
                    "process_group_id": result.get("process_group_id")}
                event("motion_shutdown_revocation_error", **failure)
                return failure

        domain = runtime.check_domain_isolated(ros, observer)
        gate("ros_domain_isolation", domain)
        stability = runtime.collect_resource_stability_window(lambda: runtime.memory_snapshot(
            cgroup_waiver=waiver,
            min_available_kib=runtime.WAIVER_STARTUP_MIN_AVAILABLE_KIB,
            min_cgroup_headroom_bytes=runtime.CGROUP_STARTUP_MIN_HEADROOM_BYTES,
            memavailable_warning_only=True,
        ))
        preflight["gates"]["resource_stability_30s"] = stability
        if stability.get("status") != "VERIFIED":
            fail("resource stability window did not verify", "preflight")
        domain_after = runtime.check_domain_isolated(ros, observer)
        gate("ros_domain_after_stability", domain_after)
        gate("process_conflicts_after_stability", runtime.process_visibility_and_conflicts())
        gate("gazebo_port_after_stability", runtime.port_free(11704))
        cgroup_gate = runtime.cgroup_startup_gate(
            runtime.cgroup_memory_snapshot(), runtime.wsl_cgroup_waiver_preflight(),
            stable_window=stability,
            auxiliary_gates_ok=all(preflight["gates"][name].get("ok") is True for name in (
                "simulation_rtabmap_safety_prefixes", "ros_domain_after_stability",
                "process_conflicts_after_stability", "gazebo_port_after_stability")),
        )
        gate("cgroup_startup_gate", cgroup_gate)
        preflight["ok"] = True
        result["preflight"] = preflight
        event("preflight_passed", cgroup_disposition=cgroup_disposition,
              stability_samples=stability.get("sample_count"),
              stability_seconds=stability.get("actual_duration_seconds"))
        if LOCALIZATION_DIAGNOSTIC or historical_motion or motion_mode:
            import sqlite3
            from localization_diagnostics import DiagnosticRecorder, DiagnosticWorker
            from rclpy.executors import SingleThreadedExecutor
            sim_lib=Path(prefixes['cleannav_simulation'])/'lib/cleannav_simulation'
            sys.path.insert(0,str(sim_lib))
            from localization_health import accepted_match
            with sqlite3.connect('file:'+str(map_directory/'rtabmap.db')+'?mode=ro',uri=True) as db:
                saved_ids=[row[0] for row in db.execute('SELECT id FROM Node')]
            diagnostic_node=Node('cleannav_localization_diagnostics')
            recorder=DiagnosticRecorder(diagnostic_node,run_dir,args.run_id,saved_ids,accepted_match)
            diagnostic_executor=SingleThreadedExecutor(context=observer.context)
            diagnostic_executor.add_node(diagnostic_node)
            diagnostic=DiagnosticWorker(recorder,diagnostic_executor)
            diagnostic.start()
        if (LOCALIZATION_DIAGNOSTIC or any_motion) and elapsed() > PREFLIGHT_LATEST_LAUNCH_S:
            fail('preflight exceeded the selected profile latest launch T0+55s; bounded observation/task window cannot be guaranteed', 'preflight_budget')
        if elapsed() + MIN_CLEANUP_RESERVE_S + NO_MOTION_OBSERVATION_S >= SOFT_STOP_FROM_T0_S:
            fail("insufficient T0 budget remains for launch and minimum observation", "preflight_budget")

        # Diagnostic soft stop already precedes the cleanup/evidence cutoff.
        # Reserving cleanup twice lets the child timer kill active Nav2 before
        # diagnostic observation or historical GUI/task time has completed.
        launch_cleanup_reserve = 0 if (LOCALIZATION_DIAGNOSTIC or historical_motion) else MIN_CLEANUP_RESERVE_S
        runtime_budget = min(
            MAX_LAUNCH_RUNTIME_S,
            SOFT_STOP_FROM_T0_S - elapsed() - launch_cleanup_reserve)
        cmd = bounded_launch_argv(world=world, map_directory=map_directory,
                                  map_profile=args.map_profile,
                                  output_dir=run_dir, runtime_timeout_s=runtime_budget,
                                  motion_profile=args.motion_profile,
                                  odometry_mode=args.odometry_mode)
        launch_mode = (verify_short_goal_launch_argv(cmd, expected_profile=args.motion_profile)
                       if any_motion else verify_no_motion_launch_argv(cmd))
        result["launch_mode_contract"] = launch_mode
        diagnostic_memory_policy = (LOCALIZATION_DIAGNOSTIC
                                    and launch_mode["enable_motion"] is False
                                    and launch_mode["auto_start_task"] is False)
        result["runtime_memory_policy"] = (
            "localization-diagnostic-no-motion-pressure-evidence"
            if diagnostic_memory_policy else ("historical-motion-pc-pressure-evidence"
                if historical_motion else "existing_runtime_gate"))
        result["launch_command"] = cmd
        result["launch_source"] = {"path": str(ENTRY), "sha256": sha256(ENTRY),
                                   "is_repository_source": True}
        launch_log = (run_dir / "launch.log").open("wb")

        def spawn():
            return subprocess.Popen(cmd, cwd=str(NAV), env={**os.environ,
                                    'CLEANNAV_RVIZ_RUN_ID': args.run_id,
                                    'CLEANNAV_RVIZ_RUN_DIR': str(run_dir)},
                                    stdout=launch_log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True,
                                    close_fds=True)

        launch_monotonic_ns = time.monotonic_ns()
        process = launch_after_preflight(preflight, spawn)
        launched = True
        result.update(launch_started=True, launch_pid=process.pid, process_group_id=process.pid,
                      ros_log_dir=str(ros_log_dir))
        result["launch_started_monotonic_ns"] = launch_monotonic_ns
        launch_start = proc_start_ticks(process.pid)
        atomic_json(run_dir / "supervisor_state.json", {
            "run_id": args.run_id, "supervisor_pid": os.getpid(),
            "supervisor_start_ticks": proc_start_ticks(os.getpid()),
            "launch_pid": process.pid, "launch_pgid": process.pid,
            "launch_start_ticks": launch_start,
            "launch_started_monotonic_ns": launch_monotonic_ns,
            "timing_profile": TIMING_PROFILE,
            "readiness_all_passed": False,
            "readiness_phases": dict(ledger.states),
            "t0_monotonic_ns": args.t0_ns,
            "startup_verification_complete": False,
        })
        event("launch_started", argv=cmd, pid=process.pid, pgid=process.pid)
        if diagnostic_memory_policy or historical_motion:
            try:
                memory_telemetry = ProcessMemoryTelemetry(run_dir, args.run_id)
                result["memory_attribution"] = {
                    "status": "RECORDING", "timeseries_csv": "process-memory-timeseries.csv",
                    "summary_json": "memory-attribution-summary.json",
                    "sample_interval_target_s": 1.0,
                    "late_samples_affect_readiness_or_stop": False,
                }
                telemetry_sample = memory_telemetry.sample(
                    launch_pgid=process.pid, supervisor_pid=os.getpid(),
                    t0_ns=args.t0_ns, launch_start_ns=launch_monotonic_ns,
                    label="first_after_launch")
                event("process_memory_sample", **telemetry_sample)
            except Exception as telemetry_error:
                result["memory_attribution"] = {
                    "status": "PARTIAL", "error": f"{type(telemetry_error).__name__}: {telemetry_error}",
                    "late_samples_affect_readiness_or_stop": False,
                }
                event("process_memory_telemetry_error", error=str(telemetry_error))
        log_read = (run_dir / "launch.log").open("r", encoding="utf-8", errors="replace")
        graph_gate = None
        initial_wait_marker = False
        phase_records = {}
        previous_conditions = {}
        previous_initialization_graph = None
        last_stale_guard_stamp = None
        previous_cpu = linux_cpu_counters()
        previous_resource_monotonic = time.monotonic()
        last_resource_sample_monotonic = previous_resource_monotonic
        while True:
            watchdog_failure = (read_json(run_dir / "hard_deadline.json")
                                or read_json(run_dir / "watchdog_failure.json"))
            if watchdog_failure:
                fail("independent deadline watchdog reported stop/failure: "
                     + str(watchdog_failure.get("reason", watchdog_failure.get("error", "unknown"))),
                     "hard_deadline" if watchdog_failure.get("hard_deadline_enforced") else "watchdog")
            if diagnostic is not None and diagnostic.error:
                fail('localization diagnostic receiver failed: '+diagnostic.error, 'diagnostics')
            if STOP_REQUESTED:
                stop_reason = "external_" + STOP_REQUESTED
                break
            if elapsed() >= SOFT_STOP_FROM_T0_S:
                stop_reason = f"soft_stop_deadline_T0_plus_{SOFT_STOP_FROM_T0_S:g}s"
                break
            resource_now = time.monotonic()
            if resource_now - last_resource_sample_monotonic >= 1.0:
                current_cpu = linux_cpu_counters()
                cpu = cpu_usage_percent(previous_cpu, current_cpu,
                    resource_now - previous_resource_monotonic, os.sysconf('SC_CLK_TCK'))
                memory = runtime.memory_snapshot(
                    cgroup_waiver=waiver,
                    min_available_kib=(0 if diagnostic_memory_policy or historical_motion
                                       else runtime.WAIVER_STARTUP_MIN_AVAILABLE_KIB),
                    min_cgroup_headroom_bytes=runtime.CGROUP_STARTUP_MIN_HEADROOM_BYTES,
                    memavailable_warning_only=True)
                telemetry_sample = None
                if memory_telemetry is not None:
                    try:
                        telemetry_sample = memory_telemetry.sample(
                            launch_pgid=process.pid, supervisor_pid=os.getpid(),
                            t0_ns=args.t0_ns, launch_start_ns=launch_monotonic_ns,
                            label="runtime")
                        memory["oom_kill_delta"] = telemetry_sample.get("oom_kill_delta")
                        memory["oom_kill_count"] = telemetry_sample.get("oom_kill_count")
                        event("process_memory_sample", **telemetry_sample)
                    except Exception as telemetry_error:
                        event("process_memory_telemetry_error",
                              error=f"{type(telemetry_error).__name__}: {telemetry_error}")
                memory_decision = runtime_memory_decision(
                    memory, timing_profile=TIMING_PROFILE,
                    no_motion_contract=diagnostic_memory_policy)
                pressure = pressure_snapshot()
                resource_sample = {
                    "sample_index": resource_samples + 1,
                    "monotonic_ns": time.monotonic_ns(),
                    "t0_elapsed_s": round(elapsed(), 6),
                    "launch_elapsed_s": round((time.monotonic_ns()-launch_monotonic_ns)/1e9, 6),
                    **cpu,
                    "cpu_counters": current_cpu,
                    "MemAvailable_kib": memory.get("MemAvailable_kib"),
                    "SwapUsed_kib": memory.get("SwapUsed_kib"),
                    "memory_full_psi_avg10": memory.get("memory_full_psi_avg10"),
                    "pressure": pressure,
                    "resource_snapshot_ok": memory.get("ok"),
                    "resource_snapshot_reasons": memory.get("reasons"),
                    "resource_gate_ok": memory_decision.get("ok"),
                    "resource_gate_reasons": memory_decision.get("reasons"),
                    "runtime_memory_policy": memory_decision.get("policy"),
                    "memory_soft_warning": memory_decision.get("soft_warning"),
                    "memory_soft_warning_threshold_kib": memory_decision.get(
                        "soft_warning_threshold_kib"),
                    "memory_hard_stop_threshold_kib": memory_decision.get(
                        "hard_stop_threshold_kib"),
                    "memory_historical_single_sample_stop_line_kib": memory_decision.get(
                        "historical_single_sample_stop_line_kib"),
                    "memory_oom_kill_delta": memory_decision.get("oom_kill_delta"),
                    "process_memory_sample_index": (telemetry_sample or {}).get("sample_index"),
                    "process_memory_sample_duration_s": (telemetry_sample or {}).get("duration_s"),
                    "process_memory_sample_late": (telemetry_sample or {}).get("late"),
                    "process_memory_scan_status": (telemetry_sample or {}).get("scan_status"),
                    "cgroup_memory_status": memory.get("cgroup_memory_status"),
                    "cgroup_memory_disposition": memory.get("cgroup_memory_disposition"),
                }
                resource_samples += 1
                event("runtime_resource_sample", **resource_sample)
                if memory_decision.get("soft_warning") is True:
                    runtime_memory_soft_warning_samples += 1
                    event("runtime_memory_soft_warning",
                          sample_index=resource_sample["sample_index"],
                          MemAvailable_kib=memory_decision.get("MemAvailable_kib"),
                          warning_threshold_kib=memory_decision.get(
                              "soft_warning_threshold_kib"),
                          hard_stop_threshold_kib=memory_decision.get(
                              "hard_stop_threshold_kib"),
                          historical_single_sample_stop_line_kib=memory_decision.get(
                              "historical_single_sample_stop_line_kib"),
                          policy=memory_decision.get("policy"))
                if memory_decision.get("ok") is not True:
                    fail("runtime memory/swap/PSI/cgroup gate rejected: "
                         + ",".join(memory_decision.get("reasons")
                                    or ["unknown resource failure"]),
                         "resource")
                previous_cpu = current_cpu
                previous_resource_monotonic = resource_now
                last_resource_sample_monotonic = resource_now
            try:
                ros.spin_once(observer, timeout_sec=0.05)
                # The three command streams can outpace one callback per
                # graph/resource cycle. Drain ready callbacks within a small
                # bound so Safety updates are checked before they age out.
                observer_spin_deadline = time.monotonic() + .02
                while time.monotonic() < observer_spin_deadline:
                    ros.spin_once(observer, timeout_sec=0.0)
            except ContractError as error:
                fail(str(error), "safety")
            observer.request_ground_truth(time.monotonic())
            if process.poll() is not None:
                fail("launch main process exited before supervised observation completed: "
                     + str(process.returncode), "launch_process")
            log_read.seek(launch_log_offset)
            new_text = log_read.read()
            launch_log_offset = log_read.tell()
            if new_text:
                log_text += new_text
                event("launch_log_append", bytes=len(new_text.encode(errors="replace")))
                if "[ERROR] FAILED" in new_text:
                    fail("integrated launch readiness probe reported failure",
                         phase_from_failure_text(new_text))
            map_coordinator = read_json(discover_artifact(
                ros_log_dir, "map-server-lifecycle-coordinator-result.json"))
            if map_coordinator and map_coordinator.get("status") == "FAILED_CLOSED":
                fail("Map Server lifecycle coordinator failed: "
                     + str(map_coordinator.get("error", "unknown failure")), "map_localization")
            collision_path = discover_artifact(ros_log_dir, "collision-lifecycle.json")
            guard_path = discover_artifact(ros_log_dir, "safety-guard.json")
            collision = read_json(collision_path)
            guard, before_graph_read = refresh_guard_receipt(guard_path, None)
            event('safety_receipt_read', point='before_graph', **before_graph_read)
            if not LOCALIZATION_DIAGNOSTIC and collision is not None and collision.get("passed") is not True:
                fail("Collision Monitor lifecycle receipt rejected: "
                     + str(collision.get("decision", "missing PASS decision")), "collision_monitor")
            if guard and guard.get("failed"):
                fail("integrated Safety guard failed: " + str(guard["failed"]), "safety")
            if guard and guard.get("failed"):
                fail("integrated Safety guard failed: " + str(guard.get("failed")), "safety")
            procs = process_group_processes(process.pid)
            gazebo_process = any(item["comm"] == "gzserver"
                                 or "gzserver" in item.get("cmdline", "") for item in procs)
            rviz_process = any(item["comm"] == "rviz2" or
                               "rviz2" in item.get("cmdline", "") for item in procs)
            nodes = {(name, namespace) for name, namespace in observer.get_node_names_and_namespaces()}
            rviz_node = expected_rviz_graph_node(nodes)
            mission_node = ("mission_manager_node", "/") in nodes
            readiness = phase_readiness(log_text, collision, guard,
                                        rviz_node=rviz_node, rviz_process=rviz_process,
                                        gazebo_process=gazebo_process, mission_node=mission_node,
                                        localization_scan_topic=map_contract['scan_topic'],
                                        now_monotonic=time.monotonic(), motion_mode=any_motion,
                                        motion_admitted=(historical_motion or motion_admitted),
                                        expected_owner=(observer.patrol_owner if patrol_motion else "ros-execution-1"),
                    mission_lease_active=bool(observer.safety_state and
                                            observer.safety_state.get('owner')==(observer.patrol_owner if patrol_motion else 'ros-execution-1')))
            initialization_graph = dict(topic='/rtabmap/initialpose',
                message_type='geometry_msgs/msg/PoseWithCovarianceStamped', expected_frame='map',
                subscribers=[endpoint_record(e) for e in observer.get_subscriptions_info_by_topic('/rtabmap/initialpose')],
                publishers=[endpoint_record(e) for e in observer.get_publishers_info_by_topic('/rtabmap/initialpose')],
                rviz_node=rviz_node, rviz_process=rviz_process)
            initialization_graph['rtabmap_receiver_confirmed'] = any(
                e['node_name']=='rtabmap' and e['node_namespace']=='/rtabmap'
                and e['topic_type']=='geometry_msgs/msg/PoseWithCovarianceStamped'
                for e in initialization_graph['subscribers'])
            if initialization_graph != previous_initialization_graph:
                event('initialization_interface_changed', evidence=initialization_graph)
                previous_initialization_graph = initialization_graph
            result['localization_initialization'] = dict(initialization_graph,
                observed_initial_pose_messages=observer.initial_pose_samples)
            atomic_json(run_dir/'localization-initialization.json',
                dict(run_id=args.run_id,monotonic_ns=time.monotonic_ns(),**result['localization_initialization']))
            if readiness["safety"][0] and graph_gate is None:
                graph_gate = PublisherGraphConvergence(time.monotonic_ns())
            if graph_gate is not None:
                graph_sample = graph_gate.sample_query(
                    lambda: observer.get_publishers_info_by_topic("/cmd_vel"),
                    time.monotonic_ns,
                )
                event("cmd_vel_publisher_graph", sample=graph_sample)
                if graph_gate.failure_reason:
                    fail("/cmd_vel publisher graph rejected: " + graph_gate.failure_reason, "safety")
                graph_records.append(graph_sample)
                if graph_gate.converged:
                    navigation_probe_ready = "[NAVIGATION] READY" in log_text
                    raw_records = [endpoint_record(endpoint) for endpoint in
                                   observer.get_publishers_info_by_topic(
                                       "/cleannav/navigation/cmd_vel_raw")]
                    raw_contract = raw_publisher_phase_contract(
                        raw_records, navigation_probe_ready=navigation_probe_ready)
                    event("command_input_graph", topic="/cleannav/navigation/cmd_vel_raw",
                          expected="controller_server", phase="nav2",
                          navigation_probe_ready=navigation_probe_ready,
                          **raw_contract)
                    if not raw_contract["ok"]:
                        failure_phase = "nav2" if navigation_probe_ready else "safety"
                        fail("command input publisher phase contract rejected: "
                             "/cleannav/navigation/cmd_vel_raw state="
                             + raw_contract["state"], failure_phase)
                    ok, records = one_expected_publisher(
                        observer.get_publishers_info_by_topic("/cleannav/cmd_vel_candidate"),
                        "collision_monitor")
                    if not ok:
                        fail("command input publisher identity rejected: "
                             "/cleannav/cmd_vel_candidate", "safety")
                    event("command_input_graph", topic="/cleannav/cmd_vel_candidate",
                          expected="collision_monitor", ok=ok, endpoints=records)
                    if list(observer.get_publishers_info_by_topic("/cleannav/manual/cmd_vel_raw")):
                        fail("unexpected manual raw command publisher exists", "safety")
                    if list(observer.get_publishers_info_by_topic("/cleannav/mapping/cmd_vel_raw")):
                        fail("unexpected mapping raw command publisher exists", "safety")
            ready_safety, safety_evidence = readiness["safety"]
            zero_commands = zero_command_observation_ok(
                observer.command_samples, observer.nonzero_samples)
            readiness["safety"] = (ready_safety and bool(graph_gate and graph_gate.converged)
                                    and zero_commands,
                                    {**safety_evidence,
                                     "supervisor_cmd_vel_graph_converged": bool(graph_gate and graph_gate.converged),
                                     "zero_cmd_vel_observed": zero_commands,
                                     "cmd_vel_message_count": observer.command_samples,
                                     "nonzero_cmd_vel_message_count": observer.nonzero_samples})
            # ROS graph inspection above may take longer than one guard timer
            # period. Re-read the source receipt immediately before making any
            # phase/loss decision so an old local object cannot create a false
            # stale failure while the guard keeps writing fresh receipts.
            previous_guard = guard
            latest_guard, receipt_snapshot = refresh_guard_receipt(
                guard_path, previous_guard)
            event("safety_receipt_read", point="after_graph", **receipt_snapshot)
            stale_guard_stamp = (previous_guard or {}).get("monotonic")
            if (receipt_snapshot["previous_status"] == "FAIL"
                    and stale_guard_stamp != last_stale_guard_stamp):
                event("safety_guard_receipt_snapshot_stale", status="FAIL",
                      health_readiness="FAIL", source_refresh=receipt_snapshot,
                      previous_guard_monotonic=(previous_guard or {}).get("monotonic"),
                      latest_guard_monotonic=(latest_guard or {}).get("monotonic"),
                      continued_from_refreshed_receipt=receipt_snapshot[
                          "usable_latest_receipt"])
                last_stale_guard_stamp = stale_guard_stamp
            guard = latest_guard
            if guard and guard.get("failed"):
                fail("integrated Safety guard failed after receipt refresh: "
                     + str(guard.get("failed")), "safety")
            if guard and guard.get("ready") and not LOCALIZATION_DIAGNOSTIC:
                try:
                    if motion_mode or (historical_motion and historical_gui_ready_at is not None):
                        if historical_motion:
                            safety_sync_started = time.monotonic()
                            safety_sync_zero_count = observer.nonzero_samples
                            while True:
                                try:
                                    validate_motion_guard_runtime_state(
                                        guard, now_monotonic=time.monotonic(),
                                        motion_admitted=True,
                                        expected_owner=(observer.patrol_owner if patrol_motion else "ros-execution-1"),
                                        mission_lease_active=bool(guard.get('safety', {}).get('owner')),
                                        observer_safety=observer.safety_state,
                                        minimum_safety_stamp_ns=observer.rviz_goal_stamp_ns,
                                        maximum_distance_m=((guard['route_budget']['odometry_origin_m']
                                            +guard['route_budget']['odometry_m'])
                                            if guard.get('route_budget') and guard['route_budget'].get('command_id')==motion_click_command_id
                                            else (patrol_spec["max_odometry_m"] if patrol_motion else MAX_HISTORICAL_ODOMETRY_M)))
                                    if (not (guard.get('safety') or {}).get('owner')
                                            and any(x.get('pending_safety_values') is not None
                                                for x in observer.command_last.values())):
                                        raise ContractError('Safety observer snapshot pending')
                                    break
                                except ContractError as error:
                                    if str(error) != 'Safety observer snapshot pending':
                                        raise
                                    if ((observer.last_command or {}).get('nonzero')
                                            or observer.nonzero_samples != safety_sync_zero_count):
                                        raise ContractError('Safety snapshots differ while final command is nonzero')
                                    if time.monotonic()-safety_sync_started >= .5:
                                        raise ContractError('Safety observer synchronization timeout (0.5s)')
                                    event('safety_snapshot_wait', guard=guard.get('safety'),
                                          observer=observer.safety_state,
                                          receipt_sequence=guard.get('receipt_sequence'))
                                    ros.spin_once(observer, timeout_sec=.01)
                                    if observer.nonzero_samples != safety_sync_zero_count:
                                        raise ContractError('Safety snapshots differ while final command is nonzero')
                                    guard, _ = refresh_guard_receipt(guard_path, guard)
                                    if guard is None:
                                        raise ContractError('Safety guard receipt unavailable during synchronization')
                            from guard_policy import command_error
                            for pending_topic in ('/cleannav/navigation/cmd_vel_raw',
                                                  '/cleannav/cmd_vel_candidate'):
                                pending_record = observer.command_last.get(pending_topic, {})
                                pending_values = pending_record.get('pending_safety_values')
                                if pending_values is not None:
                                    pending_error = command_error(pending_values, True,
                                        observer.safety_state, expected_owner=(observer.patrol_owner if patrol_motion else 'ros-execution-1'), motion_profile=args.motion_profile)
                                    if pending_error:
                                        raise ContractError(pending_topic+' synchronized command: '+pending_error)
                                    pending_record.pop('pending_safety_values')
                                    event('command_safety_check_synchronized', topic=pending_topic,
                                          safety=observer.safety_state,
                                          receipt_sequence=guard.get('receipt_sequence'))
                            if time.monotonic()-safety_sync_started > .005:
                                event('safety_snapshot_synchronized', guard=guard.get('safety'),
                                      observer=observer.safety_state,
                                      receipt_sequence=guard.get('receipt_sequence'))
                        else:
                            if motion_admitted:
                                confirm_admitted_safety_receipt(admitted=True)
                            else:
                                validate_motion_guard_runtime_state(
                                    guard, now_monotonic=time.monotonic(),
                                    motion_admitted=False,
                                    expected_owner=(observer.patrol_owner if patrol_motion else "ros-execution-1"),
                                    mission_lease_active=False)
                    elif not historical_motion:
                        validate_guard_state(guard, now_monotonic=time.monotonic())
                except ContractError as error:
                    fail(str(error), "safety")
            # Recompute from the just-read guard. A previous stale snapshot is
            # retained as a FAIL event above; only this current receipt may
            # contribute readiness facts.
            readiness = phase_readiness(log_text, collision, guard,
                                        rviz_node=rviz_node, rviz_process=rviz_process,
                                        gazebo_process=gazebo_process, mission_node=mission_node,
                                        localization_scan_topic=map_contract['scan_topic'],
                                        now_monotonic=time.monotonic(), motion_mode=any_motion,
                                        motion_admitted=(historical_motion or motion_admitted),
                                        expected_owner=(observer.patrol_owner if patrol_motion else "ros-execution-1"),
                    mission_lease_active=bool(observer.safety_state and
                                            observer.safety_state.get('owner')==(observer.patrol_owner if patrol_motion else 'ros-execution-1')))
            ready_safety, safety_evidence = readiness['safety']
            mission_accepted_now = (observer.patrol_accepted if patrol_motion else any(item.get('event') == 'ACCEPTED'
                and item.get('execution_id') == 'ros-execution-1'
                for item in observer.motion_events))
            if historical_motion:
                zero_contract = (mission_accepted_now or zero_command_observation_ok(
                    observer.command_samples, observer.nonzero_samples))
            elif motion_mode and not mission_accepted_now:
                zero_contract = all_command_streams_zero(
                    observer.command_counts, observer.command_nonzero_counts)
            elif any_motion:
                zero_contract = True
            else:
                zero_contract = zero_command_observation_ok(
                    observer.command_samples, observer.nonzero_samples)
            readiness['safety'] = (ready_safety and bool(graph_gate and graph_gate.converged)
                and zero_contract,
                {**safety_evidence,'supervisor_cmd_vel_graph_converged':bool(graph_gate and graph_gate.converged),
                 'zero_command_contract':zero_contract,
                 'command_counts':dict(observer.command_counts),
                 'command_nonzero_counts':dict(observer.command_nonzero_counts)})
            now_ns = time.monotonic_ns()
            for phase, (ready, detail) in readiness.items():
                conditions = dict(ready=ready, facts=detail.get('facts'),
                                  guard_receipt_fresh=detail.get('guard_receipt_fresh'))
                if previous_conditions.get(phase) != conditions:
                    event('phase_readiness_changed',phase=phase,ready=ready,evidence=detail,
                          input_observations=(guard or {}).get('observations'))
                    previous_conditions[phase] = conditions
            for changed in ledger.advance(readiness, monotonic_ns=now_ns):
                phase_records[changed] = ledger.evidence[changed]
                event("phase_passed", phase=changed, evidence=ledger.evidence[changed])
                state_record = read_json(run_dir / "supervisor_state.json") or {}
                state_record["readiness_phases"] = dict(ledger.states)
                state_record["readiness_all_passed"] = all(
                    ledger.states[name] == "PASS" for name in PHASES)
                atomic_json(run_dir / "supervisor_state.json", state_record)
            loss = first_admitted_phase_loss(ledger, readiness, guard)
            if loss is not None:
                phase = loss['phase']
                result.setdefault("readiness_losses", []).append(loss)
                event("phase_readiness_lost", phase=phase, evidence=loss)
                if ((motion_mode and not motion_interlock_open and not motion_authority_issued)
                        or (historical_motion and not mission_accepted_now)):
                    previous_phase_evidence = ledger.reopen_from(phase)
                    loss['reopened_phase_evidence'] = previous_phase_evidence
                    loss['continued_before_motion_admission'] = True
                    observation_started = None
                    motion_stable_since = None
                    motion_admission_wait_started = None
                    motion_initial_ground_truth = None
                    state_record = read_json(run_dir / "supervisor_state.json") or {}
                    state_record["readiness_phases"] = dict(ledger.states)
                    state_record["readiness_all_passed"] = False
                    state_record.setdefault("motion_readiness_recoveries", 0)
                    state_record["motion_readiness_recoveries"] += 1
                    atomic_json(run_dir / "supervisor_state.json", state_record)
                    if historical_motion:
                        motion_admission_wait_started=time.monotonic()
                    event('motion_readiness_loss_waiting_for_recovery', phase=phase,
                          phase_states=dict(ledger.states),
                          motion_remains_blocked=True)
                elif diagnostic_memory_policy:
                    continuation = diagnostic_readiness_loss_decision(
                        phase, guard, observer.nonzero_samples,
                        now_monotonic=time.monotonic(),
                        no_motion_contract=diagnostic_memory_policy)
                    event('safety_receipt_decision', sequence=(guard or {}).get('receipt_sequence'),
                          writer_monotonic=(guard or {}).get('monotonic'),
                          decision=continuation)
                    loss['diagnostic_continuation_decision'] = continuation
                    if continuation['allowed']:
                        previous_phase_evidence = ledger.reopen_from(phase)
                        loss['reopened_phase_evidence'] = previous_phase_evidence
                        loss['continued_without_readiness'] = True
                        observation_started = None
                        state_record = read_json(run_dir / "supervisor_state.json") or {}
                        state_record["readiness_phases"] = dict(ledger.states)
                        state_record["readiness_all_passed"] = False
                        state_record.setdefault("diagnostic_readiness_losses", 0)
                        state_record["diagnostic_readiness_losses"] += 1
                        atomic_json(run_dir / "supervisor_state.json", state_record)
                        event('diagnostic_readiness_loss_continued', phase=phase,
                              decision=continuation, phase_states=dict(ledger.states),
                              readiness_remains_false=True)
                    else:
                        false_facts = loss["guard_false_facts"]
                        suffix = ("; guard facts false: " + ", ".join(false_facts)
                                  if false_facts else "; guard facts did not identify a missing input")
                        fail("readiness was lost after phase admission: " + phase + suffix
                             + "; diagnostic hard gate: "
                             + ",".join(continuation.get('guard_safety_decision', {}).get(
                                 'hard_failures', [])), phase)
                else:
                    false_facts = loss["guard_false_facts"]
                    suffix = ("; guard facts false: " + ", ".join(false_facts)
                              if false_facts else "; guard facts did not identify a missing input")
                    fail("readiness was lost after motion admission: " + phase + suffix, phase)
            if ledger.failed_at:
                fail("readiness phase failed: " + ledger.failed_at, ledger.failed_at)
            all_phases = all(ledger.states[name] == "PASS" for name in PHASES)
            if all_phases and not any_motion and observation_started is None:
                observation_started = time.monotonic()
                result["observation_started_t0_s"] = round(elapsed(), 3)
                event("no_motion_observation_started", required_s=NO_MOTION_OBSERVATION_S)
            observation_elapsed = (time.monotonic() - observation_started
                                   if observation_started is not None else None)
            launch_elapsed_s = (time.monotonic_ns() - launch_monotonic_ns) / 1e9
            if patrol_motion:
                if all_phases and historical_gui_ready_at is None:
                    historical_gui_ready_at = time.monotonic()
                    startup_verification_complete = True
                    state_record = read_json(run_dir/'supervisor_state.json') or {}
                    state_record['startup_verification_complete'] = True
                    atomic_json(run_dir/'supervisor_state.json', state_record)
                    event('patrol_ready', map_identity=patrol_spec['map_identity'])
                if all_phases and observer.patrol_started is None:
                    if not all(observer.command_last.get(t, {}).get('nonzero') is False for t in observer.command_topics):
                        fail('patrol requires a zero command baseline', 'safety')
                    observer.submit_patrol()
                if observer.patrol_started is not None:
                    if not observer.patrol_accepted and time.monotonic()-observer.patrol_started > 6:
                        fail('SubmitTaskSequence response timed out', 'mission_manager')
                    if time.monotonic()-observer.patrol_started > patrol_spec['task_timeout_s']:
                        fail('single-lap patrol exceeded its retained 300s task bound', 'mission_manager')
                    status = observer.patrol_status or {}
                    if status.get('state') in (3, 4, 5, 6):
                        motion_terminal_monotonic = motion_terminal_monotonic or time.monotonic()
                        motion_terminal_payload = status
                        if status.get('state') != 3:
                            fail('patrol sequence terminal was not SUCCEEDED: '+str(status), 'mission_manager')
                        state = observer.safety_state or {}
                        zero = all(observer.command_last.get(t, {}).get('nonzero') is False for t in observer.command_topics)
                        released = bool(state) and not state.get('owner') and not state.get('enabled')
                        after = observer.last_command and observer.last_command['monotonic_ns'] >= int(motion_terminal_monotonic*1e9)
                        if released and zero and after:
                            if not (observer.patrol_accepted and status.get('completed_steps') == 2
                                    and observer.patrol_progress >= patrol_spec['route_segments']
                                    and (guard or {}).get('distance_m', 0) > .1
                                    and all(observer.command_nonzero_counts.get(t, 0)>0 for t in observer.command_topics)
                                    and any(x.get('owner') in observer.patrol_owners for x in observer.safety_owner_history)):
                                fail('sequence success lacks full-lap motion/progress/Lease witnesses', 'mission_manager')
                            result['motion_execution'] = dict(status='PASS', path='Mission Manager TaskSequence',
                                sequence=status, executions=sorted(observer.patrol_owners),
                                progress=observer.patrol_progress, wheel_distance_m=guard['distance_m'],
                                final_zero=True, lease_released=True, created_auth_scope=False)
                            stop_reason = 'patrol_complete'
                            event('patrol_complete', **result['motion_execution'])
                            break
                        if time.monotonic()-motion_terminal_monotonic > 5:
                            fail('patrol terminal did not release Lease and stop', 'safety')
            elif historical_motion:
                if all_phases:
                    motion_admission_wait_started=None
                elif historical_gui_ready_at is not None and motion_admission_wait_started is not None \
                        and time.monotonic()-motion_admission_wait_started>=2.:
                    fail("GUI readiness did not recover within 2s", "readiness")
                if historical_gui_ready_at is None and all_phases:
                    historical_gui_ready_at=time.monotonic()
                    startup_verification_complete=True
                    state_record=read_json(run_dir / "supervisor_state.json") or {}
                    state_record["startup_verification_complete"]=True
                    atomic_json(run_dir / "supervisor_state.json",state_record)
                    historical_initial_truth=(observer.ground_truth_samples[-1]
                        if observer.ground_truth_samples else None)
                    result['historical_motion_business_path'] = dict(
                        ready=True, authority_owner='existing localization_status_monitor',
                        lease_owner='existing Mission Manager', created_auth_scope=False)
                    result['historical_gui_wait'] = dict(status='WAITING',
                        started_t0_s=elapsed(), limit_s=HISTORICAL_GUI_GOAL_WAIT_S)
                    event('historical_motion_ready_for_gui', single_goal=True,
                          goal_wait_limit_s=HISTORICAL_GUI_GOAL_WAIT_S,
                          task_limit_s=HISTORICAL_TASK_TIMEOUT_S)
                    announce_navigation_ready()
                for item in observer.motion_events[motion_event_index:]:
                    kind=item.get('event')
                    if kind=='CLICK':
                        if motion_click_command_id is not None:
                            fail('second GUI click exceeds this one-goal validation', 'safety')
                        motion_click_command_id=item.get('command_id')
                        motion_goal_seen_at=time.monotonic()
                        motion_task_deadline=motion_goal_seen_at+HISTORICAL_TASK_TIMEOUT_S
                        result['historical_gui_click']=item
                        result.setdefault('historical_gui_wait', {})['status']='RECEIVED'
                        event('historical_gui_goal_received', goal=item,
                              task_limit_s=HISTORICAL_TASK_TIMEOUT_S,
                              wait_elapsed_s=(motion_goal_seen_at-historical_gui_ready_at
                                  if historical_gui_ready_at is not None else None))
                    elif kind=='ACCEPTED' and item.get('command_id')==motion_click_command_id:
                        motion_task_id=item.get('execution_id')
                        result['historical_mission_accepted']=item
                    elif kind in ('REJECTED','SERVICE_ERROR'):
                        fail('existing RViz adapter rejected goal: '+str(item), 'mission_manager')
                    elif (kind=='TASK_STATUS' and item.get('command_id')==motion_click_command_id
                          and item.get('state') in (13,14,15)):
                        motion_terminal_payload=item
                        motion_terminal_monotonic=motion_terminal_monotonic or time.monotonic()
                motion_event_index=len(observer.motion_events)
                historical_lease_seen = historical_lease_seen or any(
                    x.get('owner')==motion_task_id for x in observer.safety_owner_history
                    if motion_task_id)
                result["authorization_or_lease_created"] = historical_lease_seen
                result["local_authorization_scope_created"] = False
                budget=(guard or {}).get('route_budget')
                if budget and budget.get('command_id')==motion_click_command_id and motion_goal_seen_at:
                    motion_task_deadline=motion_goal_seen_at+budget['task_s']
                    result['historical_route_budget']=budget
                else:
                    budget=None
                displacement=None
                if historical_initial_truth and observer.ground_truth_samples:
                    a=historical_initial_truth['position']; b=observer.ground_truth_samples[-1]['position']
                    displacement=((a[0]-b[0])**2+(a[1]-b[1])**2)**.5
                    if displacement>=(budget['displacement_m'] if budget else MAX_HISTORICAL_DISPLACEMENT_M):
                        fail(f'historical validation exceeded {MAX_HISTORICAL_DISPLACEMENT_M:g}m displacement', 'safety')
                if historical_gui_ready_at and motion_goal_seen_at is None:
                    if time.monotonic()-historical_gui_ready_at>HISTORICAL_GUI_GOAL_WAIT_S:
                        result['historical_gui_wait']['status']='TIMED_OUT'
                        event('historical_gui_goal_wait_timeout',
                              wait_elapsed_s=time.monotonic()-historical_gui_ready_at,
                              limit_s=HISTORICAL_GUI_GOAL_WAIT_S)
                        fail('bounded historical GUI goal wait expired', 'rviz')
                if motion_task_deadline and time.monotonic()>motion_task_deadline and not motion_terminal_payload:
                    fail(f'existing historical Mission task exceeded {motion_task_deadline-motion_goal_seen_at:g}s',
                         'mission_manager')
                if motion_terminal_payload:
                    state=observer.safety_state or {}
                    released=(historical_lease_seen and not state.get('owner')
                        and not state.get('enabled'))
                    last_zero=all(observer.command_last.get(t,{}).get('nonzero') is False
                        for t in observer.command_topics)
                    final_after=observer.last_command and observer.last_command['monotonic_ns']>=int(motion_terminal_monotonic*1e9)
                    if released and last_zero and final_after:
                        if motion_terminal_payload.get('state')!=13:
                            fail('existing Mission terminal was not NAVIGATION_SUCCEEDED: '+str(motion_terminal_payload), 'mission_manager')
                        witnessed=(motion_task_id and historical_lease_seen and displacement is not None
                            and displacement>=.10 and all(observer.command_nonzero_counts.get(t,0)>0
                                for t in observer.command_topics))
                        if not witnessed:
                            fail('Mission succeeded without complete command/movement witness', 'mission_manager')
                        result['motion_execution']=dict(status='PASS',
                            path='restored historical Mission/Lease', mission=motion_terminal_payload,
                            lease_acquired=True, lease_released=True, displacement_m=displacement,
                            command_counts=observer.command_counts,
                            command_nonzero_counts=observer.command_nonzero_counts,
                            final_zero=True, created_auth_scope=False)
                        stop_reason='historical_motion_goal_complete'
                        event('historical_motion_goal_complete', **result['motion_execution'])
                        break
                    if time.monotonic()-motion_terminal_monotonic>5:
                        fail('existing Mission terminal did not release Lease and stop commands', 'safety')
            elif motion_mode:
                from short_goal_policy import (
                    MAX_AUTHORITY_TTL_S, PROFILE_ID, issue_motion_receipts,
                    validate_profile,
                )

                while motion_event_index < len(observer.motion_events):
                    item = observer.motion_events[motion_event_index]
                    motion_event_index += 1
                    kind = item.get("event")
                    if kind == "SERVICE_READY":
                        if motion_goal_wait_started is None:
                            motion_goal_wait_started = time.monotonic()
                            event("motion_gui_goal_wait_started",
                              deadline_s=MAX_GUI_GOAL_WAIT_S, service=item.get("service"),
                                  execution_id=item.get("execution_id"))
                    elif kind == "CLICK":
                        endpoints = item.get("publisher_endpoints")
                        click_ok = (item.get("frame") == "map"
                            and isinstance(endpoints, list) and len(endpoints) == 1
                            and endpoints[0].get("node_name") == "rviz"
                            and endpoints[0].get("node_namespace") == "/"
                            and endpoints[0].get("topic_type")
                                == "geometry_msgs/msg/PoseStamped"
                            and endpoints[0].get("endpoint_gid") not in (None, "", [], {}))
                        result.setdefault("motion_execution", {})["rviz_click"] = {
                            "status": "OBSERVED" if click_ok else "REJECTED_IDENTITY",
                            "event": item,
                        }
                        if not click_ok or not motion_admitted or motion_goal_seen_at is not None:
                            fail("RViz click was not a unique, admitted map-frame GUI goal", "safety")
                        motion_click_command_id = item.get("command_id")
                        motion_goal_seen_at = time.monotonic()
                    elif kind == "GOAL_BOUNDS_PASS":
                        result.setdefault("motion_execution", {})["goal_bounds"] = item
                    elif kind == "ACCEPTED":
                        if (motion_goal_seen_at is None
                                or item.get("execution_id") != "ros-execution-1"
                                or item.get("command_id") != motion_click_command_id
                                or not isinstance(item.get("task_id"), int)
                                or item.get("task_id") <= 0):
                            fail("Mission acceptance is not bound to the observed RViz goal", "mission_manager")
                        motion_task_id = item["task_id"]
                        motion_task_deadline = time.monotonic() + MAX_GOAL_RUNTIME_S
                        result.setdefault("motion_execution", {})["mission_acceptance"] = item
                    elif kind == "LEASE_ACQUIRED":
                        if item.get("execution_id") != "ros-execution-1":
                            fail("Safety Lease owner differs from the accepted Mission", "safety")
                        result.setdefault("motion_execution", {})["lease_acquired"] = item
                    elif kind == "LEASE_RELEASED":
                        if item.get("execution_id") != "ros-execution-1":
                            fail("Safety Lease release identity mismatch", "safety")
                        result.setdefault("motion_execution", {})["lease_released"] = item
                    elif kind == "MISSION_TERMINAL":
                        if (item.get("execution_id") != "ros-execution-1"
                                or item.get("command_id") != motion_click_command_id
                                or item.get("task_id") != motion_task_id):
                            fail("Mission terminal is not bound to the accepted GUI task",
                                 "mission_manager")
                        motion_terminal_payload = item
                        motion_terminal_monotonic = time.monotonic()
                        result.setdefault("motion_execution", {})["mission_terminal"] = item
                    elif kind in {"REJECTED", "IDENTITY_FAILURE", "SERVICE_ERROR",
                                  "GOAL_TIMEOUT", "STOP_UNCONFIRMED",
                                  "RECEIPT_UPDATE_FAILED", "LEASE_OWNER_MISMATCH"}:
                        fail("short-goal adapter reported a fail-closed event: "
                             + str(item.get("reason", kind)), "mission_manager")

                if motion_mode and observer.ground_truth_samples:
                    latest_gt = observer.ground_truth_samples[-1]
                    if all_phases and motion_initial_ground_truth is None:
                        motion_initial_ground_truth = latest_gt
                        result["motion_execution"] = {
                            "status": "PRE_ADMISSION_STABLE_BASELINE",
                            "ground_truth_start": latest_gt,
                        }
                        event("motion_ground_truth_baseline", sample=latest_gt)
                    if motion_initial_ground_truth is not None:
                        start_xy = motion_initial_ground_truth["position"][:2]
                        for sample in observer.ground_truth_samples:
                            displacement = math.hypot(
                                sample["position"][0] - start_xy[0],
                                sample["position"][1] - start_xy[1])
                            sample["displacement_from_admission_baseline_m"] = displacement
                            if displacement > MAX_MOTION_SESSION_DISPLACEMENT_M:
                                fail("Gazebo displacement exceeded the 2.5 m short-session bound",
                                     "safety")
                            if not motion_admitted and displacement > MAX_NO_MOTION_DISPLACEMENT_M:
                                fail("vehicle moved beyond the 0.05 m pre-admission stationary bound",
                                     "safety")

                if all_phases and motion_admission_wait_started is None and not motion_authority_issued:
                    motion_admission_wait_started = time.monotonic()
                if not all_phases and not motion_authority_issued:
                    motion_admission_wait_started = None
                    motion_stable_since = None
                    motion_last_graph_signature = None

                mission_accepted = any(item.get("event") == "ACCEPTED"
                    and item.get("execution_id") == "ros-execution-1"
                    and item.get("command_id") == motion_click_command_id
                    and item.get("task_id") == motion_task_id
                    for item in observer.motion_events)
                zero_baseline = all_command_streams_zero(
                    observer.command_counts, observer.command_nonzero_counts)
                guard_ok = False
                guard_reason = None
                try:
                    validate_motion_guard_state(guard, now_monotonic=time.monotonic())
                    guard_ok = True
                except (ContractError, TypeError, ValueError) as error:
                    guard_reason = str(error)
                final_records = [endpoint_record(endpoint)
                    for endpoint in observer.get_publishers_info_by_topic("/cmd_vel")]
                raw_records = [endpoint_record(endpoint)
                    for endpoint in observer.get_publishers_info_by_topic(
                        "/cleannav/navigation/cmd_vel_raw")]
                candidate_records = [endpoint_record(endpoint)
                    for endpoint in observer.get_publishers_info_by_topic(
                        "/cleannav/cmd_vel_candidate")]
                graph_signature = json.dumps({
                    "/cmd_vel": final_records,
                    "/cleannav/navigation/cmd_vel_raw": raw_records,
                    "/cleannav/cmd_vel_candidate": candidate_records,
                }, sort_keys=True)
                admission_inputs_ok = bool(
                    all_phases and guard_ok and graph_gate and graph_gate.converged
                    and zero_baseline and not mission_accepted and observer.rviz_goal_count == 0
                    and len(observer.ground_truth_samples) >= 2
                    and observer.safety_client.service_is_ready())
                if admission_inputs_ok:
                    if graph_signature != motion_last_graph_signature:
                        motion_stable_since = time.monotonic()
                        motion_last_graph_signature = graph_signature
                        event("motion_admission_graph_stability_reset",
                              graph_signature_sha256=hashlib.sha256(
                                  graph_signature.encode()).hexdigest())
                    elif motion_stable_since is None:
                        motion_stable_since = time.monotonic()
                else:
                    motion_stable_since = None
                    motion_last_graph_signature = None
                stable_s = (time.monotonic() - motion_stable_since
                            if motion_stable_since is not None else 0.0)
                if motion_admission_wait_started is not None and not motion_authority_issued:
                    admission_sample = {
                        "run_id": args.run_id,
                        "monotonic_ns": time.monotonic_ns(),
                        "all_phases_pass": all_phases,
                        "guard_ready": guard_ok,
                        "guard_reason": guard_reason,
                        "publisher_graph_converged": bool(graph_gate and graph_gate.converged),
                        "publisher_graph_signature_sha256": hashlib.sha256(
                            graph_signature.encode()).hexdigest(),
                        "command_counts": dict(observer.command_counts),
                        "command_nonzero_counts": dict(observer.command_nonzero_counts),
                        "zero_baseline": zero_baseline,
                        "ground_truth_samples": len(observer.ground_truth_samples),
                        "goal_seen": observer.rviz_goal_count > 0,
                        "stable_s": stable_s,
                        "admission_inputs_ok": admission_inputs_ok,
                    }
                    admission_path = run_dir / "motion-admission-observations.jsonl"
                    if (not admission_path.exists() or time.monotonic()
                            - float(result.get("last_motion_admission_sample", 0.0)) >= 1.0):
                        with admission_path.open("a") as stream:
                            stream.write(json.dumps(admission_sample, sort_keys=True) + "\n")
                        result["last_motion_admission_sample"] = time.monotonic()
                    if admission_inputs_ok and stable_s >= MOTION_ADMISSION_STABLE_S:
                        decision = motion_admission_decision(
                            phases=ledger.states, guard=guard,
                            final_publishers=final_records, raw_publishers=raw_records,
                            candidate_publishers=candidate_records,
                            command_samples=observer.command_counts,
                            zero_command_counts=observer.command_zero_counts,
                            goal_seen=observer.rviz_goal_count > 0,
                            ground_truth_samples=len(observer.ground_truth_samples),
                            stable_s=stable_s, now_monotonic=time.monotonic())
                        result["motion_admission"] = decision
                        event("motion_admission_decision", **decision)
                        remaining_before_cleanup = (SOFT_STOP_FROM_T0_S - elapsed()
                            - MIN_CLEANUP_RESERVE_S)
                        if decision.get("allowed") and remaining_before_cleanup < MAX_AUTHORITY_TTL_S:
                            fail("insufficient bounded runtime remains for one GUI goal and cleanup",
                                 "motion_admission")
                        if decision.get("allowed"):
                            profile = json.loads((HERE / "short_goal_profile.json").read_text())
                            result["motion_profile_contract"] = validate_profile(profile)
                            issued_ns = time.monotonic_ns()
                            expiry_limit_ns = args.t0_ns + int(
                                (SOFT_STOP_FROM_T0_S - MIN_CLEANUP_RESERVE_S) * 1e9)
                            expiry_ns = min(
                                issued_ns + int(MAX_AUTHORITY_TTL_S * 1e9), expiry_limit_ns)
                            if expiry_ns - issued_ns < int(MAX_AUTHORITY_TTL_S * 1e9):
                                fail("motion Authority TTL cannot cover the bounded task window",
                                     "motion_admission")
                            authorization, scope, lifecycle = issue_motion_receipts(
                                run_id=args.run_id, run_dir=run_dir,
                                issued_monotonic_ns=issued_ns,
                                expires_monotonic_ns=expiry_ns,
                                source_manifest_sha256=identity["source_manifest_sha256"],
                                map_contract_sha256=identity["map"]["contract_sha256"],
                                world_sha256=identity["map"]["world_sha256"])
                            motion_authority_issued = True
                            result["authorization_or_lease_created"] = True
                            result["motion_authority"] = {
                                "authorization_created": True,
                                "scope_created": True,
                                "safety_lease_created_by_supervisor": False,
                                "safety_lease_owner": "ros-execution-1 after Mission acceptance",
                                "authorization_sha256": sha256(run_dir / "motion-authorization.json"),
                                "scope_sha256": sha256(run_dir / "motion-scope.json"),
                                "lifecycle_sha256": sha256(run_dir / "motion-lifecycle.json"),
                                "expires_monotonic_ns": expiry_ns,
                            }
                            admission_receipt = {
                                "schema_version": 1, "run_id": args.run_id,
                                "profile_id": PROFILE_ID, "status": "OPENING",
                                "issued_monotonic_ns": issued_ns,
                                "expires_monotonic_ns": expiry_ns,
                                "authorization_sha256": result["motion_authority"][
                                    "authorization_sha256"],
                                "scope_sha256": result["motion_authority"]["scope_sha256"],
                                "all_phases_passed": True,
                                "goal_seen_before_admission": False,
                                "stability_s": stable_s,
                                "guard_receipt_monotonic": guard.get("monotonic"),
                                "publisher_graph_signature_sha256": admission_sample[
                                    "publisher_graph_signature_sha256"],
                            }
                            atomic_json(run_dir / "motion-admission.json", admission_receipt)
                            motion_interlock_stage = "OPENING"
                            event("motion_interlock_open_requested",
                                  admission=admission_receipt,
                                  safety_lease_granted=False)
                            interlock = set_simulation_interlock(True)
                            result["motion_admission"]["interlock"] = interlock
                            if interlock.get("success") is not True:
                                atomic_json(run_dir / "motion-admission.json",
                                    {**admission_receipt, "status": "OPEN_FAILED",
                                     "interlock_result": interlock})
                                fail("existing simulation interlock failed to open safely",
                                     "motion_admission")
                            motion_interlock_open = True
                            try:
                                confirm_admitted_safety_receipt(admitted=True)
                            except ContractError as error:
                                fail(str(error), "safety")
                            motion_interlock_stage = "ADMITTED"
                            admission_receipt.update(
                                status="ADMITTED",
                                admitted_monotonic_ns=time.monotonic_ns(),
                                guard_path=str(guard_path.resolve()),
                                guard_writer_pid=guard_writer_pid,
                                interlock_response_success=True,
                                interlock_result=interlock,
                                safety_unblocked_unleased=True,
                                safety_status=observer.safety_state,
                                all_phases_passed=True,
                                goal_seen_before_admission=False)
                            atomic_json(run_dir / "motion-admission.json", admission_receipt)
                            motion_admitted = True
                            result["motion_admission"] = {
                                "status": "PASS", "decision": decision,
                                "receipt": admission_receipt,
                                "authorization_sha256": admission_receipt[
                                    "authorization_sha256"],
                                "scope_sha256": admission_receipt["scope_sha256"],
                                "interlock": interlock,
                            }
                            event("motion_admission_passed", run_id=args.run_id,
                                  profile_id=PROFILE_ID,
                                  authorization_sha256=admission_receipt[
                                      "authorization_sha256"],
                                  scope_sha256=admission_receipt["scope_sha256"],
                                  safety_lease_not_yet_created=True,
                                  safety=observer.safety_state)
                if (motion_admission_wait_started is not None
                        and time.monotonic() - motion_admission_wait_started > 40.0
                        and not motion_authority_issued):
                    fail("Motion Admission prerequisites did not remain stable within 40 s",
                         "motion_admission")

                if motion_admitted:
                    if motion_goal_wait_started is not None and motion_goal_seen_at is None \
                            and time.monotonic() - motion_goal_wait_started > MAX_GUI_GOAL_WAIT_S:
                        fail("no real RViz GUI goal arrived within the bounded admission window",
                             "rviz")
                    if motion_goal_seen_at is not None and not mission_accepted \
                            and time.monotonic() - motion_goal_seen_at > 6.0:
                        fail("Mission Manager did not accept the observed RViz goal within 6 s",
                             "mission_manager")
                    if motion_task_deadline is not None and time.monotonic() > motion_task_deadline:
                        fail("one-goal Mission exceeded its 60 s bound", "mission_manager")
                    if motion_terminal_payload is not None:
                        if not motion_terminal_payload.get("succeeded"):
                            if time.monotonic() - motion_terminal_monotonic > 5.0:
                                fail("Mission ended without successful goal completion",
                                     "mission_manager")
                        else:
                            lifecycle_value = read_json(run_dir / "motion-lifecycle.json") or {}
                            final_zero = (motion_post_terminal_zero_counts["/cmd_vel"] >= 5
                                and motion_post_terminal_zero_counts[
                                    "/cleannav/navigation/cmd_vel_raw"] >= 1
                                and motion_post_terminal_zero_counts[
                                    "/cleannav/cmd_vel_candidate"] >= 1)
                            latest_after_terminal = [sample for sample in observer.ground_truth_samples
                                if sample["monotonic_ns"] > int(
                                    motion_terminal_monotonic * 1e9)]
                            final_gt = latest_after_terminal[-1] if latest_after_terminal else None
                            displacement = None
                            if final_gt is not None and motion_initial_ground_truth is not None:
                                start_xy = motion_initial_ground_truth["position"][:2]
                                displacement = math.hypot(final_gt["position"][0]-start_xy[0],
                                                         final_gt["position"][1]-start_xy[1])
                            lease_acquired = any(item.get("event") == "LEASE_ACQUIRED"
                                and item.get("execution_id") == "ros-execution-1"
                                for item in observer.motion_events)
                            lease_released = any(item.get("event") == "LEASE_RELEASED"
                                and item.get("execution_id") == "ros-execution-1"
                                for item in observer.motion_events)
                            from short_goal_policy import validate_revocation_receipt
                            try:
                                revocation = validate_revocation_receipt(
                                    run_id=args.run_id,
                                    authorization_path=run_dir / "motion-authorization.json",
                                    scope_path=run_dir / "motion-scope.json",
                                    lifecycle_path=run_dir / "motion-lifecycle.json",
                                    receipt_path=run_dir / "motion-revocation.json",
                                    lease_acquired=lease_acquired,
                                    lease_released=lease_released)
                                lifecycle_closed = True
                            except (OSError, ValueError) as error:
                                revocation = {"status": "INVALID", "reason": str(error)}
                                lifecycle_closed = False
                            completion = motion_completion_decision(
                                gui_goal_seen=motion_goal_seen_at is not None,
                                mission_accepted=mission_accepted,
                                task_succeeded=motion_terminal_payload.get("succeeded") is True,
                                lease_acquired=lease_acquired,
                                lease_released=lease_released,
                                lifecycle_closed=lifecycle_closed,
                                command_counts=observer.command_counts,
                                nonzero_counts=observer.command_nonzero_counts,
                                final_zero_after_terminal=final_zero,
                                gazebo_displacement_m=displacement)
                            result["motion_completion"] = completion
                            result["motion_execution"] = {
                                "status": "COMPLETION_CHECK",
                                "rviz_goal_seen": motion_goal_seen_at is not None,
                                "mission_accepted": mission_accepted,
                                "mission_terminal": motion_terminal_payload,
                                "lease_acquired": lease_acquired,
                                "lease_released": lease_released,
                                "lifecycle": lifecycle_value,
                                "revocation_receipt": revocation,
                                "post_terminal_zero_counts": dict(
                                    motion_post_terminal_zero_counts),
                                "final_ground_truth": final_gt,
                                "ground_truth_start": motion_initial_ground_truth,
                                "ground_truth_displacement_m": displacement,
                                "completion": completion,
                            }
                            if completion.get("passed") is True:
                                close_result = set_simulation_interlock(False)
                                result["motion_execution"]["interlock_closed"] = close_result
                                if close_result.get("success") is not True:
                                    fail("completed task did not return Safety to block-all idle",
                                         "safety")
                                motion_close_confirmed = True
                                motion_interlock_open = False
                                motion_admitted = False
                                result["motion_execution"]["status"] = "PASS"
                                result["motion_execution"]["authority_effectively_revoked"] = (
                                    lifecycle_value.get("authorization_effectively_revoked") is True)
                                result["motion_execution"]["scope_effectively_closed"] = (
                                    lifecycle_value.get("scope_effectively_closed") is True)
                                stop_reason = "short_goal_completed"
                                event("short_goal_motion_completed",
                                      completion=completion,
                                      interlock_close=close_result)
                                break
                            elif motion_terminal_monotonic is not None and (
                                    time.monotonic()-motion_terminal_monotonic > 5.0):
                                fail("Mission terminal lacked required motion/lease/zero evidence: "
                                     + ",".join(completion.get("failures", [])),
                                     "motion_completion")
            if diagnostic_memory_policy and not startup_verification_complete:
                current_distance = (guard.get("distance_m")
                                    if isinstance(guard, dict) else None)
                startup_verification_complete = no_motion_startup_qualifies(
                    all_phases_pass=all_phases,
                    observation_elapsed_s=observation_elapsed,
                    enable_motion=launch_mode.get("enable_motion") is True,
                    auto_start_task=launch_mode.get("auto_start_task") is True,
                    command_samples=observer.command_samples,
                    nonzero_samples=observer.nonzero_samples,
                    displacement_m=current_distance)
                if startup_verification_complete:
                    verified_startup = startup_verification_record(
                        run_id=args.run_id, launch_elapsed_s=launch_elapsed_s,
                        observation_elapsed=observation_elapsed, phases=ledger.states,
                        launch_mode=launch_mode, command_samples=observer.command_samples,
                        nonzero_samples=observer.nonzero_samples, distance_m=current_distance)
                    result["verified_no_motion_startup"] = verified_startup
                    result["startup_soak"] = {
                        "status": "IN_PROGRESS",
                        "target_launch_elapsed_s": MIN_LAUNCH_OBSERVATION_S,
                        "started_at_monotonic_ns": launch_monotonic_ns,
                        "level_a_verified_at_monotonic_ns": verified_startup[
                            "completed_monotonic_ns"],
                        "readiness_loss_count_at_level_a": len(
                            result.get("readiness_losses", [])),
                    }
                    atomic_json(run_dir / "verified-no-motion-startup.json", verified_startup)
                    state_record = read_json(run_dir / "supervisor_state.json") or {}
                    state_record["startup_verification_complete"] = True
                    state_record["verified_no_motion_startup"] = verified_startup
                    atomic_json(run_dir / "supervisor_state.json", state_record)
                    event("verified_no_motion_startup", **verified_startup)
            stop_reason = deadline_stop_reason(elapsed_s=elapsed(), all_phases_pass=all_phases,
                                               observation_elapsed_s=observation_elapsed,
                                               launch_elapsed_s=launch_elapsed_s,
                                               startup_verification_complete=startup_verification_complete)
            if stop_reason:
                break
            if (not LOCALIZATION_DIAGNOSTIC and not startup_verification_complete
                    and elapsed() >= READINESS_DEADLINE_FROM_T0_S and not all_phases):
                fail(f"required lifecycle phases did not become ready before "
                     f"T0+{READINESS_DEADLINE_FROM_T0_S:g}s", "readiness")
            time.sleep(0.05)
        result["launch_observation_elapsed_s"] = round((time.monotonic_ns()-launch_monotonic_ns)/1e9, 6)
        result["runtime_resource_sample_count"] = resource_samples
        result["runtime_memory_soft_warning_sample_count"] = runtime_memory_soft_warning_samples
        result["stop_reason"] = stop_reason
        result["phases"] = dict(ledger.states)
        result["phase_evidence"] = ledger.evidence
        result["command_observation"] = {
            "samples": observer.command_samples,
            "nonzero_samples": observer.nonzero_samples,
            "last_command": observer.last_command,
            "publisher_graph_samples": len(graph_records),
            "publisher_graph_converged": bool(graph_gate and graph_gate.converged),
        }
        result["readiness_artifacts"] = {
            "collision_lifecycle": str(collision_path) if collision_path else None,
            "safety_guard": str(guard_path) if guard_path else None,
            "guard_final_state": guard,
        }
        if diagnostic_memory_policy:
            if not startup_verification_complete:
                result["verified_no_motion_startup"] = {
                    "status": "FAIL", "reason": "Level A contract was not satisfied",
                    "run_id": args.run_id,
                    "phases_at_stop": dict(ledger.states),
                    "command_samples": (observer.command_samples if observer else 0),
                    "nonzero_command_samples": (observer.nonzero_samples if observer else 0),
                    "guard_distance_m": ((guard or {}).get("distance_m")
                                          if isinstance(guard, dict) else None),
                }
                result["startup_soak"] = {
                    "status": "NOT COMPLETED",
                    "target_launch_elapsed_s": MIN_LAUNCH_OBSERVATION_S,
                    "stop_reason": stop_reason,
                    "readiness_loss_count": len(result.get("readiness_losses", [])),
                }
                failures.append("supervisor stopped before verifying no-motion startup Level A")
            elif stop_reason == "startup_soak_complete":
                losses = result.get("readiness_losses", [])
                soak_status = "FAIL" if losses else "PASS"
                result["startup_soak"] = {
                    "status": soak_status,
                    "target_launch_elapsed_s": MIN_LAUNCH_OBSERVATION_S,
                    "actual_launch_elapsed_s": result.get("launch_observation_elapsed_s"),
                    "readiness_loss_count": len(losses),
                    "readiness_losses": losses,
                    "memory_soft_warning_samples": runtime_memory_soft_warning_samples,
                    "memory_soft_warning_is_nonfatal": True,
                }
                if soak_status == "FAIL":
                    failures.append("startup soak completed with readiness-loss events")
            else:
                health_failure = bool(failures or result.get("readiness_losses"))
                result["startup_soak"] = {
                    "status": "FAIL" if health_failure else "NOT COMPLETED",
                    "target_launch_elapsed_s": MIN_LAUNCH_OBSERVATION_S,
                    "actual_launch_elapsed_s": result.get("launch_observation_elapsed_s"),
                    "stop_reason": stop_reason,
                    "readiness_loss_count": len(result.get("readiness_losses", [])),
                    "failures_at_stop": list(failures),
                    "memory_soft_warning_samples": runtime_memory_soft_warning_samples,
                }
        elif stop_reason not in ("no_motion_observation_complete", "historical_motion_goal_complete", "patrol_complete"):
            failures.append("supervisor stopped before completing every readiness phase and observation")
    except BaseException as error:
        if not failures:
            failures.append(f"{type(error).__name__}: {error}")
        result["traceback"] = traceback.format_exc()
        result["failure_stage"] = result.get("failure_stage", "supervision")
        event("supervisor_exception", error=f"{type(error).__name__}: {error}")
    finally:
        if any_motion:
            reason = (stop_reason or (failures[-1] if failures else "supervisor shutdown"))
            motion_cleanup = revoke_motion_before_shutdown(reason)
            result["motion_shutdown_revocation"] = motion_cleanup
            if motion_cleanup.get("verified") is not True:
                failures.append("motion Authority/Scope/Lease revocation was not verified before shutdown")
        if launch_log is not None:
            try:
                launch_log.flush()
                launch_log.close()
            except OSError:
                pass
        if process is not None:
            try:
                cleanup = runtime.signal_and_reap(process, True, t0)
            except BaseException as error:
                cleanup = {"started": True, "cleanup_ok": False, "status": "UNKNOWN",
                           "error": f"{type(error).__name__}: {error}",
                           "traceback": traceback.format_exc(),
                           "process_group_snapshot_after_cleanup":
                               runtime.process_group_snapshot(process.pid)}
                failures.append("cleanup routine raised; final process state UNKNOWN")
            result["launch_exit_code"] = process.poll()
            result["cleanup"] = cleanup
            result["cleanup_verified"] = cleanup_is_verified(cleanup)
            event("cleanup_finished", cleanup_ok=cleanup.get("cleanup_ok"),
                  verified=result["cleanup_verified"],
                  process_group_status=cleanup.get("process_group_state_after_cleanup"),
                  elapsed_s=cleanup.get("elapsed_from_T0_s"))
            if not result["cleanup_verified"]:
                failures.append("process-group cleanup lacks complete empty/no-zombie/reaped evidence")
        else:
            cleanup = {"status": "NOT_STARTED", "cleanup_ok": None,
                       "reason": "preflight rejected before launch process creation"}
            result["cleanup"] = cleanup
            result["cleanup_verified"] = False
        if diagnostic is not None:
            try:diagnostic.close()
            except Exception as error:failures.append('diagnostic receipt finalization failed: '+str(error))
        if ros is not None and observer is not None:
            try:
                observer.destroy_node()
                if ros.ok():
                    ros.shutdown()
            except Exception as error:
                failures.append("ROS observer shutdown failed: " + str(error))
        if memory_telemetry is not None:
            try:
                if process is not None:
                    final_sample = memory_telemetry.sample(
                        launch_pgid=process.pid, supervisor_pid=os.getpid(),
                        t0_ns=args.t0_ns,
                        launch_start_ns=launch_monotonic_ns or time.monotonic_ns(),
                        label="after_launch_cleanup")
                    event("process_memory_sample", **final_sample)
                result["memory_attribution"] = memory_telemetry.close()
                result["memory_attribution"]["late_samples_affect_readiness_or_stop"] = False
            except Exception as telemetry_error:
                result["memory_attribution"] = {
                    "status": "PARTIAL", "error": f"{type(telemetry_error).__name__}: {telemetry_error}",
                    "timeseries_csv": "process-memory-timeseries.csv",
                    "summary_json": "memory-attribution-summary.json",
                    "late_samples_affect_readiness_or_stop": False,
                }
                try:
                    if not memory_telemetry.closed:
                        memory_telemetry.close()
                except Exception:
                    pass
                event("process_memory_telemetry_finalization_error",
                      error=f"{type(telemetry_error).__name__}: {telemetry_error}")
        result["lock_release"] = release_locks(descriptors)
        if any(not item["closed"] for item in result["lock_release"]):
            failures.append("one or more lock file descriptors did not close")
        hard_deadline_record = read_json(run_dir / "hard_deadline.json")
        watchdog_failure_record = read_json(run_dir / "watchdog_failure.json")
        if hard_deadline_record or watchdog_failure_record:
            record = hard_deadline_record or watchdog_failure_record
            failures.append("independent deadline watchdog reported a cutoff or failure")
            result["hard_deadline_enforced"] = record.get("hard_deadline_enforced") is True
            result["hard_deadline"] = record
            result["failure_stage"] = "hard_deadline" if result["hard_deadline_enforced"] else "watchdog"
        if diagnostic_memory_policy:
            if startup_verification_complete:
                # Never overwrite a previously emitted Level A PASS because a
                # later SOAK exception/stop occurred.
                result["verified_no_motion_startup"] = verified_startup or result[
                    "verified_no_motion_startup"]
                if result["startup_soak"].get("status") == "IN_PROGRESS":
                    result["startup_soak"] = {
                        "status": "FAIL" if failures else "NOT COMPLETED",
                        "target_launch_elapsed_s": MIN_LAUNCH_OBSERVATION_S,
                        "actual_launch_elapsed_s": result.get("launch_observation_elapsed_s"),
                        "stop_reason": stop_reason,
                        "readiness_loss_count": len(result.get("readiness_losses", [])),
                        "failures_at_stop": list(failures),
                        "memory_soft_warning_samples": runtime_memory_soft_warning_samples,
                    }
            elif result["verified_no_motion_startup"].get("status") == "NOT ACHIEVED":
                result["verified_no_motion_startup"] = {
                    "status": "FAIL", "reason": "Level A contract was not satisfied",
                    "run_id": args.run_id, "failure_stage": result.get("failure_stage"),
                    "phases_at_stop": dict(ledger.states),
                }
                if result["startup_soak"].get("status") == "NOT COMPLETED":
                    result["startup_soak"].update(
                        stop_reason=stop_reason,
                        readiness_loss_count=len(result.get("readiness_losses", [])),
                        failures_at_stop=list(failures))
        result["runtime_resource_sample_count"] = resource_samples
        result["runtime_memory_soft_warning_sample_count"] = runtime_memory_soft_warning_samples
        result["completed_at_utc"] = utc_now()
        result["total_elapsed_s"] = round(elapsed(), 3)
        result["phases"] = dict(ledger.states)
        result["failures"] = failures
        if any_motion and launched and result.get("motion_execution", {}).get("status") != "PASS":
            result.setdefault("motion_execution", {})["status"] = "FAIL"
            result.setdefault("motion_execution", {})["reason"] = (
                ("one real A-region lap, sequence success, actual motion, lease release, " if patrol_motion else
                 "one real RViz GUI goal, Mission success, actual motion, lease release, ") +
                "zero-command return and verified shutdown are all required")
            failures.append(f"{args.motion_profile} motion session did not satisfy the full completion contract")
            result["failures"] = failures
        result["status"] = final_status(
            failures=failures, phases=ledger.states, cleanup=cleanup,
            launch_started=launched, elapsed_s=elapsed(), cgroup_disposition=cgroup_disposition)
        if (diagnostic_memory_policy
                and result["verified_no_motion_startup"].get("status") == "PASS"
                and result["startup_soak"].get("status") == "NOT COMPLETED"
                and result["status"] == "PASS"):
            result["status"] = "PASS_WITH_EXCEPTION"
        result["launch_started"] = launched
        result["input_identity"] = input_identity
        if not launched and result["status"] == "BLOCKED":
            result.setdefault("failure_stage", "preflight")
        if environment is not None:
            environment["status"] = "CAPTURED"
        receipt_source = {**input_identity, **(source or {"status": "PARTIAL"})}
        receipt_source.update(run_id=args.run_id, run_dir=str(run_dir),
                              process_group_id=result.get("process_group_id"))
        receipt_environment = environment or {
            "status": "PARTIAL",
            "overlay_setup": args.overlay_setup,
            "variables": environment_variable_snapshot(),
        }
        write_run_receipts(run_dir, result, cleanup=cleanup,
                           environment=receipt_environment, identity=receipt_source, events=events)
        atomic_json(run_dir / "supervisor_complete.json", {
            "run_id": args.run_id, "status": result["status"],
            "completed_at_utc": result["completed_at_utc"],
            "monotonic_ns": time.monotonic_ns(),
        })
        write_checksums(run_dir)
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
    return 0 if result["status"] in ("PASS", "PASS_WITH_EXCEPTION") else 1


def main(argv=None):
    args = parse_args(argv)
    if args.record_wrapper_failure:
        return wrapper_failure(args)
    if args.record_watchdog_failure:
        return record_watchdog_failure(args)
    if args.record_watchdog_exit:
        return record_watchdog_exit(args)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
