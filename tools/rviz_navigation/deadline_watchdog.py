#!/usr/bin/env python3
"""Independent, fail-closed process watchdog for RViz navigation sessions.

This helper runs as a separate process from navigation_supervisor.py. It never
starts ROS or Gazebo. It can request a stop from the identity-bound wrapper or
supervisor, and signal only the launch process group recorded for this run.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import sys
import time
import traceback

HERE = Path(__file__).resolve().parent
NAV = HERE.parents[1]
sys.path.insert(0, str(NAV / "tools/startup_smoke"))
import roof_lidar_runtime as runtime
from supervisor_policy import (
    TIMING_PROFILE, timing_identity,
    CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S,
    HARD_PROCESS_CUTOFF_FROM_T0_S,
    MIN_LAUNCH_OBSERVATION_S,
    PHASES,
    WATCHDOG_RECEIPT_RESERVE_S,
    DeadlineWatchdogState,
)


def atomic_json(path: Path, value: dict) -> None:
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True, default=str) + "\n")
    temporary.replace(path)


def sha256(path: Path) -> str:
    import hashlib
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_checksums(run_dir: Path) -> None:
    rows = []
    for path in sorted(Path(run_dir).rglob("*")):
        if not path.is_file() or path.name == "SHA256SUMS" or path.name.endswith(".tmp"):
            continue
        rows.append(f"{sha256(path)}  {path.relative_to(run_dir).as_posix()}\n")
    (Path(run_dir) / "SHA256SUMS").write_text("".join(rows))


def process_identity(pid: int) -> dict | None:
    try:
        text = (Path("/proc") / str(pid) / "stat").read_text()
        end = text.rfind(")")
        fields = text[end + 2:].split()
        return {"pid": pid, "state": fields[0], "pgid": int(fields[2]),
                "start_ticks": int(fields[19])}
    except (OSError, ValueError, IndexError):
        return None


def identity_matches(pid: int, expected_start_ticks: int) -> bool:
    current = process_identity(pid)
    return bool(current and current["start_ticks"] == expected_start_ticks)


def timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def verified_prior_cleanup(run_dir: Path) -> dict | None:
    try:
        cleanup = json.loads((run_dir / "cleanup.json").read_text())
        snapshot = cleanup.get("process_group_snapshot_after_cleanup", {})
        if (cleanup.get("cleanup_ok") is True and cleanup.get("launch_main_reaped") is True
                and cleanup.get("process_group_alive_after_cleanup") is False
                and cleanup.get("process_group_state_after_cleanup") == "EMPTY"
                and snapshot.get("scan_complete") is True and snapshot.get("status") == "EMPTY"
                and not snapshot.get("running_members") and not snapshot.get("zombie_members")):
            return cleanup
    except (OSError, ValueError, AttributeError):
        pass
    return None


def readiness_complete_for_deadline(state: dict) -> bool:
    """Select the durable startup milestone used by the readiness cutoff."""
    if state.get("timing_profile") in ("localization-diagnostic", "historical-motion"):
        return state.get("startup_verification_complete") is True
    return state.get("readiness_all_passed") is True


class Watchdog:
    """Out-of-process monitor with injected OS operations for deterministic tests."""

    def __init__(self, *, run_dir: Path, run_id: str, t0_ns: int,
                 wrapper_pid: int | None = None, wrapper_start_ticks: int | None = None,
                 supervisor_pid: int | None = None, supervisor_start_ticks: int | None = None,
                 launch_pid: int | None = None, launch_start_ticks: int | None = None,
                 clock_ns=time.monotonic_ns, sleeper=time.sleep,
                 get_identity=process_identity, snapshot=runtime.process_group_snapshot,
                 kill_pid=os.kill, kill_group=os.killpg):
        self.run_dir = Path(run_dir)
        self.run_id = run_id
        self.t0_ns = int(t0_ns)
        self.wrapper_pid = int(wrapper_pid) if wrapper_pid is not None else None
        self.wrapper_start_ticks = int(wrapper_start_ticks) if wrapper_start_ticks is not None else None
        self.supervisor_pid = int(supervisor_pid) if supervisor_pid is not None else None
        self.supervisor_start_ticks = (int(supervisor_start_ticks)
                                       if supervisor_start_ticks is not None else None)
        self.launch_pid = int(launch_pid) if launch_pid is not None else None
        self.launch_start_ticks = int(launch_start_ticks) if launch_start_ticks is not None else None
        self.clock_ns = clock_ns
        self.sleeper = sleeper
        self.get_identity = get_identity
        self.snapshot = snapshot
        self.kill_pid = kill_pid
        self.kill_group = kill_group
        self.schedule = DeadlineWatchdogState()
        self.signals: list[dict] = []

    def _elapsed(self) -> float:
        return max(0.0, (self.clock_ns() - self.t0_ns) / 1_000_000_000)

    def _supervisor_alive(self) -> bool:
        pid, ticks = self._active_identity()
        current = self.get_identity(pid) if pid is not None else None
        return bool(current and current.get("start_ticks") == ticks
                    and current.get("state") not in ("Z", "X"))

    def _active_identity(self) -> tuple[int | None, int | None]:
        if self.supervisor_pid is not None:
            return self.supervisor_pid, self.supervisor_start_ticks
        return self.wrapper_pid, self.wrapper_start_ticks

    def _refresh_identity_from_state(self) -> None:
        try:
            state = json.loads((self.run_dir / "supervisor_state.json").read_text())
        except (OSError, ValueError):
            return
        if state.get("run_id") != self.run_id:
            return
        supervisor_pid = state.get("supervisor_pid")
        supervisor_ticks = state.get("supervisor_start_ticks")
        if isinstance(supervisor_pid, int) and isinstance(supervisor_ticks, int):
            self.supervisor_pid = supervisor_pid
            self.supervisor_start_ticks = supervisor_ticks
        launch_pid = state.get("launch_pid")
        launch_ticks = state.get("launch_start_ticks")
        if isinstance(launch_pid, int) and isinstance(launch_ticks, int) and launch_pid > 0:
            self.launch_pid = launch_pid
            self.launch_start_ticks = launch_ticks

    def _group_snapshot(self) -> dict:
        if self.launch_pid is None:
            return {"status": "NOT_STARTED", "scan_complete": True,
                    "members": [], "running_members": [], "zombie_members": [], "errors": []}
        try:
            return self.snapshot(self.launch_pid)
        except BaseException as error:
            return {"status": "UNKNOWN", "scan_complete": False,
                    "members": [], "running_members": [], "zombie_members": [],
                    "errors": [f"{type(error).__name__}: {error}"]}

    def _record_signal(self, target: str, sig: int, outcome: str,
                       before: dict | None = None, reason: str | None = None) -> None:
        self.signals.append({"target": target, "signal": signal.Signals(sig).name,
                             "t0_elapsed_s": round(self._elapsed(), 6),
                             "utc": timestamp(), "outcome": outcome,
                             "reason": reason,
                             "process_group_snapshot_before": before})
        with (self.run_dir / 'watchdog-signals.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(run_id=self.run_id, **self.signals[-1]), default=str)+'\n')

    def _signal_supervisor(self, sig: int, reason: str | None = None) -> None:
        pid, ticks = self._active_identity()
        current = self.get_identity(pid) if pid is not None else None
        target_name = "supervisor" if self.supervisor_pid is not None else "wrapper"
        if not current or current.get("start_ticks") != ticks:
            self._record_signal(target_name, sig, "identity_missing_or_changed", reason=reason)
            return
        try:
            self.kill_pid(pid, sig)
            self._record_signal(target_name, sig, "sent", reason=reason)
        except ProcessLookupError:
            self._record_signal(target_name, sig, "process_missing", reason=reason)
        except OSError as error:
            self._record_signal(target_name, sig, f"error:{type(error).__name__}:{error.errno}", reason=reason)

    def _signal_group(self, sig: int) -> dict:
        before = self._group_snapshot()
        if self.launch_pid is None:
            self._record_signal("launch_process_group", sig, "not_started", before)
            return before
        members = before.get("members", [])
        if before.get("scan_complete") is True and not members:
            self._record_signal("launch_process_group", sig, "not_sent_empty", before)
            return before
        leader = self.get_identity(self.launch_pid)
        # If the leader exists, require its start time to match this run. If it
        # exited while children remain, the extant members still own this PGID.
        if leader and (leader.get("start_ticks") != self.launch_start_ticks
                       or leader.get("pgid") != self.launch_pid):
            self._record_signal("launch_process_group", sig, "not_sent_leader_identity_mismatch", before)
            return before
        if not leader and (before.get("scan_complete") is not True or not members):
            self._record_signal("launch_process_group", sig, "not_sent_identity_unconfirmed", before)
            return before
        try:
            self.kill_group(self.launch_pid, sig)
            self._record_signal("launch_process_group", sig, "sent", before)
        except ProcessLookupError:
            self._record_signal("launch_process_group", sig, "group_missing", before)
        except OSError as error:
            self._record_signal("launch_process_group", sig,
                                f"error:{type(error).__name__}:{error.errno}", before)
        return before

    def _completion_recorded(self) -> bool:
        return (self.run_dir / "supervisor_complete.json").is_file()

    def _write_ready(self) -> None:
        identity = process_identity(os.getpid())
        if identity is None:
            raise RuntimeError("watchdog PID/start-time identity unavailable")
        atomic_json(self.run_dir / "watchdog_ready.json", {
            "status": "READY", "run_id": self.run_id, "pid": os.getpid(),
            "timing_profile": TIMING_PROFILE, "timing_contract": timing_identity(),
            "session_id": os.getsid(0), "process_group_id": os.getpgrp(),
            "start_ticks": identity["start_ticks"],
            "wrapper_pid": self.wrapper_pid,
            "wrapper_start_ticks": self.wrapper_start_ticks,
            "supervisor_pid": self.supervisor_pid,
            "supervisor_start_ticks": self.supervisor_start_ticks,
            "launch_pid": self.launch_pid, "launch_start_ticks": self.launch_start_ticks,
            "t0_monotonic_ns": self.t0_ns, "started_at_utc": timestamp(),
        })

    def _verified_pre_admission_failure(self) -> dict | None:
        """Accept only a same-run supervisor rejection that proves no launch existed."""
        try:
            failure = json.loads((self.run_dir / "supervisor_failure.json").read_text())
            state = json.loads((self.run_dir / "supervisor_state.json").read_text())
            ready = json.loads((self.run_dir / "watchdog_ready.json").read_text())
        except (OSError, ValueError):
            return None
        if not all(isinstance(item, dict) for item in (failure, state, ready)):
            return None
        supervisor_pid = self.supervisor_pid
        supervisor_ticks = self.supervisor_start_ticks
        watchdog_identity = process_identity(os.getpid())
        checks = (
            failure.get("status") == "REJECTED_BEFORE_ADMISSION",
            failure.get("failure_stage") == "run_directory_admission",
            failure.get("run_id") == self.run_id == state.get("run_id") == ready.get("run_id"),
            failure.get("run_dir") == str(self.run_dir.resolve()),
            failure.get("t0_monotonic_ns") == self.t0_ns == state.get("t0_monotonic_ns")
                == ready.get("t0_monotonic_ns"),
            failure.get("launch_started") is False,
            failure.get("launch_pid") is None and failure.get("launch_pgid") is None,
            state.get("launch_pid") is None and state.get("launch_pgid") is None
                and state.get("launch_start_ticks") is None,
            failure.get("supervisor_pid") == supervisor_pid == state.get("supervisor_pid"),
            failure.get("supervisor_start_ticks") == supervisor_ticks
                == state.get("supervisor_start_ticks"),
            failure.get("wrapper_pid") == self.wrapper_pid == ready.get("wrapper_pid"),
            failure.get("wrapper_start_ticks") == self.wrapper_start_ticks
                == ready.get("wrapper_start_ticks"),
            failure.get("watchdog_pid") == os.getpid() == ready.get("pid"),
            failure.get("watchdog_start_ticks") == (watchdog_identity or {}).get("start_ticks")
                == ready.get("start_ticks"),
            ready.get("process_group_id") == os.getpid()
                and ready.get("session_id") == os.getpid(),
            isinstance(failure.get("reason"), str) and bool(failure.get("reason")),
        )
        return failure if all(checks) else None

    def _write_failure(self, reason: str, *, hard_cutoff: bool) -> dict:
        final_snapshot = self._group_snapshot()
        supervisor_failure = self._verified_pre_admission_failure()
        if supervisor_failure is not None and not hard_cutoff:
            reason = "supervisor_rejected_before_admission: " + supervisor_failure["reason"]
        prior = verified_prior_cleanup(self.run_dir)
        group_empty = (final_snapshot.get("scan_complete") is True
                       and final_snapshot.get("status") == "EMPTY"
                       and not final_snapshot.get("running_members")
                       and not final_snapshot.get("zombie_members"))
        completed = bool(prior and group_empty)
        launch_not_started = supervisor_failure is not None
        record = {
            "schema_version": 1, "run_id": self.run_id,
            "status": "FAIL_CLOSED", "reason": reason,
            "failure_stage": ("run_directory_admission" if launch_not_started else "watchdog"),
            "first_failure": supervisor_failure,
            "launch_started": not launch_not_started,
            "hard_deadline_enforced": hard_cutoff,
            "hard_process_cutoff_t0_s": HARD_PROCESS_CUTOFF_FROM_T0_S,
            "cleanup_evidence_cutoff_t0_s": CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S,
            "observed_t0_elapsed_s": round(self._elapsed(), 6),
            "recorded_at_utc": timestamp(), "signals": self.signals,
            "supervisor_identity": self.get_identity(self.supervisor_pid)
                if self.supervisor_pid is not None else None,
            "wrapper_identity": self.get_identity(self.wrapper_pid)
                if self.wrapper_pid is not None else None,
            "supervisor_identity_matches": self._supervisor_alive(),
            "launch_pid": self.launch_pid, "launch_pgid": self.launch_pid,
            "launch_start_ticks": self.launch_start_ticks,
            "process_group_snapshot_after": final_snapshot,
            "process_group_empty_after": None if launch_not_started else group_empty,
            "prior_cleanup_receipt_verified": prior is not None,
            "launch_main_reaped": (True if prior else
                                    "NOT_APPLICABLE" if launch_not_started else "UNKNOWN"),
            "cleanup_ok": None if launch_not_started else completed,
            "cleanup_status": "NOT_STARTED" if launch_not_started else
                "PASS" if completed else "FAIL_CLOSED",
            "limitations": [
                "The watchdog is not the parent of the launch process and cannot synthesize a Popen wait status.",
                "A verified prior cleanup receipt is required to claim launch-main reaping.",
                "An incomplete or unscannable process group remains FAIL_CLOSED.",
            ],
        }
        atomic_json(self.run_dir / "hard_deadline.json", record)
        cleanup = prior if completed else ({
            "status": "NOT_STARTED", "cleanup_ok": None,
            "reason": "supervisor rejected the run directory before launch creation",
            "launch_main_reaped": "NOT_APPLICABLE",
            "process_group_alive_after_cleanup": None,
            "process_group_state_after_cleanup": "NOT_STARTED",
            "process_group_snapshot_after_cleanup": final_snapshot,
            "watchdog_signals": self.signals,
        } if launch_not_started else {
            "status": "FAIL_CLOSED_WATCHDOG", "cleanup_ok": False,
            "reason": reason, "launch_main_reaped": "UNKNOWN",
            "process_group_alive_after_cleanup": (
                bool(final_snapshot.get("running_members"))
                if final_snapshot.get("scan_complete") is True else None),
            "process_group_state_after_cleanup": final_snapshot.get("status", "UNKNOWN"),
            "process_group_snapshot_after_cleanup": final_snapshot,
            "watchdog_signals": self.signals,
        })
        try:
            result = json.loads((self.run_dir / "result.json").read_text())
            if not isinstance(result, dict):
                result = {}
        except (OSError, ValueError):
            result = {}
        if not result:
            try:
                wrapper = json.loads((self.run_dir / "wrapper.json").read_text())
            except (OSError, ValueError):
                wrapper = {}
            result = {
                "schema_version": 1, "run_id": self.run_id,
                "t0_monotonic_ns": self.t0_ns, "started_at_utc": wrapper.get("t0_utc"),
                "launch_started": self.launch_pid is not None,
                "launch_pid": self.launch_pid, "process_group_id": self.launch_pid,
                "launch_exit_code": None, "dynamic_entry_calls": 1,
                "motion_enabled": False, "automatic_task_enabled": False,
                "authorization_or_lease_created": False,
                "phases": {phase: "NOT RUN" for phase in PHASES}, "failures": [],
            }
        failures = result.setdefault("failures", [])
        try:
            state = json.loads((self.run_dir / "supervisor_state.json").read_text())
        except (OSError, ValueError):
            state = {}
        if (state.get("run_id") == self.run_id
                and state.get("timing_profile") in ("localization-diagnostic", "historical-motion")):
            verified = state.get("verified_no_motion_startup")
            if (isinstance(verified, dict) and verified.get("status") == "PASS"
                    and verified.get("run_id") == self.run_id):
                # Keep the already persisted Level A result when the separate
                # long SOAK later reaches the watchdog's failure path.
                result["verified_no_motion_startup"] = verified
                result["startup_soak"] = {
                    "status": "FAIL", "target_launch_elapsed_s": MIN_LAUNCH_OBSERVATION_S,
                    "stop_reason": reason,
                    "readiness_loss_count": len(result.get("readiness_losses", [])),
                    "failure_stage": "watchdog",
                }
            else:
                result.setdefault("verified_no_motion_startup", {
                    "status": "FAIL", "reason": "Level A was not persisted before watchdog stop",
                    "run_id": self.run_id,
                })
                result.setdefault("startup_soak", {
                    "status": "NOT COMPLETED", "target_launch_elapsed_s": MIN_LAUNCH_OBSERVATION_S,
                    "stop_reason": reason,
                })
        if supervisor_failure is not None:
            result["first_failure"] = supervisor_failure
            result["failure_stage"] = "run_directory_admission"
            result["launch_started"] = False
            result["launch_pid"] = None
            failures.append("supervisor rejected run-directory admission: "
                            + supervisor_failure["reason"])
        failures.append("independent watchdog stopped an incomplete or overdue supervisor")
        result.update({
            "status": "FAIL", "failure_stage": (
                "hard_deadline" if hard_cutoff else
                "run_directory_admission" if launch_not_started else "watchdog"),
            "hard_deadline_enforced": hard_cutoff, "hard_deadline": record,
            "hard_process_cutoff_t0_s": HARD_PROCESS_CUTOFF_FROM_T0_S,
            "cleanup_evidence_cutoff_t0_s": CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S,
            "cleanup": cleanup, "cleanup_verified": completed,
            "failures": failures,
            "completed_at_utc": timestamp(),
            "total_elapsed_s": round(self._elapsed(), 3),
        })
        for filename, default in (("environment.json", {"status": "PARTIAL"}),
                                  ("source_identity.json", {"status": "PARTIAL"})):
            path = self.run_dir / filename
            if not path.exists():
                atomic_json(path, default)
        atomic_json(self.run_dir / "cleanup.json", cleanup)
        atomic_json(self.run_dir / "result.json", result)
        events_path = self.run_dir / "events.json"
        try:
            events = json.loads(events_path.read_text())
            rows = events.get("events", []) if isinstance(events, dict) else []
        except (OSError, ValueError):
            rows = []
        rows.append({"event": "watchdog_terminal_failure", "utc": timestamp(),
                     "monotonic_ns": self.clock_ns(), "reason": reason,
                     "hard_deadline_enforced": hard_cutoff})
        atomic_json(events_path, {"events": rows})
        write_checksums(self.run_dir)
        return record

    def _force_cutoff(self, reason: str) -> int:
        # Persist the trigger before acting so an interrupted cleanup is still
        # visible to the shell wrapper's finalizer.
        trigger = {"schema_version": 1, "run_id": self.run_id,
                   "status": "CUT_OFF_IN_PROGRESS", "reason": reason,
                   "hard_deadline_enforced": True,
                   "triggered_at_t0_elapsed_s": round(self._elapsed(), 6),
                   "signals": self.signals}
        atomic_json(self.run_dir / "hard_deadline.json", trigger)
        snapshot = self._group_snapshot()
        if snapshot.get("scan_complete") is not True or snapshot.get("members"):
            self._signal_group(signal.SIGKILL)
        if self._supervisor_alive():
            self._signal_supervisor(signal.SIGTERM)
            self.sleeper(0.1)
            if self._supervisor_alive():
                self._signal_supervisor(signal.SIGKILL)
        scan_cutoff_s = CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S - WATCHDOG_RECEIPT_RESERVE_S
        cutoff_ns = self.t0_ns + int(scan_cutoff_s * 1_000_000_000)
        while self.clock_ns() < cutoff_ns:
            final_snapshot = self._group_snapshot()
            if final_snapshot.get("scan_complete") is True and final_snapshot.get("status") == "EMPTY":
                break
            self.sleeper(min(0.05, max(0.0, (cutoff_ns - self.clock_ns()) / 1_000_000_000)))
        self._write_failure(reason, hard_cutoff=True)
        code = 124
        atomic_json(self.run_dir / "watchdog_exit.json", {
            "status": "HARD_CUTOFF", "exit_code": code,
            "run_id": self.run_id, "pid": os.getpid(),
            "t0_elapsed_s": round(self._elapsed(), 6), "recorded_at_utc": timestamp(),
        })
        return code

    def run(self, *, poll_interval_s: float = 0.05, on_action=None) -> int:
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._write_ready()
        while True:
            self._refresh_identity_from_state()
            if self._completion_recorded() and not self._supervisor_alive():
                atomic_json(self.run_dir / "watchdog_exit.json", {
                    "status": "NORMAL_COMPLETION", "exit_code": 0,
                    "run_id": self.run_id, "pid": os.getpid(),
                    "t0_elapsed_s": round(self._elapsed(), 6), "recorded_at_utc": timestamp(),
                })
                return 0
            elapsed = self._elapsed()
            self._refresh_identity_from_state()
            group = self._group_snapshot()
            alive = self._supervisor_alive()
            state = {}
            try:
                state = json.loads((self.run_dir / "supervisor_state.json").read_text())
            except (OSError, ValueError):
                pass
            actions = self.schedule.actions(
                elapsed, supervisor_alive=alive,
                process_group_status=group.get("status", "UNKNOWN"),
                # Re-read after checking liveness: completion may be written
                # between the loop-head read and the supervisor exiting.
                completion_recorded=self._completion_recorded(),
                # The diagnostic profile has two distinct milestones.  Once
                # Level A has been durably verified, later SOAK readiness
                # losses must not be reclassified as a startup timeout at
                # launch-relative T+180.  The supervisor still owns the
                # SOAK result and its normal bounded stop/cleanup.
                readiness_complete=readiness_complete_for_deadline(state),
                launch_elapsed_s=((self.clock_ns()-state['launch_started_monotonic_ns'])/1e9
                    if state.get('run_id') == self.run_id
                    and isinstance(state.get('launch_started_monotonic_ns'), int) else None))
            for action in actions:
                if on_action is not None:
                    on_action(action)
                kind = action["kind"]
                if kind == "signal_supervisor":
                    self._signal_supervisor(signal.SIGINT, action.get("reason"))
                elif kind == "signal_process_group":
                    self._signal_group(getattr(signal, action["signal"]))
                elif kind == "abnormal_supervisor_exit":
                    self._signal_group(signal.SIGTERM)
                    self.sleeper(0.1)
                    if self._group_snapshot().get("status") != "EMPTY":
                        self._signal_group(signal.SIGKILL)
                    failure = self._verified_pre_admission_failure()
                    failure_reason = ("supervisor_rejected_before_admission: " + failure["reason"]
                                      if failure else "supervisor_exited_without_completion_receipt")
                    self._write_failure(failure_reason, hard_cutoff=False)
                    atomic_json(self.run_dir / "watchdog_exit.json", {
                        "status": "SUPERVISOR_EXIT_WITHOUT_RECEIPT", "exit_code": 1,
                        "run_id": self.run_id, "pid": os.getpid(),
                        "reason": failure_reason,
                        "failure_stage": "run_directory_admission" if failure else "watchdog",
                        "t0_elapsed_s": round(self._elapsed(), 6), "recorded_at_utc": timestamp(),
                    })
                    return 1
                elif kind == "hard_process_cutoff":
                    return self._force_cutoff(f"supervisor_or_launch_group_alive_at_T0_plus_{HARD_PROCESS_CUTOFF_FROM_T0_S:g}s")
            self.sleeper(poll_interval_s)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run-dir", required=True)
    p.add_argument("--run-id", required=True)
    p.add_argument("--t0-ns", required=True, type=int)
    p.add_argument("--wrapper-pid", required=True, type=int)
    p.add_argument("--wrapper-start-ticks", required=True, type=int)
    return p


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    try:
        # Do not share the caller's terminal process group. Ctrl+C may stop the
        # supervised entry, but must leave this independent watchdog running.
        if os.getpgrp() != os.getpid():
            os.setsid()
        wrapper_identity = process_identity(args.wrapper_pid)
        if not wrapper_identity or wrapper_identity.get("start_ticks") != args.wrapper_start_ticks:
            raise RuntimeError("wrapper identity was not valid at watchdog start")
        return Watchdog(
            run_dir=Path(args.run_dir), run_id=args.run_id, t0_ns=args.t0_ns,
            wrapper_pid=args.wrapper_pid,
            wrapper_start_ticks=args.wrapper_start_ticks,
        ).run()
    except BaseException as error:
        # A watchdog implementation failure cannot leave a potentially live
        # launch unmonitored. Use the recorded PID/start-time identity before
        # signalling, and preserve uncertainty in the failure receipt.
        run_dir = Path(args.run_dir)
        run_dir.mkdir(parents=True, exist_ok=True)
        try:
            state = json.loads((Path(args.run_dir) / "supervisor_state.json").read_text())
        except (OSError, ValueError):
            state = {}
        supervisor_pid = state.get("supervisor_pid")
        supervisor_ticks = state.get("supervisor_start_ticks")
        launch_pid = state.get("launch_pid")
        launch_ticks = state.get("launch_start_ticks")
        before = runtime.process_group_snapshot(launch_pid) if isinstance(launch_pid, int) else {
            "status": "NOT_STARTED", "scan_complete": True, "members": [],
            "running_members": [], "zombie_members": [], "errors": []}
        leader = process_identity(launch_pid) if isinstance(launch_pid, int) else None
        if (leader and leader.get("start_ticks") == launch_ticks
                and leader.get("pgid") == launch_pid):
            try:
                os.killpg(launch_pid, signal.SIGKILL)
            except OSError:
                pass
        target_pid = supervisor_pid if isinstance(supervisor_pid, int) else None
        target_ticks = supervisor_ticks if isinstance(supervisor_ticks, int) else None
        sup = process_identity(target_pid) if target_pid is not None else None
        if sup and sup.get("start_ticks") == target_ticks:
            try:
                os.kill(target_pid, signal.SIGTERM)
                time.sleep(0.1)
                if identity_matches(target_pid, target_ticks):
                    os.kill(target_pid, signal.SIGKILL)
            except OSError:
                pass
        after = runtime.process_group_snapshot(launch_pid) if isinstance(launch_pid, int) else before
        failure = {"schema_version": 1, "run_id": args.run_id,
                   "status": "WATCHDOG_FAILURE", "hard_deadline_enforced": False,
                   "error": f"{type(error).__name__}: {error}",
                   "traceback": traceback.format_exc(), "recorded_at_utc": timestamp(),
                   "signals": [
                       {"target": "launch_process_group", "signal": "SIGKILL",
                        "attempted": bool(leader and leader.get("start_ticks") == launch_ticks),
                        "snapshot_before": before},
                       {"target": "supervisor", "signal": "SIGTERM_then_SIGKILL",
                        "attempted": bool(sup and sup.get("start_ticks") == target_ticks)},
                   ],
                   "process_group_snapshot_after": after,
                   "launch_main_reaped": "UNKNOWN", "cleanup_ok": False}
        try:
            atomic_json(Path(args.run_dir) / "watchdog_failure.json", failure)
            watchdog = Watchdog(
                run_dir=Path(args.run_dir), run_id=args.run_id, t0_ns=args.t0_ns,
                wrapper_pid=args.wrapper_pid, wrapper_start_ticks=args.wrapper_start_ticks)
            watchdog.supervisor_pid = supervisor_pid if isinstance(supervisor_pid, int) else None
            watchdog.supervisor_start_ticks = (supervisor_ticks if isinstance(supervisor_ticks, int)
                                               else None)
            watchdog.launch_pid = launch_pid if isinstance(launch_pid, int) else None
            watchdog.launch_start_ticks = launch_ticks if isinstance(launch_ticks, int) else None
            watchdog._write_failure(f"watchdog_failure:{type(error).__name__}", hard_cutoff=False)
        except BaseException:
            pass
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
