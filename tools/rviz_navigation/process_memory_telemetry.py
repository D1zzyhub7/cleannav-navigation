"""Lightweight, read-only /proc memory attribution for one RViz run.

The sampler is diagnostic evidence only. A late or partial sample is recorded
and never changes readiness or the supervisor's stop decision.
"""
from __future__ import annotations

import csv
import json
import os
from pathlib import Path
import re
import shlex
import time


CSV_FIELDS = (
    "sample_index", "sample_label", "sample_start_monotonic_ns",
    "sample_end_monotonic_ns", "sample_duration_s", "sample_interval_s",
    "sample_late", "scan_status", "scan_errors", "pid", "ppid", "pgid",
    "state", "process_start_ticks", "process_start_epoch_s", "comm",
    "executable", "command", "category", "rss_kib", "pss_kib",
    "shared_kib", "private_kib", "cpu_percent", "user_ticks", "system_ticks",
    "MemAvailable_kib", "MemFree_kib", "Cached_kib", "SwapFree_kib",
    "SwapUsed_kib", "memory_some_avg10", "memory_some_avg60",
    "memory_some_total_us", "memory_full_avg10", "memory_full_avg60",
    "memory_full_total_us", "cpu_some_avg10", "cpu_some_avg60",
    "cpu_some_total_us", "io_some_avg10", "io_some_avg60",
    "io_some_total_us", "loadavg_1", "loadavg_5", "loadavg_15",
    "oom_kill_count", "oom_kill_delta",
)

SUMMARY_CATEGORIES = (
    "gzserver", "gzclient", "robot_state_publisher", "controllers",
    "collision_monitor", "safety_supervisor", "rtabmap", "map_server",
    "nav2_planner", "nav2_controller", "mission_manager", "rviz",
    "navigation_guard", "supervisor", "watchdog", "other_launch_group",
    "other_run_process",
)


def parse_proc_stat(text: str) -> dict:
    """Parse Linux proc stat while allowing spaces and parentheses in comm."""
    close = text.rfind(")")
    if close < 0:
        raise ValueError("malformed /proc/PID/stat comm field")
    fields = text[close + 2:].split()
    if len(fields) < 20:
        raise ValueError("/proc/PID/stat fields are incomplete")
    return {
        "state": fields[0], "ppid": int(fields[1]), "pgid": int(fields[2]),
        "user_ticks": int(fields[11]), "system_ticks": int(fields[12]),
        "start_ticks": int(fields[19]),
    }


def _read_kib_status(path: Path) -> dict:
    result = {}
    for line in path.read_text().splitlines():
        key, separator, value = line.partition(":")
        if separator and key in ("VmRSS", "RssAnon", "RssFile", "RssShmem"):
            try:
                result[key] = int(value.strip().split()[0])
            except (ValueError, IndexError):
                pass
    return result


def _read_smaps_rollup(path: Path) -> dict:
    values = {}
    for line in path.read_text().splitlines():
        key, separator, value = line.partition(":")
        if separator and key in ("Pss", "Shared_Clean", "Shared_Dirty",
                                 "Private_Clean", "Private_Dirty"):
            try:
                values[key] = int(value.strip().split()[0])
            except (ValueError, IndexError):
                pass
    return values


def _read_pressure(path: Path) -> dict:
    result = {}
    try:
        for line in path.read_text().splitlines():
            fields = line.split()
            if not fields:
                continue
            row = {}
            for item in fields[1:]:
                key, separator, value = item.partition("=")
                if separator:
                    row[key] = float(value) if key.startswith("avg") else int(value)
            result[fields[0]] = row
        return result
    except (OSError, ValueError) as error:
        return {"_error": f"{type(error).__name__}: {error}"}


def _read_system(proc_root: Path) -> dict:
    meminfo = {}
    try:
        for line in (proc_root / "meminfo").read_text().splitlines():
            key, separator, value = line.partition(":")
            if separator:
                try:
                    meminfo[key] = int(value.strip().split()[0])
                except (ValueError, IndexError):
                    pass
    except OSError as error:
        meminfo["_error"] = f"{type(error).__name__}: {error}"
    swap_total = meminfo.get("SwapTotal")
    swap_free = meminfo.get("SwapFree")
    memory_pressure = _read_pressure(proc_root / "pressure/memory")
    cpu_pressure = _read_pressure(proc_root / "pressure/cpu")
    io_pressure = _read_pressure(proc_root / "pressure/io")
    try:
        loadavg = [float(value) for value in
                   (proc_root / "loadavg").read_text().split()[:3]]
    except (OSError, ValueError):
        loadavg = []
    try:
        btime = next(int(line.split()[1]) for line in
                     (proc_root / "stat").read_text().splitlines()
                     if line.startswith("btime "))
    except (OSError, StopIteration, ValueError, IndexError):
        btime = None
    try:
        vmstat = dict(line.split()[:2] for line in
                      (proc_root / "vmstat").read_text().splitlines()
                      if len(line.split()) >= 2)
        oom_kill = int(vmstat["oom_kill"]) if "oom_kill" in vmstat else None
    except (OSError, ValueError, IndexError):
        oom_kill = None
    return {
        "MemAvailable_kib": meminfo.get("MemAvailable"),
        "MemFree_kib": meminfo.get("MemFree"), "Cached_kib": meminfo.get("Cached"),
        "SwapFree_kib": swap_free,
        "SwapUsed_kib": (swap_total - swap_free
                         if swap_total is not None and swap_free is not None else None),
        "memory_pressure": memory_pressure, "cpu_pressure": cpu_pressure,
        "io_pressure": io_pressure,
        "loadavg": loadavg + [None] * (3 - len(loadavg)),
        "boot_time_epoch_s": btime, "oom_kill_count": oom_kill,
    }


def classify_process(comm: str, executable: str, command: str, *, role: str | None = None) -> str:
    comm_name = Path(comm).name.lower()
    executable_name = Path(executable).name.lower()
    # Prefer executable identity. Searching the complete command string for
    # "rtabmap" misclassified RViz and helper processes because their ROS
    # parameter arguments include `is_rtabmap_paused`.
    executable_rules = {
        "gzserver": "gzserver", "gzclient": "gzclient",
        "robot_state_publisher": "robot_state_publisher",
        "collision_monitor": "collision_monitor",
        "safety_supervisor_node": "safety_supervisor",
        "rtabmap": "rtabmap", "map_server": "map_server",
        "planner_server": "nav2_planner",
        "controller_server": "nav2_controller", "rviz2": "rviz",
        "controller_manager": "controllers", "ros2_control_node": "controllers",
    }
    for name, category in executable_rules.items():
        if comm_name == name or executable_name == name:
            return category

    try:
        command_names = {Path(token).name.lower() for token in shlex.split(command)}
    except ValueError:
        command_names = set()
    script_rules = {
        "navigation_supervisor.py": "supervisor",
        "deadline_watchdog.py": "watchdog",
        "run_rviz_navigation.sh": "supervisor",
        "navigation_safety_guard.py": "navigation_guard",
        "safety_supervisor_node": "safety_supervisor",
        "collision_monitor_lifecycle_startup.py": "collision_monitor",
        "spawner": "controllers",
    }
    for name, category in script_rules.items():
        if name in command_names:
            return category
    if role == "watchdog":
        return "watchdog"
    if role in ("wrapper", "supervisor"):
        return "supervisor"
    return "other_launch_group" if role == "launch" else "other_run_process"


_SENSITIVE_ARG = re.compile(r"(password|passwd|token|secret|credential|api[-_]?key)", re.I)


def _safe_command(raw: bytes) -> str:
    args = [part.decode(errors="replace") for part in raw.split(b"\0") if part]
    redact_next = False
    safe = []
    for arg in args:
        if redact_next:
            safe.append("<REDACTED>")
            redact_next = False
            continue
        if _SENSITIVE_ARG.search(arg):
            if "=" in arg:
                safe.append(arg.split("=", 1)[0] + "=<REDACTED>")
            else:
                safe.append(arg)
                redact_next = True
        else:
            safe.append(arg)
    return " ".join(safe)[:1200]


def _read_process(entry: Path, *, proc_root: Path, btime: int | None,
                  clock_ticks_per_second: int) -> dict:
    pid = int(entry.name)
    stat = parse_proc_stat((entry / "stat").read_text())
    try:
        comm = (entry / "comm").read_text().strip()
    except OSError:
        comm = "UNKNOWN"
    try:
        command = _safe_command((entry / "cmdline").read_bytes())
    except OSError:
        command = comm
    try:
        executable = str((entry / "exe").resolve(strict=True))
    except OSError:
        executable = "UNKNOWN"
    status = _read_kib_status(entry / "status")
    try:
        smaps = _read_smaps_rollup(entry / "smaps_rollup")
    except OSError:
        smaps = {}
    return {
        "pid": pid, **stat, "comm": comm, "executable": executable,
        "command": command,
        "process_start_epoch_s": (btime + stat["start_ticks"] / clock_ticks_per_second
                                  if btime is not None else None),
        "rss_kib": status.get("VmRSS"), "pss_kib": smaps.get("Pss"),
        "shared_kib": (smaps.get("Shared_Clean", 0) + smaps.get("Shared_Dirty", 0)
                       if smaps else None),
        "private_kib": (smaps.get("Private_Clean", 0) + smaps.get("Private_Dirty", 0)
                        if smaps else None),
        "user_ticks": stat["user_ticks"], "system_ticks": stat["system_ticks"],
    }


def _read_process_index(proc_root: Path) -> tuple[dict[int, dict], list[str]]:
    """Read only cheap identity fields system-wide to find owned processes."""
    records: dict[int, dict] = {}
    errors = []
    try:
        entries = list(proc_root.iterdir())
    except OSError as error:
        return {}, [f"proc_scan:{type(error).__name__}:{error}"]
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            stat = parse_proc_stat((entry / "stat").read_text())
            records[int(entry.name)] = stat
        except FileNotFoundError:
            # Process exit during a /proc scan is an expected race.
            continue
        except (OSError, ValueError, IndexError) as error:
            errors.append(f"pid={entry.name}:{type(error).__name__}:{error}")
    return records, errors


class ProcessMemoryTelemetry:
    """Write one system/process sample per call and summarize the run."""

    def __init__(self, run_dir: Path, run_id: str, *, proc_root: Path = Path("/proc"),
                 clock_ns=time.monotonic_ns, wall_ns=time.time_ns,
                 clock_ticks_per_second: int | None = None):
        self.run_dir = Path(run_dir)
        self.run_id = run_id
        self.proc_root = Path(proc_root)
        self.clock_ns = clock_ns
        self.wall_ns = wall_ns
        self.ticks = clock_ticks_per_second or os.sysconf("SC_CLK_TCK")
        self.path = self.run_dir / "process-memory-timeseries.csv"
        self.stream = self.path.open("x", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        self.writer.writeheader()
        self.sample_index = 0
        self.previous_sample_start_ns = None
        self.previous_process_ticks = {}
        self.previous_cpu_sample_ns = None
        self.rows: list[dict] = []
        self.sample_meta: list[dict] = []
        # Take the kernel OOM counter baseline as soon as the run telemetry
        # object is created, before the integrated launch is spawned.
        self.oom_baseline = _read_system(self.proc_root).get("oom_kill_count")
        self.closed = False

    def _identity_roots(self, supervisor_pid: int) -> list[tuple[int, str, int | None]]:
        roots = [(supervisor_pid, "supervisor", None)]
        for filename in ("watchdog_ready.json", "supervisor_state.json"):
            try:
                record = json.loads((self.run_dir / filename).read_text())
            except (OSError, ValueError):
                continue
            candidates = ((record.get("wrapper_pid"), record.get("wrapper_start_ticks"), "wrapper"),
                          (record.get("pid"), record.get("start_ticks"), "watchdog"),
                          (record.get("supervisor_pid"), record.get("supervisor_start_ticks"), "supervisor"),
                          (record.get("launch_pid"), record.get("launch_start_ticks"), "launch"))
            for pid, start, role in candidates:
                if isinstance(pid, int) and isinstance(start, int):
                    roots.append((pid, role, start))
        return roots

    def sample(self, *, launch_pgid: int, supervisor_pid: int, t0_ns: int,
               launch_start_ns: int, label: str = "runtime") -> dict:
        if self.closed:
            raise RuntimeError("process telemetry is closed")
        start_ns = self.clock_ns()
        system = _read_system(self.proc_root)
        process_index, errors = _read_process_index(self.proc_root)
        roots = self._identity_roots(supervisor_pid)
        selected_ids = {pid for pid, _role, expected_start in roots
                        if pid in process_index and (expected_start is None
                           or process_index[pid]["start_ticks"] == expected_start)}
        # Include complete descendants of wrapper/supervisor/watchdog roots.
        changed = True
        while changed:
            changed = False
            for pid, record in process_index.items():
                if record["ppid"] in selected_ids and pid not in selected_ids:
                    selected_ids.add(pid)
                    changed = True
        # The owned launch process group is authoritative even after reparenting.
        selected_ids.update(pid for pid, record in process_index.items()
                             if record["pgid"] == launch_pgid)
        roles = {pid: role for pid, role, expected_start in roots
                 if pid in process_index and (expected_start is None
                    or process_index[pid]["start_ticks"] == expected_start)}
        processes = {}
        for pid in sorted(selected_ids):
            try:
                processes[pid] = _read_process(
                    self.proc_root / str(pid), proc_root=self.proc_root,
                    btime=system["boot_time_epoch_s"],
                    clock_ticks_per_second=self.ticks)
            except FileNotFoundError:
                # Exiting processes during the selected detail pass are normal.
                continue
            except (OSError, ValueError, IndexError) as error:
                errors.append(f"pid={pid}:detail:{type(error).__name__}:{error}")
        selected_ids.intersection_update(processes)
        for pid in selected_ids:
            roles.setdefault(pid, "launch" if processes[pid]["pgid"] == launch_pgid
                              else "run_tree")
        end_ns = self.clock_ns()
        self.sample_index += 1
        interval_s = (None if self.previous_sample_start_ns is None else
                      (start_ns - self.previous_sample_start_ns) / 1e9)
        duration_s = max(0.0, (end_ns - start_ns) / 1e9)
        late = interval_s is not None and interval_s > 1.5
        oom = system.get("oom_kill_count")
        oom_delta = (None if oom is None or self.oom_baseline is None else
                     oom - self.oom_baseline)
        if self.oom_baseline is None and oom is not None:
            self.oom_baseline = oom
            oom_delta = 0
        now_process_ticks = {}
        rows = []
        for pid in sorted(selected_ids):
            item = processes[pid]
            key = (pid, item["start_ticks"])
            ticks = item["user_ticks"] + item["system_ticks"]
            previous = self.previous_process_ticks.get(key)
            cpu_pct = (None if previous is None or self.previous_cpu_sample_ns is None
                       or start_ns <= self.previous_cpu_sample_ns else
                       100.0 * (ticks - previous) / self.ticks /
                       ((start_ns - self.previous_cpu_sample_ns) / 1e9))
            now_process_ticks[key] = ticks
            item_role = roles.get(pid)
            row = {
                "sample_index": self.sample_index, "sample_label": label,
                "sample_start_monotonic_ns": start_ns,
                "sample_end_monotonic_ns": end_ns,
                "sample_duration_s": round(duration_s, 6),
                "sample_interval_s": (round(interval_s, 6) if interval_s is not None else None),
                "sample_late": late, "scan_status": "PARTIAL" if errors else "COMPLETE",
                "scan_errors": json.dumps(errors[:20]),
                "pid": pid, "ppid": item["ppid"], "pgid": item["pgid"],
                "state": item["state"], "process_start_ticks": item["start_ticks"],
                "process_start_epoch_s": item["process_start_epoch_s"],
                "comm": item["comm"], "executable": item["executable"],
                "command": item["command"],
                "category": classify_process(item["comm"], item["executable"],
                                              item["command"], role=item_role),
                "rss_kib": item["rss_kib"], "pss_kib": item["pss_kib"],
                "shared_kib": item["shared_kib"], "private_kib": item["private_kib"],
                "cpu_percent": round(cpu_pct, 4) if cpu_pct is not None else None,
                "user_ticks": item["user_ticks"], "system_ticks": item["system_ticks"],
                "MemAvailable_kib": system["MemAvailable_kib"],
                "MemFree_kib": system["MemFree_kib"], "Cached_kib": system["Cached_kib"],
                "SwapFree_kib": system["SwapFree_kib"], "SwapUsed_kib": system["SwapUsed_kib"],
                "oom_kill_count": oom, "oom_kill_delta": oom_delta,
                "loadavg_1": system["loadavg"][0], "loadavg_5": system["loadavg"][1],
                "loadavg_15": system["loadavg"][2],
            }
            for prefix, pressure, key in (("memory", system["memory_pressure"], "some"),
                                          ("memory", system["memory_pressure"], "full"),
                                          ("cpu", system["cpu_pressure"], "some"),
                                          ("io", system["io_pressure"], "some")):
                data = pressure.get(key, {}) if isinstance(pressure, dict) else {}
                row[f"{prefix}_{key}_avg10"] = data.get("avg10")
                row[f"{prefix}_{key}_avg60"] = data.get("avg60")
                row[f"{prefix}_{key}_total_us"] = data.get("total")
            rows.append(row)
        if not rows:
            rows.append({
                "sample_index": self.sample_index, "sample_label": label,
                "sample_start_monotonic_ns": start_ns,
                "sample_end_monotonic_ns": end_ns,
                "sample_duration_s": round(duration_s, 6),
                "sample_interval_s": round(interval_s, 6) if interval_s is not None else None,
                "sample_late": late, "scan_status": "PARTIAL" if errors else "COMPLETE",
                "scan_errors": json.dumps(errors[:20]), "category": "other_run_process",
                "MemAvailable_kib": system["MemAvailable_kib"],
                "MemFree_kib": system["MemFree_kib"], "Cached_kib": system["Cached_kib"],
                "SwapFree_kib": system["SwapFree_kib"], "SwapUsed_kib": system["SwapUsed_kib"],
                "oom_kill_count": oom, "oom_kill_delta": oom_delta,
                "loadavg_1": system["loadavg"][0], "loadavg_5": system["loadavg"][1],
                "loadavg_15": system["loadavg"][2],
            })
        for row in rows:
            self.writer.writerow(row)
        self.stream.flush()
        self.rows.extend(rows)
        self.sample_meta.append({
            "sample_index": self.sample_index, "label": label,
            "start_monotonic_ns": start_ns, "end_monotonic_ns": end_ns,
            "duration_s": duration_s, "interval_s": interval_s,
            "late": late, "scan_status": "PARTIAL" if errors else "COMPLETE",
            "process_count": len(selected_ids), "scan_error_count": len(errors),
            "MemAvailable_kib": system["MemAvailable_kib"],
            "MemFree_kib": system["MemFree_kib"], "Cached_kib": system["Cached_kib"],
            "SwapFree_kib": system["SwapFree_kib"], "SwapUsed_kib": system["SwapUsed_kib"],
            "memory_pressure": system["memory_pressure"],
            "cpu_pressure": system["cpu_pressure"], "io_pressure": system["io_pressure"],
            "loadavg": system["loadavg"], "oom_kill_count": oom,
            "oom_kill_delta": oom_delta,
        })
        self.previous_process_ticks = now_process_ticks
        self.previous_cpu_sample_ns = start_ns
        self.previous_sample_start_ns = start_ns
        return self.sample_meta[-1]

    def summary(self) -> dict:
        if not self.sample_meta:
            return {"status": "NOT_SAMPLED", "run_id": self.run_id,
                    "timeseries_csv": self.path.name, "samples": 0}
        sample_ids = sorted({row["sample_index"] for row in self.rows})
        identities = {}
        for row in self.rows:
            if row.get("pid") is None:
                continue
            key = (row["pid"], row["process_start_ticks"])
            rec = identities.setdefault(key, {
                "pid": row["pid"], "process_start_ticks": row["process_start_ticks"],
                "comm": row.get("comm"), "executable": row.get("executable"),
                "command": row.get("command"), "category": row.get("category"),
                "first_sample": row["sample_index"], "last_sample": row["sample_index"],
                "startup_rss_kib": row.get("rss_kib"), "startup_pss_kib": row.get("pss_kib"),
                "peak_rss_kib": row.get("rss_kib"), "peak_pss_kib": row.get("pss_kib"),
                "last_rss_kib": row.get("rss_kib"), "last_pss_kib": row.get("pss_kib"),
                "cpu_percent_peak": row.get("cpu_percent"),
            })
            rec["last_sample"] = row["sample_index"]
            for field in ("rss_kib", "pss_kib"):
                value = row.get(field)
                if value is not None:
                    rec[f"peak_{field}"] = max(rec.get(f"peak_{field}") or value, value)
                    rec[f"last_{field}"] = value
            cpu = row.get("cpu_percent")
            if cpu is not None:
                rec["cpu_percent_peak"] = max(rec.get("cpu_percent_peak") or cpu, cpu)
        per_process = []
        run_start_id, run_end_id = sample_ids[0], sample_ids[-1]
        process_rows = {}
        for row in self.rows:
            if row.get("pid") is not None:
                process_rows.setdefault((row["pid"], row["process_start_ticks"]), {})[
                    row["sample_index"]] = row
        for key, rec in identities.items():
            first_row = process_rows[key].get(run_start_id)
            last_row = process_rows[key].get(run_end_id)
            for metric in ("rss_kib", "pss_kib"):
                # Growth compares each process's first observation with its
                # last observed resident state. A process that has exited by
                # the final cleanup sample is absent, not a zero-RSS process.
                start_value = rec.get(f"startup_{metric}")
                end_value = rec.get(f"last_{metric}")
                rec[f"run_start_{metric}"] = start_value
                rec[f"run_end_{metric}"] = end_value
                rec[f"run_growth_{metric}"] = (
                    end_value - start_value
                    if end_value is not None and start_value is not None else None)
            rec["present_in_final_sample"] = last_row is not None
            per_process.append(rec)
        positive_rss = sum(max(0, row["run_growth_rss_kib"] or 0) for row in per_process)
        positive_pss = sum(max(0, row["run_growth_pss_kib"] or 0) for row in per_process)
        for row in per_process:
            row["positive_rss_growth_share"] = (
                max(0, row["run_growth_rss_kib"] or 0) / positive_rss if positive_rss else None)
            row["positive_pss_growth_share"] = (
                max(0, row["run_growth_pss_kib"] or 0) / positive_pss if positive_pss else None)
        category_summary = {}
        for category in SUMMARY_CATEGORIES:
            members = [item for item in per_process if item["category"] == category]
            category_summary[category] = {
                "process_identities": len(members),
                "startup_rss_kib": sum((item.get("startup_rss_kib") or 0) for item in members),
                "startup_pss_kib": sum((item.get("startup_pss_kib") or 0) for item in members),
                "peak_rss_kib": sum((item.get("peak_rss_kib") or 0) for item in members),
                "peak_pss_kib": sum((item.get("peak_pss_kib") or 0) for item in members),
                "final_observed_rss_kib": sum((item.get("last_rss_kib") or 0)
                                               for item in members if item["present_in_final_sample"]),
                "final_observed_pss_kib": sum((item.get("last_pss_kib") or 0)
                                               for item in members if item["present_in_final_sample"]),
                "run_growth_rss_kib": sum((item["run_growth_rss_kib"] or 0) for item in members),
                "run_growth_pss_kib": sum((item["run_growth_pss_kib"] or 0) for item in members),
                "positive_rss_growth_share": (
                    sum(max(0, item["run_growth_rss_kib"] or 0) for item in members) / positive_rss
                    if positive_rss else None),
                "positive_pss_growth_share": (
                    sum(max(0, item["run_growth_pss_kib"] or 0) for item in members) / positive_pss
                    if positive_pss else None),
            }
        first_meta, last_meta = self.sample_meta[0], self.sample_meta[-1]
        return {
            "schema_version": 1, "run_id": self.run_id, "status": "RECORDED",
            "sampling_method": "Linux /proc owned launch process group plus wrapper/supervisor/watchdog descendant tree",
            "sample_interval_target_s": 1.0, "sample_count": len(self.sample_meta),
            "late_sample_count": sum(bool(item["late"]) for item in self.sample_meta),
            "partial_sample_count": sum(item["scan_status"] != "COMPLETE" for item in self.sample_meta),
            "first_monotonic_ns": first_meta["start_monotonic_ns"],
            "last_monotonic_ns": last_meta["end_monotonic_ns"],
            "duration_s": round((last_meta["end_monotonic_ns"] -
                                  first_meta["start_monotonic_ns"]) / 1e9, 6),
            "system_baseline": first_meta, "system_final": last_meta,
            "system_MemAvailable_change_kib": (
                last_meta["MemAvailable_kib"] - first_meta["MemAvailable_kib"]
                if last_meta["MemAvailable_kib"] is not None
                and first_meta["MemAvailable_kib"] is not None else None),
            "system_SwapUsed_change_kib": (
                last_meta["SwapUsed_kib"] - first_meta["SwapUsed_kib"]
                if last_meta["SwapUsed_kib"] is not None
                and first_meta["SwapUsed_kib"] is not None else None),
            "process_group_sum_rss_change_kib": sum(
                (item["run_growth_rss_kib"] or 0) for item in per_process),
            "process_group_sum_pss_change_kib": sum(
                (item["run_growth_pss_kib"] or 0) for item in per_process),
            "positive_growth_attribution_note": (
                "Per-process growth is last-observed minus first-observed memory for that process "
                "identity; a process absent in the final cleanup sample retains its last observed "
                "memory and is marked present_in_final_sample=false. Positive shares normalize only "
                "positive per-process growth; PSS is preferred because RSS sums include shared pages. "
                "This process attribution is not the same as host MemAvailable change."),
            "categories": category_summary,
            "processes": sorted(per_process,
                                 key=lambda item: (item["category"], item["pid"],
                                                   item["process_start_ticks"])),
            "samples": self.sample_meta,
            "timeseries_csv": self.path.name,
        }

    def close(self) -> dict:
        if not self.closed:
            self.stream.flush()
            self.stream.close()
            self.closed = True
        value = self.summary()
        target = self.run_dir / "memory-attribution-summary.json"
        target.write_text(json.dumps(value, indent=2, sort_keys=True, default=str) + "\n")
        return value
