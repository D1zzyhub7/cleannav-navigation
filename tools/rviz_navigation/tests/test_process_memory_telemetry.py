"""Process resource attribution tests using a synthetic /proc tree."""
from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

from process_memory_telemetry import (
    ProcessMemoryTelemetry, classify_process, parse_proc_stat,
)


def _proc_stat(pid: int, *, ppid: int, pgid: int, start: int,
               user: int = 10, system: int = 5, comm: str = "proc") -> str:
    # Linux fields after comm: state is field 3 and starttime is field 22.
    rest = ["S", str(ppid), str(pgid), "1", "0", "-1", "0", "0", "0", "0",
            "0", str(user), str(system), "0", "0", "20", "0", "1", "0",
            str(start), "4096", "1"]
    return f"{pid} ({comm}) " + " ".join(rest) + "\n"


def _fake_process(proc: Path, pid: int, *, ppid: int, pgid: int, start: int,
                  comm: str, rss: int, pss: int, user: int = 10) -> None:
    root = proc / str(pid)
    root.mkdir(exist_ok=True)
    (root / "stat").write_text(_proc_stat(pid, ppid=ppid, pgid=pgid,
                                          start=start, user=user, comm=comm))
    (root / "comm").write_text(comm + "\n")
    (root / "cmdline").write_bytes((comm + "\0--mode\0test\0").encode())
    exe = root / "exe"
    if not exe.exists():
        exe.symlink_to("/usr/bin/python3")
    (root / "status").write_text(
        f"Name:\t{comm}\nVmRSS:\t{rss} kB\nRssAnon:\t{rss - 10} kB\n")
    (root / "smaps_rollup").write_text(
        f"Pss:\t{pss} kB\nShared_Clean:\t10 kB\nShared_Dirty:\t2 kB\n"
        f"Private_Clean:\t{pss - 12} kB\nPrivate_Dirty:\t0 kB\n")


def _fake_proc(tmp_path: Path) -> Path:
    proc = tmp_path / "proc"
    proc.mkdir()
    (proc / "pressure").mkdir()
    (proc / "meminfo").write_text(
        "MemTotal: 16000000 kB\nMemFree: 3000000 kB\n"
        "MemAvailable: 6000000 kB\nCached: 2000000 kB\n"
        "SwapTotal: 1000000 kB\nSwapFree: 900000 kB\n")
    (proc / "stat").write_text("cpu 100 0 50 850 0 0 0 0 0 0\nbtime 1700000000\n")
    (proc / "vmstat").write_text("oom_kill 2\n")
    (proc / "loadavg").write_text("1.00 0.80 0.60 1/20 99\n")
    (proc / "pressure/memory").write_text(
        "some avg10=0.10 avg60=0.20 avg300=0.30 total=40\n"
        "full avg10=0.01 avg60=0.02 avg300=0.03 total=4\n")
    (proc / "pressure/cpu").write_text("some avg10=2.0 avg60=2.1 avg300=2.2 total=100\n")
    (proc / "pressure/io").write_text("some avg10=0.0 avg60=0.0 avg300=0.0 total=0\n")
    _fake_process(proc, 70, ppid=1, pgid=70, start=700, comm="bash", rss=1000, pss=800)
    _fake_process(proc, 71, ppid=70, pgid=71, start=710, comm="python3", rss=2000, pss=1500)
    _fake_process(proc, 72, ppid=70, pgid=72, start=720, comm="python3", rss=3000, pss=2500)
    _fake_process(proc, 80, ppid=72, pgid=80, start=800, comm="ros2", rss=4000, pss=3500)
    _fake_process(proc, 81, ppid=80, pgid=80, start=810, comm="gzserver", rss=8000, pss=6000)
    _fake_process(proc, 90, ppid=1, pgid=90, start=900, comm="unrelated", rss=9000, pss=7000)
    return proc


def _identity_receipts(run: Path) -> None:
    (run / "watchdog_ready.json").write_text(json.dumps({
        "wrapper_pid": 70, "wrapper_start_ticks": 700,
        "pid": 71, "start_ticks": 710,
    }))
    (run / "supervisor_state.json").write_text(json.dumps({
        "supervisor_pid": 72, "supervisor_start_ticks": 720,
        "launch_pid": 80, "launch_start_ticks": 800,
    }))


def test_parse_proc_stat_handles_parenthesized_comm():
    record = parse_proc_stat(_proc_stat(10, ppid=1, pgid=10, start=123,
                                        comm="name (with parens)"))
    assert record["ppid"] == 1
    assert record["pgid"] == 10
    assert record["start_ticks"] == 123


def test_process_classification_uses_executable_not_parameter_substrings():
    rviz = classify_process(
        "rviz2", "/opt/ros/humble/lib/rviz2/rviz2",
        "rviz2 -d navigation.rviz --ros-args -p is_rtabmap_paused:=True",
        role="launch",
    )
    rtabmap = classify_process(
        "rtabmap", "/opt/ros/humble/lib/rtabmap_slam/rtabmap",
        "rtabmap --ros-args -r __ns:=/rtabmap -p use_sim_time:=True",
        role="launch",
    )
    helper = classify_process(
        "python3", "/usr/bin/python3.10",
        "python3 /opt/cleannav/localization_status_monitor --ros-args "
        "-p is_rtabmap_paused:=True",
        role="launch",
    )
    assert rviz == "rviz"
    assert rtabmap == "rtabmap"
    assert helper == "other_launch_group"


def test_sampler_attributes_only_run_tree_and_owned_process_group(tmp_path):
    proc = _fake_proc(tmp_path)
    run = tmp_path / "run"
    run.mkdir()
    _identity_receipts(run)
    ticks = iter([1_000_000_000, 1_000_000_001,
                  2_000_000_000, 2_000_000_001])
    sampler = ProcessMemoryTelemetry(
        run, "run-test", proc_root=proc, clock_ns=lambda: next(ticks),
        clock_ticks_per_second=100)
    first = sampler.sample(launch_pgid=80, supervisor_pid=72, t0_ns=1,
                           launch_start_ns=1, label="startup")
    _fake_process(proc, 81, ppid=80, pgid=80, start=810, comm="gzserver",
                  rss=10000, pss=7500, user=20)
    (proc / "vmstat").write_text("oom_kill 3\n")
    second = sampler.sample(launch_pgid=80, supervisor_pid=72, t0_ns=1,
                            launch_start_ns=1, label="runtime")
    summary = sampler.close()

    assert first["scan_status"] == "COMPLETE"
    assert second["late"] is False
    assert second["oom_kill_delta"] == 1
    with (run / "process-memory-timeseries.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    pids = {int(row["pid"]) for row in rows if row["pid"]}
    assert pids == {70, 71, 72, 80, 81}
    gzserver = next(item for item in summary["processes"] if item["pid"] == 81)
    assert gzserver["category"] == "gzserver"
    assert gzserver["startup_rss_kib"] == 8000
    assert gzserver["peak_pss_kib"] == 7500
    assert gzserver["run_growth_rss_kib"] == 2000
    assert gzserver["positive_rss_growth_share"] > 0
    assert summary["system_baseline"]["MemAvailable_kib"] == 6_000_000
    assert summary["system_final"]["memory_pressure"]["full"]["avg10"] == 0.01
    assert (run / "memory-attribution-summary.json").is_file()


def test_late_proc_sample_is_recorded_and_does_not_raise(tmp_path):
    proc = _fake_proc(tmp_path)
    run = tmp_path / "run"
    run.mkdir()
    _identity_receipts(run)
    ticks = iter([1_000_000_000, 1_000_000_001,
                  3_000_000_000, 3_000_000_001])
    sampler = ProcessMemoryTelemetry(
        run, "run-late", proc_root=proc, clock_ns=lambda: next(ticks),
        clock_ticks_per_second=100)
    sampler.sample(launch_pgid=80, supervisor_pid=72, t0_ns=1,
                   launch_start_ns=1, label="first")
    late = sampler.sample(launch_pgid=80, supervisor_pid=72, t0_ns=1,
                          launch_start_ns=1, label="late")
    assert late["late"] is True
    assert sampler.close()["late_sample_count"] == 1
