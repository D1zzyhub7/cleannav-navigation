"""Pure, fail-closed contracts for the RViz navigation supervisor."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from pathlib import Path


PHASES = (
    "gazebo_vehicle",
    "collision_monitor",
    "dual_lidar_tf",
    "map_localization",
    "safety",
    "nav2",
    "mission_manager",
    "rviz",
)
SOFT_STOP_FROM_T0_S = 145.0
WATCHDOG_SIGTERM_FROM_T0_S = 159.0
WATCHDOG_SIGKILL_FROM_T0_S = 167.0
HARD_PROCESS_CUTOFF_FROM_T0_S = 175.0
CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S = 180.0
WATCHDOG_RECEIPT_RESERVE_S = 1.0
ACCOUNTING_LIMIT_FROM_T0_S = CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S
MIN_CLEANUP_RESERVE_S = 35.0
READINESS_DEADLINE_FROM_T0_S = 110.0
NO_MOTION_OBSERVATION_S = 10.0
MAX_NO_MOTION_DISPLACEMENT_M = 0.05
MOTION_ADMISSION_STABLE_S = 10.0
MAX_MOTION_SESSION_DISPLACEMENT_M = 2.5
# Historical mode: accumulated odometry and Gazebo net displacement differ.
MAX_HISTORICAL_ODOMETRY_M = 12.0
MAX_HISTORICAL_DISPLACEMENT_M = 11.0
HISTORICAL_TASK_TIMEOUT_S = 120.0
# Operational route envelope; task budgets below are derived from admitted geometry.
HISTORICAL_MAX_ROUTE_M = float(os.environ.get('CLEANNAV_RVIZ_MAX_ROUTE_M', '100'))
if not math.isfinite(HISTORICAL_MAX_ROUTE_M) or not 1 <= HISTORICAL_MAX_ROUTE_M <= 1000:
    raise ValueError('CLEANNAV_RVIZ_MAX_ROUTE_M must be finite and in [1,1000] metres')
HISTORICAL_TASK_MAX_S = max(HISTORICAL_TASK_TIMEOUT_S, HISTORICAL_MAX_ROUTE_M/.2+30.)


def historical_route_budget(goal_distance_m, path_length_m, path_extent_m=None):
    path_extent_m=goal_distance_m if path_extent_m is None else path_extent_m
    if not all(math.isfinite(x) and x >= 0 for x in (goal_distance_m, path_length_m, path_extent_m)):
        raise ValueError('route distances must be finite and nonnegative')
    length=max(goal_distance_m,path_length_m,path_extent_m)
    if length > HISTORICAL_MAX_ROUTE_M:
        raise ValueError('admitted path exceeds configured historical route envelope')
    return dict(goal_distance_m=goal_distance_m,path_length_m=path_length_m,path_extent_m=path_extent_m,
                odometry_m=max(MAX_HISTORICAL_ODOMETRY_M,1.5*length+2.),
                displacement_m=max(MAX_HISTORICAL_DISPLACEMENT_M,path_extent_m+2.),
                task_s=max(HISTORICAL_TASK_TIMEOUT_S,length/.2+30.))

RUNTIME_MEMORY_WARNING_KIB = 5 * 1024 * 1024
LOCALIZATION_DIAGNOSTIC_MEMORY_HISTORICAL_STOP_KIB = 4_718_592
LOCALIZATION_DIAGNOSTIC_MEMORY_EMERGENCY_KIB = 4 * 1024 * 1024

# The public wrapper resets this variable from an explicit, allow-listed CLI
# option. The bounded GUI wait below affects historical interaction only.
# Ordinary and diagnostic runs retain their existing schedules.
TIMING_PROFILE = os.environ.get('CLEANNAV_RVIZ_TIMING_PROFILE', 'standard')
if TIMING_PROFILE not in ('standard', 'localization-diagnostic', 'short-goal-motion', 'historical-motion'):
    raise ValueError('unsupported RViz timing profile: ' + TIMING_PROFILE)
LOCALIZATION_DIAGNOSTIC = TIMING_PROFILE == 'localization-diagnostic'
SHORT_GOAL_MOTION = TIMING_PROFILE == 'short-goal-motion'
HISTORICAL_MOTION = TIMING_PROFILE == 'historical-motion'
# Operator preparation time is separate from task and safety deadlines.
try:
    HISTORICAL_GUI_GOAL_WAIT_S = float(os.environ.get('CLEANNAV_RVIZ_GUI_GOAL_WAIT_S', '300'))
except ValueError as error:
    raise ValueError('CLEANNAV_RVIZ_GUI_GOAL_WAIT_S must be finite seconds in [1, 3600]') from error
if not math.isfinite(HISTORICAL_GUI_GOAL_WAIT_S) or not 1 <= HISTORICAL_GUI_GOAL_WAIT_S <= 3600:
    raise ValueError('CLEANNAV_RVIZ_GUI_GOAL_WAIT_S must be finite seconds in [1, 3600]')
EXTENDED_STARTUP = LOCALIZATION_DIAGNOSTIC or SHORT_GOAL_MOTION or HISTORICAL_MOTION
MIN_LAUNCH_OBSERVATION_S = 180.0 if LOCALIZATION_DIAGNOSTIC else 0.0
PREFLIGHT_LATEST_LAUNCH_S = 55.0 if LOCALIZATION_DIAGNOSTIC else None
if LOCALIZATION_DIAGNOSTIC:
    # Reserve a small scheduling margin after the latest permitted launch
    # (T0+55) before the launch-relative 180 s soak target (T0+235).
    READINESS_DEADLINE_FROM_T0_S = 240.0
    SOFT_STOP_FROM_T0_S = 245.0
    WATCHDOG_SIGTERM_FROM_T0_S = 259.0
    WATCHDOG_SIGKILL_FROM_T0_S = 267.0
    HARD_PROCESS_CUTOFF_FROM_T0_S = 275.0
    CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S = 280.0
    ACCOUNTING_LIMIT_FROM_T0_S = CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S
elif SHORT_GOAL_MOTION or HISTORICAL_MOTION:
    # Shared startup bound; historical mode gets its own GUI/task budget below.
    PREFLIGHT_LATEST_LAUNCH_S = 55.0
    READINESS_DEADLINE_FROM_T0_S = 205.0
    SOFT_STOP_FROM_T0_S = 335.0
    WATCHDOG_SIGTERM_FROM_T0_S = 349.0
    WATCHDOG_SIGKILL_FROM_T0_S = 357.0
    HARD_PROCESS_CUTOFF_FROM_T0_S = 365.0
    CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S = 370.0
    ACCOUNTING_LIMIT_FROM_T0_S = CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S

if HISTORICAL_MOTION:
    # Latest readiness + human input + historical 120 s task + 10 s release margin.
    # Keep the existing 14/22/30/35 s stop-escalation/cleanup offsets.
    SOFT_STOP_FROM_T0_S = (READINESS_DEADLINE_FROM_T0_S + HISTORICAL_GUI_GOAL_WAIT_S
                           + HISTORICAL_TASK_MAX_S + 10.0)
    WATCHDOG_SIGTERM_FROM_T0_S = SOFT_STOP_FROM_T0_S + 14.0
    WATCHDOG_SIGKILL_FROM_T0_S = SOFT_STOP_FROM_T0_S + 22.0
    HARD_PROCESS_CUTOFF_FROM_T0_S = SOFT_STOP_FROM_T0_S + 30.0
    CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S = SOFT_STOP_FROM_T0_S + 35.0
    ACCOUNTING_LIMIT_FROM_T0_S = CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S

MAX_LAUNCH_RUNTIME_S = (SOFT_STOP_FROM_T0_S if HISTORICAL_MOTION else
                        250.0 if SHORT_GOAL_MOTION else
                        195.0 if LOCALIZATION_DIAGNOSTIC else 145.0)


def timing_identity() -> dict:
    return dict(profile=TIMING_PROFILE,
                gui_goal_wait_s=HISTORICAL_GUI_GOAL_WAIT_S if HISTORICAL_MOTION else None,
                historical_task_timeout_s=HISTORICAL_TASK_TIMEOUT_S if HISTORICAL_MOTION else None,
                preflight_latest_launch_s=PREFLIGHT_LATEST_LAUNCH_S,
                min_launch_observation_s=MIN_LAUNCH_OBSERVATION_S,
                no_motion_startup_stable_s=NO_MOTION_OBSERVATION_S,
                startup_soak_target_launch_s=MIN_LAUNCH_OBSERVATION_S,
                readiness_latest_t0_s=READINESS_DEADLINE_FROM_T0_S,
                max_launch_runtime_s=MAX_LAUNCH_RUNTIME_S,
                historical_single_sample_memory_stop_kib=(
                    LOCALIZATION_DIAGNOSTIC_MEMORY_HISTORICAL_STOP_KIB
                    if LOCALIZATION_DIAGNOSTIC else None),
                soft_stop_t0_s=SOFT_STOP_FROM_T0_S,
                sigterm_t0_s=WATCHDOG_SIGTERM_FROM_T0_S,
                sigkill_t0_s=WATCHDOG_SIGKILL_FROM_T0_S,
                hard_process_cutoff_t0_s=HARD_PROCESS_CUTOFF_FROM_T0_S,
                evidence_cutoff_t0_s=CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S)


class ContractError(ValueError):
    """An identity or safety contract rejected the run."""


def runtime_memory_decision(snapshot: dict, *, timing_profile: str,
                            no_motion_contract: bool) -> dict:
    """Apply explicitly scoped PC diagnostic/restored-RViz resource policies.

    MemAvailable below 5 GiB is recorded as a warning. Existing PSI, swap,
    cgroup, and incomplete-snapshot failures remain hard stops. A changed
    kernel OOM-kill counter is also a hard stop when available. The explicit user-approved
    no-motion diagnostic floor remains unchanged. Formal motion uses pressure/OOM,
    swap and cgroup evidence, with no standalone MemAvailable hard floor.
    """
    if not isinstance(snapshot, dict):
        return {"ok": False, "reasons": ["memory_snapshot_invalid"],
                "policy": "invalid_snapshot", "soft_warning": False}
    if not ((timing_profile == "localization-diagnostic" and no_motion_contract is True)
            or (timing_profile in ("historical-motion", "short-goal-motion") and no_motion_contract is False)):
        reasons = list(snapshot.get("reasons") or [])
        if snapshot.get("ok") is not True and not reasons:
            reasons = ["resource_gate_rejected_without_reason"]
        return {"ok": snapshot.get("ok") is True, "reasons": reasons,
                "policy": "existing_runtime_gate", "soft_warning": False,
                "MemAvailable_kib": snapshot.get("MemAvailable_kib")}

    original_reasons = list(snapshot.get("reasons") or [])
    reasons = [reason for reason in original_reasons
               if not str(reason).startswith("MemAvailable_below_")]
    available = snapshot.get("MemAvailable_kib")
    if available is None:
        if "memory_or_psi_unavailable" not in reasons:
            reasons.append("memory_or_psi_unavailable")
        reasons.append("MemAvailable_unavailable")
        warning = False
    elif (not isinstance(available, (int, float)) or isinstance(available, bool)
          or not math.isfinite(available)):
        reasons.append("MemAvailable_invalid")
        available = None
        warning = False
    else:
        warning = available < RUNTIME_MEMORY_WARNING_KIB
        if timing_profile not in ('historical-motion', 'short-goal-motion') and available < LOCALIZATION_DIAGNOSTIC_MEMORY_EMERGENCY_KIB:
            reasons.append("MemAvailable_below_diagnostic_4GiB_emergency_floor")

    oom_delta = snapshot.get("oom_kill_delta")
    if (isinstance(oom_delta, (int, float)) and not isinstance(oom_delta, bool)
            and oom_delta > 0):
        reasons.append("kernel_oom_kill_counter_increased")

    legacy_memavailable_only = (
        snapshot.get("ok") is not True and bool(original_reasons)
        and all(str(reason).startswith("MemAvailable_below_")
                for reason in original_reasons))
    if snapshot.get("ok") is not True and not reasons and not legacy_memavailable_only:
        reasons.append("resource_snapshot_rejected_without_reason")

    return {
        "ok": not reasons,
        "reasons": reasons,
        "policy": ("historical-motion-pc-pressure-evidence" if timing_profile in ("historical-motion", "short-goal-motion")
                   else "localization-diagnostic-no-motion-pressure-evidence"),
        "soft_warning": warning,
        "soft_warning_threshold_kib": RUNTIME_MEMORY_WARNING_KIB,
        "historical_single_sample_stop_line_kib": LOCALIZATION_DIAGNOSTIC_MEMORY_HISTORICAL_STOP_KIB,
        "hard_stop_threshold_kib": (None if timing_profile in ('historical-motion','short-goal-motion')
                                    else LOCALIZATION_DIAGNOSTIC_MEMORY_EMERGENCY_KIB),
        "hard_stop_reasons": reasons,
        "oom_kill_delta": oom_delta,
        "MemAvailable_kib": available,
    }


def verify_no_motion_launch_argv(argv: list[str]) -> dict:
    """Require the fixed integrated launch argv to remain explicitly no-motion."""
    motion = [value for value in argv if value.startswith("enable_motion:=")]
    automatic = [value for value in argv if value.startswith("auto_start_task:=")]
    valid = (motion == ["enable_motion:=false"]
             and automatic == ["auto_start_task:=false"]
             and not any("authorization" in value.lower()
                         or "lease" in value.lower()
                         or "scope" in value.lower() for value in argv))
    if not valid:
        raise ContractError("integrated launch argv violates the fixed no-motion/no-task contract")
    return {"enable_motion": False, "auto_start_task": False,
            "authorization_or_lease_options_present": False}


def verify_short_goal_launch_argv(argv: list[str], expected_profile="short-goal") -> dict:
    """Require explicit, single-profile motion and keep automatic work off."""
    motion = [value for value in argv if value.startswith("enable_motion:=")]
    automatic = [value for value in argv if value.startswith("auto_start_task:=")]
    profile = [value for value in argv if value.startswith("motion_profile:=")]
    if (motion != ["enable_motion:=true"]
            or automatic != ["auto_start_task:=false"]
            or expected_profile not in ("short-goal", "historical", "patrol")
            or profile != [f"motion_profile:={expected_profile}"]):
        raise ContractError("launch argv violates the explicit one-goal motion contract")
    return {"enable_motion": True, "auto_start_task": False,
            "motion_profile": (expected_profile if expected_profile in ("historical", "patrol") else "rviz_short_goal_015"),
            "one_goal_only": expected_profile != "patrol", "single_lap": expected_profile == "patrol"}


def idle_command_baseline_ok(command_samples: dict, zero_command_counts: dict) -> tuple[bool, list[str]]:
    """Accept a stopped chain before any Nav2 goal exists.

    /cmd_vel is published continuously by Safety and must be observed at zero.
    controller_server and collision_monitor create their Twist publishers when
    they activate, then publish only after a velocity command exists. Silence
    on those two topics is the idle baseline. A sample that is not all zeros
    still fails.
    """
    failures = []
    final = "/cmd_vel"
    samples = command_samples.get(final, 0)
    zeros = zero_command_counts.get(final, 0)
    if samples <= 0:
        failures.append(final + ":no_zero_baseline_samples")
    elif zeros != samples:
        failures.append(final + ":nonzero_or_unclassified_baseline_command")
    for topic in ("/cleannav/navigation/cmd_vel_raw", "/cleannav/cmd_vel_candidate"):
        samples = command_samples.get(topic, 0)
        zeros = zero_command_counts.get(topic, 0)
        if samples < 0 or zeros < 0 or zeros > samples or (samples > 0 and zeros != samples):
            failures.append(topic + ":nonzero_or_unclassified_baseline_command")
    return not failures, failures


def motion_admission_decision(*, phases: dict[str, str], guard: dict,
                              final_publishers: list[dict], raw_publishers: list[dict],
                              candidate_publishers: list[dict], command_samples: dict,
                              zero_command_counts: dict, goal_seen: bool,
                              ground_truth_samples: int, stable_s: float,
                              now_monotonic: float) -> dict:
    """One-shot, read-only admission check before issuing a bounded GUI scope."""
    failures = []
    if any(phases.get(name) != "PASS" for name in PHASES):
        failures.append("required_startup_phase_not_passed")
    try:
        validate_motion_guard_state(guard, now_monotonic=now_monotonic)
    except ContractError as error:
        failures.append("motion_guard:" + str(error))
    for topic, records, expected in (
        ("/cmd_vel", final_publishers, "cleannav_safety_supervisor"),
        ("/cleannav/navigation/cmd_vel_raw", raw_publishers, "controller_server"),
        ("/cleannav/cmd_vel_candidate", candidate_publishers, "collision_monitor"),
    ):
        if len(records) != 1:
            failures.append(topic + ":publisher_count_not_one")
        elif (records[0].get("node_name") != expected
              or records[0].get("node_namespace") != "/"
              or records[0].get("topic_type") != "geometry_msgs/msg/Twist"
              or records[0].get("endpoint_gid") in (None, "", [], {})):
            failures.append(topic + ":publisher_identity_mismatch")
    failures.extend(idle_command_baseline_ok(command_samples, zero_command_counts)[1])
    if goal_seen:
        failures.append("goal_observed_before_admission")
    if ground_truth_samples < 2:
        failures.append("gazebo_ground_truth_requires_two_independent_samples")
    if (not isinstance(stable_s, (int, float)) or isinstance(stable_s, bool)
            or not math.isfinite(stable_s) or stable_s < MOTION_ADMISSION_STABLE_S):
        failures.append("readiness_stability_window_incomplete")
    return {
        "allowed": not failures, "stage": "MOTION_ADMISSION",
        "failures": failures, "stable_s": stable_s,
        "required_stable_s": MOTION_ADMISSION_STABLE_S,
        "phases": dict(phases), "ground_truth_samples": ground_truth_samples,
        "goal_seen": goal_seen,
    }


def motion_completion_decision(*, gui_goal_seen: bool, mission_accepted: bool,
                               task_succeeded: bool, lease_acquired: bool,
                               lease_released: bool, lifecycle_closed: bool,
                               command_counts: dict, nonzero_counts: dict,
                               final_zero_after_terminal: bool,
                               gazebo_displacement_m: float | None,
                               max_displacement_m: float = MAX_MOTION_SESSION_DISPLACEMENT_M) -> dict:
    """Require GUI-to-Gazebo completion and an observed return to zero."""
    failures = []
    for key, value in (("gui_goal_seen", gui_goal_seen),
                       ("mission_accepted", mission_accepted),
                       ("task_succeeded", task_succeeded),
                       ("lease_acquired", lease_acquired),
                       ("lease_released", lease_released),
                       ("lifecycle_closed", lifecycle_closed),
                       ("final_zero_after_terminal", final_zero_after_terminal)):
        if value is not True:
            failures.append(key)
    for topic in ("/cleannav/navigation/cmd_vel_raw", "/cleannav/cmd_vel_candidate", "/cmd_vel"):
        if command_counts.get(topic, 0) <= 0:
            failures.append(topic + ":no_samples")
        if nonzero_counts.get(topic, 0) <= 0:
            failures.append(topic + ":no_nonzero_motion_evidence")
    if (isinstance(gazebo_displacement_m, bool)
            or not isinstance(gazebo_displacement_m, (int, float))
            or not math.isfinite(gazebo_displacement_m)):
        failures.append("gazebo_ground_truth_displacement_unknown")
    elif gazebo_displacement_m < 0.10:
        failures.append("gazebo_vehicle_movement_below_0.10m")
    elif gazebo_displacement_m > max_displacement_m:
        failures.append("gazebo_vehicle_displacement_exceeded_bound")
    return {"passed": not failures, "failures": failures,
            "gazebo_displacement_m": gazebo_displacement_m,
            "maximum_displacement_m": max_displacement_m}


def expected_prefix_environment(dependency_manifest: dict) -> dict[str, str]:
    return {
        binding["prefix_env"]: package
        for package, binding in dependency_manifest["cleannav_packages"].items()
    }


def validate_environment(environment: dict[str, str | None], dependency_manifest: dict) -> dict:
    """Validate only the public ROS identity variables needed by this entry."""
    expected = {
        "ROS_DISTRO": "humble",
        "ROS_DOMAIN_ID": "204",
        "ROS_LOCALHOST_ONLY": "1",
    }
    actual = {key: environment.get(key) for key in expected}
    errors = [f"{key} expected {value!r}, got {actual[key]!r}"
              for key, value in expected.items() if actual[key] != value]
    prefixes = expected_prefix_environment(dependency_manifest)
    prefix_values = {}
    for variable, package in prefixes.items():
        value = environment.get(variable)
        prefix_values[variable] = value
        if not value:
            errors.append(f"{variable} required for {package}")
        elif not value.startswith("/"):
            errors.append(f"{variable} must be an absolute installation prefix")
    if errors:
        raise ContractError("environment identity rejected: " + "; ".join(errors))
    return {**actual, **prefix_values}


def validate_guard_state(state: dict, *, now_monotonic: float,
                         maximum_age_s: float = 0.5) -> None:
    """Require the integrated read-only guard's fresh no-motion admission."""
    if state.get("failed"):
        raise ContractError("safety guard failed: " + str(state["failed"]))
    stamp = state.get("monotonic")
    if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise ContractError("safety guard monotonic timestamp is missing or invalid")
    age = now_monotonic - stamp
    if age < 0 or age > maximum_age_s:
        raise ContractError(f"safety guard receipt is stale or future dated: age={age:.3f}s")
    facts = state.get("facts")
    if not isinstance(facts, dict):
        raise ContractError("safety guard facts are missing")
    required = (
        "/scan", "/mapping/scan", "/scan_tf", "/mapping/scan_tf", "clock", "odom",
        "safety", "cmd", "cm", "safety_no_estop", "cm_active", "cmd_publisher",
        "/cleannav/navigation/cmd_vel_raw_publisher",
        "/cleannav/cmd_vel_candidate_publisher", "localization", "no_motion_authority",
    )
    missing = [name for name in required if facts.get(name) is not True]
    if missing:
        raise ContractError("safety guard required facts are not true: " + ", ".join(missing))
    if state.get("ready") is not True or state.get("missing"):
        raise ContractError("safety guard did not issue a ready receipt")
    if not math.isfinite(float(state.get("distance_m", math.nan))) or \
            float(state["distance_m"]) > MAX_NO_MOTION_DISPLACEMENT_M:
        raise ContractError("no-motion displacement limit exceeded or unavailable")
    safety = state.get("safety")
    if not isinstance(safety, dict):
        raise ContractError("Safety status is missing")
    if safety.get("block_all") is not True or safety.get("estop") is not False \
            or safety.get("enabled") is not False or safety.get("owner") not in ("", None):
        raise ContractError("Safety is not in the required blocked, unleased state")


def validate_motion_guard_state(state: dict, *, now_monotonic: float,
                                maximum_age_s: float = 0.5) -> None:
    """Require fresh pre-lease readiness; this function creates no authority."""
    if not isinstance(state, dict):
        raise ContractError("motion guard receipt is unavailable or malformed")
    if state.get("failed"):
        raise ContractError("motion safety guard failed: " + str(state["failed"]))
    stamp = state.get("monotonic")
    if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise ContractError("motion guard monotonic timestamp is missing or invalid")
    age = now_monotonic - stamp
    if age < 0 or age > maximum_age_s:
        raise ContractError(f"motion safety guard receipt is stale or future dated: age={age:.3f}s")
    facts = state.get("facts")
    if not isinstance(facts, dict):
        raise ContractError("motion guard facts are missing")
    required = (
        "/scan", "/mapping/scan", "/scan_tf", "/mapping/scan_tf", "clock", "odom",
        "safety", "cmd", "cm", "safety_no_estop", "cm_active", "cmd_publisher",
        "/cleannav/navigation/cmd_vel_raw_publisher",
        "/cleannav/cmd_vel_candidate_publisher", "/cmd_vel_authorized_publisher",
        "localization",
    )
    missing = [name for name in required if facts.get(name) is not True]
    if missing or state.get("ready") is not True or state.get("missing"):
        raise ContractError("motion guard is not READY: " + ", ".join(missing or state.get("missing", [])))
    safety = state.get("safety")
    # Admission is checked before enabling the simulation interlock. Safety
    # must still be blocked, disabled, ESTOP-clear, and unleased here.
    if (not isinstance(safety, dict) or safety.get("block_all") is not True
            or safety.get("estop") is not False or safety.get("enabled") is not False
            or safety.get("owner") not in ("", None)):
        raise ContractError("pre-lease Safety must be unblocked, ESTOP-clear, disabled and unleased")
    distance = state.get("distance_m")
    if (not isinstance(distance, (int, float)) or isinstance(distance, bool)
            or not math.isfinite(distance) or distance > MAX_NO_MOTION_DISPLACEMENT_M):
        raise ContractError("pre-lease displacement exceeds the no-motion admission bound")


def gazebo_entity_state_from_response(response):
    """Return the Humble GetEntityState state without assuming a status_message field."""
    if getattr(response, "success", None) is not True:
        raise ContractError("GetEntityState returned success=false or missing success")
    state = getattr(response, "state", None)
    if state is None:
        raise ContractError("GetEntityState success response has no state")
    return state


def validate_motion_guard_runtime_state(state: dict, *, now_monotonic: float,
                                        motion_admitted: bool,
                                        mission_lease_active: bool,
                                        maximum_age_s: float = 0.5,
                                        observer_safety=..., minimum_safety_stamp_ns=None,
                                        maximum_distance_m=MAX_MOTION_SESSION_DISPLACEMENT_M,
                                        expected_owner="ros-execution-1") -> None:
    """Validate live health after admission without freezing motion telemetry."""
    if state.get("failed"):
        raise ContractError("motion safety guard failed: " + str(state["failed"]))
    stamp = state.get("monotonic")
    if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
            or not math.isfinite(stamp)):
        raise ContractError("motion guard monotonic timestamp is missing or invalid")
    age = now_monotonic - stamp
    if age < 0 or age > maximum_age_s:
        raise ContractError(f"motion guard receipt is stale or future dated: age={age:.3f}s")
    facts = state.get("facts")
    if not isinstance(facts, dict):
        raise ContractError("motion guard facts are missing")
    required = (
        "/scan", "/mapping/scan", "/scan_tf", "/mapping/scan_tf", "clock", "odom",
        "safety", "cmd", "cm", "safety_no_estop", "cm_active", "cmd_publisher",
        "/cleannav/navigation/cmd_vel_raw_publisher",
        "/cleannav/cmd_vel_candidate_publisher", "/cmd_vel_authorized_publisher",
        "localization",
    )
    missing = [name for name in required if facts.get(name) is not True]
    if missing or state.get("ready") is not True or state.get("missing"):
        raise ContractError("motion guard lost required runtime health: "
                            + ", ".join(missing or state.get("missing", [])))
    safety = state.get("safety")
    if not isinstance(safety, dict) or safety.get("estop") is not False:
        raise ContractError("Safety status is missing or ESTOP is active")
    if observer_safety is not Ellipsis:
        for label, snapshot in [('guard', safety), ('observer', observer_safety)]:
            if not isinstance(snapshot, dict):
                raise ContractError(label+' Safety state is unknown')
            received = snapshot.get('monotonic_ns')
            if (not isinstance(received, int) or isinstance(received, bool)
                    or not 0 <= now_monotonic*1e9-received <= 1_000_000_000):
                raise ContractError(label+' Safety status is stale or future dated')
            if (snapshot.get('owner') and minimum_safety_stamp_ns is not None
                    and (not isinstance(snapshot.get('stamp_ns'), int)
                         or snapshot['stamp_ns'] < minimum_safety_stamp_ns)):
                raise ContractError(label+' Safety state predates current goal/Lease')
            if snapshot.get('estop') is not False:
                raise ContractError(label+' Safety ESTOP is active or unknown')
            if snapshot.get('owner') not in ('', expected_owner):
                raise ContractError(label+' Safety Lease owner mismatch')
            if snapshot.get('block_all') is not False or snapshot.get('enabled') is not bool(snapshot.get('owner')):
                raise ContractError(label+' Safety Lease/enabled state is invalid')
        keys = ('owner', 'enabled', 'block_all', 'estop')
        if any(safety[k] != observer_safety[k] for k in keys):
            raise ContractError('Safety observer snapshot pending')
    if not motion_admitted:
        if (safety.get("block_all") is not True or safety.get("enabled") is not False
                or safety.get("owner") not in ("", None)):
            raise ContractError("pre-admission Safety must remain blocked and unleased")
    elif mission_lease_active:
        if (safety.get("block_all") is not False or safety.get("enabled") is not True
                or safety.get("owner") != expected_owner):
            raise ContractError("active task does not own the expected Safety Lease")
    elif (safety.get("block_all") is not False or safety.get("enabled") is not False
          or safety.get("owner") not in ("", None)):
        raise ContractError("admitted idle Safety is not unblocked and unleased")
    distance = state.get("distance_m")
    if (isinstance(distance, bool) or not isinstance(distance, (int, float))
            or not math.isfinite(distance) or distance < 0
            or distance > maximum_distance_m):
        raise ContractError(f"motion-session accumulated odometry is unavailable or exceeds {maximum_distance_m:g} m")


def guard_receipt_snapshot_status(previous: dict | None, latest: dict | None, *,
                                   now_monotonic: float,
                                   maximum_age_s: float = 0.5) -> dict:
    """Compare the supervisor's cached guard read with the latest disk receipt.

    The freshness threshold remains the health contract. This only detects
    whether a stale local snapshot was superseded by a fresh receipt while the
    supervisor performed graph queries; it never changes readiness itself.
    """
    def age(state):
        stamp = state.get("monotonic") if isinstance(state, dict) else None
        if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                or not math.isfinite(stamp)):
            return None
        return now_monotonic - stamp

    previous_age = age(previous)
    latest_age = age(latest)
    previous_fresh = (previous_age is not None
                      and 0 <= previous_age <= maximum_age_s)
    latest_fresh = (latest_age is not None
                    and 0 <= latest_age <= maximum_age_s)
    recovered = bool(
        not previous_fresh and latest_fresh and isinstance(previous, dict)
        and isinstance(latest, dict)
        and isinstance(previous.get("monotonic"), (int, float))
        and isinstance(latest.get("monotonic"), (int, float))
        and latest["monotonic"] > previous["monotonic"])
    previous_status = ("PASS" if previous_fresh else
                       "FAIL" if isinstance(previous, dict) else "UNKNOWN")
    latest_status = ("PASS" if latest_fresh else
                     "FAIL" if isinstance(latest, dict) else "UNKNOWN")
    return {
        "maximum_age_s": maximum_age_s,
        "previous_age_s": previous_age,
        "previous_status": previous_status,
        "latest_age_s": latest_age,
        "latest_status": latest_status,
        "latest_receipt_available": isinstance(latest, dict),
        "recovered_from_newer_receipt": recovered,
        "usable_latest_receipt": latest_fresh,
    }


def _exact_int(value, *, positive=False) -> bool:
    return (isinstance(value, int) and not isinstance(value, bool)
            and (value > 0 if positive else value >= 0))


SAFETY_DECISION_FIELDS = ("block_all", "estop", "enabled", "owner")


def observe_safety_sample(previous: dict | None, sample: dict) -> dict:
    """Record one Safety sample and the stamp where its decision fields began.

    A heartbeat that repeats block_all, estop, enabled, and owner keeps the
    generation already in force. A repeated or backward header stamp is
    rejected. This does not write a receipt.
    """
    if not isinstance(sample, dict):
        raise ContractError("Safety status is missing or ESTOP is active")
    stamp = sample.get("stamp_ns")
    if not _exact_int(stamp):
        raise ContractError("Safety status stamp repeated or moved backward")
    state = {key: sample.get(key) for key in (*SAFETY_DECISION_FIELDS, "monotonic_ns", "stamp_ns")}
    if not isinstance(previous, dict):
        state["generation_stamp_ns"] = stamp
        return state
    previous_stamp = previous.get("stamp_ns")
    if not _exact_int(previous_stamp) or stamp <= previous_stamp:
        raise ContractError("Safety status stamp repeated or moved backward")
    same = all(previous.get(key) == state[key] for key in SAFETY_DECISION_FIELDS)
    generation = previous.get("generation_stamp_ns") if same else stamp
    state["generation_stamp_ns"] = generation if _exact_int(generation) else stamp
    return state


def admitted_safety_receipt_decision(receipt: dict | None, confirmed: dict | None, *,
                                     now_monotonic: float, deadline_monotonic: float,
                                     expected_run_id: str,
                                     expected_writer_pid: int | None,
                                     motion_admitted: bool, mission_lease_active: bool,
                                     expected_owner: str = "ros-execution-1",
                                     maximum_age_s: float = 0.5,
                                     generation_stamp_ns: int | None = None,
                                     minimum_receipt_stamp_ns: int | None = None) -> dict:
    """Decide whether a guard receipt covers the current Safety decision.

    ``stamp_ns`` identifies one SafetyStatus sample. Ordinary heartbeats keep
    the same decision fields, so a fresh receipt is current once its stamp
    reaches the generation where those fields began. It does not have to copy
    the newest heartbeat. A receipt from before that generation is not
    accepted. ESTOP, block_all, owner, and Lease checks on the current sample
    still fail immediately. The 0.5 s freshness limit is unchanged.
    """
    def fail(reason):
        return {"action": "fail", "reason": reason}

    if not isinstance(confirmed, dict) or confirmed.get("estop") is not False:
        return fail("Safety status is missing or ESTOP is active")
    if confirmed.get("owner") not in ("", expected_owner):
        return fail("Safety Lease owner mismatch")
    if mission_lease_active:
        if (confirmed.get("block_all") is not False or confirmed.get("enabled") is not True
                or confirmed.get("owner") != expected_owner):
            return fail("active task does not own the expected Safety Lease")
    elif motion_admitted:
        if (confirmed.get("block_all") is not False or confirmed.get("enabled") is not False
                or confirmed.get("owner") not in ("", None)):
            return fail("admitted idle Safety is not unblocked and unleased")
    if not isinstance(receipt, dict):
        return fail("Safety guard receipt unavailable during confirmation")
    writer_pid = receipt.get("writer_pid")
    if (receipt.get("run_id") != expected_run_id or not _exact_int(writer_pid, positive=True)
            or (expected_writer_pid is not None and writer_pid != expected_writer_pid)):
        return fail("safety guard receipt process identity rejected")
    stamp = receipt.get("monotonic")
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        return fail("motion guard monotonic timestamp is missing or invalid")
    age = now_monotonic - stamp
    if age < 0 or age > maximum_age_s:
        return fail(f"motion guard receipt is stale or future dated: age={age:.3f}s")
    if now_monotonic >= deadline_monotonic:
        return fail("safety guard receipt was not confirmed before the phase deadline")
    safety = receipt.get("safety")
    receipt_stamp = safety.get("stamp_ns") if isinstance(safety, dict) else None
    confirmed_stamp = confirmed.get("stamp_ns")
    if not _exact_int(receipt_stamp) or not _exact_int(confirmed_stamp):
        return fail("Safety guard receipt has no comparable status stamp")
    if _exact_int(minimum_receipt_stamp_ns) and receipt_stamp < minimum_receipt_stamp_ns:
        return fail("Safety guard receipt stamp moved backward")
    generation = (generation_stamp_ns if generation_stamp_ns is not None
                  else confirmed.get("generation_stamp_ns", confirmed_stamp))
    if not _exact_int(generation):
        return fail("Safety guard receipt has no comparable status stamp")
    fields_match = (isinstance(safety, dict) and all(
        safety.get(key) == confirmed.get(key) for key in SAFETY_DECISION_FIELDS))
    if not (fields_match and receipt_stamp >= generation) and receipt_stamp < confirmed_stamp:
        return {"action": "wait",
                "reason": "Safety guard receipt has not processed the confirmed status",
                "writer_pid": writer_pid, "stamp_ns": receipt_stamp}
    try:
        validate_motion_guard_runtime_state(
            receipt, now_monotonic=now_monotonic, motion_admitted=motion_admitted,
            mission_lease_active=mission_lease_active, maximum_age_s=maximum_age_s,
            expected_owner=expected_owner)
    except ContractError as error:
        return fail(str(error))
    return {"action": "accept", "reason": None, "writer_pid": writer_pid,
            "stamp_ns": receipt_stamp}


def raw_publisher_phase_contract(endpoint_records: list[dict], *,
                                 navigation_probe_ready: bool) -> dict:
    """Treat the raw controller endpoint as absent until Nav2's active probe.

    Before the probe confirms planner/controller readiness, absence is an
    expected lifecycle state. Any endpoint that does appear must already have
    the exact controller identity. Once Navigation READY is reported, absence
    is a hard phase failure.
    """
    records = endpoint_records if isinstance(endpoint_records, list) else []
    if not records:
        state = "MISSING_REQUIRED" if navigation_probe_ready else "ABSENT_EXPECTED"
        return {"ok": not navigation_probe_ready, "state": state,
                "required_from": "navigation_probe_ready", "endpoints": []}
    if len(records) != 1:
        return {"ok": False, "state": "INVALID",
                "reason": "publisher_count_not_one", "endpoints": records}
    endpoint = records[0]
    gid = endpoint.get("endpoint_gid")
    known_gid = gid is not None and gid not in ("", [], {})
    valid = (
        endpoint.get("node_name") == "controller_server"
        and endpoint.get("node_namespace") == "/"
        and endpoint.get("topic_type") == "geometry_msgs/msg/Twist"
        and known_gid)
    return {"ok": valid, "state": "PRESENT_EXPECTED" if valid else "INVALID",
            "reason": None if valid else "publisher_identity_mismatch_or_unknown",
            "required_from": "navigation_probe_ready", "endpoints": records}


def validate_zero_command(values: list[float], *, tolerance: float = 1e-8) -> None:
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ContractError("invalid /cmd_vel message in no-motion mode")
    if any(abs(value) > tolerance for value in values):
        raise ContractError("nonzero /cmd_vel observed in supervisor no-motion mode")


def zero_command_observation_ok(message_count: int, nonzero_count: int) -> bool:
    return message_count > 0 and nonzero_count == 0


def format_navigation_ready(*, run_id: str, odometry_mode: str,
                            motion_profile: str, map_profile: str) -> str:
    """Operator notice after every existing readiness phase has passed."""
    return (
        "[CLEANNAV] NAVIGATION READY - YOU CAN SEND GOAL NOW\n"
        f"[CLEANNAV] run_id={run_id} odometry_mode={odometry_mode} "
        f"motion_profile={motion_profile} map_profile={map_profile}\n"
        "[CLEANNAV] Click 2D Goal Pose in RViz now.\n"
    )


def format_navigation_not_ready(*, stage: str, reason: str) -> str:
    """Operator notice when startup fails before the ready transition."""
    return (
        "[CLEANNAV] NOT READY\n"
        f"[CLEANNAV] stage={stage}\n"
        f"[CLEANNAV] reason={reason}\n"
    )


def phase_readiness(log_text: str, collision_receipt: dict | None,
                    guard_state: dict | None, *, rviz_node: bool,
                    rviz_process: bool, gazebo_process: bool,
                    mission_node: bool, localization_scan_topic: str = '/mapping/scan',
                    now_monotonic: float | None = None,
                    motion_mode: bool = False, motion_admitted: bool = False,
                    mission_lease_active: bool = False, expected_owner="ros-execution-1") -> dict[str, tuple[bool, dict]]:
    """Derive staged evidence from existing launch probes and integrated guards."""
    if localization_scan_topic not in ('/scan', '/mapping/scan'):
        raise ContractError('unsupported localization scan topic in readiness contract')
    log = log_text
    facts = guard_state.get("facts", {}) if isinstance(guard_state, dict) else {}
    receipt_age = (now_monotonic - guard_state.get('monotonic', -math.inf)
                   if now_monotonic is not None and isinstance(guard_state, dict) else None)
    receipt_fresh = receipt_age is None or (math.isfinite(receipt_age) and 0 <= receipt_age <= .5)
    if not receipt_fresh:
        facts = {}
    observations = (guard_state or {}).get('observations', {})
    cm_ok = isinstance(collision_receipt, dict) and collision_receipt.get("passed") is True \
        and str(collision_receipt.get("decision", "")).startswith("PASS")
    collision_id = (collision_receipt or {}).get("decision")
    sensors = all(facts.get(key) is True for key in
                  ("/scan", "/mapping/scan", "/scan_odom_tf", "/mapping/scan_odom_tf", "clock", "odom"))
    localization_map_tf = localization_scan_topic + '_map_tf'
    localization = ("[LOCALIZATION] READY" in log and facts.get("localization") is True
                    and facts.get(localization_scan_topic) is True
                    and facts.get(localization_map_tf) is True)
    safety_keys = ["safety", "safety_no_estop", "cm_active", "cmd_publisher",
                   "/cleannav/cmd_vel_candidate_publisher"]
    if motion_mode:
        safety_keys.append("/cmd_vel_authorized_publisher")
    if not motion_mode:
        safety_keys.append("no_motion_authority")
    safety = all(facts.get(key) is True for key in safety_keys)
    safety_state = (guard_state or {}).get("safety", {})
    if motion_mode:
        # Before admission the simulation interlock must remain blocked. Once
        # admitted it is open, and only the actual Mission Lease may enable
        # Safety output. This keeps one readiness contract across the whole
        # supervised lifecycle without mistaking a valid Lease for a failure.
        if mission_lease_active:
            safety_state_ok = (isinstance(safety_state, dict)
                and safety_state.get("block_all") is False
                and safety_state.get("estop") is False
                and safety_state.get("enabled") is True
                and safety_state.get("owner") == expected_owner)
        elif motion_admitted:
            safety_state_ok = (isinstance(safety_state, dict)
                and safety_state.get("block_all") is False
                and safety_state.get("estop") is False
                and safety_state.get("enabled") is False
                and safety_state.get("owner") in ("", None))
        else:
            safety_state_ok = (isinstance(safety_state, dict)
                and safety_state.get("block_all") is True
                and safety_state.get("estop") is False
                and safety_state.get("enabled") is False
                and safety_state.get("owner") in ("", None))
        safety = safety and safety_state_ok
    navigation_probe_ready = "[NAVIGATION] READY" in log
    raw_publisher_ready = facts.get("/cleannav/navigation/cmd_vel_raw_publisher") is True
    nav2 = (navigation_probe_ready and raw_publisher_ready
            and isinstance(guard_state, dict) and guard_state.get("ready") is True)
    mapping_observation = observations.get('/mapping/scan')
    mapping_message = (mapping_observation.get('last_message')
                       if isinstance(mapping_observation, dict) else None)
    mapping_return_status = (mapping_message.get('return_status', 'UNKNOWN')
                             if isinstance(mapping_message, dict) else 'UNKNOWN')
    readiness = {
        "gazebo_vehicle": ("[SIMULATION] READY" in log and gazebo_process,
                           {"probe_marker": "[SIMULATION] READY" in log,
                            "owned_gazebo_process": gazebo_process}),
        "collision_monitor": (cm_ok and facts.get("cm") is True and facts.get("cm_active") is True,
                              {"receipt_decision": collision_id, "runtime_cm_fresh": facts.get("cm"),
                               "runtime_cm_active": facts.get("cm_active")}),
        "dual_lidar_tf": (sensors, {"facts": {k: facts.get(k) for k in
                             ("/scan", "/mapping/scan", "/scan_odom_tf", "/mapping/scan_odom_tf", "clock", "odom")},
                             "observations": observations, "guard_receipt_age_s": receipt_age,
                             "guard_receipt_fresh": receipt_fresh,
                             "mapping_return_status": mapping_return_status}),
        "map_localization": (localization, {"localization_probe": "[LOCALIZATION] READY" in log,
                              "localization_fact": facts.get("localization"),
                              "localization_scan_topic": localization_scan_topic,
                              "localization_scan_map_tf": facts.get(localization_map_tf),
                              "health": (guard_state or {}).get('localization_health')}),
        "safety": (safety, {"facts": {k: facts.get(k) for k in (
                  "safety", "safety_no_estop", "cm_active", "cmd_publisher",
                  "/cleannav/cmd_vel_candidate_publisher", "no_motion_authority")},
                  "motion_mode": motion_mode,
                  "motion_admitted": motion_admitted,
                  "mission_lease_active": mission_lease_active,
                  "safety_state": safety_state if motion_mode else None}),
        "nav2": (nav2, {"probe_marker": "[NAVIGATION] READY" in log,
                          "guard_ready": bool(guard_state and guard_state.get("ready")),
                          "raw_publisher_ready": raw_publisher_ready,
                          "raw_publisher_required": navigation_probe_ready}),
        "mission_manager": ("[MISSION] READY" in log and mission_node,
                            {"probe_marker": "[MISSION] READY" in log,
                             "mission_manager_node": mission_node}),
        "rviz": (rviz_node and rviz_process,
                 {"ros_graph_node": rviz_node, "owned_process_group_member": rviz_process}),
    }
    return readiness


def readiness_loss_record(phase: str, readiness: dict[str, tuple[bool, dict]],
                          guard_state: dict | None,
                          prior_evidence: dict | None) -> dict:
    """Preserve the exact evidence that made an admitted phase go false."""
    ready, evidence = readiness.get(phase, (False, {}))
    facts = guard_state.get("facts") if isinstance(guard_state, dict) else None
    facts = facts if isinstance(facts, dict) else {}
    return {
        "phase": phase,
        "run_id": (guard_state or {}).get('run_id'),
        "guard_monotonic": (guard_state or {}).get('monotonic'),
        "ready": bool(ready),
        "prior_pass_evidence": prior_evidence,
        "current_evidence": evidence,
        "guard_available": isinstance(guard_state, dict),
        "guard_ready": guard_state.get("ready") if isinstance(guard_state, dict) else None,
        "guard_missing": guard_state.get("missing") if isinstance(guard_state, dict) else None,
        "guard_false_facts": sorted(key for key, value in facts.items() if value is not True),
        "input_observations": (guard_state or {}).get('observations'),
        "localization_health": (guard_state or {}).get('localization_health'),
        "diagnostic_hard_safety": (guard_state or {}).get('diagnostic_continuation'),
    }


def diagnostic_readiness_loss_decision(phase: str, guard_state: dict | None,
                                       nonzero_command_count: int, *,
                                       now_monotonic: float,
                                       maximum_guard_age_s: float = 0.5,
                                       no_motion_contract: bool = False) -> dict:
    """Unknown observations cannot grant READY, but need not stop idle diagnostics.

    This branch requires the real verified no-motion launch contract. Explicit
    guard failures remain sticky even when their receipt is old. The standard
    and future motion paths retain their existing fail-closed admission rules.
    """
    state = guard_state if isinstance(guard_state, dict) else {}
    gate = state.get('diagnostic_continuation') or {}
    stamp = state.get('monotonic')
    age = (now_monotonic - stamp if isinstance(stamp, (int, float))
           and math.isfinite(stamp) else math.inf)
    fresh = 0.0 <= age <= maximum_guard_age_s
    hard = list(gate.get('hard_failures') or [])
    if state.get('failed'):
        hard.append('guard_explicit_failure:' + str(state['failed']))
    if nonzero_command_count:
        hard.append('nonzero_cmd_vel_observed')
    allowed = bool(LOCALIZATION_DIAGNOSTIC and no_motion_contract is True
                   and phase in PHASES and not hard)
    return dict(allowed=allowed, phase=phase, mode=TIMING_PROFILE,
                no_motion_contract_verified=no_motion_contract,
                guard_diagnostic_mode=state.get('diagnostic_mode'),
                guard_receipt_sequence=state.get('receipt_sequence'),
                guard_receipt_age_s=age if math.isfinite(age) else None,
                guard_receipt_fresh=fresh,
                health_readiness='PASS' if fresh else 'FAIL',
                guard_safety_decision=gate, hard_failures=hard,
                nonzero_command_count=nonzero_command_count,
                reason=('CONTINUE_OBSERVING_WITHOUT_READINESS' if allowed
                        else 'explicit hard safety failure or non-diagnostic contract'))


@dataclass
class PhaseLedger:
    """Advance phases in order; a failed phase blocks every later phase."""
    states: dict[str, str] = field(default_factory=lambda: {name: "NOT RUN" for name in PHASES})
    evidence: dict[str, dict] = field(default_factory=dict)
    failed_at: str | None = None

    def advance(self, readiness: dict[str, tuple[bool, dict]], *, monotonic_ns: int) -> list[str]:
        changed = []
        if self.failed_at:
            return changed
        for index, name in enumerate(PHASES):
            if self.states[name] == "PASS":
                continue
            if any(self.states[prior] != "PASS" for prior in PHASES[:index]):
                break
            ready, detail = readiness.get(name, (False, {}))
            if not ready:
                break
            self.states[name] = "PASS"
            self.evidence[name] = {"monotonic_ns": monotonic_ns, **detail}
            changed.append(name)
        return changed

    def fail(self, phase: str, reason: str, *, monotonic_ns: int) -> None:
        if phase not in PHASES:
            raise ContractError("unknown readiness phase: " + phase)
        self.failed_at = phase
        self.states[phase] = "FAIL"
        self.evidence[phase] = {"monotonic_ns": monotonic_ns, "reason": reason}
        for later in PHASES[PHASES.index(phase) + 1:]:
            self.states[later] = "BLOCKED"

    def reopen_from(self, phase: str) -> dict:
        """Return an admitted phase to pending after a diagnostic health loss.

        The previous PASS evidence is returned to the caller for the loss
        receipt; it is removed from current evidence so final status cannot
        mistake an earlier admission for current readiness.
        """
        if phase not in PHASES:
            raise ContractError("unknown readiness phase: " + phase)
        index = PHASES.index(phase)
        previous = {name: self.evidence.get(name) for name in PHASES[index:]}
        for name in PHASES[index:]:
            self.states[name] = "NOT RUN"
            self.evidence.pop(name, None)
        return previous


def linux_cpu_counters(proc_root: Path = Path('/proc')) -> dict:
    """Read total CPU and this process CPU counters from a procfs-like root."""
    proc_root = Path(proc_root)
    cpu_line = next(line for line in (proc_root / 'stat').read_text().splitlines()
                    if line.startswith('cpu '))
    values = [int(value) for value in cpu_line.split()[1:]]
    if len(values) < 4:
        raise ValueError('/proc/stat aggregate CPU line is incomplete')
    total = sum(values)
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    stat = (proc_root / 'self/stat').read_text()
    close = stat.rfind(')')
    if close < 0:
        raise ValueError('/proc/self/stat process name is malformed')
    fields = stat[close + 1:].split()
    if len(fields) < 13:
        raise ValueError('/proc/self/stat CPU fields are incomplete')
    process_ticks = int(fields[11]) + int(fields[12])
    return {'system_total_ticks': total, 'system_idle_ticks': idle,
            'process_ticks': process_ticks}


def cpu_usage_percent(previous: dict | None, current: dict, elapsed_s: float,
                      clock_ticks_per_second: int) -> dict:
    """Calculate system busy and current-process CPU from monotonic samples."""
    if (previous is None or elapsed_s <= 0 or clock_ticks_per_second <= 0):
        return {'system_busy_pct': None, 'process_cpu_pct': None}
    total = current['system_total_ticks'] - previous['system_total_ticks']
    idle = current['system_idle_ticks'] - previous['system_idle_ticks']
    process = current['process_ticks'] - previous['process_ticks']
    if total <= 0 or idle < 0 or process < 0:
        return {'system_busy_pct': None, 'process_cpu_pct': None}
    return {
        'system_busy_pct': max(0.0, min(100.0, 100.0 * (total - idle) / total)),
        'process_cpu_pct': 100.0 * process / clock_ticks_per_second / elapsed_s,
    }


def preflight_allows_launch(preflight: dict) -> bool:
    return isinstance(preflight, dict) and preflight.get("ok") is True


def deadline_stop_reason(elapsed_s: float, *, all_phases_pass: bool,
                         observation_elapsed_s: float | None,
                         launch_elapsed_s: float | None = None,
                         startup_verification_complete: bool = False) -> str | None:
    if elapsed_s >= SOFT_STOP_FROM_T0_S:
        return f"soft_stop_deadline_T0_plus_{SOFT_STOP_FROM_T0_S:g}s"
    readiness_expired = elapsed_s >= READINESS_DEADLINE_FROM_T0_S
    if LOCALIZATION_DIAGNOSTIC and launch_elapsed_s is not None:
        readiness_expired = launch_elapsed_s >= MIN_LAUNCH_OBSERVATION_S
    if LOCALIZATION_DIAGNOSTIC and launch_elapsed_s is not None \
            and launch_elapsed_s >= MIN_LAUNCH_OBSERVATION_S:
        return ("startup_soak_complete" if startup_verification_complete
                else "STARTUP_TIMEOUT")
    if readiness_expired and not all_phases_pass and not (HISTORICAL_MOTION and startup_verification_complete):
        return ('STARTUP_TIMEOUT' if LOCALIZATION_DIAGNOSTIC
                else f"readiness_deadline_T0_plus_{READINESS_DEADLINE_FROM_T0_S:g}s")
    if all_phases_pass and observation_elapsed_s is not None \
            and observation_elapsed_s >= NO_MOTION_OBSERVATION_S \
            and not LOCALIZATION_DIAGNOSTIC:
        return "no_motion_observation_complete"
    return None


def no_motion_startup_qualifies(*, all_phases_pass: bool,
                                observation_elapsed_s: float | None,
                                enable_motion: bool, auto_start_task: bool,
                                command_samples: int, nonzero_samples: int,
                                displacement_m: float | None) -> bool:
    """Level A startup contract, independent of the later 180 s soak."""
    return bool(
        all_phases_pass is True
        and observation_elapsed_s is not None
        and math.isfinite(observation_elapsed_s)
        and observation_elapsed_s >= NO_MOTION_OBSERVATION_S
        and enable_motion is False and auto_start_task is False
        and command_samples > 0 and nonzero_samples == 0
        and displacement_m is not None and math.isfinite(displacement_m)
        and displacement_m <= MAX_NO_MOTION_DISPLACEMENT_M
    )


@dataclass
class DeadlineWatchdogState:
    """Idempotent schedule for an out-of-process stop watchdog.

    The watchdog asks the Python supervisor to stop at T0+145, independently
    escalates the owned launch group at T0+159/T0+167, and enforces the process
    cutoff at T0+175. The remaining five seconds are reserved for final scans
    and a fail-closed receipt before the T0+180 evidence cutoff.
    """
    soft_stop_sent: bool = False
    readiness_stop_sent: bool = False
    sigterm_sent: bool = False
    sigkill_sent: bool = False
    cutoff_triggered: bool = False

    def actions(self, elapsed_s: float, *, supervisor_alive: bool,
                process_group_status: str, completion_recorded: bool,
                readiness_complete: bool = False,
                launch_elapsed_s: float | None = None) -> list[dict]:
        if completion_recorded:
            return []
        if not supervisor_alive:
            self.cutoff_triggered = True
            return [{"kind": "abnormal_supervisor_exit"}]
        actions: list[dict] = []
        readiness_expired = elapsed_s >= READINESS_DEADLINE_FROM_T0_S
        if LOCALIZATION_DIAGNOSTIC and launch_elapsed_s is not None:
            readiness_expired = launch_elapsed_s >= MIN_LAUNCH_OBSERVATION_S
        if (readiness_expired and not readiness_complete
                and not self.readiness_stop_sent):
            self.readiness_stop_sent = True
            self.soft_stop_sent = True
            actions.append({"kind": "signal_supervisor", "signal": "SIGINT",
                            "reason": ('STARTUP_TIMEOUT'
                                       if LOCALIZATION_DIAGNOSTIC else
                                       f'readiness_deadline_T0_plus_{READINESS_DEADLINE_FROM_T0_S:g}s'),
                            "deadline_t0_s": READINESS_DEADLINE_FROM_T0_S})
        if elapsed_s >= SOFT_STOP_FROM_T0_S and not self.soft_stop_sent:
            self.soft_stop_sent = True
            actions.append({"kind": "signal_supervisor", "signal": "SIGINT",
                            "reason": f"soft_stop_T0_plus_{SOFT_STOP_FROM_T0_S:g}s",
                            "deadline_t0_s": SOFT_STOP_FROM_T0_S})
        group_not_empty = process_group_status not in ("EMPTY", "NOT_STARTED")
        if elapsed_s >= WATCHDOG_SIGTERM_FROM_T0_S and not self.sigterm_sent and group_not_empty:
            self.sigterm_sent = True
            actions.append({"kind": "signal_process_group", "signal": "SIGTERM",
                            "deadline_t0_s": WATCHDOG_SIGTERM_FROM_T0_S})
        if elapsed_s >= WATCHDOG_SIGKILL_FROM_T0_S and not self.sigkill_sent and group_not_empty:
            self.sigkill_sent = True
            actions.append({"kind": "signal_process_group", "signal": "SIGKILL",
                            "deadline_t0_s": WATCHDOG_SIGKILL_FROM_T0_S})
        if elapsed_s >= HARD_PROCESS_CUTOFF_FROM_T0_S:
            self.cutoff_triggered = True
            actions.append({"kind": "hard_process_cutoff",
                            "deadline_t0_s": HARD_PROCESS_CUTOFF_FROM_T0_S,
                            "process_group_not_empty": group_not_empty})
        return actions


def cleanup_is_verified(cleanup: dict | None) -> bool:
    if not isinstance(cleanup, dict) or cleanup.get("cleanup_ok") is not True:
        return False
    snapshot = cleanup.get("process_group_snapshot_after_cleanup", {})
    return (
        cleanup.get("launch_main_reaped") is True
        and cleanup.get("process_group_alive_after_cleanup") is False
        and cleanup.get("process_group_state_after_cleanup") == "EMPTY"
        and snapshot.get("scan_complete") is True
        and snapshot.get("status") == "EMPTY"
        and not snapshot.get("running_members")
        and not snapshot.get("zombie_members")
    )


def final_status(*, failures: list[str], phases: dict[str, str], cleanup: dict | None,
                 launch_started: bool, elapsed_s: float, cgroup_disposition: str | None) -> str:
    if failures:
        return "FAIL" if launch_started else "BLOCKED"
    if not launch_started:
        return "BLOCKED"
    if elapsed_s > ACCOUNTING_LIMIT_FROM_T0_S:
        return "FAIL"
    if any(phases.get(name) != "PASS" for name in PHASES) or not cleanup_is_verified(cleanup):
        return "FAIL"
    if cgroup_disposition == "WAIVED":
        return "PASS_WITH_EXCEPTION"
    return "PASS"
