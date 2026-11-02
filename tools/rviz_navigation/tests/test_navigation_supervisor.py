"""Behavioral tests for the no-motion supervisor contracts; no ROS nodes launch."""
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import shlex
import subprocess
import tempfile
import time
import uuid

import pytest

HERE = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / "tools/startup_smoke"))

import navigation_supervisor as supervisor
import navigation_safety_guard as safety_guard
import supervisor_policy as policy
from readiness_inputs import InputEvidence
from guard_policy import diagnostic_continuation_decision
import deadline_watchdog as watchdog_module
import roof_lidar_runtime as runtime


def environment(**overrides):
    value = {"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "204", "ROS_LOCALHOST_ONLY": "1"}
    manifest = {"cleannav_packages": {"cleannav_simulation": {"prefix_env": "CLEANNAV_EXPECTED_SIMULATION_PREFIX"},
                                      "cleannav_navigation": {"prefix_env": "CLEANNAV_EXPECTED_NAVIGATION_PREFIX"}}}
    value.update({"CLEANNAV_EXPECTED_SIMULATION_PREFIX": "/opt/cleannav/simulation",
                  "CLEANNAV_EXPECTED_NAVIGATION_PREFIX": "/opt/cleannav/navigation"})
    value.update(overrides)
    return value, manifest


@pytest.mark.parametrize("key,value", [
    ("ROS_DISTRO", "iron"), ("ROS_DISTRO", None),
    ("ROS_DOMAIN_ID", "205"), ("ROS_LOCALHOST_ONLY", "0"),
    ("CLEANNAV_EXPECTED_SIMULATION_PREFIX", None),
    ("CLEANNAV_EXPECTED_NAVIGATION_PREFIX", "relative/prefix"),
])
def test_environment_identity_rejects_wrong_or_missing_values(key, value):
    env, manifest = environment(**{key: value})
    with pytest.raises(policy.ContractError):
        policy.validate_environment(env, manifest)


def test_environment_identity_accepts_humble_domain_and_all_expected_prefixes():
    env, manifest = environment()
    result = policy.validate_environment(env, manifest)
    assert result["ROS_DISTRO"] == "humble"
    assert result["ROS_DOMAIN_ID"] == "204"
    assert result["CLEANNAV_EXPECTED_SIMULATION_PREFIX"] == "/opt/cleannav/simulation"


def test_scan_range_diagnostics_separate_real_returns_from_no_return_values():
    ranges = [0.5, 10.0, 0.05, 11.0, float("nan"), float("inf"), -float("inf")]
    assert safety_guard.scan_range_summary(ranges, 0.12, 10.0) == {
        "beam_count": 7,
        "finite_in_range": 2,
        "finite_below_min": 1,
        "finite_above_max": 1,
        "nan": 1,
        "positive_infinity": 1,
        "negative_infinity": 1,
    }


def test_no_return_roof_scan_is_valid_only_when_not_the_selected_localization_input():
    summary = safety_guard.scan_range_summary([float("inf")] * 360, 0.12, 10.0)
    assert summary["positive_infinity"] == 360
    assert summary["finite_in_range"] == 0
    roof = safety_guard.scan_admission('/mapping/scan', 'mapping_lidar_link',
        [float('inf')] * 360, -3.14159, 3.12414, 0.0174533, .12, 10.,
        localization_scan_topic='/scan')
    assert roof == {
        'valid': True, 'reason': None, 'finite_ranges': 0,
        'finite_return_required': False, 'return_status': 'NO_RETURN_WITHIN_RANGE',
        'range_values_valid': True,
    }
    low = safety_guard.scan_admission('/scan', 'base_scan',
        [float('inf')] * 181, -1.57, 1.57, .01745, .12, 10.,
        localization_scan_topic='/scan')
    assert not low['valid']
    assert low['reason'] == 'no_finite_return_on_localization_or_safety_scan'
    selected_roof = safety_guard.scan_admission('/mapping/scan', 'mapping_lidar_link',
        [float('inf')] * 360, -3.14159, 3.12414, 0.0174533, .12, 10.,
        localization_scan_topic='/mapping/scan')
    assert not selected_roof['valid']
    assert selected_roof['finite_return_required'] is True


@pytest.mark.parametrize('bad_value', [float('nan'), -float('inf'), 11.0])
def test_non_localization_roof_scan_still_rejects_malformed_range_values(bad_value):
    ranges = [float('inf')] * 360
    ranges[7] = bad_value
    result = safety_guard.scan_admission('/mapping/scan', 'mapping_lidar_link',
        ranges, -3.14159, 3.12414, .0174533, .12, 10.,
        localization_scan_topic='/scan')
    assert not result['valid']
    assert result['reason'] == 'invalid_range_value'


def low_scan(ranges, *, frame='base_scan', angle_increment=0.017453333362936974,
             range_min=0.12, range_max=10.0):
    return safety_guard.scan_admission(
        '/scan', frame, ranges, -1.5707999467849731, 1.5707999467849731,
        angle_increment, range_min, range_max,
        localization_scan_topic='/mapping/scan')


def test_low_scan_all_positive_infinity_is_a_valid_no_hit():
    result = low_scan([float('inf')] * 181)
    assert result == {
        'valid': True, 'reason': None, 'finite_ranges': 0,
        'finite_return_required': False, 'return_status': 'NO_RETURN_WITHIN_RANGE',
        'range_values_valid': True,
    }


def test_low_scan_accepts_finite_values_mixed_with_positive_infinity():
    ranges = [float('inf')] * 181
    ranges[0] = 1.5
    ranges[90] = 9.9
    result = low_scan(ranges)
    assert result['valid'] is True
    assert result['reason'] is None
    assert result['finite_ranges'] == 2
    assert result['return_status'] == 'FINITE_RETURN'
    assert result['range_values_valid'] is True


def test_low_scan_accepts_normal_finite_returns():
    result = low_scan([2.0] * 181)
    assert result['valid'] is True
    assert result['finite_ranges'] == 181
    assert result['return_status'] == 'FINITE_RETURN'


@pytest.mark.parametrize('bad_value', [float('nan'), -float('inf')])
def test_low_scan_rejects_nan_and_negative_infinity(bad_value):
    ranges = [float('inf')] * 181
    ranges[3] = bad_value
    result = low_scan(ranges)
    assert result['valid'] is False
    assert result['reason'] == 'invalid_range_value'
    assert result['range_values_valid'] is False


def test_low_scan_rejects_wrong_beam_count():
    result = low_scan([float('inf')] * 180)
    assert result['valid'] is False
    assert result['reason'] == 'invalid_scan_parameters_or_identity'


@pytest.mark.parametrize('override', [
    {'frame': 'laser'},
    {'angle_increment': 0.0},
    {'angle_increment': float('nan')},
    {'range_min': 10.0, 'range_max': 10.0},
    {'range_min': -0.1},
])
def test_low_scan_rejects_wrong_parameters(override):
    result = low_scan([float('inf')] * 181, **override)
    assert result['valid'] is False
    assert result['reason'] == 'invalid_scan_parameters_or_identity'


def test_mapping_scan_all_positive_infinity_stays_rejected():
    result = safety_guard.scan_admission('/mapping/scan', 'mapping_lidar_link',
        [float('inf')] * 360, -3.14159, 3.12414, 0.0174533, .12, 10.,
        localization_scan_topic='/mapping/scan')
    assert result['valid'] is False
    assert result['finite_return_required'] is True
    assert result['reason'] == 'no_finite_return_on_localization_or_safety_scan'
    assert result['range_values_valid'] is True


def test_stale_scan_and_missing_tf_stay_rejected():
    inputs = InputEvidence()
    inputs.observe('clock', 10_000_000_000, 1.)
    inputs.observe('odom', 10_000_000_000, 1., frame='odom')
    inputs.observe('/scan', 10_000_000_000, 1., frame='base_scan')
    inputs.observe('/mapping/scan', 10_000_000_000, 1., frame='mapping_lidar_link')
    facts, observations = inputs.snapshot(1.301, lambda *args: {'ok': True})
    assert facts['/scan'] is False
    assert observations['/scan']['reason'] == 'receive_age_exceeded'
    assert observations['/scan']['freshness_limit_s'] == .3

    def missing(*args):
        raise LookupError('odom -> base_scan missing at exact stamp')

    fresh = InputEvidence()
    fresh.observe('clock', 10_000_000_000, 1.)
    fresh.observe('odom', 10_000_000_000, 1., frame='odom')
    fresh.observe('/scan', 10_000_000_000, 1., frame='base_scan')
    fresh.observe('/mapping/scan', 10_000_000_000, 1., frame='mapping_lidar_link')
    facts, observations = fresh.snapshot(1.1, missing)
    assert facts['/scan'] is True
    assert facts['/scan_odom_tf'] is False
    assert facts['/scan_tf'] is False
    assert observations['/scan_odom_tf']['reason'] == 'exact_time_tf_unavailable'


def test_environment_receipt_snapshots_exact_six_manifest_prefix_bindings(monkeypatch):
    expected = {
        binding["prefix_env"]
        for binding in supervisor.DEPENDENCIES["cleannav_packages"].values()
    }
    for key in expected:
        monkeypatch.setenv(key, "/audited/" + key.lower())
    snapshot = supervisor.environment_variable_snapshot()
    assert expected <= snapshot.keys()
    assert snapshot.keys().isdisjoint({
        "CLEANNAV_EXPECTED_MISSION_PREFIX", "CLEANNAV_EXPECTED_SAFETY_PREFIX",
    })
    assert {key: snapshot[key] for key in expected} == {
        key: "/audited/" + key.lower() for key in expected
    }


@pytest.mark.parametrize("failed_gate", ["memory_swap_psi_cgroup", "gazebo_port_11704", "exclusive_locks"])
def test_failed_resource_port_or_lock_gate_never_calls_process_factory(failed_gate):
    calls = []
    with pytest.raises(policy.ContractError, match="preflight"):
        supervisor.launch_after_preflight({"ok": False, "failed_gate": failed_gate},
                                          lambda: calls.append("Popen"))
    assert calls == []


def resource_snapshot(available, *, reasons=(), ok=True):
    return {"ok": ok, "MemAvailable_kib": available,
            "SwapUsed_kib": 204, "memory_full_psi_avg10": 0.0,
            "cgroup_memory_status": "UNKNOWN",
            "cgroup_memory_disposition": "WAIVED",
            "reasons": list(reasons)}


@pytest.mark.parametrize("available", [5 * 1024 * 1024, 5 * 1024 * 1024 + 1])
def test_diagnostic_runtime_memory_gate_accepts_at_or_above_soft_line(available):
    decision = policy.runtime_memory_decision(
        resource_snapshot(available), timing_profile="localization-diagnostic",
        no_motion_contract=True)
    assert decision["ok"] is True
    assert decision["soft_warning"] is False
    assert decision["hard_stop_threshold_kib"] == 4 * 1024 * 1024


def test_diagnostic_runtime_memory_gate_warns_and_continues_below_historical_line():
    samples = [
        resource_snapshot(5_226_312),
        resource_snapshot(5_100_000),
        resource_snapshot(4_800_000),
        resource_snapshot(4_702_332),
        resource_snapshot(4_650_000),
    ]
    decisions = [policy.runtime_memory_decision(
        sample, timing_profile="localization-diagnostic", no_motion_contract=True)
        for sample in samples]
    assert all(item["ok"] is True and item["soft_warning"] is True
               for item in decisions[1:])
    assert [item["MemAvailable_kib"] for item in decisions] == [
        5_226_312, 5_100_000, 4_800_000, 4_702_332, 4_650_000]
    assert all(item["hard_stop_threshold_kib"] == 4 * 1024 * 1024 for item in decisions)


def test_diagnostic_memavailable_alone_is_not_a_runtime_hard_stop():
    for available in (4_718_591, 4_500_000, 4 * 1024 * 1024):
        decision = policy.runtime_memory_decision(
            resource_snapshot(available, reasons=["MemAvailable_below_4718592KiB"],
                              ok=False),
            timing_profile="localization-diagnostic", no_motion_contract=True)
        assert decision["ok"] is True
        assert decision["soft_warning"] is True
        assert decision["reasons"] == []


@pytest.mark.parametrize("reason", [
    "swap_used_above_1GiB", "memory_full_PSI_avg10_at_least_1",
    "cgroup_headroom_below_runtime_threshold",
    "cgroup_memory_headroom_unknown_or_unwaived", "memory_or_psi_unavailable",
])
def test_diagnostic_soft_warning_never_masks_emergency_resource_reasons(reason):
    decision = policy.runtime_memory_decision(
        resource_snapshot(5_100_000, reasons=[reason], ok=False),
        timing_profile="localization-diagnostic", no_motion_contract=True)
    assert decision["ok"] is False
    assert reason in decision["reasons"]
    assert decision["soft_warning"] is True


def test_diagnostic_kernel_oom_counter_increase_is_a_hard_stop():
    decision = policy.runtime_memory_decision(
        {**resource_snapshot(5_100_000), "oom_kill_delta": 1},
        timing_profile="localization-diagnostic", no_motion_contract=True)
    assert decision["ok"] is False
    assert "kernel_oom_kill_counter_increased" in decision["reasons"]


def test_diagnostic_startup_level_a_requires_full_no_motion_contract():
    qualifies = policy.no_motion_startup_qualifies
    required = dict(all_phases_pass=True, observation_elapsed_s=10.0,
                    enable_motion=False, auto_start_task=False, command_samples=100,
                    nonzero_samples=0, displacement_m=0.01)
    assert qualifies(**required)
    assert not qualifies(**{**required, "observation_elapsed_s": 9.99})
    assert not qualifies(**{**required, "nonzero_samples": 1})
    assert not qualifies(**{**required, "enable_motion": True})
    assert not qualifies(**{**required, "auto_start_task": True})
    assert not qualifies(**{**required, "displacement_m": 0.051})


@pytest.mark.parametrize("timing_profile,no_motion", [
    ("standard", True), ("localization-diagnostic", False),
])
def test_other_profiles_keep_existing_five_gib_runtime_gate(timing_profile, no_motion):
    snapshot = resource_snapshot(5_100_000,
                                 reasons=["MemAvailable_below_5GiB"], ok=False)
    decision = policy.runtime_memory_decision(
        snapshot, timing_profile=timing_profile, no_motion_contract=no_motion)
    assert decision["ok"] is False
    assert decision["soft_warning"] is False
    assert decision["reasons"] == ["MemAvailable_below_5GiB"]


def test_diagnostic_soft_band_requires_explicit_valid_no_motion_launch_argv():
    argv = supervisor.bounded_launch_argv(
        world=Path("/world.sdf"), map_directory=Path("/saved/map"),
        map_profile="mcity-localization07-canonical-debug",
        output_dir=Path("/run"), runtime_timeout_s=45.0)
    assert "odometry_mode:=wheel" in argv
    assert "enable_diagnostics:=false" in argv
    contract = policy.verify_no_motion_launch_argv(argv)
    assert contract == {"enable_motion": False, "auto_start_task": False,
                        "authorization_or_lease_options_present": False}
    with pytest.raises(policy.ContractError, match="no-motion/no-task"):
        policy.verify_no_motion_launch_argv(
            [*argv, "enable_motion:=true"])
    with pytest.raises(policy.ContractError, match="no-motion/no-task"):
        policy.verify_no_motion_launch_argv(
            [*argv, "auto_start_task:=true"])
    with pytest.raises(policy.ContractError, match="no-motion/no-task"):
        policy.verify_no_motion_launch_argv([*argv, "create_lease:=true"])

    oracle_argv = supervisor.bounded_launch_argv(
        world=Path("/world.sdf"), map_directory=Path("/saved/map"),
        map_profile="mcity-localization07-canonical-debug",
        output_dir=Path("/run"), runtime_timeout_s=45.0,
        odometry_mode="oracle")
    assert "odometry_mode:=oracle" in oracle_argv
    assert "enable_diagnostics:=false" in oracle_argv
    assert policy.verify_no_motion_launch_argv(oracle_argv)["enable_motion"] is False
    with pytest.raises(policy.ContractError, match="unsupported odometry mode"):
        supervisor.bounded_launch_argv(
            world=Path("/world.sdf"), map_directory=Path("/saved/map"),
            map_profile="mcity-localization07-canonical-debug",
            output_dir=Path("/run"), runtime_timeout_s=45.0,
            odometry_mode="both")


def complete_guard(now):
    required = (
        "/scan", "/mapping/scan", "/scan_tf", "/mapping/scan_tf", "/scan_odom_tf", "/mapping/scan_odom_tf",
        "/scan_map_tf", "/mapping/scan_map_tf", "clock", "odom",
        "safety", "cmd", "cm", "safety_no_estop", "cm_active", "cmd_publisher",
        "/cleannav/navigation/cmd_vel_raw_publisher",
        "/cleannav/cmd_vel_candidate_publisher", "localization", "no_motion_authority",
    )
    return {"ready": True, "failed": False, "monotonic": now, "missing": [], "distance_m": 0.0,
            "facts": {key: True for key in required},
            "safety": {"block_all": True, "estop": False, "enabled": False, "owner": ""}}


def test_ordered_readiness_requires_each_stage_and_failed_stage_blocks_later():
    now = 10.0
    guard = complete_guard(now)
    logs = "\n".join(f"[{name}] READY" for name in ("SIMULATION", "LOCALIZATION", "NAVIGATION", "MISSION"))
    readiness = policy.phase_readiness(logs, {"passed": True, "decision": "PASS"}, guard,
                                       rviz_node=True, rviz_process=True,
                                       gazebo_process=True, mission_node=True)
    ledger = policy.PhaseLedger()
    changed = ledger.advance(readiness, monotonic_ns=1)
    assert changed == list(policy.PHASES)
    assert all(ledger.states[name] == "PASS" for name in policy.PHASES)

    ledger = policy.PhaseLedger()
    ledger.advance({**readiness, "collision_monitor": (False, {"receipt": "missing"})},
                   monotonic_ns=2)
    assert ledger.states["gazebo_vehicle"] == "PASS"
    assert ledger.states["collision_monitor"] == "NOT RUN"
    assert ledger.states["dual_lidar_tf"] == "NOT RUN"
    ledger.fail("collision_monitor", "not ACTIVE", monotonic_ns=3)
    assert all(ledger.states[name] == "BLOCKED" for name in policy.PHASES[2:])
    assert ledger.advance(readiness, monotonic_ns=4) == []


def test_safety_phase_does_not_require_nav2_raw_publisher_before_navigation_probe():
    guard = complete_guard(20.0)
    guard["facts"]["/cleannav/navigation/cmd_vel_raw_publisher"] = False
    guard["ready"] = False
    guard["missing"] = ["/cleannav/navigation/cmd_vel_raw_publisher"]
    logs = "[SIMULATION] READY\n[LOCALIZATION] READY"
    readiness = policy.phase_readiness(
        logs, {"passed": True, "decision": "PASS"}, guard,
        rviz_node=False, rviz_process=False, gazebo_process=True,
        mission_node=False, now_monotonic=20.0)
    assert readiness["safety"][0] is True
    assert readiness["nav2"][0] is False
    assert readiness["nav2"][1]["raw_publisher_required"] is False


def test_nav2_readiness_requires_raw_publisher_after_navigation_probe():
    guard = complete_guard(20.0)
    guard["facts"]["/cleannav/navigation/cmd_vel_raw_publisher"] = False
    logs = "[SIMULATION] READY\n[LOCALIZATION] READY\n[NAVIGATION] READY"
    readiness = policy.phase_readiness(
        logs, {"passed": True, "decision": "PASS"}, guard,
        rviz_node=False, rviz_process=False, gazebo_process=True,
        mission_node=False, now_monotonic=20.0)
    assert readiness["safety"][0] is True
    assert readiness["nav2"][0] is False
    assert readiness["nav2"][1]["raw_publisher_required"] is True
    assert readiness["nav2"][1]["raw_publisher_ready"] is False


def test_raw_publisher_phase_contract_marks_absence_expected_until_nav2_probe():
    before = policy.raw_publisher_phase_contract([], navigation_probe_ready=False)
    after = policy.raw_publisher_phase_contract([], navigation_probe_ready=True)
    assert before["ok"] is True and before["state"] == "ABSENT_EXPECTED"
    assert after["ok"] is False and after["state"] == "MISSING_REQUIRED"


def test_raw_publisher_phase_contract_accepts_only_one_exact_controller_endpoint():
    expected = {"node_name": "controller_server", "node_namespace": "/",
                "topic_type": "geometry_msgs/msg/Twist", "endpoint_gid": [1, 2, 3]}
    result = policy.raw_publisher_phase_contract([expected], navigation_probe_ready=True)
    assert result["ok"] is True and result["state"] == "PRESENT_EXPECTED"
    for invalid in (
        {**expected, "node_name": "unknown"},
        {**expected, "node_namespace": None},
        {**expected, "topic_type": "unknown"},
        {**expected, "endpoint_gid": None},
    ):
        assert not policy.raw_publisher_phase_contract(
            [invalid], navigation_probe_ready=False)["ok"]
    assert not policy.raw_publisher_phase_contract(
        [expected, expected], navigation_probe_ready=False)["ok"]


def test_guard_receipt_refresh_distinguishes_stale_cache_from_stale_source(tmp_path):
    path = tmp_path / "safety-guard.json"
    path.write_text(json.dumps({"monotonic": 10.55, "failed": False}))
    prior = {"monotonic": 10.0, "failed": False}
    latest, status = supervisor.refresh_guard_receipt(
        path, prior, now_monotonic=10.6)
    assert latest["monotonic"] == 10.55
    assert status["previous_status"] == "FAIL"
    assert status["latest_status"] == "PASS"
    assert status["recovered_from_newer_receipt"] is True
    assert status["usable_latest_receipt"] is True

    path.write_text(json.dumps({"monotonic": 10.0, "failed": False}))
    latest, status = supervisor.refresh_guard_receipt(
        path, prior, now_monotonic=10.6)
    assert latest["monotonic"] == 10.0
    assert status["latest_status"] == "FAIL"
    assert status["usable_latest_receipt"] is False


def test_readiness_requires_actual_collision_receipt_guard_and_rviz_process():
    guard = complete_guard(20.0)
    logs = "[SIMULATION] READY\n[LOCALIZATION] READY\n[NAVIGATION] READY\n[MISSION] READY"
    extra = dict(rviz_node=True, rviz_process=True, gazebo_process=True, mission_node=True)
    no_cm = policy.phase_readiness(logs, None, guard, **extra)
    assert not no_cm["collision_monitor"][0]
    no_rviz = policy.phase_readiness(logs, {"passed": True, "decision": "PASS"}, guard,
                                     rviz_node=True, rviz_process=False,
                                     gazebo_process=True, mission_node=True)
    assert not no_rviz["rviz"][0]
    guard["facts"]["/mapping/scan_odom_tf"] = False
    missing_roof_tf = policy.phase_readiness(logs, {"passed": True, "decision": "PASS"}, guard,
                                             rviz_node=True, rviz_process=True,
                                             gazebo_process=True, mission_node=True)
    assert not missing_roof_tf["dual_lidar_tf"][0]


def test_initial_readiness_handles_unobserved_roof_scan_message_as_unknown():
    guard = complete_guard(20.0)
    guard['facts']['/mapping/scan'] = False
    guard['observations'] = {'/mapping/scan': {'ready': False, 'last_message': None}}
    readiness = policy.phase_readiness(
        '', None, guard, rviz_node=False, rviz_process=False,
        gazebo_process=False, mission_node=False)
    assert readiness['dual_lidar_tf'][1]['mapping_return_status'] == 'UNKNOWN'
    assert not readiness['dual_lidar_tf'][0]


def test_phase_readiness_loss_receipt_preserves_false_guard_facts():
    guard = complete_guard(20.0)
    guard["ready"] = False
    guard["missing"] = ["/mapping/scan_odom_tf"]
    guard["facts"]["/mapping/scan_odom_tf"] = False
    readiness = policy.phase_readiness("[SIMULATION] READY", {"passed": True}, guard,
                                       rviz_node=False, rviz_process=False,
                                       gazebo_process=True, mission_node=False)
    previous = {"facts": {"/mapping/scan_odom_tf": True}}
    receipt = policy.readiness_loss_record("dual_lidar_tf", readiness, guard, previous)
    assert receipt["phase"] == "dual_lidar_tf"
    assert receipt["ready"] is False
    assert receipt["prior_pass_evidence"] == previous
    assert receipt["guard_false_facts"] == ["/mapping/scan_odom_tf"]
    assert receipt["guard_missing"] == ["/mapping/scan_odom_tf"]


def test_actual_supervisor_loss_branch_records_age_and_run_id_without_name_error():
    guard=complete_guard(67.7)
    guard.update(run_id='rviz-nav-test',ready=False,observations={
        '/scan':dict(ready=False,reason='receive_age_exceeded',
                     receive_age_s=.379,freshness_limit_s=.3)})
    guard['facts']['/scan']=False
    readiness=policy.phase_readiness('[SIMULATION] READY',{'passed':True,'decision':'PASS'},guard,
        rviz_node=False,rviz_process=True,gazebo_process=True,mission_node=False,
        now_monotonic=67.7)
    ledger=policy.PhaseLedger()
    for phase in ('gazebo_vehicle','collision_monitor','dual_lidar_tf'):
        ledger.states[phase]='PASS'
    loss=supervisor.first_admitted_phase_loss(ledger,readiness,guard)
    assert loss['phase']=='dual_lidar_tf'
    assert loss['run_id']=='rviz-nav-test'
    assert loss['input_observations']['/scan']['receive_age_s']==.379
    assert loss['input_observations']['/scan']['reason']=='receive_age_exceeded'


def test_no_motion_diagnostic_continues_sensor_loss_without_claiming_readiness():
    facts={
        '/scan':False,'/scan_tf':False,'localization':False,
        'safety':True,'safety_no_estop':True,'cm':True,'cm_active':True,
        'cmd_publisher':True,'no_motion_authority':True,
    }
    endpoints={
        '/cmd_vel':[{'node_name':'cleannav_safety_supervisor','node_namespace':'/',
                     'topic_type':'geometry_msgs/msg/Twist','endpoint_gid':'safety-gid'}],
        # An idle Nav2 stage may not have created its publisher yet. This keeps
        # readiness false, while the missing endpoint itself is not ambiguous.
        '/cleannav/navigation/cmd_vel_raw':[],
        '/cleannav/cmd_vel_candidate':[{'node_name':'collision_monitor','node_namespace':'/',
                     'topic_type':'geometry_msgs/msg/Twist','endpoint_gid':'cm-gid'}],
        '/cleannav/manual/cmd_vel_raw':[],
        '/cleannav/mapping/cmd_vel_raw':[],
    }
    decision=diagnostic_continuation_decision(facts,endpoints,
        nonzero_command_count=0,distance_m=.001)
    assert decision['allowed'] is True
    assert decision['ordinary_readiness_missing']==['/scan','/scan_tf','localization']
    assert decision['maximum_displacement_m']==.05


@pytest.mark.parametrize('mutation,expected',[
    ('safety',True),('estop',False),('cm',True),('cm_stale',True),('unknown_cmd',False),
    ('nonzero',False),('displacement',False),('unmanaged',False),
])
def test_no_motion_diagnostic_stops_on_hard_safety_or_motion_evidence(mutation,expected):
    facts={'/scan':False,'localization':False,'safety':True,'safety_no_estop':True,
           'cm':True,'cm_active':True,'cmd_publisher':True,'no_motion_authority':True}
    endpoints={
        '/cmd_vel':[{'node_name':'cleannav_safety_supervisor','node_namespace':'/',
                     'topic_type':'geometry_msgs/msg/Twist','endpoint_gid':'safety-gid'}],
        '/cleannav/navigation/cmd_vel_raw':[],
        '/cleannav/cmd_vel_candidate':[],
        '/cleannav/manual/cmd_vel_raw':[],
        '/cleannav/mapping/cmd_vel_raw':[],
    }
    nonzero=0;distance=.001
    if mutation=='safety':facts['safety']=False
    elif mutation=='estop':facts['safety_no_estop']=False
    elif mutation=='cm':facts['cm_active']=False
    elif mutation=='cm_stale':facts['cm']=False
    elif mutation=='unknown_cmd':endpoints['/cleannav/navigation/cmd_vel_raw']=[
        {'node_name':'unknown','node_namespace':'/','topic_type':'geometry_msgs/msg/Twist'}]
    elif mutation=='nonzero':nonzero=1
    elif mutation=='displacement':distance=.050001
    elif mutation=='unmanaged':endpoints['/cleannav/manual/cmd_vel_raw']=[
        {'node_name':'manual_node','node_namespace':'/','topic_type':'geometry_msgs/msg/Twist'}]
    decision=diagnostic_continuation_decision(facts,endpoints,
        nonzero_command_count=nonzero,distance_m=distance)
    assert decision['allowed'] is expected
    assert bool(decision['hard_failures']) is (not expected)


def test_phase_ledger_reopens_lost_admission_and_does_not_retain_ready_state():
    ledger=policy.PhaseLedger()
    for phase in ('gazebo_vehicle','collision_monitor','dual_lidar_tf','map_localization'):
        ledger.states[phase]='PASS';ledger.evidence[phase]={'prior':phase}
    previous=ledger.reopen_from('dual_lidar_tf')
    assert previous['dual_lidar_tf']=={'prior':'dual_lidar_tf'}
    assert ledger.states['gazebo_vehicle']=='PASS'
    assert ledger.states['collision_monitor']=='PASS'
    assert ledger.states['dual_lidar_tf']=='NOT RUN'
    assert ledger.states['map_localization']=='NOT RUN'
    assert 'dual_lidar_tf' not in ledger.evidence


def test_diagnostic_readiness_loss_requires_explicit_profile_and_guard_hard_gate():
    script='''
import sys
sys.path.insert(0,sys.argv[1])
import supervisor_policy as p
state={'diagnostic_mode':True,'failed':False,'monotonic':10.0,
       'diagnostic_continuation':{'allowed':True}}
decide=lambda *args: p.diagnostic_readiness_loss_decision(*args,now_monotonic=10.1,no_motion_contract=True)
assert decide('dual_lidar_tf',state,0)['allowed']
assert decide('safety',state,0)['allowed']
assert not decide('map_localization',state,1)['allowed']
for invalid in (True, 'false'):
    state['failed']=invalid
    assert not decide('dual_lidar_tf',state,0)['allowed']
state['failed']=False
assert p.diagnostic_readiness_loss_decision('dual_lidar_tf',state,0,
    now_monotonic=10.5,no_motion_contract=True)['allowed']
assert p.diagnostic_readiness_loss_decision('dual_lidar_tf',state,0,
    now_monotonic=10.500001,no_motion_contract=True)['allowed']
assert p.diagnostic_readiness_loss_decision('dual_lidar_tf',state,0,
    now_monotonic=9.9,no_motion_contract=True)['allowed']
state['diagnostic_continuation']={'allowed':False,'hard_failures':['explicit violation']}
assert not decide('dual_lidar_tf',state,0)['allowed']
'''
    result=subprocess.run([sys.executable,'-c',script,str(HERE)],
        env={**os.environ,'CLEANNAV_RVIZ_TIMING_PROFILE':'localization-diagnostic'},
        capture_output=True,text=True,timeout=5)
    assert result.returncode==0,result.stdout+result.stderr


def test_live_guard_receipt_serializes_healthy_failure_as_explicit_false(monkeypatch):
    monkeypatch.setattr(policy, "LOCALIZATION_DIAGNOSTIC", True)
    assert safety_guard.explicit_failure_state(None) is False
    assert safety_guard.explicit_failure_state("actual guard failure") == "actual guard failure"
    state = {
        "diagnostic_mode": True,
        "failed": safety_guard.explicit_failure_state(None),
        "monotonic": 10.0,
        "diagnostic_continuation": {"allowed": True, "hard_failures": []},
    }
    decision = policy.diagnostic_readiness_loss_decision(
        "dual_lidar_tf", state, 0, now_monotonic=10.1,no_motion_contract=True)
    assert decision["allowed"] is True
    assert state["failed"] is False


def test_actual_supervisor_loss_branch_does_not_fail_a_not_yet_admitted_phase():
    ledger=policy.PhaseLedger()
    readiness={p:(False,{}) for p in policy.PHASES}
    assert supervisor.first_admitted_phase_loss(ledger,readiness,None) is None


def test_localization_stage_requires_the_actual_installed_probe_marker_and_native_health():
    guard=complete_guard(20.)
    args=dict(rviz_node=True,rviz_process=True,gazebo_process=True,mission_node=False)
    assert not policy.phase_readiness('[SLAM] READY',None,guard,**args)['map_localization'][0]
    assert policy.phase_readiness('[LOCALIZATION] READY native accepted saved-node matches=1',None,guard,**args)['map_localization'][0]
    guard['facts']['localization']=False
    assert not policy.phase_readiness('[LOCALIZATION] READY',None,guard,**args)['map_localization'][0]


def test_legacy_low_scan_localization_does_not_require_roof_map_transform():
    guard=complete_guard(20.)
    guard['facts'].pop('/mapping/scan_map_tf')
    args=dict(rviz_node=True,rviz_process=True,gazebo_process=True,mission_node=False,
              localization_scan_topic='/scan')
    result=policy.phase_readiness(
        '[LOCALIZATION] READY native accepted saved-node matches=1',None,guard,**args)
    assert result['map_localization'][0]
    assert result['map_localization'][1]['localization_scan_topic']=='/scan'


def test_roof_scan_localization_still_requires_roof_map_transform():
    guard=complete_guard(20.)
    guard['facts']['/mapping/scan_map_tf']=False
    args=dict(rviz_node=True,rviz_process=True,gazebo_process=True,mission_node=False,
              localization_scan_topic='/mapping/scan')
    result=policy.phase_readiness(
        '[LOCALIZATION] READY native accepted saved-node matches=1',None,guard,**args)
    assert not result['map_localization'][0]


def test_deadline_schedule_separates_soft_escalation_hard_cutoff_and_evidence_cutoff():
    assert policy.deadline_stop_reason(145.0, all_phases_pass=False,
                                       observation_elapsed_s=None) == "soft_stop_deadline_T0_plus_145s"
    assert policy.deadline_stop_reason(110.0, all_phases_pass=False,
                                       observation_elapsed_s=None) == "readiness_deadline_T0_plus_110s"
    assert policy.deadline_stop_reason(20.0, all_phases_pass=True,
                                       observation_elapsed_s=10.0) == "no_motion_observation_complete"
    assert policy.ACCOUNTING_LIMIT_FROM_T0_S == 180
    assert policy.SOFT_STOP_FROM_T0_S < policy.ACCOUNTING_LIMIT_FROM_T0_S
    assert policy.WATCHDOG_SIGTERM_FROM_T0_S == 159
    assert policy.WATCHDOG_SIGKILL_FROM_T0_S == 167
    assert policy.HARD_PROCESS_CUTOFF_FROM_T0_S == 175
    assert policy.CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S == 180
    state = policy.DeadlineWatchdogState()
    assert state.actions(110, supervisor_alive=True, process_group_status="ALIVE",
                         completion_recorded=False, readiness_complete=False) == [
        {"kind": "signal_supervisor", "signal": "SIGINT",
         "reason": "readiness_deadline_T0_plus_110s", "deadline_t0_s": 110.0}]
    assert state.actions(145, supervisor_alive=True, process_group_status="ALIVE",
                         completion_recorded=False, readiness_complete=True) == []
    fresh_state = policy.DeadlineWatchdogState()
    assert fresh_state.actions(145, supervisor_alive=True, process_group_status="ALIVE",
                               completion_recorded=False, readiness_complete=True) == [
        {"kind": "signal_supervisor", "signal": "SIGINT",
         "reason": "soft_stop_T0_plus_145s", "deadline_t0_s": 145.0}]
    assert fresh_state.actions(159, supervisor_alive=True, process_group_status="ALIVE",
                               completion_recorded=False, readiness_complete=True)[0]["signal"] == "SIGTERM"
    assert fresh_state.actions(167, supervisor_alive=True, process_group_status="ALIVE",
                               completion_recorded=False, readiness_complete=True)[0]["signal"] == "SIGKILL"
    assert fresh_state.actions(175, supervisor_alive=True, process_group_status="EMPTY",
                               completion_recorded=False, readiness_complete=True)[0]["kind"] == "hard_process_cutoff"
    assert policy.final_status(failures=[], phases={p: "PASS" for p in policy.PHASES},
                               cleanup={"cleanup_ok": True}, launch_started=True,
                               elapsed_s=180.1, cgroup_disposition="KNOWN") == "FAIL"


def test_readiness_deadline_failure_message_uses_active_mode_bound():
    import ast
    tree = ast.parse((HERE / "navigation_supervisor.py").read_text())
    branch = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
                  and ast.unparse(n.test).startswith("not LOCALIZATION_DIAGNOSTIC and (not startup_verification_complete)"))
    code = compile(ast.Module(body=[branch], type_ignores=[]), "<readiness-deadline>", "exec")
    failures = []
    scope = {
        "LOCALIZATION_DIAGNOSTIC": False,
        "startup_verification_complete":False,
        "elapsed": lambda: 205.0,
        "READINESS_DEADLINE_FROM_T0_S": 205.0,
        "all_phases": False,
        "fail": lambda reason, stage: failures.append((reason, stage)),
    }
    exec(code, scope)
    assert failures == [(
        "required lifecycle phases did not become ready before T0+205s", "readiness")]


def test_historical_readiness_deadline_receipt_reports_205_seconds():
    env = dict(os.environ, PYTHONPATH=str(HERE),
               CLEANNAV_RVIZ_TIMING_PROFILE="historical-motion")
    code = (
        "import json,supervisor_policy as p; "
        "s=p.DeadlineWatchdogState(); "
        "print(json.dumps({'stop':p.deadline_stop_reason(205,all_phases_pass=False,"
        "observation_elapsed_s=None), 'watchdog':s.actions(205,supervisor_alive=True,"
        "process_group_status='ALIVE',completion_recorded=False,readiness_complete=False)}))"
    )
    result = json.loads(subprocess.check_output([sys.executable, "-c", code], env=env, text=True))
    assert result["stop"] == "readiness_deadline_T0_plus_205s"
    assert result["watchdog"] == [{
        "kind": "signal_supervisor", "signal": "SIGINT",
        "reason": "readiness_deadline_T0_plus_205s", "deadline_t0_s": 205.0}]


def test_diagnostic_deadline_uses_persistent_level_a_not_current_phase_readiness():
    state = {"timing_profile": "localization-diagnostic",
             "readiness_all_passed": False,
             "startup_verification_complete": True}
    assert watchdog_module.readiness_complete_for_deadline(state) is True
    assert watchdog_module.readiness_complete_for_deadline({
        **state, "startup_verification_complete": False}) is False
    assert watchdog_module.readiness_complete_for_deadline({
        "timing_profile": "standard", "readiness_all_passed": True,
        "startup_verification_complete": False}) is True


def test_cpu_resource_counters_and_psi_snapshot_are_parsed_without_ros_runtime(tmp_path):
    proc=tmp_path/'proc';(proc/'self').mkdir(parents=True);(proc/'pressure').mkdir()
    (proc/'stat').write_text('cpu 100 20 30 400 5 0 0 0\n')
    fields=['R']+['1']*13;fields[11]='10';fields[12]='5'
    (proc/'self/stat').write_text('42 (test process) '+' '.join(fields)+'\n')
    first=policy.linux_cpu_counters(proc)
    (proc/'stat').write_text('cpu 200 20 30 500 5 0 0 0\n')
    fields[11]='20';fields[12]='10'
    (proc/'self/stat').write_text('42 (test process) '+' '.join(fields)+'\n')
    second=policy.linux_cpu_counters(proc)
    assert first=={'system_total_ticks':555,'system_idle_ticks':405,'process_ticks':15}
    assert policy.cpu_usage_percent(first,second,.5,100)=={
        'system_busy_pct':50.0,'process_cpu_pct':30.0}
    (proc/'pressure/cpu').write_text('some avg10=1.20 avg60=0.50 total=10\n')
    (proc/'pressure/memory').write_text('some avg10=0.10 avg60=0.00 total=11\nfull avg10=0.02 avg60=0.00 total=3\n')
    snapshot=supervisor.pressure_snapshot(proc)
    assert snapshot['cpu']['values']['some']['avg10']==1.2
    assert snapshot['memory']['values']['full']['total']==3
    assert snapshot['io']['status']=='UNKNOWN'


def test_watchdog_is_out_of_supervisor_loop_and_emergency_signals_when_loop_blocks(tmp_path):
    now = [0]
    supervisor_alive = [True]
    group_alive = [True]
    sent = []

    def clock_ns():
        return now[0]

    def sleep(seconds):
        now[0] += max(1, int(seconds * 1_000_000_000))

    def get_identity(pid):
        if pid == 222 and supervisor_alive[0]:
            return {"pid": pid, "start_ticks": 22, "state": "S", "pgid": 2}
        if pid == 333 and group_alive[0]:
            return {"pid": pid, "start_ticks": 33, "state": "S", "pgid": 333}
        return None

    def snapshot(_pgid):
        if group_alive[0]:
            return {"status": "ALIVE", "scan_complete": True,
                    "members": [{"pid": 333, "state": "S"}],
                    "running_members": [{"pid": 333, "state": "S"}],
                    "zombie_members": [], "errors": []}
        return {"status": "EMPTY", "scan_complete": True, "members": [],
                "running_members": [], "zombie_members": [], "errors": []}

    def kill_pid(pid, sig):
        sent.append(("pid", pid, sig))
        if sig == signal.SIGKILL:
            supervisor_alive[0] = False

    def kill_group(pgid, sig):
        sent.append(("pgid", pgid, sig))
        if sig == signal.SIGKILL:
            group_alive[0] = False

    watchdog = watchdog_module.Watchdog(
        run_dir=tmp_path, run_id="rviz-nav-20261021T000000Z-deadbeef", t0_ns=0,
        supervisor_pid=222, supervisor_start_ticks=22,
        launch_pid=333, launch_start_ticks=33, clock_ns=clock_ns,
        sleeper=sleep, get_identity=get_identity, snapshot=snapshot,
        kill_pid=kill_pid, kill_group=kill_group)
    # No completion marker is written: the simulated supervisor main loop is blocked.
    assert watchdog.run(poll_interval_s=0.1) == 124
    assert ("pid", 222, signal.SIGINT) in sent
    assert ("pgid", 333, signal.SIGTERM) in sent
    assert ("pgid", 333, signal.SIGKILL) in sent
    assert ("pid", 222, signal.SIGKILL) in sent
    receipt = json.loads((tmp_path / "hard_deadline.json").read_text())
    assert receipt["hard_deadline_enforced"] is True
    assert receipt["process_group_empty_after"] is True
    assert receipt["launch_main_reaped"] == "UNKNOWN"
    assert receipt["cleanup_ok"] is False
    assert receipt["observed_t0_elapsed_s"] < policy.CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S
    assert json.loads((tmp_path / "watchdog_exit.json").read_text())["status"] == "HARD_CUTOFF"
    terminal = json.loads((tmp_path / "result.json").read_text())
    assert terminal["status"] == "FAIL"
    assert terminal["hard_deadline_enforced"] is True
    assert json.loads((tmp_path / "cleanup.json").read_text())["cleanup_ok"] is False
    assert (tmp_path / "SHA256SUMS").is_file()


def test_watchdog_normal_completion_does_not_signal_any_process(tmp_path):
    (tmp_path / "supervisor_complete.json").write_text("{}\n")
    sent = []
    watchdog = watchdog_module.Watchdog(
        run_dir=tmp_path, run_id="rviz-nav-20261021T000000Z-deadbeef", t0_ns=0,
        supervisor_pid=222, supervisor_start_ticks=22,
        launch_pid=333, launch_start_ticks=33,
        clock_ns=lambda: 1, sleeper=lambda _seconds: None,
        get_identity=lambda _pid: None,
        snapshot=lambda _pgid: {"status": "ALIVE", "scan_complete": True,
                                "members": [{"pid": 333}], "running_members": [{"pid": 333}],
                                "zombie_members": [], "errors": []},
        kill_pid=lambda *args: sent.append(("pid", args)),
        kill_group=lambda *args: sent.append(("group", args)))
    assert watchdog.run() == 0
    assert sent == []
    assert json.loads((tmp_path / "watchdog_exit.json").read_text())["status"] == "NORMAL_COMPLETION"


def test_watchdog_unexpected_supervisor_exit_records_failure_and_stops_owned_group(tmp_path):
    sent = []
    group_alive = [True]

    def snapshot(_pgid):
        if group_alive[0]:
            return {"status": "ALIVE", "scan_complete": True,
                    "members": [{"pid": 333, "state": "S"}],
                    "running_members": [{"pid": 333, "state": "S"}],
                    "zombie_members": [], "errors": []}
        return {"status": "EMPTY", "scan_complete": True, "members": [],
                "running_members": [], "zombie_members": [], "errors": []}

    def kill_group(pgid, sig):
        sent.append((pgid, sig))
        if sig == signal.SIGTERM:
            group_alive[0] = False

    watchdog = watchdog_module.Watchdog(
        run_dir=tmp_path, run_id="rviz-nav-20261021T000000Z-deadbeef", t0_ns=0,
        supervisor_pid=222, supervisor_start_ticks=22,
        launch_pid=333, launch_start_ticks=33,
        clock_ns=lambda: 1, sleeper=lambda _seconds: None,
        get_identity=lambda _pid: None, snapshot=snapshot,
        kill_pid=lambda *args: sent.append(("pid", *args)), kill_group=kill_group)
    assert watchdog.run() == 1
    receipt = json.loads((tmp_path / "hard_deadline.json").read_text())
    assert receipt["reason"] == "supervisor_exited_without_completion_receipt"
    assert receipt["hard_deadline_enforced"] is False
    assert receipt["process_group_empty_after"] is True
    assert receipt["launch_main_reaped"] == "UNKNOWN"
    assert sent == [(333, signal.SIGTERM)]
    assert json.loads((tmp_path / "result.json").read_text())["status"] == "FAIL"
    assert (tmp_path / "SHA256SUMS").is_file()


def test_watchdog_unknown_group_scan_stays_fail_closed_at_hard_cutoff(tmp_path):
    now = [0]
    sup_alive = [True]
    sent = []

    def sleep(seconds):
        now[0] += max(1, int(seconds * 1_000_000_000))

    def identity(pid):
        return {"start_ticks": 22} if pid == 222 and sup_alive[0] else None

    def kill_pid(pid, sig):
        sent.append((pid, sig))
        if sig == signal.SIGKILL:
            sup_alive[0] = False

    watchdog = watchdog_module.Watchdog(
        run_dir=tmp_path, run_id="rviz-nav-20261021T000000Z-deadbeef", t0_ns=0,
        supervisor_pid=222, supervisor_start_ticks=22,
        launch_pid=333, launch_start_ticks=33,
        clock_ns=lambda: now[0], sleeper=sleep, get_identity=identity,
        snapshot=lambda _pgid: {"status": "UNKNOWN", "scan_complete": False,
                                "members": [], "running_members": [],
                                "zombie_members": [], "errors": ["injected"]},
        kill_pid=kill_pid, kill_group=lambda pgid, sig: sent.append((pgid, sig)))
    assert watchdog.run(poll_interval_s=0.1) == 124
    assert not any(pgid == 333 for pgid, _sig in sent)
    receipt = json.loads((tmp_path / "hard_deadline.json").read_text())
    assert receipt["cleanup_ok"] is False
    assert receipt["process_group_snapshot_after"]["status"] == "UNKNOWN"


def test_watchdog_cleanup_timeout_keeps_live_group_as_failure(tmp_path):
    now = [0]
    sup_alive = [True]
    sent = []

    def sleep(seconds):
        now[0] += max(1, int(seconds * 1_000_000_000))

    def identity(pid):
        if pid == 222 and sup_alive[0]:
            return {"start_ticks": 22}
        if pid == 333:
            return {"start_ticks": 33, "pgid": 333}
        return None

    def kill_pid(pid, sig):
        sent.append(("pid", pid, sig))
        if pid == 222 and sig == signal.SIGKILL:
            sup_alive[0] = False

    def kill_group(pgid, sig):
        sent.append(("pgid", pgid, sig))
        # Inject a SIGKILL-resistant/uninterruptible process: group remains live.

    def snapshot(_pgid):
        return {"status": "ALIVE", "scan_complete": True,
                "members": [{"pid": 333, "state": "D"}],
                "running_members": [{"pid": 333, "state": "D"}],
                "zombie_members": [], "errors": []}

    watchdog = watchdog_module.Watchdog(
        run_dir=tmp_path, run_id="rviz-nav-20261021T000000Z-deadbeef", t0_ns=0,
        supervisor_pid=222, supervisor_start_ticks=22,
        launch_pid=333, launch_start_ticks=33,
        clock_ns=lambda: now[0], sleeper=sleep, get_identity=identity,
        snapshot=snapshot, kill_pid=kill_pid, kill_group=kill_group)
    assert watchdog.run(poll_interval_s=0.1) == 124
    receipt = json.loads((tmp_path / "hard_deadline.json").read_text())
    assert now[0] >= int(policy.HARD_PROCESS_CUTOFF_FROM_T0_S * 1_000_000_000)
    assert receipt["process_group_empty_after"] is False
    assert receipt["process_group_snapshot_after"]["status"] == "ALIVE"
    assert receipt["cleanup_ok"] is False
    assert ("pgid", 333, signal.SIGKILL) in sent


def test_watchdog_finalizer_preserves_failure_even_when_final_scan_is_empty(tmp_path):
    run_id = "rviz-nav-20261021T000000Z-deadbeef"
    (tmp_path / "wrapper.json").write_text(json.dumps({"run_id": run_id, "t0_utc": "test"}))
    (tmp_path / "hard_deadline.json").write_text(json.dumps({
        "run_id": run_id, "status": "FAIL_CLOSED", "hard_deadline_enforced": True,
        "reason": "injected cutoff", "launch_main_reaped": "UNKNOWN",
        "cleanup_ok": False,
        "process_group_snapshot_after": {"status": "EMPTY", "scan_complete": True,
            "running_members": [], "zombie_members": []},
    }))
    args = type("Args", (), {"run_dir": str(tmp_path), "run_id": run_id, "t0_ns": 1})()
    assert supervisor.record_watchdog_failure(args) == 124
    result = json.loads((tmp_path / "result.json").read_text())
    cleanup = json.loads((tmp_path / "cleanup.json").read_text())
    assert result["status"] == "FAIL"
    assert result["hard_deadline_enforced"] is True
    assert cleanup["cleanup_ok"] is False
    assert cleanup["launch_main_reaped"] == "UNKNOWN"
    assert (tmp_path / "SHA256SUMS").is_file()


def test_watchdog_soak_cutoff_preserves_verified_level_a_and_fails_soak(tmp_path):
    run_id = "rviz-nav-20261022T000000Z-abc12345"
    verified = {"status": "PASS", "run_id": run_id,
                "continuous_all_ready_observation_s": 10.0}
    (tmp_path / "supervisor_state.json").write_text(json.dumps({
        "run_id": run_id, "timing_profile": "localization-diagnostic",
        "startup_verification_complete": True,
        "verified_no_motion_startup": verified,
    }))
    (tmp_path / "result.json").write_text(json.dumps({
        "run_id": run_id, "verified_no_motion_startup": verified,
        "startup_soak": {"status": "IN_PROGRESS"}, "failures": [],
    }))
    watchdog = watchdog_module.Watchdog(
        run_dir=tmp_path, run_id=run_id, t0_ns=1,
        clock_ns=lambda: 2, sleeper=lambda _seconds: None,
        get_identity=lambda _pid: None,
        snapshot=lambda _pgid: {"status": "NOT_STARTED", "scan_complete": True,
                                "members": [], "running_members": [],
                                "zombie_members": [], "errors": []})
    watchdog._write_failure("injected soak cutoff", hard_cutoff=True)
    result = json.loads((tmp_path / "result.json").read_text())
    assert result["verified_no_motion_startup"] == verified
    assert result["verified_no_motion_startup"]["status"] == "PASS"
    assert result["startup_soak"]["status"] == "FAIL"


def test_runtime_cleanup_escalates_sigint_sigterm_sigkill_and_reaps_process_group():
    now = [0.0]

    class FakeProcess:
        pid = 43210
        code = None

        def poll(self):
            return self.code

        def send_signal(self, sig):
            assert sig == signal.SIGINT

    process = FakeProcess()
    sent = []

    def snapshot(_pgid):
        if process.code is not None:
            return {"status": "EMPTY", "scan_complete": True, "members": [],
                    "running_members": [], "zombie_members": [], "errors": []}
        return {"status": "ALIVE", "scan_complete": True,
                "members": [{"pid": process.pid, "state": "S"}],
                "running_members": [{"pid": process.pid, "state": "S"}],
                "zombie_members": [], "errors": []}

    def killpg(pgid, sig):
        assert pgid == process.pid
        sent.append(sig)
        if sig == signal.SIGKILL:
            process.code = -signal.SIGKILL

    def wait_fn(_process, timeout):
        now[0] += timeout
        raise subprocess.TimeoutExpired("launch", timeout)

    result = runtime.signal_and_reap(
        process, True, 0.0, snapshot_fn=snapshot, killpg_fn=killpg,
        clock=lambda: now[0], sleeper=lambda seconds: now.__setitem__(0, now[0] + seconds),
        wait_fn=wait_fn, sigint_grace=0.3, sigterm_grace=0.3, sigkill_grace=0.3)
    assert sent == [signal.SIGTERM, signal.SIGKILL]
    assert [item["signal"] for item in result["signal_records"]] == ["SIGINT", "SIGTERM", "SIGKILL"]
    assert result["launch_main_reaped"] is True
    assert policy.cleanup_is_verified(result) is True


@pytest.mark.parametrize("state", ["ZOMBIES_ONLY", "UNKNOWN", "ALIVE"])
def test_unknown_zombie_or_live_cleanup_never_passes(state):
    cleanup = {"cleanup_ok": True, "launch_main_reaped": True,
               "process_group_alive_after_cleanup": False,
               "process_group_state_after_cleanup": state,
               "process_group_snapshot_after_cleanup": {
                   "scan_complete": state != "UNKNOWN", "status": state,
                   "running_members": [], "zombie_members": ([{"pid": 4}] if state == "ZOMBIES_ONLY" else [])}}
    assert policy.cleanup_is_verified(cleanup) is False
    assert policy.final_status(failures=[], phases={p: "PASS" for p in policy.PHASES},
                               cleanup=cleanup, launch_started=True, elapsed_s=30,
                               cgroup_disposition="KNOWN") == "FAIL"


def test_guard_requires_blocked_unleased_safety_and_no_nonzero_commands():
    state = complete_guard(30.0)
    policy.validate_guard_state(state, now_monotonic=30.0)
    for values in ([0.1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, float("nan")]):
        with pytest.raises(policy.ContractError):
            policy.validate_zero_command(values)
    policy.validate_zero_command([0, 0, 0, 0, 0, 0])
    assert policy.zero_command_observation_ok(1, 0) is True
    assert policy.zero_command_observation_ok(0, 0) is False
    assert policy.zero_command_observation_ok(1, 1) is False
    state["safety"]["owner"] = "unexpected-lease"
    with pytest.raises(policy.ContractError, match="unleased"):
        policy.validate_guard_state(state, now_monotonic=30.0)


def test_fixed_argv_cannot_enable_motion_or_automatic_task():
    argv = supervisor.bounded_launch_argv(world=Path("/world.sdf"),
                                          map_directory=Path("/saved/map"),
                                          map_profile="mcity-localization07",
                                          output_dir=Path("/run"), runtime_timeout_s=45.0)
    assert "map_profile:=mcity-localization07" in argv
    assert "enable_motion:=false" in argv
    assert "auto_start_task:=false" in argv
    assert "enable_rviz_goals:=true" in argv
    # An empty `name:=` override is rejected by `ros2 launch` argument parsing.
    # The launch file declares an empty default, so leave this optional value
    # unset rather than emitting a malformed CLI argument.
    assert not any(item.startswith("localization_initial_pose:=") for item in argv)
    assert not any("simulation_task_client" in item or "goal_x" in item for item in argv)
    assert str(supervisor.ENTRY) in argv


def test_rviz_readiness_requires_historical_graph_identity():
    assert supervisor.expected_rviz_graph_node([('rviz', '/')]) is True
    assert supervisor.expected_rviz_graph_node([('rviz2', '/')]) is False
    assert supervisor.expected_rviz_graph_node([('rviz', '/nested')]) is False


@pytest.mark.parametrize("endpoints,expected", [
    ([{"node_name": "controller_server", "node_namespace": "/",
       "topic_type": "geometry_msgs/msg/Twist", "endpoint_gid": [1]}], True),
    ([], False),
    ([{"node_name": "unknown", "node_namespace": "/",
       "topic_type": "geometry_msgs/msg/Twist", "endpoint_gid": [1]}], False),
    ([{"node_name": "controller_server", "node_namespace": "/",
       "topic_type": "geometry_msgs/msg/Twist", "endpoint_gid": [1]},
      {"node_name": "other", "node_namespace": "/",
       "topic_type": "geometry_msgs/msg/Twist", "endpoint_gid": [2]}], False),
])
def test_raw_command_publisher_identity_is_exact(endpoints, expected):
    ok, _ = supervisor.one_expected_publisher(endpoints, "controller_server")
    assert ok is expected


def test_normal_and_exception_receipts_are_independent_and_checksummed(tmp_path):
    result = {"run_id": "rviz-nav-test", "status": "PASS_WITH_EXCEPTION"}
    cleanup = {"cleanup_ok": True, "launch_main_reaped": True,
               "process_group_alive_after_cleanup": False, "process_group_state_after_cleanup": "EMPTY",
               "process_group_snapshot_after_cleanup": {"scan_complete": True, "status": "EMPTY",
                  "running_members": [], "zombie_members": []}}
    supervisor.write_run_receipts(tmp_path, result, cleanup=cleanup,
                                  environment={"status": "CAPTURED"},
                                  identity={"source_manifest_sha256": "abc"}, events=[])
    assert json.loads((tmp_path / "result.json").read_text())["status"] == "PASS_WITH_EXCEPTION"
    hashes = {line.split("  ", 1)[1]: line.split("  ", 1)[0]
              for line in (tmp_path / "SHA256SUMS").read_text().splitlines()}
    for name in ("result.json", "cleanup.json", "environment.json", "source_identity.json", "events.json"):
        assert hashes[name] == hashlib.sha256((tmp_path / name).read_bytes()).hexdigest()

    failed = {"run_id": "rviz-nav-failed", "status": "FAIL", "launch_exit_code": 1}
    supervisor.write_run_receipts(tmp_path / "failed", failed, cleanup=cleanup,
                                  environment={"status": "CAPTURED"},
                                  identity={"source_manifest_sha256": "def"}, events=[])
    assert json.loads((tmp_path / "failed/result.json").read_text())["launch_exit_code"] == 1


def test_supervisor_preserves_verified_source_identity_on_post_identity_failure(tmp_path,monkeypatch):
    # Execute run() only to the injected lock rejection; no ROS init or child.
    run_dir=tmp_path/'run';run_dir.mkdir()
    setup=tmp_path/'setup';setup.write_text('# inert test setup')
    world=tmp_path/'world';world.write_text('test world')
    maps=tmp_path/'map';maps.mkdir();(maps/'map.yaml').write_text('test map')
    assets=tmp_path/'assets';(assets/'models').mkdir(parents=True)
    digest=hashlib.sha256(setup.read_bytes()).hexdigest()
    monkeypatch.setenv('ROS_HOME',str(tmp_path/'prior-home'))
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'prior-logs'))
    monkeypatch.setenv('DISPLAY',':not-launched')
    monkeypatch.setattr(supervisor,'NAV',Path.cwd())
    monkeypatch.setattr(supervisor,'MODEL_ASSETS',assets)
    monkeypatch.setattr(supervisor,'validate_run_directory',lambda *a:{'run_dir':str(run_dir)})
    monkeypatch.setattr(supervisor,'validate_environment',lambda *a:None)
    monkeypatch.setattr(supervisor,'source_identity',lambda:{'head':'test-head',
        'manifest_files':{'source.py':'abc'},'source_manifest_sha256':'test-manifest'})
    monkeypatch.setattr(supervisor.shutil,'which',lambda name:'/opt/ros/humble/bin/'+name)
    monkeypatch.setattr(supervisor.integration,'verify_map',lambda *a,**k:{'files':{'map.yaml':'test-map'}})
    monkeypatch.setattr(supervisor.integration,'verify_packages',lambda *a:{})
    monkeypatch.setattr(supervisor,'installed_file_identity',lambda *a:{})
    def reject_lock():raise policy.ContractError('injected lock gate rejection')
    monkeypatch.setattr(supervisor.runtime,'acquire_locks',reject_lock)
    def forbidden(*a,**k):pytest.fail('identity receipt test must never create a process')
    monkeypatch.setattr(supervisor.subprocess,'Popen',forbidden)
    args=type('Args',(),dict(run_id='rviz-nav-20261021T000000Z-deadbeef',map_profile='roof-lidar-360',
        run_dir=str(run_dir),t0_ns=time.monotonic_ns(),overlay_setup=str(setup),
        overlay_before_sha256=digest,overlay_after_sha256=digest,
            map_directory=str(maps),world=str(world),motion_profile='none',odometry_mode='wheel'))()
    assert supervisor.run(args)==1
    receipt=json.loads((run_dir/'source_identity.json').read_text())
    assert receipt['source_manifest_sha256']=='test-manifest'
    assert receipt['manifest_files']=={'source.py':'abc'}
    result=json.loads((run_dir/'result.json').read_text())
    assert result['launch_started'] is False
    assert result['cleanup']['cleanup_ok'] is None


@pytest.mark.parametrize("watchdog_rc,supervisor_rc,expected", [
    (0, 0, "PASS_WITH_EXCEPTION"),
    (2, 0, "FAIL"),
    (0, 1, "FAIL"),
])
def test_wrapper_wait_receipt_binds_actual_watchdog_and_supervisor_exit_codes(
        tmp_path, watchdog_rc, supervisor_rc, expected):
    t0_ns = time.monotonic_ns() - 1_000_000
    cleanup = {"cleanup_ok": True, "launch_main_reaped": True,
               "process_group_alive_after_cleanup": False,
               "process_group_state_after_cleanup": "EMPTY",
               "process_group_snapshot_after_cleanup": {"scan_complete": True,
                   "status": "EMPTY", "running_members": [], "zombie_members": []}}
    result = {"run_id": "rviz-nav-20261021T000000Z-deadbeef",
              "status": "PASS_WITH_EXCEPTION", "failures": [],
              "t0_monotonic_ns": t0_ns}
    supervisor.write_run_receipts(tmp_path, result, cleanup=cleanup,
                                  environment={"status": "CAPTURED"},
                                  identity={"status": "BOUND"}, events=[])
    (tmp_path / "watchdog_exit.json").write_text(json.dumps({
        "status": "NORMAL_COMPLETION" if watchdog_rc == 0 else "SUPERVISOR_EXIT_WITHOUT_RECEIPT",
        "exit_code": watchdog_rc,
    }))
    args = type("Args", (), {"run_dir": str(tmp_path), "run_id": result["run_id"],
                             "t0_ns": t0_ns, "watchdog_pid": 456,
                             "watchdog_exit_code": watchdog_rc,
                             "supervisor_exit_code": supervisor_rc})()
    assert supervisor.record_watchdog_exit(args) == 0
    final = json.loads((tmp_path / "result.json").read_text())
    assert final["status"] == expected
    assert final["wrapper_wait_status"]["supervisor_pid_reaped"] is True
    assert final["wrapper_wait_status"]["watchdog_pid_reaped"] is True
    assert final["wrapper_wait_status"]["watchdog_exit_code"] == watchdog_rc
    hashes = {line.split("  ", 1)[1]: line.split("  ", 1)[0]
              for line in (tmp_path / "SHA256SUMS").read_text().splitlines()}
    for name in ("result.json", "cleanup.json", "watchdog_exit.json"):
        assert hashes[name] == hashlib.sha256((tmp_path / name).read_bytes()).hexdigest()


def test_wrapper_setup_failure_writes_blocked_receipt_without_cleanup_pass(tmp_path):
    run_dir=tmp_path/'rviz-nav-20261021T000000Z-deadbeef'
    run_dir.mkdir()
    (run_dir/'wrapper.json').write_text(json.dumps({
        'run_id':run_dir.name, 't0_monotonic_ns':1,
        'navigation_worktree':'/repo', 'workspace_root':'/workspace'}))
    args=type('Args',(),{'run_dir':str(run_dir),'run_id':run_dir.name,'t0_ns':1,
                         'reason':'overlay source failed'})()
    assert supervisor.wrapper_failure(args)==2
    result=json.loads((run_dir/'result.json').read_text())
    assert result['status']=='BLOCKED' and result['launch_started'] is False
    assert result['cleanup']['status']=='NOT_STARTED' and result['cleanup']['cleanup_ok'] is None
    assert (run_dir/'environment.json').is_file()
    assert (run_dir/'source_identity.json').is_file()
    assert (run_dir/'SHA256SUMS').is_file()


def test_shell_runner_rejects_external_output_root_before_creating_it(tmp_path):
    external = tmp_path / "outside-deliverables"
    env = dict(os.environ)
    env["CLEANNAV_RVIZ_OUTPUT_ROOT"] = str(external)
    completed = subprocess.run(
        ["bash", str(HERE / "run_rviz_navigation.sh"),
         "--map-profile", "mcity-localization07",
         "--map-directory", "/unused/map", "--world", "/unused/world.sdf"],
        cwd=supervisor.NAV, env=env, capture_output=True, text=True, timeout=5,
    )
    assert completed.returncode == 2
    assert "must stay below" in completed.stderr
    assert not external.exists()


def _initial_run_directory_fixture(tmp_path, *, run_id="rviz-nav-20261021T000000Z-a1b2c3d4"):
    run_dir = tmp_path / run_id
    run_dir.mkdir()
    t0_ns = 123456789000
    overlay = tmp_path / "overlay_setup.bash"
    overlay.write_text("# reviewed test overlay\n")
    args = type("Args", (), {
        "run_id": run_id, "run_dir": str(run_dir), "t0_ns": t0_ns,
        "map_profile": "roof-lidar-360", "odometry_mode": "wheel",
        "map_directory": "/maps/test-map", "world": "/worlds/test.world",
        "overlay_setup": str(overlay),
    })()
    (run_dir / "wrapper.json").write_text(json.dumps({
        "run_id": run_id, "t0_monotonic_ns": t0_ns,
        "navigation_worktree": str(supervisor.NAV.resolve()),
        "workspace_root": str(runtime.ROOT.resolve()),
        "map_profile": args.map_profile,
        "map_directory_argument": args.map_directory, "world_argument": args.world,
        "overlay_setup_argument": str(overlay), "t0_utc": "2026-10-21T00:00:00Z",
    }))
    (run_dir / "environment-setup.log").write_text("test setup output\n")
    (run_dir / "watchdog.log").write_text("test watchdog output\n")
    (run_dir / "supervisor.log").write_text("test supervisor output\n")
    (run_dir / "watchdog_ready.json").write_text(json.dumps({
        "status": "READY", "run_id": run_id, "pid": 30, "start_ticks": 300,
        "process_group_id": 30, "session_id": 30,
        "wrapper_pid": 20, "wrapper_start_ticks": 200,
        "supervisor_pid": None, "supervisor_start_ticks": None,
        "launch_pid": None, "launch_start_ticks": None, "t0_monotonic_ns": t0_ns,
    }))
    (run_dir / "supervisor_state.json").write_text(json.dumps({
        "run_id": run_id, "supervisor_pid": 40, "supervisor_start_ticks": 400,
        "launch_pid": None, "launch_pgid": None, "launch_start_ticks": None,
        "readiness_all_passed": False, "readiness_phases": {},
        "t0_monotonic_ns": t0_ns,
    }))
    identities = {
        20: {"pid": 20, "state": "S", "pgid": 20, "session_id": 20, "start_ticks": 200},
        30: {"pid": 30, "state": "S", "pgid": 30, "session_id": 30, "start_ticks": 300},
        40: {"pid": 40, "state": "S", "pgid": 20, "session_id": 20, "start_ticks": 400},
    }
    return run_dir, args, identities


def _validate_fixture(run_dir, args, identities):
    return supervisor.validate_run_directory(
        run_dir, args, output_root=run_dir.parent,
        identity_reader=lambda pid: identities.get(pid), current_pid=40, parent_pid=20)


def test_run_directory_accepts_exact_same_run_startup_credentials(tmp_path):
    run_dir, args, _ = _initial_run_directory_fixture(tmp_path)
    identities = {
        20: {"pid": 20, "state": "S", "pgid": 20, "session_id": 20, "start_ticks": 200},
        30: {"pid": 30, "state": "S", "pgid": 30, "session_id": 30, "start_ticks": 300},
        40: {"pid": 40, "state": "S", "pgid": 20, "session_id": 20, "start_ticks": 400},
    }
    result = _validate_fixture(run_dir, args, identities)
    assert result["run_id"] == args.run_id
    assert result["run_dir"] == str(run_dir.resolve())
    assert result["supervisor_pid"] == 40 and result["watchdog_pid"] == 30


@pytest.mark.parametrize("mutation,match", [
    ("old_wrapper_run_id", "Run ID or T0"),
    ("old_watchdog_run_id", "watchdog_ready.json Run ID or T0"),
    ("old_supervisor_run_id", "supervisor_state.json Run ID or T0"),
    ("unknown_file", "unexpected file"),
    ("terminal_receipt", "terminal receipt"),
    ("corrupt_json", "invalid JSON"),
    ("missing_field", "wrapper PID/start-time"),
    ("path_mismatch", "map profile/path"),
    ("identity_mismatch", "supervisor_state.json PID/start-time"),
])
def test_run_directory_reuse_or_credential_mismatch_fails_closed(tmp_path, mutation, match):
    run_dir, args, identities = _initial_run_directory_fixture(tmp_path)
    if mutation == "old_wrapper_run_id":
        data = json.loads((run_dir / "wrapper.json").read_text())
        data["run_id"] = "rviz-nav-20261020T000000Z-deadbeef"
        (run_dir / "wrapper.json").write_text(json.dumps(data))
    elif mutation == "old_watchdog_run_id":
        data = json.loads((run_dir / "watchdog_ready.json").read_text())
        data["run_id"] = "rviz-nav-20261020T000000Z-deadbeef"
        (run_dir / "watchdog_ready.json").write_text(json.dumps(data))
    elif mutation == "old_supervisor_run_id":
        data = json.loads((run_dir / "supervisor_state.json").read_text())
        data["run_id"] = "rviz-nav-20261020T000000Z-deadbeef"
        (run_dir / "supervisor_state.json").write_text(json.dumps(data))
    elif mutation == "unknown_file":
        (run_dir / "unexpected.txt").write_text("not an initialization credential")
    elif mutation == "terminal_receipt":
        (run_dir / "result.json").write_text("{}\n")
    elif mutation == "corrupt_json":
        (run_dir / "watchdog_ready.json").write_text("{not json\n")
    elif mutation == "missing_field":
        data = json.loads((run_dir / "watchdog_ready.json").read_text())
        data.pop("wrapper_start_ticks")
        (run_dir / "watchdog_ready.json").write_text(json.dumps(data))
    elif mutation == "path_mismatch":
        data = json.loads((run_dir / "wrapper.json").read_text())
        data["world_argument"] = "/different.world"
        (run_dir / "wrapper.json").write_text(json.dumps(data))
    elif mutation == "identity_mismatch":
        data = json.loads((run_dir / "supervisor_state.json").read_text())
        data["supervisor_start_ticks"] += 1
        (run_dir / "supervisor_state.json").write_text(json.dumps(data))
    with pytest.raises(policy.ContractError, match=match):
        _validate_fixture(run_dir, args, identities)


def test_run_directory_rejects_wrong_live_watchdog_identity(tmp_path):
    run_dir, args, identities = _initial_run_directory_fixture(tmp_path)
    identities[30] = {**identities[30], "session_id": 99}
    with pytest.raises(policy.ContractError, match="watchdog PID/session/process-group"):
        _validate_fixture(run_dir, args, identities)


def test_watchdog_terminal_receipt_preserves_pre_admission_cause_without_cleanup_pass(
        tmp_path, monkeypatch):
    run_dir, args, _ = _initial_run_directory_fixture(tmp_path)
    test_t0_ns = time.monotonic_ns() - 1_000_000
    supervisor_pid, supervisor_ticks = 400, 400
    wrapper_pid, wrapper_ticks = 20, 200
    watchdog_identity = watchdog_module.process_identity(os.getpid())
    monkeypatch.setattr(watchdog_module.os, "getpgrp", lambda: os.getpid())
    monkeypatch.setattr(watchdog_module.os, "getsid", lambda _pid: os.getpid())
    state = json.loads((run_dir / "supervisor_state.json").read_text())
    state["supervisor_pid"] = supervisor_pid
    state["supervisor_start_ticks"] = supervisor_ticks
    state["t0_monotonic_ns"] = test_t0_ns
    (run_dir / "supervisor_state.json").write_text(json.dumps(state))
    failure = {
        "schema_version": 1, "run_id": args.run_id, "run_dir": str(run_dir.resolve()),
        "t0_monotonic_ns": test_t0_ns, "status": "REJECTED_BEFORE_ADMISSION",
        "failure_stage": "run_directory_admission", "reason": "unexpected file(s): stray.txt",
        "launch_started": False, "launch_pid": None, "launch_pgid": None,
        "supervisor_pid": supervisor_pid, "supervisor_start_ticks": supervisor_ticks,
        "wrapper_pid": wrapper_pid, "wrapper_start_ticks": wrapper_ticks,
        "watchdog_pid": os.getpid(), "watchdog_start_ticks": watchdog_identity["start_ticks"],
    }
    (run_dir / "supervisor_failure.json").write_text(json.dumps(failure))
    args.t0_ns = test_t0_ns
    watchdog = watchdog_module.Watchdog(
        run_dir=run_dir, run_id=args.run_id, t0_ns=test_t0_ns,
        wrapper_pid=wrapper_pid, wrapper_start_ticks=wrapper_ticks,
        supervisor_pid=supervisor_pid, supervisor_start_ticks=supervisor_ticks,
        clock_ns=lambda: time.monotonic_ns(), sleeper=lambda _seconds: None,
        get_identity=lambda _pid: None,
        snapshot=lambda _pid: {"status": "NOT_STARTED", "scan_complete": True,
                               "members": [], "running_members": [],
                               "zombie_members": [], "errors": []},
    )
    result_code = watchdog.run(poll_interval_s=0)
    assert result_code == 1
    hard = json.loads((run_dir / "hard_deadline.json").read_text())
    result = json.loads((run_dir / "result.json").read_text())
    cleanup = json.loads((run_dir / "cleanup.json").read_text())
    assert hard["first_failure"]["reason"] == failure["reason"]
    assert hard["failure_stage"] == "run_directory_admission"
    assert result["first_failure"]["reason"] == failure["reason"]
    assert result["launch_started"] is False
    assert result["status"] == "FAIL" and result["cleanup_verified"] is False
    assert cleanup["status"] == "NOT_STARTED"
    assert cleanup["cleanup_ok"] is None
    assert cleanup["launch_main_reaped"] == "NOT_APPLICABLE"


def _run_outer_shell_preflight(*, inject_unknown_file):
    ros_setup = Path("/opt/ros/humble/setup.bash")
    if not ros_setup.is_file():
        pytest.skip("ROS Humble setup is unavailable for the shell-entry integration test")
    deliverables = runtime.ROOT / "cleannav-deliverables"
    scratch = Path(tempfile.mkdtemp(prefix=".rviz-run-dir-contract-", dir=deliverables))
    try:
        overlay = scratch / "test-overlay.bash"
        test_bin = scratch / "test-bin"
        if inject_unknown_file:
            test_bin.mkdir()
            shim = test_bin / "python3"
            shim.write_text("""#!/usr/bin/env bash
set -eo pipefail
args=("$@")
if [[ "${args[0]:-}" == */navigation_supervisor.py ]]; then
  record_mode=false
  run_dir=
  expect_run_dir=false
  for arg in "${args[@]}"; do
    if [[ "$expect_run_dir" == true ]]; then run_dir=$arg; expect_run_dir=false; continue; fi
    [[ "$arg" == --run-dir ]] && expect_run_dir=true
    [[ "$arg" == --record-* ]] && record_mode=true
  done
  if [[ "$record_mode" == false && -n "$run_dir" ]]; then
    : > "$run_dir/injected-unknown-entry.txt"
  fi
fi
exec "$CLEANNAV_TEST_REAL_PYTHON" "${args[@]}"
""")
            shim.chmod(0o755)
            overlay.write_text(
                "export ROS_DISTRO=humble ROS_DOMAIN_ID=204 ROS_LOCALHOST_ONLY=1\n"
                "export PATH=" + shlex.quote(str(test_bin)) + ":$PATH\n")
        else:
            overlay.write_text(
                ": # test-only environment fixture; no ROS nodes are started\n"
                "export ROS_DISTRO=humble ROS_DOMAIN_ID=204 ROS_LOCALHOST_ONLY=1\n")
        output_root = scratch / "runs"
        env = dict(os.environ)
        for key in ("ROS_DISTRO", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY",
                    *supervisor.PREFIX_ENVIRONMENT_KEYS, "CLEANNAV_RVIZ_OVERLAY_SETUP",
                    "CLEANNAV_RVIZ_OUTPUT_ROOT", "CLEANNAV_TEST_REAL_PYTHON"):
            env.pop(key, None)
        env["CLEANNAV_RVIZ_OUTPUT_ROOT"] = str(output_root)
        env["CLEANNAV_RVIZ_OVERLAY_SETUP"] = str(overlay)
        if inject_unknown_file:
            env["CLEANNAV_TEST_REAL_PYTHON"] = sys.executable
        completed = subprocess.run(
            ["bash", str(HERE / "run_rviz_navigation.sh"),
             "--map-profile", "mcity-localization07",
             "--map-directory", "/not-used/map", "--world", "/not-used/world.sdf"],
            cwd=supervisor.NAV, env=env, capture_output=True, text=True, timeout=20,
        )
        runs = list(output_root.glob("rviz-nav-*") if output_root.exists() else [])
        assert len(runs) == 1, completed.stderr + completed.stdout
        run_dir = runs[0]
        wrapper_receipt = json.loads((run_dir / "wrapper.json").read_text())
        assert wrapper_receipt["motion_profile"] == "none"
        assert wrapper_receipt["timing_profile"] == "standard"
        names = {path.name for path in run_dir.iterdir()}
        assert {"wrapper.json", "environment-setup.log", "watchdog_ready.json",
                "supervisor_state.json", "watchdog.log", "supervisor.log"} <= names, (
                    f"entries={sorted(names)}\n"
                    f"result={((run_dir / 'result.json').read_text() if (run_dir / 'result.json').is_file() else 'missing')}\n"
                    f"stdout={completed.stdout}\nstderr={completed.stderr}")
        result = json.loads((run_dir / "result.json").read_text())
        events = json.loads((run_dir / "events.json").read_text())["events"]
        cleanup = json.loads((run_dir / "cleanup.json").read_text())
        assert result["launch_started"] is False
        assert not (run_dir / "launch.log").exists()
        if inject_unknown_file:
            assert any("unexpected file(s): injected-unknown-entry.txt" in item
                       for item in result["failures"])
            assert result["failure_stage"] == "run_directory_admission"
            assert result["status"] == "FAIL"
            assert cleanup["status"] == "NOT_STARTED" and cleanup["cleanup_ok"] is None
            assert cleanup["launch_main_reaped"] == "NOT_APPLICABLE"
            assert (run_dir / "supervisor_failure.json").is_file()
            assert (run_dir / "hard_deadline.json").is_file()
            assert json.loads((run_dir / "watchdog_exit.json").read_text())["failure_stage"] \
                == "run_directory_admission"
            assert completed.returncode == 124
        else:
            assert any(item.get("event") == "preflight_started" for item in events), (
                f"result={result}\nevents={events}\nsupervisor_log="
                f"{((run_dir / 'supervisor.log').read_text() if (run_dir / 'supervisor.log').is_file() else 'missing')}\n"
                f"watchdog={((run_dir / 'hard_deadline.json').read_text() if (run_dir / 'hard_deadline.json').is_file() else 'missing')}")
            assert result["status"] == "BLOCKED"
            assert cleanup["status"] == "NOT_STARTED" and cleanup["cleanup_ok"] is None
            assert (run_dir / "supervisor_complete.json").is_file()
            assert completed.returncode == 1
            assert "CLEANNAV_EXPECTED_" in " ".join(result["failures"])
        hashes = {line.split("  ", 1)[1]: line.split("  ", 1)[0]
                  for line in (run_dir / "SHA256SUMS").read_text().splitlines()}
        for name in ("result.json", "cleanup.json", "environment.json", "source_identity.json",
                     "events.json", "watchdog_exit.json"):
            assert hashes[name] == hashlib.sha256((run_dir / name).read_bytes()).hexdigest()
        return completed.returncode
    finally:
        shutil.rmtree(scratch)


def test_outer_shell_entry_accepts_real_startup_file_order_before_safe_preflight_rejection():
    _run_outer_shell_preflight(inject_unknown_file=False)


def test_outer_shell_entry_reports_unknown_file_through_watchdog_terminal_receipt():
    _run_outer_shell_preflight(inject_unknown_file=True)



def test_actual_refresh_after_blocking_operation_and_real_writer_gap(tmp_path):
    import threading
    writer=safety_guard.ReceiptWriter(tmp_path/'safety-guard.json','test-run')
    prior=writer.write(dict(monotonic=time.monotonic(),failed=False))
    done=threading.Event()
    def write_live():
        while not done.wait(.1):
            writer.write(dict(monotonic=time.monotonic(),failed=False))
    thread=threading.Thread(target=write_live);thread.start()
    try:
        time.sleep(.62)  # represent a blocking graph/service operation
        latest,status=supervisor.refresh_guard_receipt(tmp_path/'safety-guard.json',prior)
        assert status['previous_status']=='FAIL'
        assert status['latest_status']=='PASS'
        assert latest['receipt_sequence']>prior['receipt_sequence']
        assert status['latest_sequence']==latest['receipt_sequence']
        assert status['read_finished_monotonic_ns']>=status['read_started_monotonic_ns']
    finally:
        done.set();thread.join()
    time.sleep(.52)  # genuine writer gap remains unhealthy
    _,status=supervisor.refresh_guard_receipt(tmp_path/'safety-guard.json',latest)
    assert status['latest_status']=='FAIL'
    assert not status['usable_latest_receipt']
    writer.close()


@pytest.mark.parametrize('missing',['receipt','scan','cm','localization','tf'])
def test_no_motion_unknown_observations_continue_but_do_not_grant_ready(monkeypatch,missing):
    monkeypatch.setattr(policy,'LOCALIZATION_DIAGNOSTIC',True)
    guard=complete_guard(10.)
    guard.update(diagnostic_mode=True,failed=False,
                 diagnostic_continuation={'allowed':True,'hard_failures':[]})
    if missing=='receipt': guard['monotonic']=9.0
    else:
        key={'scan':'/scan','cm':'cm_active','localization':'localization','tf':'/scan_map_tf'}[missing]
        guard['facts'][key]=False
    decision=policy.diagnostic_readiness_loss_decision('dual_lidar_tf',guard,0,
        now_monotonic=10.1,no_motion_contract=True)
    assert decision['allowed']
    if missing=='receipt':
        assert not decision['guard_receipt_fresh']
        assert decision['health_readiness']=='FAIL'
    assert not policy.diagnostic_readiness_loss_decision('dual_lidar_tf',guard,0,
        now_monotonic=10.1,no_motion_contract=False)['allowed']


@pytest.mark.parametrize('hard_failure',['nonzero','publisher','authority','estop','cm_process_death','displacement'])
def test_explicit_hard_failure_stops_even_with_stale_receipt(monkeypatch,hard_failure):
    monkeypatch.setattr(policy,'LOCALIZATION_DIAGNOSTIC',True)
    state=dict(monotonic=1.,diagnostic_mode=True,failed=False,
               diagnostic_continuation={'allowed':False,'hard_failures':[hard_failure]})
    assert not policy.diagnostic_readiness_loss_decision('safety',state,0,
        now_monotonic=10.,no_motion_contract=True)['allowed']


def test_diagnostic_four_gib_emergency_floor_still_stops():
    d=policy.runtime_memory_decision(resource_snapshot(4*1024*1024-1),
        timing_profile='localization-diagnostic',no_motion_contract=True)
    assert not d['ok']
    assert 'MemAvailable_below_diagnostic_4GiB_emergency_floor' in d['reasons']


def test_actual_success_receipt_builder_and_atomic_persistence(tmp_path):
    record=supervisor.startup_verification_record(run_id='regression-success',
        launch_elapsed_s=112.5,observation_elapsed=10.1,
        phases=dict.fromkeys(policy.PHASES,'PASS'),
        launch_mode={'enable_motion':False,'auto_start_task':False},
        command_samples=100,nonzero_samples=0,distance_m=.005)
    supervisor.atomic_json(tmp_path/'verified-no-motion-startup.json',record)
    saved=json.loads((tmp_path/'verified-no-motion-startup.json').read_text())
    assert saved['status']=='PASS'
    assert saved['max_no_motion_displacement_m']==policy.MAX_NO_MOTION_DISPLACEMENT_M
    assert saved['continuous_all_ready_observation_s']>=10
    assert saved['authorization_scope_lease_created'] is False
    assert saved['rviz_ros_process_ready'] is True


def _snapshot_sync_inputs():
    from types import SimpleNamespace as N
    facts={k:True for k in ('/scan','/mapping/scan','/scan_tf','/mapping/scan_tf','clock','odom',
        'safety','cmd','cm','safety_no_estop','cm_active','cmd_publisher',
        '/cleannav/navigation/cmd_vel_raw_publisher','/cleannav/cmd_vel_candidate_publisher',
        '/cmd_vel_authorized_publisher','localization')}
    safety=dict(owner='ros-execution-1',enabled=True,block_all=False,estop=False,monotonic_ns=10_000_000_000,stamp_ns=10_000_000_000)
    guard=dict(monotonic=10.,failed=False,ready=True,missing=[],facts=facts,
               safety=safety,distance_m=0.,receipt_sequence=7)
    observer=N(safety_state=dict(safety,owner='',enabled=False),nonzero_samples=0,
               last_command={'nonzero':False},rviz_goal_stamp_ns=9_900_000_000,command_last={})
    return guard,observer


def _run_actual_snapshot_wait(guard, observer, update):
    import ast
    from types import SimpleNamespace as N
    import supervisor_policy as p
    path=Path(__file__).resolve().parents[1]/'navigation_supervisor.py'
    tree=ast.parse(path.read_text())
    block=next(x for x in ast.walk(tree) if isinstance(x,ast.If)
        and isinstance(x.test,ast.Name) and x.test.id=='historical_motion'
        and x.body and isinstance(x.body[0],ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=='safety_sync_started' for t in x.body[0].targets))
    clock=N(now=10.1)
    def spin(_observer,timeout_sec):
        clock.now+=.01
        update(observer,clock)
    events=[]
    scope=dict(args=N(motion_profile="historical"),MAX_HISTORICAL_ODOMETRY_M=p.MAX_HISTORICAL_ODOMETRY_M,
        historical_motion=True,patrol_motion=False,time=N(monotonic=lambda:clock.now),
        guard=guard,observer=observer,ros=N(spin_once=spin),ContractError=p.ContractError,
        validate_motion_guard_runtime_state=p.validate_motion_guard_runtime_state,
        event=lambda name,**kw:events.append((name,kw)),guard_path=None,
        refresh_guard_receipt=lambda path,previous:(dict(previous,monotonic=clock.now),{}))
    exec(compile(ast.Module(body=[block],type_ignores=[]),'<actual-supervisor-snapshot-wait>','exec'),scope)
    return events


def test_actual_supervisor_waits_zero_until_observer_matches_receipt():
    guard,observer=_snapshot_sync_inputs()
    def update(o,clock):
        assert o.nonzero_samples==0 and not o.last_command['nonzero']
        o.safety_state=dict(guard['safety'],monotonic_ns=int(clock.now*1e9))
    events=_run_actual_snapshot_wait(guard,observer,update)
    assert any(x[0]=='safety_snapshot_wait' for x in events)
    assert events[-1][0]=='safety_snapshot_synchronized'


def test_actual_supervisor_snapshot_wait_times_out():
    import supervisor_policy as p
    guard,observer=_snapshot_sync_inputs()
    with pytest.raises(p.ContractError,match='synchronization timeout'):
        _run_actual_snapshot_wait(guard,observer,lambda o,c:None)


@pytest.mark.parametrize('change',[
    dict(owner='other',enabled=True),dict(estop=True),dict(monotonic_ns=8_000_000_000),
    dict(stamp_ns=9_800_000_000,owner='ros-execution-1',enabled=True),
])
def test_supervisor_snapshot_sync_still_rejects_invalid_state(change):
    import supervisor_policy as p
    guard,observer=_snapshot_sync_inputs();observer.safety_state.update(change)
    with pytest.raises(p.ContractError):
        _run_actual_snapshot_wait(guard,observer,lambda o,c:None)


def test_final_nonzero_during_snapshot_wait_is_rejected_even_if_status_matches():
    import supervisor_policy as p
    guard,observer=_snapshot_sync_inputs()
    def update(o,c):
        o.nonzero_samples+=1;o.safety_state=dict(guard['safety'])
    with pytest.raises(p.ContractError,match='final command is nonzero'):
        _run_actual_snapshot_wait(guard,observer,update)


def _run_actual_raw_callback(observer, x):
    import ast
    from types import SimpleNamespace as N
    import types
    import supervisor_policy as p
    tree=ast.parse((HERE/'navigation_supervisor.py').read_text())
    method=next(v for v in ast.walk(tree) if isinstance(v,ast.FunctionDef) and v.name=='on_command')
    scope=dict(args=N(motion_profile="historical"),time=N(monotonic_ns=lambda:10_100_000_000),historical_motion=True,patrol_motion=False,
        any_motion=True,motion_mode=False,motion_terminal_monotonic=None,failures=[],
        event=lambda *a,**k:None)
    def fail(reason,stage):raise p.ContractError(reason)
    scope['fail']=fail
    exec(compile(ast.Module(body=[method],type_ignores=[]),'<actual-observer-command>','exec'),scope)
    message=N(linear=N(x=x,y=0.,z=0.),angular=N(x=0.,y=0.,z=0.))
    types.MethodType(scope['on_command'],observer)(message,'/cleannav/navigation/cmd_vel_raw')


@pytest.mark.parametrize("speed", [.05,.1935612,.5])
def test_raw_callback_retains_nonzero_until_coherent_lease_validation(speed):
    from collections import defaultdict
    guard,observer=_snapshot_sync_inputs()
    observer.command_counts=defaultdict(int);observer.command_zero_counts=defaultdict(int)
    observer.command_nonzero_counts=defaultdict(int)
    _run_actual_raw_callback(observer,speed)
    _run_actual_raw_callback(observer,0.)
    topic='/cleannav/navigation/cmd_vel_raw'
    assert observer.command_last[topic]['pending_safety_values'][0]==speed
    def update(o,c):o.safety_state=dict(guard['safety'],monotonic_ns=int(c.now*1e9))
    events=_run_actual_snapshot_wait(guard,observer,update)
    assert 'pending_safety_values' not in observer.command_last[topic]
    assert any(x[0]=='command_safety_check_synchronized' for x in events)


def test_pending_raw_does_not_pass_when_both_snapshots_still_idle():
    import supervisor_policy as p
    guard,observer=_snapshot_sync_inputs();guard['safety']=dict(observer.safety_state)
    observer.command_last={'/cleannav/navigation/cmd_vel_raw':{'pending_safety_values':[.05,0.,0.,0.,0.,0.]}}
    with pytest.raises(p.ContractError,match='synchronization timeout'):
        _run_actual_snapshot_wait(guard,observer,lambda o,c:None)


@pytest.mark.parametrize('safety', [None,
    dict(owner='other',enabled=True,block_all=False,estop=False,monotonic_ns=10_000_000_000),
    dict(owner='',enabled=False,block_all=False,estop=True,monotonic_ns=10_000_000_000)])
def test_raw_callback_does_not_defer_unknown_wrong_owner_or_estop(safety):
    from collections import defaultdict
    import supervisor_policy as p
    _,observer=_snapshot_sync_inputs();observer.safety_state=safety
    observer.command_counts=defaultdict(int);observer.command_zero_counts=defaultdict(int)
    observer.command_nonzero_counts=defaultdict(int)
    with pytest.raises(p.ContractError):_run_actual_raw_callback(observer,.05)


def test_goal_callback_delay_does_not_make_current_source_status_an_old_lease():
    guard,observer=_snapshot_sync_inputs()
    # The Safety callback arrived before delayed Goal callback handling, but
    # both messages' ROS source stamps prove the Lease state is current.
    guard['safety']['monotonic_ns']=10_000_000_000
    observer.safety_state=dict(guard['safety'])
    observer.rviz_goal_stamp_ns=9_900_000_000
    _run_actual_snapshot_wait(guard,observer,lambda o,c:None)


def test_watchdog_completion_written_between_head_read_and_exit_check(tmp_path):
    sent = []
    def exited(_pid):
        (tmp_path / 'supervisor_complete.json').write_text('{"status":"FAIL"}')
        return None
    watchdog = watchdog_module.Watchdog(
        run_dir=tmp_path, run_id='rviz-nav-20261023T000000Z-deadbeef', t0_ns=0,
        supervisor_pid=222, supervisor_start_ticks=22,
        launch_pid=333, launch_start_ticks=33,
        clock_ns=lambda: 1, sleeper=lambda _seconds: None,
        get_identity=exited,
        snapshot=lambda _pgid: {'status':'EMPTY','scan_complete':True,
                               'members':[], 'running_members':[], 'zombie_members':[], 'errors':[]},
        kill_pid=lambda *args: sent.append(args), kill_group=lambda *args: sent.append(args))
    assert watchdog.run() == 0
    assert sent == []
    assert json.loads((tmp_path/'watchdog_exit.json').read_text())['status'] == 'NORMAL_COMPLETION'
    assert json.loads((tmp_path/'supervisor_complete.json').read_text())['status'] == 'FAIL'


@pytest.mark.parametrize('diagnostic,soft_stop,maximum,elapsed_s,expected', [
    (True,245.,195.,43.175923587,195.),
    (True,245.,195.,55.,190.),
    (False,335.,250.,43.175923587,250.),
    (False,180.,145.,10.,135.),
])
def test_launch_budget_does_not_reserve_diagnostic_cleanup_twice(
        diagnostic, soft_stop, maximum, elapsed_s, expected):
    import ast
    tree = ast.parse((HERE/'navigation_supervisor.py').read_text())
    assignments = [node for node in ast.walk(tree) if isinstance(node, ast.Assign)
                   and any(isinstance(t,ast.Name) and t.id in
                           ('launch_cleanup_reserve','runtime_budget') for t in node.targets)]
    assignments.sort(key=lambda node:node.lineno)
    scope = dict(LOCALIZATION_DIAGNOSTIC=diagnostic, historical_motion=False, MIN_CLEANUP_RESERVE_S=35.,
                 MAX_LAUNCH_RUNTIME_S=maximum, SOFT_STOP_FROM_T0_S=soft_stop,
                 elapsed=lambda:elapsed_s)
    exec(compile(ast.Module(body=assignments,type_ignores=[]),'<actual launch budget>','exec'),scope)
    assert scope['runtime_budget'] == pytest.approx(expected)
    if diagnostic:
        assert scope['runtime_budget'] >= 180.
        assert elapsed_s+scope['runtime_budget'] <= soft_stop


def test_observer_ready_callback_batch_prevents_command_backlog_starving_safety():
    import ast
    tree = ast.parse((HERE/'navigation_supervisor.py').read_text())
    assignment = next(node for node in ast.walk(tree) if isinstance(node,ast.Assign)
                      and any(isinstance(t,ast.Name) and t.id=='observer_spin_deadline' for t in node.targets))
    loop = next(node for node in ast.walk(tree) if isinstance(node,ast.While)
                and isinstance(node.test,ast.Compare) and isinstance(node.test.comparators[0],ast.Name)
                and node.test.comparators[0].id=='observer_spin_deadline')
    now = [0.]
    queue = ['command']*15+['fresh_safety']
    observed = []
    class Clock:
        @staticmethod
        def monotonic():
            now[0] += .0005
            return now[0]
    class Ros:
        @staticmethod
        def spin_once(observer,timeout_sec):
            assert timeout_sec==0.0
            if queue: observed.append(queue.pop(0))
    # Old one-callback behavior cannot reach Safety behind the command batch.
    assert queue.pop(0)=='command' and 'fresh_safety' not in observed
    scope=dict(time=Clock,ros=Ros,observer=object())
    exec(compile(ast.Module(body=[assignment,loop],type_ignores=[]),'<actual callback drain>','exec'),scope)
    assert observed[-1]=='fresh_safety'
    assert now[0] <= .022


def test_integrated_safety_status_cadence_has_margin_inside_guard_freshness():
    import ast
    tree=ast.parse((HERE.parents[1]/'cleannav_simulation/launch/cleannav_full_sim.launch.py').read_text())
    keyword=next(node for node in ast.walk(tree) if isinstance(node,ast.keyword)
                 and node.arg=='status_publish_frequency')
    expr=ast.Expression(body=keyword.value)
    assert eval(compile(expr,'<actual status rate>','eval'),{'integration':True})=='10.0'
    assert eval(compile(expr,'<actual status rate>','eval'),{'integration':False})=='2.0'
    # Consumer bound stays strict; this test does not increase it.
    from guard_policy import command_error
    safety=dict(monotonic_ns=0,block_all=False,estop=False,enabled=True,owner='ros-execution-1')
    assert command_error([.1,0,0,0,0,0],True,safety,now_ns=500_000_001)=='Safety state stale or future dated'


@pytest.mark.parametrize('launch_elapsed', [1., 55.])
@pytest.mark.parametrize('wait', [300., 600.])
def test_historical_child_timer_covers_latest_readiness_gui_and_task(launch_elapsed,wait):
    import ast
    tree=ast.parse((HERE/'navigation_supervisor.py').read_text())
    assignments=[n for n in ast.walk(tree) if isinstance(n,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id in ('launch_cleanup_reserve','runtime_budget')
                for t in n.targets)]
    assignments.sort(key=lambda n:n.lineno)
    soft=205+wait+policy.HISTORICAL_TASK_TIMEOUT_S+10
    scope=dict(LOCALIZATION_DIAGNOSTIC=False,historical_motion=True,
        MIN_CLEANUP_RESERVE_S=35.,MAX_LAUNCH_RUNTIME_S=soft,
        SOFT_STOP_FROM_T0_S=soft,elapsed=lambda:launch_elapsed)
    exec(compile(ast.Module(body=assignments,type_ignores=[]),'<actual-child-budget>','exec'),scope)
    assert launch_elapsed+scope['runtime_budget']==soft
    assert launch_elapsed+scope['runtime_budget']>=205+wait+120+10


@pytest.mark.parametrize('distance', [2.01, 2.51, 8.0, 11.99])
def test_historical_accumulated_distance_above_old_bounds_is_valid(distance):
    guard, _ = _snapshot_sync_inputs()
    guard['distance_m'] = distance
    policy.validate_motion_guard_runtime_state(guard, now_monotonic=10.1,
        motion_admitted=True, mission_lease_active=True,
        maximum_distance_m=policy.MAX_HISTORICAL_ODOMETRY_M)


@pytest.mark.parametrize('fault', ['localization', 'cm_active', 'safety', 'stale', 'owner', 'estop'])
def test_historical_distance_contract_preserves_other_hard_stops(fault):
    guard, _ = _snapshot_sync_inputs()
    guard['distance_m'] = 8.0
    if fault in ('localization', 'cm_active', 'safety'):
        guard['facts'][fault] = False
    elif fault == 'stale':
        guard['monotonic'] = 9.0
    elif fault == 'owner':
        guard['safety']['owner'] = 'old-execution'
    else:
        guard['safety']['estop'] = True
    with pytest.raises(policy.ContractError):
        policy.validate_motion_guard_runtime_state(guard, now_monotonic=10.1,
            motion_admitted=True, mission_lease_active=True,
            maximum_distance_m=policy.MAX_HISTORICAL_ODOMETRY_M)


def test_historical_accumulated_distance_receipt_limit_stops():
    guard, _ = _snapshot_sync_inputs()
    guard['distance_m'] = 12.01
    with pytest.raises(policy.ContractError, match='12 m'):
        policy.validate_motion_guard_runtime_state(guard, now_monotonic=10.1,
            motion_admitted=True, mission_lease_active=True,
            maximum_distance_m=policy.MAX_HISTORICAL_ODOMETRY_M)

@pytest.mark.parametrize('receipt,stop', [
    (None, False),
    ({'status': 'RUNNING'}, False),
    ({'status': 'ACTIVE_CONFIRMED', 'current_state': 3}, False),
    ({'status': 'FAILED_CLOSED', 'error': 'initial lifecycle state is unknown: None'}, True),
])
def test_map_coordinator_terminal_receipt_stops_without_readiness_deadline(receipt, stop):
    import ast
    tree = ast.parse((HERE/'navigation_supervisor.py').read_text())
    check = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
                 and 'map_coordinator.get' in ast.unparse(n.test))
    calls = []
    def fail(reason, phase):
        calls.append((reason, phase))
        raise policy.ContractError(reason)
    scope = {'map_coordinator': receipt, 'fail': fail}
    code = compile(ast.Module(body=[check], type_ignores=[]), '<actual map failure check>', 'exec')
    if stop:
        with pytest.raises(policy.ContractError, match='initial lifecycle state is unknown'):
            exec(code, scope)
        assert calls[0][1] == 'map_localization'
    else:
        exec(code, scope)
        assert not calls


def _installed_map_coordinator_tree():
    import ast
    import integration
    dependencies = json.loads((HERE/'RUNTIME_DEPENDENCIES.json').read_text())
    # The approved prefix is supplied by the same static contract as integration tests.
    prefixes = json.loads(Path(os.environ['CLEANNAV_STATIC_PREFIXES_JSON']).read_text())
    source = Path(prefixes['cleannav_simulation'])/'lib/cleannav_simulation/campus_map_server_lifecycle_coordinator'
    assert integration.digest(source) == dependencies['cleannav_packages']['cleannav_simulation']['files']['lib/cleannav_simulation/campus_map_server_lifecycle_coordinator']
    return ast.parse(source.read_text())


@pytest.mark.parametrize('outcome,expected', [('ok', 1), ('timeout', None), ('error', None)])
def test_actual_get_state_rpc_remains_bounded_and_records_response(outcome, expected):
    import ast
    from types import SimpleNamespace
    tree = _installed_map_coordinator_tree()
    function = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'call_state')
    emitted = []; removed = []; spins = []
    def result():
        if outcome == 'error': raise RuntimeError('RPC failed')
        return SimpleNamespace(current_state=SimpleNamespace(id=1, label='unconfigured'))
    future = SimpleNamespace(done=lambda: outcome != 'timeout', result=result)
    scope = dict(GetState=SimpleNamespace(Request=lambda: object()),
                 time=SimpleNamespace(monotonic_ns=lambda: 123),
                 state_client=SimpleNamespace(call_async=lambda request: future, remove_pending_request=removed.append),
                 rclpy=SimpleNamespace(spin_until_future_complete=lambda node, fut, timeout_sec: spins.append(timeout_sec)),
                 node=object(), args=SimpleNamespace(state_timeout=2.),
                 recorder=SimpleNamespace(emit=lambda event, **fields: emitted.append(event)))
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<actual installed call_state>', 'exec'), scope)
    assert scope['call_state']() == expected
    assert spins == [2.]
    assert removed == ([future] if outcome == 'timeout' else [])
    assert emitted == [{'ok':'get_state_response','timeout':'get_state_timeout','error':'get_state_error'}[outcome]]


@pytest.mark.parametrize('ready,expect_error', [(True,False),(False,True)])
def test_actual_lifecycle_service_discovery_has_bounded_wait(ready, expect_error):
    import ast
    from types import SimpleNamespace
    tree = _installed_map_coordinator_tree()
    loop = next(n for n in ast.walk(tree) if isinstance(n, ast.While)
                and 'change_client.service_is_ready' in ast.unparse(n))
    now = [0.]; emitted=[]
    def sleep(duration): now[0] += duration
    class Error(RuntimeError): pass
    scope = dict(rclpy=SimpleNamespace(ok=lambda: True), deadline=.3,
                 time=SimpleNamespace(monotonic=lambda:now[0], sleep=sleep),
                 change_client=SimpleNamespace(service_is_ready=lambda:ready),
                 state_client=SimpleNamespace(service_is_ready=lambda:ready),
                 LifecycleActivationError=Error)
    code = compile(ast.Module(body=[loop], type_ignores=[]), '<actual service wait>', 'exec')
    if expect_error:
        with pytest.raises(Error,match='not discovered before deadline'): exec(code,scope)
        assert .3 <= now[0] <= .4
    else:
        exec(code,scope); assert now[0] == 0.


def _motion_receipt(now, stamp, *, block_all=False, estop=False, enabled=False, owner="",
                    writer_pid=4242, run_id="run-a", sequence=1):
    required = (
        "/scan", "/mapping/scan", "/scan_tf", "/mapping/scan_tf", "clock", "odom",
        "safety", "cmd", "cm", "safety_no_estop", "cm_active", "cmd_publisher",
        "/cleannav/navigation/cmd_vel_raw_publisher",
        "/cleannav/cmd_vel_candidate_publisher", "/cmd_vel_authorized_publisher",
        "localization",
    )
    return {
        "ready": True, "failed": False, "monotonic": now, "missing": [],
        "distance_m": 0.0, "run_id": run_id, "writer_pid": writer_pid,
        "receipt_sequence": sequence,
        "facts": {key: True for key in required},
        "safety": {
            "block_all": block_all, "estop": estop, "enabled": enabled, "owner": owner,
            "stamp_ns": stamp, "monotonic_ns": int(now * 1e9),
        },
    }


def _receipt_decision(receipt, confirmed, **overrides):
    fields = dict(now_monotonic=100.0, deadline_monotonic=300.0, expected_run_id="run-a",
                  expected_writer_pid=4242, motion_admitted=True, mission_lease_active=False)
    fields.update(overrides)
    return policy.admitted_safety_receipt_decision(receipt, confirmed, **fields)


def test_receipt_behind_confirmed_unblock_waits_and_then_accepts():
    old = _motion_receipt(100.0, 59_200_000_000, block_all=True)
    confirmed = {"block_all": False, "estop": False, "enabled": False, "owner": "",
                 "stamp_ns": 59_300_000_000}
    waiting = _receipt_decision(old, confirmed)
    assert waiting["action"] == "wait"
    assert waiting["reason"] == "Safety guard receipt has not processed the confirmed status"
    current = _motion_receipt(100.05, 59_300_000_000)
    assert _receipt_decision(current, confirmed, now_monotonic=100.05)["action"] == "accept"


def test_older_receipt_cannot_cover_a_later_safety_sample():
    confirmed = {"block_all": False, "estop": False, "enabled": False, "owner": "",
                 "stamp_ns": 30}
    assert _receipt_decision(_motion_receipt(100.0, 10), confirmed)["action"] == "wait"
    assert _receipt_decision(_motion_receipt(100.0, 20), confirmed)["action"] == "wait"
    assert _receipt_decision(_motion_receipt(100.0, 30), confirmed)["action"] == "accept"
    assert _receipt_decision(_motion_receipt(100.0, 30), {**confirmed, "stamp_ns": 40})["action"] == "wait"


def _idle(stamp, **extra):
    sample = {"block_all": False, "estop": False, "enabled": False, "owner": "",
              "stamp_ns": stamp, "monotonic_ns": stamp}
    sample.update(extra)
    return sample


def test_heartbeat_keeps_one_generation_and_a_matching_receipt_stays_current():
    first = policy.observe_safety_sample(None, _idle(57_100_000_000))
    latest = first
    for stamp in range(57_200_000_000, 58_500_000_000, 100_000_000):
        latest = policy.observe_safety_sample(latest, _idle(stamp))
    assert latest["generation_stamp_ns"] == 57_100_000_000
    assert latest["stamp_ns"] == 58_400_000_000
    receipt = _motion_receipt(100.0, 58_300_000_000)
    admitted = _receipt_decision(receipt, latest, generation_stamp_ns=57_100_000_000)
    assert admitted["action"] == "accept"
    assert admitted["stamp_ns"] == 58_300_000_000
    goal = _receipt_decision(receipt, latest, generation_stamp_ns=57_100_000_000,
                             minimum_receipt_stamp_ns=58_300_000_000)
    assert goal["action"] == "accept"


def test_receipt_from_before_the_current_generation_is_not_accepted():
    current = policy.observe_safety_sample(
        _idle(57_000_000_000, generation_stamp_ns=56_000_000_000, block_all=True),
        _idle(57_100_000_000))
    assert current["generation_stamp_ns"] == 57_100_000_000
    previous_occurrence = _motion_receipt(100.0, 56_500_000_000)
    waiting = _receipt_decision(previous_occurrence, current,
                                generation_stamp_ns=57_100_000_000)
    assert waiting["action"] == "wait"
    covered = _motion_receipt(100.05, 57_100_000_000)
    assert _receipt_decision(covered, current, now_monotonic=100.05,
                             generation_stamp_ns=57_100_000_000)["action"] == "accept"


def test_real_safety_changes_fail_before_an_old_receipt_can_cover_them():
    current = _idle(20, block_all=True)
    blocked = _receipt_decision(_motion_receipt(100.0, 20), current)
    assert blocked["action"] == "fail"
    assert blocked["reason"] == "admitted idle Safety is not unblocked and unleased"
    estop = _receipt_decision(
        _motion_receipt(100.0, 10), _idle(20, estop=True))
    assert estop["reason"] == "Safety status is missing or ESTOP is active"
    owner = _receipt_decision(
        _motion_receipt(100.0, 20), _idle(20, owner="other-owner"))
    assert owner["reason"] == "Safety Lease owner mismatch"
    leased = _receipt_decision(
        _motion_receipt(100.0, 20),
        _idle(20, enabled=True, owner="ros-execution-1"),
        mission_lease_active=False)
    assert leased["reason"] == "admitted idle Safety is not unblocked and unleased"


def test_stale_writer_stopped_and_backward_receipts_fail_closed():
    idle = _idle(20, generation_stamp_ns=10)
    stale = _receipt_decision(_motion_receipt(99.0, 20), idle, now_monotonic=100.0,
                              generation_stamp_ns=10)
    assert stale["reason"].startswith("motion guard receipt is stale or future dated")
    foreign = _receipt_decision(_motion_receipt(100.0, 20, writer_pid=7), idle,
                                generation_stamp_ns=10)
    assert foreign["reason"] == "safety guard receipt process identity rejected"
    stopped = _receipt_decision(_motion_receipt(99.4, 20), idle, now_monotonic=100.0,
                                generation_stamp_ns=10)
    assert stopped["action"] == "fail"
    backward = _receipt_decision(_motion_receipt(100.0, 15), idle,
                                 generation_stamp_ns=10,
                                 minimum_receipt_stamp_ns=20)
    assert backward["reason"] == "Safety guard receipt stamp moved backward"
    repeated = _receipt_decision(_motion_receipt(100.0, 20), _idle(30, generation_stamp_ns=10),
                                 generation_stamp_ns=10, minimum_receipt_stamp_ns=20)
    assert repeated["action"] == "accept"


def test_supervisor_safety_callback_tracks_generation_and_rejects_bad_stamps():
    import ast
    from types import SimpleNamespace as N
    source = ast.parse((HERE / "navigation_supervisor.py").read_text())
    method = next(node for node in ast.walk(source)
                  if isinstance(node, ast.FunctionDef) and node.name == "on_safety"
                  and any(isinstance(item, ast.Assign) and "sample" in ast.unparse(item)
                          for item in node.body))
    errors = []

    def fail(why, stage="supervision"):
        errors.append((why, stage))
        raise policy.ContractError(why)

    scope = dict(fail=fail, time=time, observe_safety_sample=policy.observe_safety_sample,
                 ContractError=policy.ContractError, args=N(run_id="run-a"),
                 event=lambda *args, **kwargs: None)
    exec(compile(ast.Module(body=[method], type_ignores=[]), "<supervisor on_safety>", "exec"), scope)
    observer = N(safety_state=None, safety_owner_history=[], patrol_owner="")
    observer.on_safety = scope["on_safety"].__get__(observer)

    def message(stamp, **fields):
        sec, nanosec = divmod(stamp, 10**9)
        values = dict(block_all=False, emergency_stop_active=False, autonomous_enabled=False,
                      lease_owner_execution_id="")
        values.update(fields)
        return N(**values, header=N(stamp=N(sec=sec, nanosec=nanosec)))

    observer.on_safety(message(57_100_000_000))
    observer.on_safety(message(58_400_000_000))
    assert observer.safety_state["generation_stamp_ns"] == 57_100_000_000
    assert observer.safety_state["stamp_ns"] == 58_400_000_000
    with pytest.raises(policy.ContractError, match="repeated or moved backward"):
        observer.on_safety(message(58_300_000_000))
    assert errors[-1][1] == "safety"
    assert observer.safety_state["stamp_ns"] == 58_400_000_000
    observer.on_safety(message(58_600_000_000, block_all=True, emergency_stop_active=True))
    assert observer.safety_state["generation_stamp_ns"] == 58_600_000_000
    assert observer.safety_state["block_all"] is True and observer.safety_state["estop"] is True


def test_unconfirmed_receipt_fails_at_the_existing_phase_deadline():
    decision = _receipt_decision(
        _motion_receipt(100.0, 10, block_all=True),
        {"block_all": False, "estop": False, "enabled": False, "owner": "", "stamp_ns": 20},
        now_monotonic=100.0, deadline_monotonic=100.0)
    assert decision["action"] == "fail"
    assert decision["reason"] == "safety guard receipt was not confirmed before the phase deadline"


def test_estop_blocks_without_waiting_for_the_receipt():
    decision = _receipt_decision(
        _motion_receipt(100.0, 10),
        {"block_all": False, "estop": True, "enabled": False, "owner": "", "stamp_ns": 20})
    assert decision == {"action": "fail", "reason": "Safety status is missing or ESTOP is active"}
    caught_up = _receipt_decision(
        _motion_receipt(100.0, 20, estop=True),
        {"block_all": False, "estop": False, "enabled": False, "owner": "", "stamp_ns": 20})
    assert caught_up["action"] == "fail"
    assert caught_up["reason"] == "Safety status is missing or ESTOP is active"


def test_block_all_and_owner_and_lease_still_fail_closed():
    blocked = _receipt_decision(
        _motion_receipt(100.0, 20),
        {"block_all": True, "estop": False, "enabled": False, "owner": "", "stamp_ns": 20})
    assert blocked["reason"] == "admitted idle Safety is not unblocked and unleased"
    owned = _receipt_decision(
        _motion_receipt(100.0, 20),
        {"block_all": False, "estop": False, "enabled": False, "owner": "other", "stamp_ns": 20})
    assert owned["reason"] == "Safety Lease owner mismatch"
    leased = _receipt_decision(
        _motion_receipt(100.0, 20, owner="other", enabled=True),
        {"block_all": False, "estop": False, "enabled": True, "owner": "ros-execution-1",
         "stamp_ns": 20},
        mission_lease_active=True, expected_owner="ros-execution-1")
    assert leased["action"] == "fail"
    assert leased["reason"] == "active task does not own the expected Safety Lease"


def test_stale_receipt_and_writer_identity_still_fail():
    stale = _receipt_decision(
        _motion_receipt(99.0, 20),
        {"block_all": False, "estop": False, "enabled": False, "owner": "", "stamp_ns": 20},
        now_monotonic=100.0)
    assert stale["action"] == "fail"
    assert stale["reason"].startswith("motion guard receipt is stale or future dated")
    foreign = _receipt_decision(
        _motion_receipt(100.0, 20, writer_pid=7),
        {"block_all": False, "estop": False, "enabled": False, "owner": "", "stamp_ns": 20},
        expected_writer_pid=4242)
    assert foreign["reason"] == "safety guard receipt process identity rejected"
    wrong_run = _receipt_decision(
        _motion_receipt(100.0, 20, run_id="other-run"),
        {"block_all": False, "estop": False, "enabled": False, "owner": "", "stamp_ns": 20})
    assert wrong_run["reason"] == "safety guard receipt process identity rejected"


def test_receipt_replace_hides_partial_bytes_and_callback_does_not_skip_the_write(tmp_path, monkeypatch):
    import ast
    import threading
    from types import SimpleNamespace as N
    run_id = "rviz-nav-unit"
    monkeypatch.setenv("CLEANNAV_RVIZ_RUN_DIR", str(tmp_path))
    (tmp_path / "motion-admission.json").write_text(json.dumps(
        {"run_id": run_id, "status": "OPENING"}))
    source = ast.parse((HERE / "navigation_safety_guard.py").read_text())
    method = next(node for node in ast.walk(source)
                  if isinstance(node, ast.FunctionDef) and node.name == "on_safety")
    scope = dict(observe_safety_sample=policy.observe_safety_sample,
                 short_goal=True, historical=False, EXPECTED_EXECUTION_ID="ros-execution-1",
                 args=N(run_id=run_id, diagnostic_continue_on_readiness_loss=False),
                 os=os, Path=Path, json=json, time=time)
    exec(compile(ast.Module(body=[method], type_ignores=[]), "<on_safety>", "exec"), scope)
    path = tmp_path / "safety-guard.json"
    writer = safety_guard.ReceiptWriter(path, run_id)
    now = time.monotonic()
    first = writer.write(_motion_receipt(now, 59_200_000_000, block_all=True, run_id=run_id,
                                         writer_pid=os.getpid()))
    writer.flush()
    errors = []
    guard = N(safety=None, failed=None, mission_goal_accepted=False,
              expected_owner="ros-execution-1", receipt_writer=writer, mark=lambda _key: None,
              fail=lambda why: errors.append(why))
    guard.on_safety = scope["on_safety"].__get__(guard)
    message = N(block_all=False, emergency_stop_active=False, autonomous_enabled=False,
                lease_owner_execution_id="",
                header=N(stamp=N(sec=59, nanosec=300_000_000)))
    guard.get_clock = lambda: N(now=lambda: N(nanoseconds=59_300_000_000))
    guard.on_safety(message)
    assert errors == []
    assert guard.safety["stamp_ns"] == 59_300_000_000
    assert guard.safety["block_all"] is False
    on_disk = supervisor.read_json(path)
    assert on_disk["receipt_sequence"] == first["receipt_sequence"]
    assert on_disk["safety"]["stamp_ns"] == 59_200_000_000
    confirmed = {"block_all": False, "estop": False, "enabled": False, "owner": "",
                 "stamp_ns": 59_300_000_000}
    observed_now = on_disk["monotonic"] + 0.05
    assert _receipt_decision(on_disk, confirmed, now_monotonic=observed_now,
                             deadline_monotonic=observed_now + 1.0,
                             expected_run_id=run_id,
                             expected_writer_pid=os.getpid())["action"] == "wait"
    written = writer.write(_motion_receipt(time.monotonic(), 59_300_000_000, run_id=run_id,
                                           writer_pid=os.getpid()))
    writer.flush()
    latest, _status = supervisor.refresh_guard_receipt(path, on_disk, now_monotonic=time.monotonic())
    assert latest["receipt_sequence"] == written["receipt_sequence"]
    accepted_now = latest["monotonic"] + 0.05
    assert _receipt_decision(latest, confirmed, now_monotonic=accepted_now,
                             deadline_monotonic=accepted_now + 1.0,
                             expected_run_id=run_id,
                             expected_writer_pid=os.getpid())["action"] == "accept"
    path.with_suffix(".tmp").write_text("{partial")
    assert supervisor.read_json(path)["receipt_sequence"] == written["receipt_sequence"]
    seen = []
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            value = supervisor.read_json(path)
            if value is not None:
                seen.append((value.get("receipt_sequence"), (value.get("safety") or {}).get("stamp_ns")))

    threads = [threading.Thread(target=reader) for _ in range(4)]
    for thread in threads:
        thread.start()
    for stamp in (59_400_000_000, 59_500_000_000):
        writer.write(_motion_receipt(time.monotonic(), stamp, run_id=run_id, writer_pid=os.getpid()))
    writer.flush()
    stop.set()
    for thread in threads:
        thread.join()
    assert seen
    assert all(sequence >= 1 and isinstance(stamp, int) for sequence, stamp in seen)
    assert supervisor.read_json(path)["safety"]["stamp_ns"] == 59_500_000_000
    writer.close()


def test_short_goal_admission_waits_for_receipt_confirmation_before_admitted_status():
    text = (HERE / "navigation_supervisor.py").read_text()
    opened = text.index('if interlock.get("success") is not True:')
    confirmed = text.index("confirm_admitted_safety_receipt(admitted=True)", opened)
    admitted = text.index('status="ADMITTED"', opened)
    assert opened < confirmed < admitted
