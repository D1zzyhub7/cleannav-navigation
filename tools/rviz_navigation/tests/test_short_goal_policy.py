"""Behavioral tests for run-bound one-goal motion receipts and admission."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import short_goal_policy as motion
import supervisor_policy as policy


RUN_ID = "rviz-nav-20261022T130000Z-abcdef12"
DIGESTS = ("a" * 64, "b" * 64, "c" * 64)


def _receipts(tmp_path: Path):
    return motion.issue_motion_receipts(
        run_id=RUN_ID, run_dir=tmp_path,
        issued_monotonic_ns=1_000_000_000,
        expires_monotonic_ns=1_000_000_000 + int(motion.MAX_AUTHORITY_TTL_S * 1e9),
        source_manifest_sha256=DIGESTS[0], map_contract_sha256=DIGESTS[1],
        world_sha256=DIGESTS[2])


def test_short_goal_profile_and_ttl_cover_gui_wait_task_and_release():
    assert motion.MAX_AUTHORITY_TTL_S == (
        motion.MAX_GUI_GOAL_WAIT_S + motion.MAX_GOAL_RUNTIME_S
        + motion.LEASE_RELEASE_CONFIRMATION_S)
    assert motion.MAX_AUTHORITY_TTL_S == 85.0
    profile = json.loads((HERE / "short_goal_profile.json").read_text())
    validated = motion.validate_profile(profile)
    assert validated["max_forward_linear_x"] == 0.15
    assert validated["max_reverse_linear_x"] == 0.0
    assert validated["reverse_allowed"] is False
    assert validated["max_goal_distance_m"] == 2.0


def test_authorization_and_scope_are_unique_run_bound_and_fail_closed(tmp_path):
    auth, scope, lifecycle = _receipts(tmp_path)
    assert auth["safety_lease_created"] is False
    assert scope["accepted_goal_count"] == 0
    assert scope["maximum_accepted_goals"] == 1
    assert lifecycle["status"] == "AUTHORIZED"
    checked_auth, checked_scope = motion.validate_authority_files(
        run_id=RUN_ID, authorization_path=tmp_path / "motion-authorization.json",
        scope_path=tmp_path / "motion-scope.json", now_monotonic_ns=2_000_000_000)
    assert checked_auth["authorization_id"] == auth["authorization_id"]
    assert checked_scope["scope_id"] == scope["scope_id"]
    with pytest.raises(ValueError, match="identity"):
        motion.validate_authority_files(
            run_id="rviz-nav-other", authorization_path=tmp_path / "motion-authorization.json",
            scope_path=tmp_path / "motion-scope.json", now_monotonic_ns=2_000_000_000)
    with pytest.raises(ValueError, match="expired"):
        motion.validate_authority_files(
            run_id=RUN_ID, authorization_path=tmp_path / "motion-authorization.json",
            scope_path=tmp_path / "motion-scope.json",
            now_monotonic_ns=auth["expires_monotonic_ns"])
    tampered = dict(auth, limits={**auth["limits"], "max_reverse_linear_x_mps": 0.01})
    (tmp_path / "motion-authorization.json").write_text(json.dumps(tampered))
    with pytest.raises(ValueError, match="identity|malformed"):
        motion.validate_authority_files(
            run_id=RUN_ID, authorization_path=tmp_path / "motion-authorization.json",
            scope_path=tmp_path / "motion-scope.json", now_monotonic_ns=2_000_000_000)


def test_scope_and_lifecycle_bind_real_mission_task_lease_and_release(tmp_path):
    _, scope, lifecycle = _receipts(tmp_path)
    auth_path = tmp_path / "motion-authorization.json"
    scope_path = tmp_path / "motion-scope.json"
    lifecycle_path = tmp_path / "motion-lifecycle.json"
    now = 2_000_000_000
    command_id = "rviz-short-0123456789abcdef"
    task_id = 7

    scope = motion.update_scope(scope_path, expected_run_id=RUN_ID,
        expected_profile_id=motion.PROFILE_ID, updates={
            "status": "GOAL_ACCEPTED", "accepted_goal_count": 1,
            "accepted_command_id": command_id, "accepted_task_id": task_id})
    lifecycle = motion.update_lifecycle(lifecycle_path, expected_run_id=RUN_ID,
        expected_profile_id=motion.PROFILE_ID, updates={
            "status": "GOAL_ACCEPTED", "accepted_goal_count": 1,
            "command_id": command_id, "task_id": task_id,
            "scope_sha256": motion.sha256(scope_path)})
    assert motion.validate_lifecycle(run_id=RUN_ID, lifecycle_path=lifecycle_path,
        authorization_path=auth_path, scope_path=scope_path,
        now_monotonic_ns=now)["task_id"] == task_id

    scope = motion.update_scope(scope_path, expected_run_id=RUN_ID,
        expected_profile_id=motion.PROFILE_ID, updates={
            "status": "LEASE_ACTIVE", "lease_owner_execution_id": motion.EXPECTED_EXECUTION_ID,
            "lease_acquired_monotonic_ns": now})
    lifecycle = motion.update_lifecycle(lifecycle_path, expected_run_id=RUN_ID,
        expected_profile_id=motion.PROFILE_ID, updates={
            "status": "LEASE_ACTIVE", "lease_owner_execution_id": motion.EXPECTED_EXECUTION_ID,
            "lease_acquired_monotonic_ns": now, "scope_sha256": motion.sha256(scope_path)})
    with pytest.raises(ValueError, match="release"):
        motion.close_lifecycle(lifecycle_path, expected_run_id=RUN_ID,
            expected_profile_id=motion.PROFILE_ID, monotonic_ns=now + 1,
            release_reason="test", lease_released=False)
    with pytest.raises(ValueError, match="Lease release"):
        motion.update_scope(scope_path, expected_run_id=RUN_ID,
            expected_profile_id=motion.PROFILE_ID, updates={
                "status": "CLOSED", "lease_owner_execution_id": None})

    scope = motion.update_scope(scope_path, expected_run_id=RUN_ID,
        expected_profile_id=motion.PROFILE_ID, updates={
            "status": "CLOSED", "lease_owner_execution_id": None,
            "lease_released_monotonic_ns": now + 10, "closed_monotonic_ns": now + 10,
            "release_reason": "Mission task terminal and Safety owner cleared"})
    lifecycle = motion.update_lifecycle(lifecycle_path, expected_run_id=RUN_ID,
        expected_profile_id=motion.PROFILE_ID, updates={
            "status": "LEASE_RELEASED", "lease_owner_execution_id": None,
            "lease_released_monotonic_ns": now + 10,
            "scope_sha256": motion.sha256(scope_path)})
    lifecycle = motion.close_lifecycle(lifecycle_path, expected_run_id=RUN_ID,
        expected_profile_id=motion.PROFILE_ID, monotonic_ns=now + 11,
        release_reason="Mission task terminal and Safety owner cleared", lease_released=True)
    receipt = motion.write_revocation_receipt(run_id=RUN_ID,
        profile_id=motion.PROFILE_ID, run_dir=tmp_path, authorization_path=auth_path,
        scope_path=scope_path, lifecycle_path=lifecycle_path, monotonic_ns=now + 12,
        release_reason=lifecycle["release_reason"], lease_acquired=True, lease_released=True)
    verified = motion.validate_revocation_receipt(run_id=RUN_ID,
        authorization_path=auth_path, scope_path=scope_path,
        lifecycle_path=lifecycle_path, receipt_path=tmp_path / "motion-revocation.json",
        lease_acquired=True, lease_released=True)
    assert receipt == verified
    assert verified["status"] == "REVOKED"


def test_unleased_admission_failure_can_close_scope_but_not_forge_lease_release(tmp_path):
    _, _, lifecycle = _receipts(tmp_path)
    scope_path = tmp_path / "motion-scope.json"
    lifecycle_path = tmp_path / "motion-lifecycle.json"
    auth_path = tmp_path / "motion-authorization.json"
    scope = motion.update_scope(scope_path, expected_run_id=RUN_ID,
        expected_profile_id=motion.PROFILE_ID, updates={
            "status": "CLOSED", "closed_monotonic_ns": 3_000_000_000,
            "release_reason": "no GUI goal before bounded timeout"})
    closed = motion.close_lifecycle(lifecycle_path, expected_run_id=RUN_ID,
        expected_profile_id=motion.PROFILE_ID, monotonic_ns=3_000_000_001,
        release_reason="no GUI goal before bounded timeout", lease_released=False)
    receipt = motion.write_revocation_receipt(run_id=RUN_ID,
        profile_id=motion.PROFILE_ID, run_dir=tmp_path, authorization_path=auth_path,
        scope_path=scope_path, lifecycle_path=lifecycle_path, monotonic_ns=3_000_000_002,
        release_reason=closed["release_reason"], lease_acquired=False, lease_released=False)
    assert scope["status"] == closed["status"] == "CLOSED"
    assert receipt["lease_acquired"] is False and receipt["lease_released"] is False
    with pytest.raises(ValueError, match="closed or mismatched"):
        motion.update_scope(scope_path, expected_run_id=RUN_ID,
            expected_profile_id=motion.PROFILE_ID, updates={"status": "ACTIVE"})


def test_goal_bounds_require_nearby_forward_known_free_map_point():
    accepted = motion.validate_forward_goal(goal_x=1.2, goal_y=0.0, goal_yaw=0.0,
        base_x=0.0, base_y=0.0, base_yaw=0.0, map_free=True)
    assert accepted["distance_m"] == pytest.approx(1.2)
    for goal_x, goal_y, free in ((2.1, 0.0, True), (-1.0, 0.0, True), (1.0, 0.0, False)):
        with pytest.raises(ValueError):
            motion.validate_forward_goal(goal_x=goal_x, goal_y=goal_y, goal_yaw=0.0,
                base_x=0.0, base_y=0.0, base_yaw=0.0, map_free=free)


def test_command_policy_requires_admission_goal_and_actual_mission_lease():
    moving = [0.1, 0, 0, 0, 0, 0.02]
    assert motion.command_policy(moving, safety=None, admitted=False, goal_seen=False)
    assert motion.command_policy(moving,
        safety={"block_all": False, "estop": False, "enabled": True,
                "owner": motion.EXPECTED_EXECUTION_ID},
        admitted=True, goal_seen=False)
    assert motion.command_policy(moving,
        safety={"block_all": False, "estop": False, "enabled": True,
                "owner": "other-execution"}, admitted=True, goal_seen=True)
    assert motion.command_policy([-0.01, 0, 0, 0, 0, 0], safety=None,
                                 admitted=True, goal_seen=True)
    assert motion.command_policy([0.151, 0, 0, 0, 0, 0], safety=None,
                                 admitted=True, goal_seen=True)
    assert motion.command_policy(moving,
        safety={"block_all": False, "estop": False, "enabled": True,
                "owner": motion.EXPECTED_EXECUTION_ID},
        admitted=True, goal_seen=True) is None


def test_motion_admission_and_completion_require_full_evidence():
    now = 10.0
    facts = {name: True for name in (
        "/scan", "/mapping/scan", "/scan_tf", "/mapping/scan_tf", "clock", "odom",
        "safety", "cmd", "cm", "safety_no_estop", "cm_active", "cmd_publisher",
        "/cleannav/navigation/cmd_vel_raw_publisher", "/cleannav/cmd_vel_candidate_publisher",
        "/cmd_vel_authorized_publisher", "localization")}
    guard = {"monotonic": now - 0.1, "facts": facts, "ready": True,
             "missing": [], "distance_m": 0.0,
             "safety": {"block_all": True, "estop": False, "enabled": False, "owner": None}}
    endpoints = [{"node_name": node, "node_namespace": "/",
                  "topic_type": "geometry_msgs/msg/Twist", "endpoint_gid": [1, 2, 3]}
                 for node in ("cleannav_safety_supervisor", "controller_server", "collision_monitor")]
    args = dict(phases=dict.fromkeys(policy.PHASES, "PASS"), guard=guard,
        final_publishers=endpoints[0:1], raw_publishers=endpoints[1:2],
        candidate_publishers=endpoints[2:3],
        command_samples={"/cmd_vel": 4, "/cleannav/navigation/cmd_vel_raw": 4,
                         "/cleannav/cmd_vel_candidate": 4},
        zero_command_counts={"/cmd_vel": 4, "/cleannav/navigation/cmd_vel_raw": 4,
                             "/cleannav/cmd_vel_candidate": 4},
        goal_seen=False, ground_truth_samples=2, stable_s=10.0, now_monotonic=now)
    assert policy.motion_admission_decision(**args)["allowed"] is True
    args["zero_command_counts"]["/cmd_vel"] = 3
    assert policy.motion_admission_decision(**args)["allowed"] is False
    args["zero_command_counts"]["/cmd_vel"] = 4
    args["command_samples"]["/cleannav/navigation/cmd_vel_raw"] = 0
    args["command_samples"]["/cleannav/cmd_vel_candidate"] = 0
    args["zero_command_counts"]["/cleannav/navigation/cmd_vel_raw"] = 0
    args["zero_command_counts"]["/cleannav/cmd_vel_candidate"] = 0
    assert policy.motion_admission_decision(**args)["allowed"] is True
    args["command_samples"]["/cleannav/navigation/cmd_vel_raw"] = 2
    args["zero_command_counts"]["/cleannav/navigation/cmd_vel_raw"] = 1
    rejected = policy.motion_admission_decision(**args)
    assert rejected["allowed"] is False
    assert any("cmd_vel_raw" in item for item in rejected["failures"])

    topics = ("/cleannav/navigation/cmd_vel_raw", "/cleannav/cmd_vel_candidate", "/cmd_vel")
    counts = dict.fromkeys(topics, 5)
    nonzero = dict.fromkeys(topics, 1)
    completion_args = dict(gui_goal_seen=True, mission_accepted=True, task_succeeded=True,
        lease_acquired=True, lease_released=True, lifecycle_closed=True,
        command_counts=counts, nonzero_counts=nonzero, final_zero_after_terminal=True,
        gazebo_displacement_m=1.0)
    assert policy.motion_completion_decision(**completion_args)["passed"] is True
    completion_args["lease_released"] = False
    assert policy.motion_completion_decision(**completion_args)["passed"] is False


@pytest.mark.parametrize("missing_receipt", [None, [], "not-json-object"])
def test_missing_or_malformed_motion_guard_receipt_fails_closed_as_contract_error(
        missing_receipt):
    with pytest.raises(policy.ContractError,
                       match="motion guard receipt is unavailable or malformed"):
        policy.validate_motion_guard_state(missing_receipt, now_monotonic=10.0)


def test_gazebo_entity_state_response_uses_humble_success_and_state_fields():
    state = object()
    response = SimpleNamespace(success=True, state=state)
    assert policy.gazebo_entity_state_from_response(response) is state


def test_gazebo_entity_state_failure_without_status_message_fails_closed():
    # The installed Humble GetEntityState response has header, state, success;
    # it does not define status_message.
    response = SimpleNamespace(success=False, state=object())
    with pytest.raises(policy.ContractError,
                       match="GetEntityState returned success=false"):
        policy.gazebo_entity_state_from_response(response)


def test_gazebo_entity_state_missing_or_invalid_success_fails_closed():
    for response in (SimpleNamespace(state=object()), None):
        with pytest.raises(policy.ContractError,
                           match="GetEntityState returned success=false or missing success"):
            policy.gazebo_entity_state_from_response(response)


def test_gazebo_entity_state_success_without_state_fails_closed():
    with pytest.raises(policy.ContractError,
                       match="GetEntityState success response has no state"):
        policy.gazebo_entity_state_from_response(SimpleNamespace(success=True, state=None))


@pytest.mark.parametrize("profile,expected", [
    ("standard", (110.0, 145.0, 159.0, 167.0, 175.0, 180.0, 145.0)),
    ("localization-diagnostic", (240.0, 245.0, 259.0, 267.0, 275.0, 280.0, 195.0)),
    ("short-goal-motion", (205.0, 335.0, 349.0, 357.0, 365.0, 370.0, 250.0)),
])
def test_timing_profile_deadlines_preserve_existing_profiles_and_fit_motion(profile, expected):
    script = (
        "import json,supervisor_policy as p;"
        "print(json.dumps([p.READINESS_DEADLINE_FROM_T0_S,p.SOFT_STOP_FROM_T0_S,"
        "p.WATCHDOG_SIGTERM_FROM_T0_S,p.WATCHDOG_SIGKILL_FROM_T0_S,"
        "p.HARD_PROCESS_CUTOFF_FROM_T0_S,p.CLEANUP_EVIDENCE_CUTOFF_FROM_T0_S,"
        "p.MAX_LAUNCH_RUNTIME_S]))"
    )
    env = dict(os.environ, PYTHONPATH=str(HERE))
    env["CLEANNAV_RVIZ_TIMING_PROFILE"] = profile
    actual = json.loads(subprocess.check_output([sys.executable, "-c", script], env=env, text=True))
    assert tuple(actual) == expected
    if profile == "short-goal-motion":
        assert expected[1] - expected[0] >= (
            10 + motion.MAX_AUTHORITY_TTL_S + 35)
        assert min(expected[6], expected[1] - 55 - 35) == 245.0
        assert expected[2] > expected[1] and expected[3] > expected[2]
        assert expected[4] > expected[3] and expected[5] > expected[4]


def test_short_goal_supervisor_launch_argv_carries_bounded_motion_and_runtime():
    script = (
        "import json; from pathlib import Path; import navigation_supervisor as s;"
        "argv=s.bounded_launch_argv(world=Path('/world.sdf'),"
        "map_directory=Path('/map'),map_profile='mcity-localization07-canonical-debug',"
        "output_dir=Path('/run'),runtime_timeout_s=245,motion_profile='short-goal');"
        "print(json.dumps(argv));"
        "\ntry: s.bounded_launch_argv(world=Path('/world.sdf'),map_directory=Path('/map'),"
        "map_profile='mcity-localization07-canonical-debug',output_dir=Path('/run'),"
        "runtime_timeout_s=250.1,motion_profile='short-goal')"
        "\nexcept s.ContractError: print('OVER_LIMIT_REJECTED')"
        "\nelse: raise SystemExit('over-limit launch timeout was accepted')"
    )
    env = dict(os.environ,
        CLEANNAV_RVIZ_TIMING_PROFILE="short-goal-motion",
        PYTHONPATH=os.pathsep.join((str(HERE), str(HERE.parents[1] / "tools/startup_smoke"))),
    )
    output = subprocess.check_output([sys.executable, "-c", script], env=env, text=True)
    argv = json.loads(output.splitlines()[0])
    assert "enable_motion:=true" in argv
    assert "motion_profile:=short-goal" in argv
    assert "auto_start_task:=false" in argv
    assert "runtime_timeout_sec:=245" in argv
    assert output.splitlines()[1] == "OVER_LIMIT_REJECTED"


def test_authority_expiry_applies_ttl_as_a_scalar():
    import ast
    tree = ast.parse((HERE / "navigation_supervisor.py").read_text())
    matches = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or getattr(node.func, "id", None) != "min":
            continue
        if len(node.args) != 2:
            continue
        text = ast.unparse(node)
        if "MAX_AUTHORITY_TTL_S" in text and "expiry_limit_ns" in text:
            matches.append(text)
    assert len(matches) == 1
    assert "min(MAX_AUTHORITY_TTL_S)" not in matches[0]
    assert matches[0].startswith("min(issued_ns + int(MAX_AUTHORITY_TTL_S * ")
    assert matches[0].endswith(", expiry_limit_ns)")
    issued_ns = 1_000_000_000_000
    expiry_limit_ns = issued_ns + int(200 * 1e9)
    expiry_ns = min(issued_ns + int(motion.MAX_AUTHORITY_TTL_S * 1e9), expiry_limit_ns)
    assert expiry_ns - issued_ns == int(motion.MAX_AUTHORITY_TTL_S * 1e9)
