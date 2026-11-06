"""Pure bounds and receipt checks for the one-goal RViz motion profile."""
from __future__ import annotations

import hashlib
import json
import math
import os
import time
from pathlib import Path


PROFILE_ID = "rviz_short_goal_015"
EXPECTED_EXECUTION_ID = "ros-execution-1"
MAX_GOAL_DISTANCE_M = 2.0
MIN_GOAL_DISTANCE_M = 0.75
MAX_GOAL_YAW_ERROR_RAD = 0.35
MAX_GOAL_ANGLE_FROM_FORWARD_RAD = 0.45
MAX_GUI_GOAL_WAIT_S = 20.0
MAX_GOAL_RUNTIME_S = 60.0
LEASE_RELEASE_CONFIRMATION_S = 5.0
MAX_AUTHORITY_TTL_S = (MAX_GUI_GOAL_WAIT_S + MAX_GOAL_RUNTIME_S
                       + LEASE_RELEASE_CONFIRMATION_S)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, value: dict) -> None:
    """Persist a run receipt without exposing a partial JSON document."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def issue_motion_receipts(*, run_id: str, run_dir: Path, issued_monotonic_ns: int,
                          expires_monotonic_ns: int, source_manifest_sha256: str,
                          map_contract_sha256: str, world_sha256: str) -> tuple[dict, dict, dict]:
    """Issue a one-run operator authorization and one-task scope after admission.

    These are audit/adapter receipts created only by the outer supervisor after
    Motion Admission passes. They do not create the Safety Lease. The real
    Mission Manager acquires that Lease for the accepted GUI task.
    """
    if not run_id.startswith("rviz-nav-") or issued_monotonic_ns <= 0:
        raise ValueError("motion receipts require a valid current Run ID and monotonic issue time")
    ttl = expires_monotonic_ns - issued_monotonic_ns
    if not 0 < ttl <= int(MAX_AUTHORITY_TTL_S * 1e9):
        raise ValueError("motion authorization TTL is outside the bounded profile")
    if not all(isinstance(value, str) and len(value) == 64
               and all(c in "0123456789abcdef" for c in value)
               for value in (source_manifest_sha256, map_contract_sha256, world_sha256)):
        raise ValueError("motion receipt source identities must be SHA-256 digests")
    directory = Path(run_dir).resolve(strict=True)
    authorization = {
        "schema_version": 1,
        "status": "ACTIVE",
        "authority_kind": "explicit_user_task_authorization",
        "authorization_id": f"{run_id}:rviz_short_goal_015",
        "run_id": run_id,
        "profile_id": PROFILE_ID,
        "issued_monotonic_ns": issued_monotonic_ns,
        "expires_monotonic_ns": expires_monotonic_ns,
        "source_manifest_sha256": source_manifest_sha256,
        "map_contract_sha256": map_contract_sha256,
        "world_sha256": world_sha256,
        "limits": {
            "max_forward_linear_x_mps": 0.15,
            "max_reverse_linear_x_mps": 0.0,
            "max_goal_distance_m": MAX_GOAL_DISTANCE_M,
            "max_goal_runtime_s": MAX_GOAL_RUNTIME_S,
            "maximum_accepted_goals": 1,
        },
        "safety_lease_created": False,
        "revoked_monotonic_ns": None,
        "release_reason": None,
    }
    auth_path = directory / "motion-authorization.json"
    # These are immutable issuance records. Mutable task/Lease lifecycle goes
    # into a separate receipt so the Scope's authorization digest stays valid.
    _write_json_exclusive(auth_path, authorization)
    if auth_path.exists() and json.loads(auth_path.read_text()) != authorization:
        raise ValueError("motion authorization already exists with different content")
    scope = {
        "schema_version": 1,
        "status": "ACTIVE",
        "scope_id": f"{run_id}:single-rviz-goal",
        "run_id": run_id,
        "profile_id": PROFILE_ID,
        "authorization_sha256": sha256(auth_path),
        "mission_execution_id": EXPECTED_EXECUTION_ID,
        "issued_monotonic_ns": issued_monotonic_ns,
        "expires_monotonic_ns": expires_monotonic_ns,
        "maximum_accepted_goals": 1,
        "accepted_goal_count": 0,
        "accepted_command_id": None,
        "accepted_task_id": None,
        "lease_owner_execution_id": None,
        "lease_acquired_monotonic_ns": None,
        "lease_released_monotonic_ns": None,
        "closed_monotonic_ns": None,
        "release_reason": None,
    }
    scope_path = directory / "motion-scope.json"
    _write_json_exclusive(scope_path, scope)
    if scope_path.exists() and json.loads(scope_path.read_text()) != scope:
        raise ValueError("motion Scope already exists with different content")
    lifecycle = {
        "schema_version": 1, "status": "AUTHORIZED", "run_id": run_id,
        "profile_id": PROFILE_ID, "authorization_sha256": sha256(auth_path),
        "scope_sha256": sha256(scope_path), "mission_execution_id": EXPECTED_EXECUTION_ID,
        "accepted_goal_count": 0, "command_id": None, "task_id": None,
        "lease_owner_execution_id": None, "lease_id": None,
        "lease_acquired_monotonic_ns": None, "lease_released_monotonic_ns": None,
        "revoked_monotonic_ns": None, "release_reason": None,
    }
    lifecycle_path = directory / "motion-lifecycle.json"
    _write_json_exclusive(lifecycle_path, lifecycle)
    if lifecycle_path.exists() and json.loads(lifecycle_path.read_text()) != lifecycle:
        raise ValueError("motion lifecycle already exists with different content")
    return authorization, scope, lifecycle


def _write_json_exclusive(path: Path, value: dict) -> bool:
    payload = (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        return False
    with os.fdopen(fd, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    return True


def update_lifecycle(path: Path, *, expected_run_id: str,
                     expected_profile_id: str, updates: dict) -> dict:
    """Update the current run's lifecycle receipt, never its immutable grants."""
    path = Path(path)
    value = json.loads(path.read_text(encoding="utf-8"))
    if (value.get("status") not in {"AUTHORIZED", "GOAL_ACCEPTED", "LEASE_ACTIVE", "LEASE_RELEASED"}
            or value.get("run_id") != expected_run_id
            or value.get("profile_id") != expected_profile_id):
        raise ValueError("cannot update a closed or mismatched motion lifecycle receipt")
    next_value = dict(value)
    next_value.update(updates)
    transitions = {
        "AUTHORIZED": {"AUTHORIZED", "GOAL_ACCEPTED", "LEASE_ACTIVE"},
        "GOAL_ACCEPTED": {"GOAL_ACCEPTED", "LEASE_ACTIVE", "LEASE_RELEASED"},
        "LEASE_ACTIVE": {"LEASE_ACTIVE", "LEASE_RELEASED"},
        "LEASE_RELEASED": {"LEASE_RELEASED"},
    }
    if next_value.get("status") not in transitions[value["status"]]:
        raise ValueError("motion lifecycle state transition is invalid")
    for key in ("run_id", "profile_id", "authorization_sha256", "mission_execution_id"):
        if next_value.get(key) != value.get(key):
            raise ValueError("motion lifecycle identity fields are immutable")
    if next_value.get("accepted_goal_count", 0) not in (0, 1):
        raise ValueError("motion lifecycle exceeds the one-goal Scope")
    _atomic_json(path, next_value)
    return next_value


def update_scope(path: Path, *, expected_run_id: str, expected_profile_id: str,
                 updates: dict) -> dict:
    """Bind the single Scope to the accepted Mission and actual Lease owner."""
    path = Path(path)
    value = json.loads(path.read_text(encoding="utf-8"))
    if (value.get("run_id") != expected_run_id
            or value.get("profile_id") != expected_profile_id
            or value.get("scope_id") != f"{expected_run_id}:single-rviz-goal"
            or value.get("status") not in {"ACTIVE", "GOAL_ACCEPTED", "LEASE_ACTIVE"}):
        raise ValueError("cannot update a closed or mismatched motion Scope")
    next_value = dict(value)
    next_value.update(updates)
    transitions = {
        "ACTIVE": {"ACTIVE", "GOAL_ACCEPTED", "LEASE_ACTIVE", "CLOSED"},
        "GOAL_ACCEPTED": {"GOAL_ACCEPTED", "LEASE_ACTIVE", "CLOSED"},
        "LEASE_ACTIVE": {"LEASE_ACTIVE", "CLOSED"},
    }
    if next_value.get("status") not in transitions[value["status"]]:
        raise ValueError("motion Scope state transition is invalid")
    if (next_value.get("maximum_accepted_goals") != 1
            or next_value.get("mission_execution_id") != EXPECTED_EXECUTION_ID
            or next_value.get("accepted_goal_count", 0) not in (0, 1)):
        raise ValueError("motion Scope update broadened its goal or Mission contract")
    if next_value.get("accepted_goal_count", 0) < value.get("accepted_goal_count", 0):
        raise ValueError("motion Scope cannot unconsume an accepted goal")
    if next_value.get("accepted_goal_count") == 1:
        command_id = next_value.get("accepted_command_id")
        task_id = next_value.get("accepted_task_id")
        if not isinstance(command_id, str) or not command_id.startswith("rviz-short-"):
            raise ValueError("accepted motion Scope lacks its RViz command identity")
        if task_id is not None and (isinstance(task_id, bool) or not isinstance(task_id, int)
                                    or task_id <= 0):
            raise ValueError("accepted motion Scope task identity is invalid")
    if next_value.get("status") == "LEASE_ACTIVE":
        if (next_value.get("lease_owner_execution_id") != EXPECTED_EXECUTION_ID
                or not isinstance(next_value.get("lease_acquired_monotonic_ns"), int)):
            raise ValueError("active motion Scope lacks the observed Mission Safety Lease")
    if value.get("status") == "LEASE_ACTIVE" and next_value.get("status") == "CLOSED":
        if (next_value.get("lease_owner_execution_id") not in (None, "")
                or not isinstance(next_value.get("lease_released_monotonic_ns"), int)):
            raise ValueError("active motion Scope cannot close before observed Lease release")
    _atomic_json(path, next_value)
    return next_value


def write_revocation_receipt(*, run_id: str, profile_id: str, run_dir: Path,
                             authorization_path: Path, scope_path: Path,
                             lifecycle_path: Path, monotonic_ns: int,
                             release_reason: str, lease_acquired: bool,
                             lease_released: bool) -> dict:
    """Record revocation without rewriting immutable authorization evidence."""
    if profile_id != PROFILE_ID or monotonic_ns <= 0:
        raise ValueError("invalid motion authority revocation identity")
    directory = Path(run_dir).resolve(strict=True)
    auth = json.loads(Path(authorization_path).read_text(encoding="utf-8"))
    scope = json.loads(Path(scope_path).read_text(encoding="utf-8"))
    lifecycle = json.loads(Path(lifecycle_path).read_text(encoding="utf-8"))
    if (auth.get("run_id") != run_id or scope.get("run_id") != run_id
            or lifecycle.get("run_id") != run_id
            or auth.get("profile_id") != profile_id
            or scope.get("profile_id") != profile_id
            or lifecycle.get("profile_id") != profile_id
            or lifecycle.get("status") != "CLOSED"
            or lifecycle.get("authorization_effectively_revoked") is not True
            or lifecycle.get("scope_effectively_closed") is not True
            or scope.get("status") != "CLOSED"
            or (lease_acquired and (not lease_released
                or lifecycle.get("lease_release_confirmed") is not True))):
        raise ValueError("motion authorization/Scope revocation is not fully evidenced")
    receipt = {
        "schema_version": 1, "status": "REVOKED", "run_id": run_id,
        "profile_id": profile_id, "authorization_id": auth.get("authorization_id"),
        "authorization_sha256": sha256(Path(authorization_path)),
        "scope_id": scope.get("scope_id"), "scope_sha256": sha256(Path(scope_path)),
        "lifecycle_sha256": sha256(Path(lifecycle_path)),
        "lease_acquired": bool(lease_acquired), "lease_released": bool(lease_released),
        "monotonic_ns": monotonic_ns, "release_reason": release_reason,
    }
    target = directory / "motion-revocation.json"
    if _write_json_exclusive(target, receipt):
        return receipt
    existing = json.loads(target.read_text(encoding="utf-8"))
    stable_fields = ("status", "run_id", "profile_id", "authorization_id",
                     "authorization_sha256", "scope_id", "scope_sha256",
                     "lifecycle_sha256", "lease_acquired", "lease_released",
                     "release_reason")
    if any(existing.get(key) != receipt.get(key) for key in stable_fields):
        raise ValueError("motion revocation receipt already exists with different identity")
    return existing


def validate_revocation_receipt(*, run_id: str, authorization_path: Path,
                               scope_path: Path, lifecycle_path: Path,
                               receipt_path: Path, lease_acquired: bool,
                               lease_released: bool) -> dict:
    """Verify the closed Auth/Scope lifecycle and its immutable identities."""
    try:
        receipt = json.loads(Path(receipt_path).read_text(encoding="utf-8"))
        auth = json.loads(Path(authorization_path).read_text(encoding="utf-8"))
        scope = json.loads(Path(scope_path).read_text(encoding="utf-8"))
        lifecycle = json.loads(Path(lifecycle_path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ValueError(f"motion revocation evidence unavailable: {error}") from error
    if (receipt.get("status") != "REVOKED" or receipt.get("run_id") != run_id
            or receipt.get("profile_id") != PROFILE_ID
            or auth.get("run_id") != run_id or scope.get("run_id") != run_id
            or lifecycle.get("run_id") != run_id
            or scope.get("status") != "CLOSED" or lifecycle.get("status") != "CLOSED"
            or lifecycle.get("authorization_effectively_revoked") is not True
            or lifecycle.get("scope_effectively_closed") is not True
            or receipt.get("authorization_sha256") != sha256(Path(authorization_path))
            or receipt.get("scope_sha256") != sha256(Path(scope_path))
            or receipt.get("lifecycle_sha256") != sha256(Path(lifecycle_path))
            or receipt.get("lease_acquired") is not lease_acquired
            or receipt.get("lease_released") is not lease_released
            or (lease_acquired and (not lease_released
                or lifecycle.get("lease_release_confirmed") is not True))):
        raise ValueError("motion revocation receipt does not prove the current run's closed scope")
    return receipt


def close_lifecycle(path: Path, *, expected_run_id: str,
                    expected_profile_id: str, monotonic_ns: int,
                    release_reason: str, lease_released: bool) -> dict:
    """Close/revoke one run and fail if a previously acquired Lease is unconfirmed."""
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if value.get("run_id") != expected_run_id or value.get("profile_id") != expected_profile_id:
        raise ValueError("cannot close a mismatched motion lifecycle receipt")
    if value.get("status") == "CLOSED":
        return value
    acquired = value.get("lease_acquired_monotonic_ns") is not None
    if acquired and not lease_released:
        raise ValueError("cannot close motion lifecycle while an acquired Lease is unreleased")
    value.update(status="CLOSED", revoked_monotonic_ns=monotonic_ns,
                 release_reason=release_reason,
                 lease_release_confirmed=(not acquired or lease_released),
                 authorization_effectively_revoked=True,
                 scope_effectively_closed=True)
    _atomic_json(Path(path), value)
    return value


def validate_profile(profile: dict) -> dict:
    """Reject profiles that broaden this exact forward-only test contract."""
    safety = profile.get("safety", {}) if isinstance(profile, dict) else {}
    valid = (
        profile.get("profile_id") == PROFILE_ID
        and profile.get("reverse_allowed") is False
        and profile.get("max_reverse_linear_x") == 0.0
        and 0 < profile.get("max_forward_linear_x", math.inf) <= 0.15
        and 0 < profile.get("max_angular_z", math.inf) <= 0.13392857142857142
        and safety.get("max_reverse_linear_x") == 0.0
        and safety.get("max_forward_linear_x") == profile.get("max_forward_linear_x")
        and safety.get("max_angular_z") == profile.get("max_angular_z")
    )
    if not valid:
        raise ValueError("short-goal profile is not the approved forward-only 0.15 m/s profile")
    return {
        "profile_id": PROFILE_ID,
        "max_forward_linear_x": float(profile["max_forward_linear_x"]),
        "max_reverse_linear_x": 0.0,
        "max_angular_z": float(profile["max_angular_z"]),
        "max_goal_distance_m": MAX_GOAL_DISTANCE_M,
        "min_goal_distance_m": MIN_GOAL_DISTANCE_M,
        "max_goal_runtime_s": MAX_GOAL_RUNTIME_S,
        "reverse_allowed": False,
        "single_goal": True,
    }


def validate_authority_files(*, run_id: str, authorization_path: Path,
                             scope_path: Path, now_monotonic_ns: int) -> tuple[dict, dict]:
    """Require matching, live receipts bound to this run and profile."""
    try:
        auth = json.loads(Path(authorization_path).read_text())
        scope = json.loads(Path(scope_path).read_text())
    except (OSError, ValueError) as error:
        raise ValueError(f"motion admission receipts unavailable: {error}") from error
    if (auth.get("status") != "ACTIVE"
            or scope.get("status") not in {"ACTIVE", "GOAL_ACCEPTED", "LEASE_ACTIVE"}
            or auth.get("run_id") != run_id or scope.get("run_id") != run_id
            or auth.get("profile_id") != PROFILE_ID or scope.get("profile_id") != PROFILE_ID
            or scope.get("authorization_sha256") != sha256(Path(authorization_path))):
        raise ValueError("motion admission receipts have mismatched run/profile/authorization identity")
    expiry = auth.get("expires_monotonic_ns")
    issued = auth.get("issued_monotonic_ns")
    expected_limits = {
        "max_forward_linear_x_mps": 0.15,
        "max_reverse_linear_x_mps": 0.0,
        "max_goal_distance_m": MAX_GOAL_DISTANCE_M,
        "max_goal_runtime_s": MAX_GOAL_RUNTIME_S,
        "maximum_accepted_goals": 1,
    }
    if (not isinstance(expiry, int) or isinstance(expiry, bool)
            or not isinstance(issued, int) or isinstance(issued, bool)
            or expiry <= issued or expiry-issued > int(MAX_AUTHORITY_TTL_S*1e9)
            or now_monotonic_ns <= 0 or now_monotonic_ns >= expiry
            or scope.get("expires_monotonic_ns") != expiry
            or scope.get("issued_monotonic_ns") != issued
            or auth.get("authority_kind") != "explicit_user_task_authorization"
            or auth.get("authorization_id") != f"{run_id}:{PROFILE_ID}"
            or auth.get("safety_lease_created") is not False
            or auth.get("limits") != expected_limits
            or scope.get("maximum_accepted_goals") != 1
            or scope.get("accepted_goal_count", 0) not in (0, 1)
            or scope.get("scope_id") != f"{run_id}:single-rviz-goal"
            or not isinstance(scope.get("mission_execution_id"), str)
            or not scope.get("mission_execution_id")):
        raise ValueError("motion admission receipts are expired or malformed")
    return auth, scope


def validate_lifecycle(*, run_id: str, lifecycle_path: Path,
                       authorization_path: Path, scope_path: Path,
                       now_monotonic_ns: int) -> dict:
    try:
        lifecycle = json.loads(Path(lifecycle_path).read_text())
        auth, scope = validate_authority_files(
            run_id=run_id, authorization_path=authorization_path,
            scope_path=scope_path, now_monotonic_ns=now_monotonic_ns)
    except (OSError, ValueError) as error:
        raise ValueError(f"motion lifecycle validation failed: {error}") from error
    if (lifecycle.get("run_id") != run_id or lifecycle.get("profile_id") != PROFILE_ID
            or lifecycle.get("authorization_sha256") != sha256(Path(authorization_path))
            or lifecycle.get("scope_sha256") != sha256(Path(scope_path))
            or lifecycle.get("mission_execution_id") != scope.get("mission_execution_id")
            or lifecycle.get("status") not in {"AUTHORIZED", "GOAL_ACCEPTED", "LEASE_ACTIVE"}
            or lifecycle.get("accepted_goal_count") not in (0, 1)
            or lifecycle.get("accepted_goal_count") != scope.get("accepted_goal_count", 0)
            or lifecycle.get("command_id") != scope.get("accepted_command_id")
            or lifecycle.get("task_id") != scope.get("accepted_task_id")):
        raise ValueError("motion lifecycle identity or state is invalid")
    expiry = auth.get("expires_monotonic_ns")
    if not isinstance(expiry, int) or now_monotonic_ns >= expiry:
        raise ValueError("motion lifecycle authorization has expired")
    return lifecycle


def validate_motion_admission(*, run_id: str, admission_path: Path,
                              authorization_path: Path, scope_path: Path,
                              now_monotonic_ns: int) -> dict:
    """Require the supervisor's post-Admission, post-interlock receipt."""
    try:
        value = json.loads(Path(admission_path).read_text(encoding="utf-8"))
        auth, scope = validate_authority_files(
            run_id=run_id, authorization_path=authorization_path,
            scope_path=scope_path, now_monotonic_ns=now_monotonic_ns)
    except (OSError, ValueError) as error:
        raise ValueError(f"motion Admission is unavailable or invalid: {error}") from error
    if (value.get("schema_version") != 1 or value.get("run_id") != run_id
            or value.get("profile_id") != PROFILE_ID
            or value.get("status") != "ADMITTED"
            or value.get("authorization_sha256") != sha256(Path(authorization_path))
            or value.get("scope_sha256") != sha256(Path(scope_path))
            or value.get("interlock_response_success") is not True
            or value.get("safety_unblocked_unleased") is not True
            or value.get("all_phases_passed") is not True
            or value.get("goal_seen_before_admission") is not False
            or value.get("issued_monotonic_ns") != auth.get("issued_monotonic_ns")
            or value.get("expires_monotonic_ns") != auth.get("expires_monotonic_ns")):
        raise ValueError("Motion Admission receipt does not prove the current run's complete gate")
    if now_monotonic_ns >= min(auth.get("expires_monotonic_ns", 0),
                                scope.get("expires_monotonic_ns", 0)):
        raise ValueError("Motion Admission receipts expired")
    return value


def validate_runtime_admission(*, execution_id=EXPECTED_EXECUTION_ID):
    """Consume the existing Supervisor proof at each executable boundary.

    This function never issues authorization, opens an interlock, or grants a
    Lease. It is shared by the scoped monitor and Mission/Safety transports.
    """
    from deadline_watchdog import process_identity
    from supervisor_policy import admitted_safety_receipt_decision
    run_id=os.environ.get('CLEANNAV_RVIZ_RUN_ID')
    directory=os.environ.get('CLEANNAV_RVIZ_RUN_DIR')
    if not run_id or not directory or not Path(directory).is_absolute():
        raise ValueError('supervised Motion Admission context missing')
    directory=Path(directory).resolve(strict=True)
    now=time.monotonic_ns()
    proof=validate_motion_admission(run_id=run_id,
        admission_path=directory/'motion-admission.json',
        authorization_path=directory/'motion-authorization.json',
        scope_path=directory/'motion-scope.json',now_monotonic_ns=now)
    validate_lifecycle(run_id=run_id,lifecycle_path=directory/'motion-lifecycle.json',
        authorization_path=directory/'motion-authorization.json',
        scope_path=directory/'motion-scope.json',now_monotonic_ns=now)
    auth=json.loads((directory/'motion-authorization.json').read_text())
    if execution_id!=EXPECTED_EXECUTION_ID or auth['source_manifest_sha256']!=sha256(Path(__file__).with_name('SOURCE_MANIFEST.json')):
        raise ValueError('supervised execution/source identity mismatch')
    supervisor=json.loads((directory/'supervisor_state.json').read_text())
    pid=supervisor.get('supervisor_pid');ticks=supervisor.get('supervisor_start_ticks')
    if (supervisor.get('run_id')!=run_id or type(pid) is not int or pid<=0
            or type(ticks) is not int or ticks<=0):
        raise ValueError('supervised process identity missing')
    current=process_identity(pid)
    if not current or current['start_ticks']!=ticks or current['state']=='Z':
        raise ValueError('Motion Admission supervisor is not alive')
    guard_path=Path(proof.get('guard_path','')).resolve(strict=True)
    if not guard_path.is_relative_to(directory):
        raise ValueError('Guard receipt escaped its Run directory')
    guard=json.loads(guard_path.read_text())
    confirmed=proof.get('safety_status')
    actual=(guard.get('safety') or {}).get('generation_stamp_ns')
    generation=(confirmed or {}).get('generation_stamp_ns')
    if type(generation) is not int or actual!=generation:
        raise ValueError('Guard generation does not match confirmed Admission')
    writer=proof.get('guard_writer_pid')
    if type(writer) is not int or writer<=0:
        raise ValueError('confirmed Guard writer identity missing')
    decision=admitted_safety_receipt_decision(guard,confirmed,
        now_monotonic=now/1e9,deadline_monotonic=proof['expires_monotonic_ns']/1e9,
        expected_run_id=run_id,expected_writer_pid=writer,motion_admitted=True,
        mission_lease_active=confirmed.get('owner')==EXPECTED_EXECUTION_ID,
        generation_stamp_ns=generation)
    if decision['action']!='accept':
        raise ValueError(decision.get('reason') or 'Guard has not confirmed Admission')
    return proof


def validate_forward_goal(*, goal_x: float, goal_y: float, goal_yaw: float,
                          base_x: float, base_y: float, base_yaw: float,
                          map_free: bool) -> dict:
    """Require a nearby, map-frame, mostly forward goal in known free space."""
    values = (goal_x, goal_y, goal_yaw, base_x, base_y, base_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("goal or current pose contains a non-finite value")
    dx, dy = goal_x - base_x, goal_y - base_y
    distance = math.hypot(dx, dy)
    if not MIN_GOAL_DISTANCE_M <= distance <= MAX_GOAL_DISTANCE_M:
        raise ValueError("goal distance is outside the [0.75, 2.0] m short-goal bound")
    heading = math.atan2(dy, dx)
    direction_error = math.atan2(math.sin(heading-base_yaw), math.cos(heading-base_yaw))
    yaw_error = math.atan2(math.sin(goal_yaw-base_yaw), math.cos(goal_yaw-base_yaw))
    if abs(direction_error) > MAX_GOAL_ANGLE_FROM_FORWARD_RAD:
        raise ValueError("goal is not predominantly forward")
    if abs(yaw_error) > MAX_GOAL_YAW_ERROR_RAD:
        raise ValueError("goal yaw exceeds the short-goal heading bound")
    if map_free is not True:
        raise ValueError("goal neighborhood is not confirmed known free in the loaded map")
    return {"distance_m": distance, "direction_error_rad": direction_error,
            "yaw_error_rad": yaw_error, "map_free": True}


def command_policy(values: list[float], *, safety: dict | None,
                   admitted: bool, goal_seen: bool) -> str | None:
    """Enforce limits and require Admission, GUI goal, and the real Mission Lease."""
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        return "invalid command components"
    if any(abs(values[index]) > 1e-8 for index in (1, 2, 3, 4)):
        return "nonplanar command"
    if values[0] < -1e-8 or values[0] > 0.15 + 1e-8:
        return "linear command outside forward-only profile"
    if abs(values[5]) > 0.13392857142857142 + 1e-8:
        return "angular command outside short-goal profile"
    moving = abs(values[0]) > 1e-8 or abs(values[5]) > 1e-8
    if moving and not goal_seen:
        return "nonzero command before the single RViz goal"
    if moving and (not admitted or not goal_seen or not safety or safety.get("block_all") is not False
                   or safety.get("estop") is not False or safety.get("enabled") is not True
                   or safety.get("owner") != EXPECTED_EXECUTION_ID):
        return "nonzero command without the admitted Mission Safety Lease"
    return None
