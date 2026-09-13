"""Unit tests for Safety Supervisor control ownership semantics."""

import rclpy
import pytest
from cleannav_interfaces.msg import SafetyStatus
from cleannav_interfaces.srv import SafetyLease
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from cleannav_safety_supervisor import safety_supervisor_node as module
from cleannav_safety_supervisor.safety_supervisor_node import (
    SAFETY_ACQUIRE_LEASE_SERVICE,
    SAFETY_ESTOP_TOPIC,
    SAFETY_RELEASE_LEASE_SERVICE,
    SAFETY_RESET_ESTOP_SERVICE,
    SAFETY_STATUS_TOPIC,
    SafetySupervisorNode,
)


@pytest.fixture
def node():
    rclpy.init()
    value = SafetySupervisorNode()
    try:
        yield value
    finally:
        value.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _lease_request(execution_id):
    request = SafetyLease.Request()
    request.execution_id = execution_id
    return request


def _lease_response():
    return SafetyLease.Response()


def _run_candidate(node, linear_x, *, angular_z=0.0, autonomous=True):
    published = []
    node._cmd_vel_pub.publish = published.append
    candidate = Twist()
    candidate.linear.x = linear_x
    candidate.angular.z = angular_z
    node._candidate_cb(candidate)
    node._autonomous_enabled = autonomous
    if autonomous:
        node._lease_owner_execution_id = 'execution-a'
    node._cmd_timer_cb()
    return published[-1]


def test_forward_velocity_below_limit_passes(node):
    output = _run_candidate(node, 0.04)

    assert output.linear.x == pytest.approx(0.04)


def test_forward_velocity_above_limit_is_clamped(node):
    output = _run_candidate(node, 0.20)

    assert output.linear.x == pytest.approx(0.05)


def test_reverse_velocity_below_limit_passes(node):
    node._max_reverse_linear_x = 0.10

    output = _run_candidate(node, -0.06)

    assert output.linear.x == pytest.approx(-0.06)


def test_reverse_velocity_above_limit_is_clamped(node):
    node._max_reverse_linear_x = 0.10

    output = _run_candidate(node, -0.25)

    assert output.linear.x == pytest.approx(-0.10)


def test_zero_velocity_remains_zero(node):
    node._max_reverse_linear_x = 0.10

    output = _run_candidate(node, 0.0)

    assert output.linear.x == 0.0


def test_autonomous_disabled_blocks_reverse(node):
    node._max_reverse_linear_x = 0.10

    output = _run_candidate(node, -0.06, autonomous=False)

    assert output.linear.x == 0.0


def test_emergency_stop_blocks_reverse(node):
    node._max_reverse_linear_x = 0.10
    candidate = Twist()
    candidate.linear.x = -0.06
    node._candidate_cb(candidate)
    node._autonomous_enabled = True
    node._lease_owner_execution_id = 'execution-a'
    node._estop_cb(Bool(data=True))
    published = []
    node._cmd_vel_pub.publish = published.append

    node._cmd_timer_cb()

    assert published[-1].linear.x == 0.0


def test_stale_reverse_candidate_is_blocked(node):
    node._max_reverse_linear_x = 0.10
    candidate = Twist()
    candidate.linear.x = -0.06
    node._candidate_cb(candidate)
    node._last_candidate_time = (
        node.get_clock().now() - Duration(seconds=1.0)
    )
    node._autonomous_enabled = True
    node._lease_owner_execution_id = 'execution-a'
    published = []
    node._cmd_vel_pub.publish = published.append

    node._cmd_timer_cb()

    assert published[-1].linear.x == 0.0


def test_lease_release_blocks_previously_allowed_reverse(node):
    node._max_reverse_linear_x = 0.10
    node._acquire_lease_cb(_lease_request('execution-a'), _lease_response())
    candidate = Twist()
    candidate.linear.x = -0.06
    node._candidate_cb(candidate)
    published = []
    node._cmd_vel_pub.publish = published.append
    node._cmd_timer_cb()
    assert published[-1].linear.x == pytest.approx(-0.06)

    node._release_lease_cb(_lease_request('execution-a'), _lease_response())
    node._cmd_timer_cb()

    assert published[-1].linear.x == 0.0


def test_forward_and_reverse_limits_are_independent(node):
    node._max_forward_linear_x = 0.04
    node._max_reverse_linear_x = 0.10

    forward = node._limit_twist(Twist())
    assert forward.linear.x == 0.0

    forward_input = Twist()
    forward_input.linear.x = 0.50
    reverse_input = Twist()
    reverse_input.linear.x = -0.50

    assert node._limit_twist(forward_input).linear.x == pytest.approx(0.04)
    assert node._limit_twist(reverse_input).linear.x == pytest.approx(-0.10)


def test_default_owner_and_autonomous_state(node):
    assert node._lease_owner_execution_id is None
    assert node._autonomous_enabled is False
    assert node._emergency_stop is False


def test_acquire_a_succeeds_and_duplicate_is_idempotent(node):
    first = node._acquire_lease_cb(
        _lease_request('execution-a'),
        _lease_response(),
    )
    duplicate = node._acquire_lease_cb(
        _lease_request('execution-a'),
        _lease_response(),
    )

    assert first.success is True
    assert duplicate.success is True
    assert node._lease_owner_execution_id == 'execution-a'
    assert node._autonomous_enabled is True
    assert node._autonomous_start_time is None


def test_acquire_b_while_a_owns_fails_without_mutation(node):
    node._acquire_lease_cb(_lease_request('execution-a'), _lease_response())

    response = node._acquire_lease_cb(
        _lease_request('execution-b'),
        _lease_response(),
    )

    assert response.success is False
    assert node._lease_owner_execution_id == 'execution-a'
    assert node._autonomous_enabled is True


def test_release_wrong_owner_fails_and_release_a_succeeds(node):
    node._acquire_lease_cb(_lease_request('execution-a'), _lease_response())

    wrong = node._release_lease_cb(
        _lease_request('execution-b'),
        _lease_response(),
    )
    correct = node._release_lease_cb(
        _lease_request('execution-a'),
        _lease_response(),
    )

    assert wrong.success is False
    assert node._lease_owner_execution_id is None
    assert correct.success is True
    assert node._autonomous_enabled is False


def test_release_without_owner_is_idempotent_success(node):
    response = node._release_lease_cb(
        _lease_request('execution-a'),
        _lease_response(),
    )

    assert response.success is True
    assert node._lease_owner_execution_id is None
    assert node._autonomous_enabled is False


def test_estop_rejects_acquire_and_true_clears_owner(node):
    node._acquire_lease_cb(_lease_request('execution-a'), _lease_response())
    node._estop_cb(Bool(data=True))

    response = node._acquire_lease_cb(
        _lease_request('execution-b'),
        _lease_response(),
    )

    assert node._emergency_stop is True
    assert node._lease_owner_execution_id is None
    assert node._autonomous_enabled is False
    assert response.success is False


def test_estop_false_cannot_clear_latched_estop(node):
    node._estop_cb(Bool(data=True))
    node._estop_cb(Bool(data=False))

    assert node._emergency_stop is True


def test_reset_clears_estop_and_does_not_restore_lease(node):
    node._acquire_lease_cb(_lease_request('execution-a'), _lease_response())
    node._estop_cb(Bool(data=True))

    response = node._reset_estop_cb(None, Trigger.Response())

    assert response.success is True
    assert node._emergency_stop is False
    assert node._lease_owner_execution_id is None
    assert node._autonomous_enabled is False
    assert node._autonomous_start_time is None


def test_lease_owned_autonomous_ignores_legacy_timeout(node, monkeypatch):
    published = []
    node._cmd_vel_pub.publish = published.append
    node._candidate_count = 1
    node._last_candidate_twist = Twist()
    node._last_candidate_twist.linear.x = 0.2
    node._last_candidate_time = node.get_clock().now()
    node._autonomous_enabled = True
    node._autonomous_start_time = 0.0
    node._lease_owner_execution_id = 'execution-a'
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.0)

    node._cmd_timer_cb()

    assert node._autonomous_enabled is True
    assert node._lease_owner_execution_id == 'execution-a'
    assert published[-1].linear.x == pytest.approx(0.05)


def test_legacy_autonomous_still_times_out_without_lease(node, monkeypatch):
    published = []
    node._cmd_vel_pub.publish = published.append
    node._candidate_count = 0
    node._autonomous_enabled = True
    node._autonomous_start_time = 0.0
    node._lease_owner_execution_id = None
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.0)

    node._cmd_timer_cb()

    assert node._autonomous_enabled is False
    assert node._autonomous_start_time is None
    assert published[-1].linear.x == 0.0


def test_control_endpoints_are_frozen():
    assert SAFETY_ACQUIRE_LEASE_SERVICE == (
        '/cleannav/safety/acquire_lease'
    )
    assert SAFETY_RELEASE_LEASE_SERVICE == (
        '/cleannav/safety/release_lease'
    )
    assert SAFETY_RESET_ESTOP_SERVICE == (
        '/cleannav/safety/reset_emergency_stop'
    )
    assert SAFETY_ESTOP_TOPIC == '/cleannav/safety/emergency_stop'
    assert SAFETY_STATUS_TOPIC == '/cleannav/safety_status'


def test_structured_status_initial_state_and_legacy_publisher_preserved(node):
    published = []
    node._structured_status_pub.publish = published.append

    node._status_timer_cb()

    assert len(published) == 1
    status = published[0]
    assert isinstance(status, SafetyStatus)
    assert status.header.frame_id == ''
    assert status.interface_version == '1.0'
    assert status.emergency_stop_active is False
    assert status.autonomous_enabled is False
    assert status.lease_owner_execution_id == ''


def test_structured_status_reflects_lease_and_estop(node):
    published = []
    node._structured_status_pub.publish = published.append
    node._acquire_lease_cb(
        _lease_request('execution-a'),
        _lease_response(),
    )
    node._status_timer_cb()
    assert published[-1].autonomous_enabled is True
    assert published[-1].lease_owner_execution_id == 'execution-a'

    node._estop_cb(Bool(data=True))
    node._status_timer_cb()
    assert published[-1].emergency_stop_active is True
    assert published[-1].autonomous_enabled is False
    assert published[-1].lease_owner_execution_id == ''
