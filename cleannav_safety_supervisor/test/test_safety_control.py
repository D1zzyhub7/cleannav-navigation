"""Unit tests for Safety Supervisor control ownership semantics."""

import rclpy
import math
import pytest
from cleannav_interfaces.msg import SafetyStatus
from cleannav_interfaces.srv import SafetyLease
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool

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


def _run_candidate(
    node,
    linear_x,
    *,
    angular_z=0.0,
    autonomous=True,
    block_all=False,
):
    published = []
    node._cmd_vel_pub.publish = published.append
    candidate = Twist()
    candidate.linear.x = linear_x
    candidate.angular.z = angular_z
    node._candidate_cb(candidate)
    node._autonomous_enabled = autonomous
    node._block_all = block_all
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


def test_block_all_blocks_even_with_autonomous_lease(node):
    node._max_reverse_linear_x = 0.10

    output = _run_candidate(
        node, -0.06, autonomous=True, block_all=True
    )

    assert output.linear.x == 0.0
    assert output.angular.z == 0.0
    assert node._structured_status_message().block_all is True


def test_block_all_false_allows_lease_after_limits(node):
    output = _run_candidate(
        node, 0.04, autonomous=True, block_all=False
    )

    assert output.linear.x == pytest.approx(0.04)
    assert node._structured_status_message().block_all is False


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
    node._block_all = False
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
    assert node._block_all is True
    assert node._lease_owner_execution_id is None
    assert node._autonomous_enabled is False
    assert node._emergency_stop is False


def test_simulation_authorization_is_disabled_by_default(node):
    response = node._simulation_motion_cb(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    assert node._block_all
    assert not node._autonomous_enabled


def test_simulation_authorization_requires_sim_time_and_preserves_lease_gate(node):
    from rclpy.parameter import Parameter
    node._simulation_motion_authorization = True
    response = node._simulation_motion_cb(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    node.set_parameters([Parameter('use_sim_time', value=True)])
    response = node._simulation_motion_cb(SetBool.Request(data=True), SetBool.Response())
    assert response.success
    assert not node._block_all
    assert not node._autonomous_enabled
    assert node._lease_owner_execution_id is None
    output = _run_candidate(node, 0.04, autonomous=False)
    assert output == Twist()


def test_simulation_authorization_never_overrides_estop(node):
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter('use_sim_time', value=True)])
    node._simulation_motion_authorization = True
    node._estop_cb(Bool(data=True))
    response = node._simulation_motion_cb(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    assert node._emergency_stop
    assert node._block_all


def test_block_all_rejects_motion_lease_without_changing_owner(node):
    response = node._acquire_lease_cb(_lease_request('execution-a'), _lease_response())
    assert response.success is False
    assert 'block_all' in response.message
    assert node._lease_owner_execution_id is None
    assert node._autonomous_enabled is False


@pytest.mark.parametrize('component', [
    'linear.x', 'linear.y', 'linear.z', 'angular.x', 'angular.y', 'angular.z',
])
@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_candidate_invalidates_previously_valid_motion(node, component, value):
    _run_candidate(node, 0.04)
    candidate = Twist()
    vector, axis = component.split('.')
    setattr(getattr(candidate, vector), axis, value)
    published = []
    node._cmd_vel_pub.publish = published.append
    node._candidate_cb(candidate)
    node._cmd_timer_cb()
    assert node._last_candidate_twist is None
    assert all(message == Twist() for message in published)


@pytest.mark.parametrize('component', ['linear.y', 'linear.z', 'angular.x', 'angular.y'])
def test_non_ackermann_component_is_rejected(node, component):
    _run_candidate(node, 0.04)
    candidate = Twist()
    vector, axis = component.split('.')
    setattr(getattr(candidate, vector), axis, 0.1)
    node._candidate_cb(candidate)
    assert node._last_candidate_twist is None


@pytest.mark.parametrize('vx', [0.5, -0.25])
def test_clamping_never_tightens_ackermann_curvature(node, vx):
    node._max_reverse_linear_x = 0.10
    output = _run_candidate(node, vx, angular_z=0.2)
    assert abs(output.angular.z / output.linear.x) <= abs(0.2 / vx) + 1e-12
    stopped = _run_candidate(node, 0.0, angular_z=0.2)
    assert stopped == Twist()


def test_new_lease_cannot_replay_previous_execution_candidate(node):
    node._block_all = False
    node._acquire_lease_cb(_lease_request('a'), _lease_response())
    _run_candidate(node, 0.04)
    node._lease_owner_execution_id = 'a'
    node._release_lease_cb(_lease_request('a'), _lease_response())
    node._acquire_lease_cb(_lease_request('b'), _lease_response())
    published = []
    node._cmd_vel_pub.publish = published.append
    node._cmd_timer_cb()
    assert published[-1] == Twist()
    candidate = Twist()
    candidate.linear.x = 0.04
    node._candidate_cb(candidate)
    node._cmd_timer_cb()
    assert published[-1].linear.x == pytest.approx(0.04)


def test_wall_watchdog_rejects_candidate_when_ros_clock_does_not_advance(node, monkeypatch):
    _run_candidate(node, 0.04)
    monkeypatch.setattr(module.time, 'monotonic', lambda: node._last_candidate_monotonic + 1.0)
    published = []
    node._cmd_vel_pub.publish = published.append
    node._cmd_timer_cb()
    assert published[-1] == Twist()


def test_acquire_a_succeeds_and_duplicate_is_idempotent(node):
    node._block_all = False  # Explicit test-only authorization.
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
    node._block_all = False  # Explicit test-only authorization.
    node._acquire_lease_cb(_lease_request('execution-a'), _lease_response())

    response = node._acquire_lease_cb(
        _lease_request('execution-b'),
        _lease_response(),
    )

    assert response.success is False
    assert node._lease_owner_execution_id == 'execution-a'
    assert node._autonomous_enabled is True


def test_release_wrong_owner_fails_and_release_a_succeeds(node):
    node._block_all = False  # Explicit test-only authorization.
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
    node._block_all = False  # Explicit test-only authorization.
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
    node._block_all = False  # Explicit test-only authorization.
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
    node._block_all = False
    node._autonomous_enabled = True
    node._autonomous_start_time = 0.0
    node._lease_owner_execution_id = 'execution-a'
    node._last_candidate_monotonic = 100.0
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.0)

    node._cmd_timer_cb()

    assert node._autonomous_enabled is True
    assert node._lease_owner_execution_id == 'execution-a'
    assert published[-1].linear.x == pytest.approx(0.05)


def test_legacy_autonomous_still_times_out_without_lease(node, monkeypatch):
    published = []
    node._cmd_vel_pub.publish = published.append
    node._candidate_count = 0
    node._block_all = False
    node._autonomous_enabled = True
    node._autonomous_start_time = 0.0
    node._lease_owner_execution_id = None
    node._last_candidate_monotonic = 100.0
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
    assert status.interface_version == '1.1'
    assert status.block_all is True
    assert status.emergency_stop_active is False
    assert status.autonomous_enabled is False
    assert status.lease_owner_execution_id == ''


def test_structured_status_reflects_lease_and_estop(node):
    node._block_all = False  # Explicit test-only authorization.
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
