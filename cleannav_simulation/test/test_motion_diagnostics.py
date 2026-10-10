"""Regressions for diagnostic accounting and Gazebo-only test admission."""

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def load(filename):
    spec = importlib.util.spec_from_file_location(filename, ROOT / 'scripts' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_velocity_stats_distinguish_nonzero_from_direction_cancellation():
    stats = load('simulation_motion_diagnostics.py').VelocityStats()
    for value in [0.1, 0, -0.1, 0, 0.1, float('nan')]:
        stats.add(value)
    result = stats.summary()
    assert result['count'] == 5
    assert result['nonzero_ratio'] == 0.6
    assert result['direction_switches'] == 2
    assert result['mean'] == pytest.approx(0.02)


@pytest.mark.parametrize('key,value', [
    ('ROS_DOMAIN_ID', '0'), ('ROS_LOCALHOST_ONLY', '0'),
    ('GAZEBO_MASTER_URI', 'http://192.168.1.1:11345'),
])
def test_chassis_test_refuses_nonisolated_environment(monkeypatch, key, value):
    module = load('simulation_chassis_test.py')
    monkeypatch.setenv('ROS_DOMAIN_ID', '227')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('GAZEBO_MASTER_URI', 'http://127.0.0.1:11461')
    monkeypatch.setenv(key, value)
    with pytest.raises(ValueError):
        module.validate_environment()


def test_controller_joint_order_and_position_feedback_match_humble_interfaces():
    import yaml
    parameters = yaml.safe_load((ROOT / 'config/ackermann_controllers.yaml').read_text())
    controller = parameters['ackermann_controller']['ros__parameters']
    assert controller['rear_wheels_names'] == ['rear_right_wheel_joint', 'rear_left_wheel_joint']
    assert controller['front_wheels_names'] == [
        'front_right_steering_joint', 'front_left_steering_joint']
    assert controller['position_feedback'] is True
    # All joints must exist and supply the position feedback being configured.
    import xml.etree.ElementTree as ET
    robot = ET.parse(ROOT / 'urdf/cleannav_ackermann.urdf.xacro').getroot()
    for name in controller['front_wheels_names'] + controller['rear_wheels_names']:
        joint = robot.find(f"ros2_control/joint[@name='{name}']")
        assert joint.find("state_interface[@name='position']") is not None


@pytest.mark.parametrize('uncertain,release_ok,expected', [
    (False, True, ['zero', 'release']),
    (True, True, ['estop', 'zero', 'release']),
    (False, False, ['zero', 'release', 'estop']),
])
def test_chassis_cleanup_covers_unknown_acquire_and_release_failure(
        uncertain, release_ok, expected):
    from types import SimpleNamespace

    module = load('simulation_chassis_test.py')
    events = []

    def publish_zero(msg):
        assert msg.linear.x == 0 and msg.angular.z == 0
        events.append('zero')

    def publish_estop(msg):
        assert msg.data is True
        events.append('estop')

    def release(client, request):
        assert request.execution_id == 'simulation-chassis-test-unit'
        events.append('release')
        return SimpleNamespace(success=release_ok, message='refused')

    node = SimpleNamespace(pub=SimpleNamespace(publish=publish_zero),
                           estop=SimpleNamespace(publish=publish_estop),
                           call=release, release=object(), owner='simulation-chassis-test-unit')
    assert module.relinquish_motion(node, True, uncertain) is (release_ok and not uncertain)
    assert events == expected


def test_chassis_refused_before_lease_does_not_overwrite_another_owners_candidate():
    # No publisher/service is available on this double: rejected admission
    # must not send any candidate, release or ESTOP to a concurrent owner.
    assert load('simulation_chassis_test.py').relinquish_motion(object(), False, False)


def test_diagnostics_take_failure_is_identified_and_propagated(monkeypatch, capsys):
    from types import SimpleNamespace

    module = load('simulation_motion_diagnostics.py')
    def fail(self, subscription):
        raise RuntimeError('invalid ROS message type')
    monkeypatch.setattr(module.SingleThreadedExecutor, '_take_subscription', fail)
    executor = object.__new__(module.DiagnosticsExecutor)
    executor._context = SimpleNamespace(ok=lambda: True)
    executor._sigint_gc = None
    subscription = SimpleNamespace(topic_name='/scan', msg_type=module.LaserScan)
    with pytest.raises(RuntimeError, match='invalid ROS message type'):
        executor._take_subscription(subscription)
    assert 'LaserScan context_ok=True' in capsys.readouterr().out


def test_diagnostics_stop_inputs_before_flushing_and_destroying(monkeypatch):
    from types import SimpleNamespace

    module = load('simulation_motion_diagnostics.py')
    events = []
    timer = SimpleNamespace(cancel=lambda: events.append('timer_cancel'))
    future = SimpleNamespace(cancel=lambda: events.append('future_cancel'))
    # Node properties are read-only; use a small double with the same inputs.
    double = SimpleNamespace(timers=[timer], subscriptions=['scan'],
                             pending={'gazebo': (future, 0, 0)},
                             destroy_timer=lambda t: events.append('timer_destroy'),
                             destroy_subscription=lambda s: events.append('subscription_destroy'),
                             listener=SimpleNamespace(unregister=lambda: events.append('tf_stop')))
    module.Diagnostics.stop_inputs(double)
    assert not double.pending
    assert events == ['timer_cancel', 'timer_destroy', 'future_cancel',
                      'tf_stop', 'subscription_destroy']
