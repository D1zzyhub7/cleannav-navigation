"""Real LocalizationMonitor.panel() and real Mission admission; no ROS node.

Both objects are built with ``object.__new__`` so the shipped ``panel()``,
``motion_admission()`` and ``NavigationGoalService.admission()`` run unchanged
while no node, subscription, service, timer, launch, Lease or authorization is
created. Only the node plumbing (publishers, TF, action/service readiness) is
replaced.
"""
import json
import sys
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import localization_status_monitor as lsm
from readiness_inputs import MOTION_ADMISSION_MAX_AGE_S
from simulation_status_monitor import SimulationStatusMonitor, missing_requirements

try:  # The audited Mission overlay must be importable for the contract tests.
    from cleannav_mission_manager.navigation_goal_service import NavigationGoalService
    from cleannav_mission_manager.domain.command_processing import CommandReason
except Exception:  # pragma: no cover - only when that overlay is absent
    NavigationGoalService = CommandReason = None

mission_only = pytest.mark.skipif(
    NavigationGoalService is None, reason='audited Mission Manager overlay not importable')

ADMITTED = 'motion admitted by the navigation supervisor'


def safety(**values):
    """A SafetyStatus-shaped sample; block_all defaults to the safe state."""
    sample = dict(block_all=True, emergency_stop_active=False,
                  autonomous_enabled=False, lease_owner_execution_id='')
    sample.update(values)
    return types.SimpleNamespace(**sample)


def complete_facts(**overrides):
    facts = {key: True for stage in lsm.STAGES for key in missing_requirements(stage, {})}
    facts.update(map=True, localized=True, frozen_map=True, map_server_active=True)
    facts.update(overrides)
    return facts


class Panel:
    """The real monitor with a fake publisher and controllable facts."""

    def __init__(self, *, enable_motion=False):
        monitor = object.__new__(lsm.LocalizationMonitor)
        monitor.enable_motion = enable_motion
        monitor.authorization_failed = False
        monitor.ever_ready = False
        monitor.last_panel = {}
        monitor._quiet_panel = True
        monitor.received = {}
        monitor.safety = None
        monitor.facts_values = complete_facts()
        monitor.facts = lambda: dict(monitor.facts_values)
        self.monitor = monitor
        self.published = []
        self.transitions = []
        monitor.readiness_pub = types.SimpleNamespace(publish=self._publish)
        inherited_changed = monitor._changed

        def record(key, state, detail, label=None):
            self.transitions.append((key, state, detail))
            inherited_changed(key, state, detail, label)

        monitor._changed = record

    def _publish(self, message):
        self.published.append(json.loads(message.data))

    def observe(self, sample, *, age=0.0):
        self.monitor.safety = sample
        self.monitor.received['safety'] = time.monotonic() - age

    def panel(self):
        self.monitor.panel()
        return self.published[-1]


def test_panel_reports_no_motion_before_the_supervisor_opens_the_interlock():
    panel = Panel()
    panel.observe(safety(block_all=True))
    payload = panel.panel()
    assert payload['ready'] is True and payload['missing'] == []
    assert payload['motion_enabled'] is False
    assert payload['motion_admission']['source'] == 'safety_simulation_interlock'
    assert payload['motion_admission']['reason'] == 'interlock_closed'
    assert ADMITTED not in panel.transitions[-1][2]


def test_panel_reports_motion_only_from_the_observed_open_interlock():
    panel = Panel()
    panel.observe(safety(block_all=False))
    payload = panel.panel()
    admission = payload['motion_admission']
    assert payload['motion_enabled'] is True
    assert admission['admitted'] is True and admission['reason'] is None
    assert admission['safety_age_s'] <= admission['max_age_s'] == 1.5
    assert panel.transitions[-1][:2] == ('CLEANNAV', 'READY')
    assert ADMITTED in panel.transitions[-1][2]
    # One published contract per panel; nothing else is emitted.
    assert len(panel.published) == 1


def test_panel_keeps_motion_admitted_while_mission_holds_the_lease():
    """The Lease is acquired after admission; owning it must not revoke."""
    panel = Panel()
    panel.observe(safety(block_all=False, autonomous_enabled=True,
                         lease_owner_execution_id='ros-execution-1'))
    payload = panel.panel()
    assert payload['motion_enabled'] is True
    assert payload['motion_admission']['lease_owner_execution_id'] == 'ros-execution-1'
    assert payload['motion_admission']['autonomous_enabled'] is True


def test_panel_revokes_motion_on_every_revocation_evidence():
    panel = Panel()
    panel.observe(safety(block_all=False))
    assert panel.panel()['motion_enabled'] is True

    # The same sample object now carries a closed interlock (new message).
    panel.monitor.safety.block_all = True
    revoked = panel.panel()
    assert revoked['motion_enabled'] is False
    assert revoked['motion_admission']['reason'] == 'interlock_closed'

    # Interlock still open, but Safety latched an ESTOP.
    panel.observe(safety(block_all=False, emergency_stop_active=True))
    estop = panel.panel()
    assert estop['ready'] is False and estop['motion_enabled'] is False
    assert estop['motion_admission']['reason'] == 'emergency_stop_active'

    # No new sample for longer than the Mission freshness bound.
    panel.observe(safety(block_all=False), age=1.6)
    stale = panel.panel()
    assert stale['motion_enabled'] is False
    assert stale['motion_admission']['reason'] == 'safety_sample_stale'
    assert panel.panel()['motion_enabled'] is False


def test_panel_motion_never_outruns_localization_readiness():
    panel = Panel()
    panel.observe(safety(block_all=False))
    panel.monitor.facts_values = complete_facts(localized=False)
    blocked = panel.panel()
    assert blocked['ready'] is False and blocked['motion_enabled'] is False
    assert 'localized' in blocked['missing']
    assert blocked['motion_admission']['admitted'] is True
    assert panel.transitions[-1][:2] == ('CLEANNAV', 'PARTIAL_READY')
    assert panel.panel()['ready'] is False

    panel.monitor.facts_values = complete_facts()
    recovered = panel.panel()
    assert recovered['ready'] is True and recovered['motion_enabled'] is True
    assert panel.transitions[-1][:2] == ('CLEANNAV', 'READY')


def test_panel_explicit_enable_motion_still_delegates_to_the_inherited_panel(monkeypatch):
    calls = []

    def inherited(self):
        calls.append(1)
        self.readiness_pub.publish(
            types.SimpleNamespace(data=json.dumps({'inherited': True})))

    monkeypatch.setattr(SimulationStatusMonitor, 'panel', inherited)
    panel = Panel(enable_motion=True)
    panel.observe(safety(block_all=False))
    assert panel.panel() == {'inherited': True}
    assert calls == [1]


def test_monitor_never_issues_the_safety_interlock_itself():
    source = (Path(__file__).resolve().parents[1] / 'localization_status_monitor.py').read_text()
    assert 'set_simulation_motion_enabled' not in source
    assert 'SetBool' not in source
    panel = Panel()
    assert not hasattr(panel.monitor, 'motion_client')
    assert not hasattr(panel.monitor, 'safety_client')
    assert MOTION_ADMISSION_MAX_AGE_S == 1.5


def mission_gate(payload, sample, *, safety_age=0.0, motion_enabled=True):
    """The shipped Mission admission gate with only node readiness faked."""
    service = object.__new__(NavigationGoalService)
    service.motion_enabled = motion_enabled
    service.safety = sample
    service.safety_time = time.monotonic() - safety_age
    service.readiness = payload
    service.readiness_time = time.monotonic()
    ready = lambda: True  # noqa: E731 - action/service readiness is not under test
    service.node = types.SimpleNamespace(runtime=types.SimpleNamespace(
        navigation=types.SimpleNamespace(
            _action_client=types.SimpleNamespace(server_is_ready=ready)),
        safety=types.SimpleNamespace(
            _acquire_client=types.SimpleNamespace(service_is_ready=ready),
            _release_client=types.SimpleNamespace(service_is_ready=ready))))
    return service.admission()


@mission_only
def test_mission_accepts_the_published_admission_and_follows_its_revocation():
    panel = Panel()
    panel.observe(safety(block_all=False))
    admitted = panel.panel()
    assert mission_gate(admitted, panel.monitor.safety) is None

    panel.observe(safety(block_all=True))
    revoked = panel.panel()
    # Safety is still open for Mission; only the published contract blocks it.
    assert mission_gate(revoked, safety(block_all=False)) == CommandReason.POSE_NOT_READY
    assert mission_gate(admitted, safety(block_all=True)) == CommandReason.POSE_SAFETY_BLOCKED


@mission_only
def test_mission_keeps_its_own_safety_and_launch_gates():
    panel = Panel()
    panel.observe(safety(block_all=False))
    admitted = panel.panel()
    assert mission_gate(admitted, safety(block_all=False, emergency_stop_active=True)) \
        == CommandReason.POSE_ESTOP
    assert mission_gate(admitted, safety(block_all=False), safety_age=1.6) \
        == CommandReason.POSE_SAFETY_BLOCKED
    assert mission_gate(admitted, safety(block_all=False), motion_enabled=False) \
        == CommandReason.POSE_MOTION_DISABLED
    stale_readiness = dict(admitted, ready=False)
    assert mission_gate(stale_readiness, safety(block_all=False)) \
        == CommandReason.POSE_NOT_READY


@mission_only
def test_mission_gate_contract_is_still_the_one_this_test_mirrors():
    source = Path(sys.modules[NavigationGoalService.__module__].__file__).read_text()
    for expression in ("self.readiness.get('motion_enabled')",
                       "self.readiness.get('ready')",
                       'self.safety.block_all',
                       'self.safety.emergency_stop_active',
                       '1.5', '2.0'):
        assert expression in source, expression
