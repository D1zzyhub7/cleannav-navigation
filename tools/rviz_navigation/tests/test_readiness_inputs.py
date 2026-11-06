"""Controlled time and real input state transitions; never start a ROS node."""
import ast
import math
import sys
import types
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from readiness_inputs import InputEvidence, motion_admission, readiness_payload
from supervisor_policy import phase_readiness, readiness_loss_record


def seed():
    inputs = InputEvidence()
    inputs.observe('clock', 10_000_000_000, 1.)
    inputs.observe('odom', 10_000_000_000, 1., frame='odom')
    for topic, frame in inputs.FRAMES.items():
        if topic != 'odom':
            inputs.observe(topic, 10_000_000_000, 1., frame=frame)
    return inputs


def lookup(target, frame, stamp):
    return dict(parent=target, child=frame, stamp_ns=stamp)


def test_normal_stable_scans_clock_odom_and_exact_stamp_tf():
    inputs = seed()
    calls = []
    def query(target, source, stamp):
        calls.append((target, source, stamp))
        return lookup(target, source, stamp)
    facts, observations = inputs.snapshot(1.1, query)
    assert all(facts.values())
    assert len(calls) == 4
    assert all(stamp == 10_000_000_000 for _, _, stamp in calls)
    assert observations['/scan']['receive_age_s'] == pytest.approx(.1)


def test_new_scan_waits_for_tf_using_only_previous_still_fresh_exact_scan():
    inputs = seed()
    inputs.observe('/scan', 10_125_000_000, 1.125, frame='base_scan')
    inputs.observe('clock', 10_150_000_000, 1.15)
    def pending(target, source, stamp):
        if source == 'base_scan' and stamp > 10_000_000_000:
            raise RuntimeError('future extrapolation: transform not received yet')
        return lookup(target, source, stamp)
    facts, observations = inputs.snapshot(1.2, pending)
    assert facts['/scan_odom_tf']
    attempts = observations['/scan_odom_tf']['attempts']
    assert [a['status'] for a in attempts] == ['TF_UNAVAILABLE', 'EXACT_TIME_OK']
    # No new evidence beyond the original .3s budget: cannot latch READY.
    facts, observations = inputs.snapshot(1.31, pending)
    assert not facts['/scan_odom_tf']
    assert observations['/scan_odom_tf']['reason'] == 'exact_time_tf_unavailable'


def test_one_input_transient_and_persistent_loss_preserves_original_age_limits():
    inputs = seed()
    assert inputs.snapshot(1.299, lookup)[0]['/scan']
    facts, observations = inputs.snapshot(1.301, lookup)
    assert not facts['/scan'] and facts['/mapping/scan']
    assert observations['/scan']['reason'] == 'receive_age_exceeded'
    assert observations['/scan']['freshness_limit_s'] == .3
    inputs.observe('/scan', 10_125_000_000, 1.302, frame='base_scan')
    assert inputs.snapshot(1.303, lookup)[0]['/scan']


def _future_tf(*_args):
    raise RuntimeError(
        'Lookup would require extrapolation into the future. '
        'Requested time 69.618000 but the latest data is at time 69.603000')


def test_raw_scan_exact_tf_passes_when_the_stamp_is_already_available():
    inputs = seed()
    calls = []
    def query(target, source, stamp):
        calls.append((target, source, stamp))
        return lookup(target, source, stamp)
    facts, observations = inputs.snapshot(1.1, query)
    assert facts['/scan_odom_tf'] is True and facts['/scan_tf'] is True
    assert observations['/scan_odom_tf']['waiting_for_exact_tf'] is False
    scan_calls = [item for item in calls if item[1] == 'base_scan' and item[0] == 'odom']
    assert scan_calls == [('odom', 'base_scan', 10_000_000_000)]
    assert facts['/scan_tf'] is facts['/scan_odom_tf']


def test_raw_scan_accepts_exact_tf_that_arrives_within_fifteen_milliseconds():
    inputs = seed()
    arrived = {'odom': False}
    def query(target, source, stamp):
        if source == 'base_scan' and target == 'odom' and not arrived['odom']:
            _future_tf()
        return lookup(target, source, stamp)
    facts, observations = inputs.snapshot(1.0, query)
    assert facts['/scan_odom_tf'] is True and facts['/scan_tf'] is True
    assert observations['/scan_odom_tf']['waiting_for_exact_tf'] is True
    assert observations['/scan_odom_tf']['attempts'][0]['kind'] == 'future'
    arrived['odom'] = True
    facts, observations = inputs.snapshot(1.015, query)
    assert facts['/scan_odom_tf'] is True and facts['/scan_tf'] is True
    assert observations['/scan_odom_tf']['waiting_for_exact_tf'] is False
    assert observations['/scan_odom_tf']['attempts'][0]['status'] == 'EXACT_TIME_OK'
    assert observations['/scan_odom_tf']['attempts'][0]['stamp_ns'] == 10_000_000_000


def test_raw_scan_rejects_when_exact_tf_is_still_missing_at_fifty_milliseconds():
    inputs = seed()
    facts, _ = inputs.snapshot(1.0, _future_tf)
    assert facts['/scan_odom_tf'] is True
    facts, _ = inputs.snapshot(1.049, _future_tf)
    assert facts['/scan_odom_tf'] is True and facts['/scan_tf'] is True
    facts, observations = inputs.snapshot(1.050, _future_tf)
    assert facts['/scan_odom_tf'] is False and facts['/scan_tf'] is False
    assert observations['/scan_odom_tf']['reason'] == 'exact_time_tf_unavailable'
    assert observations['/scan_odom_tf']['waiting_for_exact_tf'] is False


def test_missing_raw_scan_tf_is_rejected_without_the_wait():
    inputs = seed()
    def missing(*_args):
        raise LookupError('"odom" passed to lookupTransform argument target_frame does not exist. ')
    facts, observations = inputs.snapshot(1.0, missing)
    assert facts['/scan_odom_tf'] is False and facts['/scan_tf'] is False
    assert observations['/scan_odom_tf']['waiting_for_exact_tf'] is False
    assert observations['/scan_odom_tf']['reason'] == 'exact_time_tf_unavailable'
    assert observations['/scan_odom_tf']['attempts'][0]['kind'] == 'missing'


def test_mapping_scan_future_tf_stays_immediate_and_raw_scan_wait_does_not_block_other_inputs():
    import time as wall
    inputs = seed()
    started = wall.monotonic()
    facts, observations = inputs.snapshot(1.0, _future_tf)
    assert wall.monotonic() - started < .02
    assert facts['/scan_odom_tf'] is True
    assert facts['/mapping/scan_odom_tf'] is False
    assert observations['/mapping/scan_odom_tf']['reason'] == 'exact_time_tf_unavailable'
    assert observations['/mapping/scan_odom_tf']['waiting_for_exact_tf'] is False
    inputs.observe('odom', 10_050_000_000, 1.01, frame='odom')
    inputs.observe('clock', 10_050_000_000, 1.01)
    facts, observations = inputs.snapshot(1.02, _future_tf)
    assert facts['odom'] is True and facts['clock'] is True
    assert observations['odom']['verified_message']['stamp_ns'] == 10_050_000_000
    assert facts['/scan_odom_tf'] is True
    assert facts['/mapping/scan_odom_tf'] is False


def test_tf_failure_records_target_stamp_and_full_exception():
    inputs = seed()
    def failed(*args):
        raise LookupError('odom -> base_scan missing at exact stamp')
    facts, observations = inputs.snapshot(1.1, failed)
    assert not facts['/scan_odom_tf']
    detail = observations['/scan_odom_tf']
    assert detail['target_frame'] == 'odom'
    assert detail['attempts'][0]['stamp_ns'] == 10_000_000_000
    assert detail['attempts'][0]['exception_type'] == 'LookupError'
    assert 'exact stamp' in detail['attempts'][0]['exception']


def test_old_message_stamp_is_not_fresh_even_when_just_received():
    inputs = seed()
    inputs.observe('clock', 12_000_000_000, 1.1)
    facts, observations = inputs.snapshot(1.11, lookup)
    assert not facts['/scan']
    assert observations['/scan']['reason'] == 'timestamp_future_or_stale'
    assert observations['/scan']['stamp_age_s'] == 2.


@pytest.mark.parametrize('kind', ['duplicate', 'frame', 'invalid'])
def test_invalid_message_does_not_refresh_valid_input(kind):
    inputs = seed()
    kwargs=dict(frame='base_scan')
    stamp=10_125_000_000
    if kind == 'duplicate': stamp=10_000_000_000
    if kind == 'frame': kwargs['frame']='wrong_frame'
    if kind == 'invalid': kwargs.update(valid=False,reason='no_valid_ranges')
    assert not inputs.observe('/scan',stamp,1.29,**kwargs)
    facts, observations=inputs.snapshot(1.31,lookup)
    assert not facts['/scan']
    assert observations['/scan']['last_message']['accepted'] is False


def test_stopped_clock_cannot_be_kept_fresh_by_duplicate_messages():
    inputs=seed()
    assert not inputs.observe('clock',10_000_000_000,2.1)
    facts, observations=inputs.snapshot(2.11,lookup)
    assert not facts['clock']
    assert observations['clock']['reason']=='receive_age_exceeded'


def test_sensor_admission_independent_of_map_tf_but_localization_stays_blocked():
    inputs=seed()
    def no_map(target,source,stamp):
        if target=='map': raise LookupError('map unavailable before initial pose')
        return lookup(target,source,stamp)
    facts,observations=inputs.snapshot(1.1,no_map)
    guard=dict(facts=facts,observations=observations,ready=False,monotonic=1.1)
    ready=phase_readiness('[SIMULATION] READY',None,guard,rviz_node=True,rviz_process=True,
                         gazebo_process=True,mission_node=False,now_monotonic=1.1)
    assert ready['dual_lidar_tf'][0]
    assert not ready['map_localization'][0]
    assert not ready['mission_manager'][0]


def test_legacy_localization_uses_low_scan_map_tf_and_roof_only_needs_odom_tf():
    inputs=seed()
    def lookup_selected(target,source,stamp):
        if source=='mapping_lidar_link' and target=='map':
            raise LookupError('map transform intentionally unavailable for non-localization roof scan')
        return lookup(target,source,stamp)
    facts,observations=inputs.snapshot(1.1,lookup_selected,localization_scan_topic='/scan')
    assert facts['/scan_map_tf'] and facts['/scan_tf']
    assert facts['/mapping/scan_odom_tf'] and facts['/mapping/scan_tf']
    assert '/mapping/scan_map_tf' not in facts
    assert observations['/mapping/scan_map_tf']['ready'] is False
    assert observations['/mapping/scan_odom_tf']['ready'] is True


def test_roof_localization_profile_keeps_roof_map_tf_as_required_fact():
    inputs=seed()
    def no_roof_map(target,source,stamp):
        if source=='mapping_lidar_link' and target=='map':
            raise LookupError('roof map transform unavailable')
        return lookup(target,source,stamp)
    facts,observations=inputs.snapshot(1.1,no_roof_map,localization_scan_topic='/mapping/scan')
    assert not facts['/mapping/scan_map_tf']
    assert not facts['/mapping/scan_tf']
    assert observations['/mapping/scan_odom_tf']['ready']


def test_ready_to_not_ready_receipt_identifies_exact_input_and_ages():
    inputs=seed()
    facts,observations=inputs.snapshot(1.1,lookup)
    previous={'facts':facts}
    facts,observations=inputs.snapshot(1.31,lookup)
    guard=dict(facts=facts,observations=observations,monotonic=1.31)
    readiness=phase_readiness('',None,guard,rviz_node=False,rviz_process=False,
        gazebo_process=True,mission_node=False,now_monotonic=1.31)
    receipt=readiness_loss_record('dual_lidar_tf',readiness,guard,previous)
    assert not receipt['ready'] and '/scan' in receipt['guard_false_facts']
    assert receipt['input_observations']['/scan']['receive_age_s']==pytest.approx(.31)
    assert receipt['input_observations']['/scan']['reason']=='receive_age_exceeded'


def test_stale_guard_file_cannot_replay_prior_true_facts():
    inputs=seed()
    facts,observations=inputs.snapshot(1.1,lookup)
    guard=dict(facts=facts,observations=observations,monotonic=1.1)
    readiness=phase_readiness('',None,guard,rviz_node=False,rviz_process=False,
        gazebo_process=True,mission_node=False,now_monotonic=1.7)
    assert not readiness['dual_lidar_tf'][0]
    assert readiness['dual_lidar_tf'][1]['guard_receipt_fresh'] is False


def test_simulation_rate_and_receiver_wall_rate_are_reported_separately():
    inputs=seed()
    inputs.observe('/scan',10_125_000_000,1.25,frame='base_scan')
    _,observations=inputs.snapshot(1.26,lookup)
    assert observations['/scan']['measured_hz']==8.
    assert observations['/scan']['measured_wall_hz']==4.


def safety(**values):
    sample = dict(block_all=True, emergency_stop_active=False,
                  autonomous_enabled=False, lease_owner_execution_id='')
    sample.update(values)
    return types.SimpleNamespace(**sample)


def payload(admission, *, ready=True, safety_present=True, estop=False,
            authorization_failed=False):
    return readiness_payload(ready=ready, missing=[] if ready else ['localized'],
        safety_present=safety_present, emergency_stop_active=estop,
        authorization_failed=authorization_failed, admission=admission)


def test_admission_stays_blocked_until_the_outer_supervisor_opens_the_interlock():
    admission = motion_admission(safety(block_all=True), now=10., received_at=9.9)
    assert admission['admitted'] is False and admission['reason'] == 'interlock_closed'
    assert admission['source'] == 'safety_simulation_interlock'
    published = payload(admission)
    assert published['ready'] is True and published['motion_enabled'] is False


def test_admission_requires_a_fresh_safety_sample():
    assert motion_admission(None, now=10., received_at=-math.inf)['reason'] == 'no_safety_sample'
    opening = payload(motion_admission(None, now=10., received_at=-math.inf),
                      safety_present=False)
    assert opening['ready'] is False and opening['motion_enabled'] is False
    stale = motion_admission(safety(block_all=False), now=11.6, received_at=10.)
    assert stale['admitted'] is False and stale['reason'] == 'safety_sample_stale'
    assert payload(stale)['motion_enabled'] is False
    # Exactly the Mission freshness bound is still admissible evidence.
    assert motion_admission(safety(block_all=False), now=11.5,
                            received_at=10.)['admitted'] is True


def test_admitted_interlock_enables_motion_and_revocation_clears_it():
    sample = safety(block_all=False)
    admitted = motion_admission(sample, now=10., received_at=9.9)
    assert admitted['admitted'] is True and admitted['reason'] is None
    assert payload(admitted)['motion_enabled'] is True
    sample.block_all = True
    revoked = motion_admission(sample, now=10.5, received_at=10.4)
    assert revoked['admitted'] is False and revoked['reason'] == 'interlock_closed'
    assert payload(revoked)['motion_enabled'] is False


def test_estop_incomplete_or_unready_state_never_enables_motion():
    estop = motion_admission(safety(block_all=False, emergency_stop_active=True),
                             now=10., received_at=9.9)
    assert estop['admitted'] is False and estop['reason'] == 'emergency_stop_active'
    assert payload(estop)['motion_enabled'] is False
    incomplete = motion_admission(types.SimpleNamespace(block_all=False),
                                  now=10., received_at=9.9)
    assert incomplete['reason'] == 'incomplete_safety_sample'
    admitted = motion_admission(safety(block_all=False), now=10., received_at=9.9)
    assert payload(admitted, ready=False)['motion_enabled'] is False
    assert payload(admitted, estop=True)['motion_enabled'] is False
    assert payload(admitted, authorization_failed=True)['motion_enabled'] is False


def test_admission_records_the_observation_the_next_run_needs():
    admission = motion_admission(safety(block_all=False, autonomous_enabled=False,
                                        lease_owner_execution_id=''), now=10.,
                                 received_at=9.75)
    assert admission['safety_age_s'] == pytest.approx(.25)
    assert admission['block_all'] is False and admission['emergency_stop_active'] is False
    assert admission['autonomous_enabled'] is False
    assert admission['lease_owner_execution_id'] == ''
    assert admission['max_age_s'] == pytest.approx(1.5)
    assert payload(admission)['motion_admission']['admitted'] is True


def test_localization_panel_reports_observed_admission_without_self_authorizing():
    tree = ast.parse((Path(__file__).resolve().parents[1]
                      / 'localization_status_monitor.py').read_text())
    monitor = next(node for node in tree.body
                   if isinstance(node, ast.ClassDef) and node.name == 'LocalizationMonitor')
    panel = next(node for node in monitor.body
                 if isinstance(node, ast.FunctionDef) and node.name == 'panel')
    calls = {ast.unparse(node.func) for node in ast.walk(panel) if isinstance(node, ast.Call)}
    assert 'readiness_payload' in calls and 'self.motion_admission' in calls
    assert not any('motion_client' in name for name in calls)
