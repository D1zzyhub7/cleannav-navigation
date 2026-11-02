"""Timestamped input/TF evidence. No ROS dependency, authority, or cached READY.

An arriving scan can precede its exact-time TF. Query recent real scans newest
first, accepting only an exact-time transform inside the original freshness
budget. A prior scan is never a substitute beyond that same budget.
"""
from collections import deque
import math


def _tf_exception_kind(error):
    """Classify a failed exact-time lookup without importing tf2."""
    text = str(error)
    name = type(error).__name__
    if name in ('LookupException', 'LookupError') or 'does not exist' in text:
        return 'missing'
    if 'into the future' in text:
        return 'future'
    return 'other'


class InputEvidence:
    LIMITS = {'/scan': .3, '/mapping/scan': 1., 'clock': 1., 'odom': 1.}
    FRAMES = {'/scan': 'base_scan', '/mapping/scan': 'mapping_lidar_link', 'odom': 'odom'}
    # Raw /scan can be stamped a few milliseconds ahead of the odom TF sample.
    # Keep the decision open across Guard ticks instead of sleeping inside one.
    SCAN_EXACT_TF_WAIT_S = .05

    def __init__(self):
        self.valid = {}
        self.last = {}
        self.counts = {}
        self.first_valid = {}
        self.accepted_counts = {}
        self.maximum_stamp_gap_s = {}
        self.pending = {topic: deque(maxlen=32) for topic in ('/scan', '/mapping/scan')}
        self.scan_exact_wait = {}

    def observe(self, key, stamp_ns, now, *, frame=None, valid=True, reason=None, **metadata):
        self.counts[key] = self.counts.get(key, 0) + 1
        previous = self.valid.get(key)
        if not isinstance(stamp_ns, int) or stamp_ns <= 0:
            valid, reason = False, 'invalid_or_zero_timestamp'
        elif previous and stamp_ns <= previous['stamp_ns']:
            valid, reason = False, 'non_increasing_timestamp'
        elif key in self.FRAMES and frame != self.FRAMES[key]:
            valid, reason = False, 'frame_mismatch'
        record = dict(stamp_ns=stamp_ns, received_monotonic=now, frame_id=frame,
                      accepted=bool(valid), reason=reason if not valid else None, **metadata)
        self.last[key] = record
        if valid:
            self.first_valid.setdefault(key, record)
            self.accepted_counts[key] = self.accepted_counts.get(key, 0) + 1
            if previous:
                self.maximum_stamp_gap_s[key] = max(self.maximum_stamp_gap_s.get(key, 0.),
                    (stamp_ns-previous['stamp_ns'])/1e9)
            self.valid[key] = record
            if key in self.pending:
                self.pending[key].append(record)
        return bool(valid)

    def _fresh(self, key, record, now, clock_ns):
        if record is None:
            return dict(ready=False, reason='no_valid_message', received_count=self.counts.get(key, 0),
                        last_message=self.last.get(key), receive_age_s=None, stamp_age_s=None)
        receive_age = now - record['received_monotonic']
        stamp_age = (clock_ns-record['stamp_ns'])/1e9 if clock_ns is not None else None
        limit = self.LIMITS[key]
        stamp_span = (self.valid[key]['stamp_ns']-self.first_valid[key]['stamp_ns'])/1e9
        wall_span = self.valid[key]['received_monotonic']-self.first_valid[key]['received_monotonic']
        reason = None
        if not math.isfinite(receive_age) or not 0 <= receive_age <= limit:
            reason = 'receive_age_exceeded'
        elif key != 'clock' and stamp_age is None:
            reason = 'clock_unavailable'
        elif key != 'clock' and not -.25 <= stamp_age <= limit:
            reason = 'timestamp_future_or_stale'
        return dict(ready=reason is None, reason=reason, freshness_limit_s=limit,
                    receive_age_s=receive_age, stamp_age_s=stamp_age,
                    received_count=self.counts.get(key, 0),
                    accepted_count=self.accepted_counts.get(key, 0),
                    measured_hz=((self.accepted_counts[key]-1)/stamp_span if stamp_span>0 else None),
                    measured_wall_hz=((self.accepted_counts[key]-1)/wall_span if wall_span>0 else None),
                    maximum_stamp_gap_s=self.maximum_stamp_gap_s.get(key),
                    last_message=self.last.get(key), verified_message=record)

    def snapshot(self, now, lookup, *, localization_scan_topic='/mapping/scan'):
        """lookup(target, source, stamp_ns) must query that exact ROS timestamp."""
        if localization_scan_topic not in ('/scan', '/mapping/scan'):
            raise ValueError('unsupported localization scan topic')
        clock = self.valid.get('clock')
        clock_ns = clock['stamp_ns'] if clock else None
        observations = {key: self._fresh(key, self.valid.get(key), now, clock_ns)
                        for key in self.LIMITS}
        facts = {key: detail['ready'] for key, detail in observations.items()}
        for topic, records in self.pending.items():
            for target in ('odom', 'map'):
                key = topic + '_' + target + '_tf'
                attempts = []
                verified = None
                future_stamp = None
                missing_frame = False
                for record in reversed(records):
                    fresh = self._fresh(topic, record, now, clock_ns)
                    if not fresh['ready']:
                        continue
                    try:
                        transform = lookup(target, record['frame_id'], record['stamp_ns'])
                        attempts.append(dict(stamp_ns=record['stamp_ns'], status='EXACT_TIME_OK',
                                             transform=transform))
                        verified = fresh
                        break
                    except Exception as error:
                        kind = _tf_exception_kind(error)
                        attempts.append(dict(stamp_ns=record['stamp_ns'], status='TF_UNAVAILABLE',
                                             exception_type=type(error).__name__, exception=str(error),
                                             kind=kind))
                        if kind == 'missing':
                            missing_frame = True
                        elif kind == 'future' and future_stamp is None:
                            future_stamp = record['stamp_ns']
                ready = verified is not None and facts[topic] and facts['clock']
                reason = None if ready else ('exact_time_tf_unavailable' if attempts else 'no_fresh_scan')
                waiting = False
                # Only the raw scan gets a bounded retry. A missing frame cannot
                # arrive, and /mapping/scan keeps the immediate exact-time result.
                if (not ready and topic == '/scan' and future_stamp is not None
                        and not missing_frame and facts[topic] and facts['clock']):
                    wait_key = (target, future_stamp)
                    started = self.scan_exact_wait.setdefault(wait_key, now)
                    if now - started < self.SCAN_EXACT_TF_WAIT_S:
                        ready = True
                        reason = None
                        waiting = True
                elif ready:
                    self.scan_exact_wait.pop((target, verified['verified_message']['stamp_ns']), None)
                observations[key] = dict(ready=ready, target_frame=target,
                    source_frame=self.FRAMES[topic], attempts=attempts, reason=reason,
                    verified_scan=verified, waiting_for_exact_tf=waiting)
                facts[key] = ready
            # The map transform belongs to the configured localization sensor.
            # The other lidar must still have an exact-time transform to odom,
            # but requiring its map transform would couple unrelated sensors.
            if topic != localization_scan_topic:
                facts.pop(topic+'_map_tf', None)
            facts[topic+'_tf'] = facts[topic+'_odom_tf'] and (
                facts[topic+'_map_tf'] if topic == localization_scan_topic else True)
        return facts, observations


MOTION_ADMISSION_SOURCE = 'safety_simulation_interlock'
# Mission rejects a Safety sample older than 1.5s. A longer local bound would
# report an admission that the Mission gate can never accept. The integrated
# launch publishes SafetyStatus at 10 Hz (2 Hz legacy), so 1.5s is 15 periods:
# it tolerates scheduler jitter without hiding a stopped publisher for longer
# than the Mission gate itself would.
MOTION_ADMISSION_MAX_AGE_S = 1.5


def _sample_field(sample, name):
    if hasattr(sample, name):
        return getattr(sample, name)
    return sample.get(name) if hasattr(sample, 'get') else None


def motion_admission(sample, *, now, received_at, max_age=MOTION_ADMISSION_MAX_AGE_S):
    """Report the interlock state the outer supervisor already established.

    The integrated entry has exactly one owner for Motion Admission and
    expresses it through the existing simulation interlock. A monitor that
    issued the same service call would be a second authority, so the only
    state it may report is the one it observes: a fresh Safety sample with the
    interlock open and no ESTOP. Missing, stale, closed, or ESTOP evidence is
    never admitted.

    ``Safety.block_all`` is the enforcement point, so this is evidence about
    permission to move, not proof that the Supervisor's own admission receipt
    still holds: Safety has no caller identity check and does not read the
    guard generation. An owned Lease is expected after Mission acceptance and
    must not clear the flag.
    """
    block_all = _sample_field(sample, 'block_all')
    estop = _sample_field(sample, 'emergency_stop_active')
    detail = dict(source=MOTION_ADMISSION_SOURCE, max_age_s=max_age,
                  block_all=None if block_all is None else bool(block_all),
                  emergency_stop_active=None if estop is None else bool(estop),
                  autonomous_enabled=_sample_field(sample, 'autonomous_enabled'),
                  lease_owner_execution_id=_sample_field(sample, 'lease_owner_execution_id'))
    if sample is None:
        return dict(detail, admitted=False, reason='no_safety_sample', safety_age_s=None)
    age = now - received_at
    detail['safety_age_s'] = age if math.isfinite(age) else None
    if not math.isfinite(age) or age < 0 or age > max_age:
        return dict(detail, admitted=False, reason='safety_sample_stale')
    if block_all is None or estop is None:
        return dict(detail, admitted=False, reason='incomplete_safety_sample')
    if estop:
        return dict(detail, admitted=False, reason='emergency_stop_active')
    if block_all:
        return dict(detail, admitted=False, reason='interlock_closed')
    return dict(detail, admitted=True, reason=None)


def readiness_payload(*, ready, missing, safety_present, emergency_stop_active,
                      authorization_failed, admission):
    """One published readiness contract; motion never outruns readiness."""
    ready_flag = bool(ready and safety_present and not emergency_stop_active
                      and not authorization_failed)
    return {
        'ready': ready_flag,
        'motion_enabled': bool(ready_flag and admission.get('admitted') is True),
        'missing': list(missing),
        'motion_admission': dict(admission),
    }


def amcl_bootstrap_ready(state, age):
    if not isinstance(state,dict) or state.get('localization_kind')!='amcl':
        return False
    return (math.isfinite(age) and 0<=age<=1.5
        and all(state.get(k) is True for k in ('localized','parameters_verified',
            'assets_unchanged','sensors_fresh','frozen_map','map_server_active',
            'amcl_active','authority_ok','map_tf_valid','scan_tf_valid'))
        and isinstance(state.get('pose_count'),int) and state['pose_count']>=1)
