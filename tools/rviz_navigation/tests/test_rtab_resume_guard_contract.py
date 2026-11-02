"""The RTAB resume barrier follows the odometry source, not the wheel mount.

Only the shipped ``MappingMonitor.odom_cb``, ``poll``, ``tf`` and
``resume_when_ready`` run. The node is built with ``object.__new__``, so no
node, subscription, timer, service, launch, Lease or authorization is created.
"""
from collections import deque
from pathlib import Path
import ast
import sys
import time
import types

from builtin_interfaces.msg import Time as TimeMessage
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mapping_status_monitor as msm

HERE = Path(__file__).resolve().parents[1]
# The barrier compares the latest TF stamp against the node clock.
NOW_NS = 10_000_000_000


def transform(x=0., stamp=(10, 0)):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=stamp[0], nanosec=stamp[1])),
        transform=types.SimpleNamespace(
            translation=types.SimpleNamespace(x=x, y=0., z=0.)))


class Buffer:
    """Only the links a run actually publishes; everything else is missing."""

    def __init__(self, links):
        self.links = dict(links)

    def lookup_transform(self, target, source, stamp=None):
        if (target, source) not in self.links:
            raise msm.TransformException(f'{target} <- {source} is missing')
        return self.links[(target, source)]


class Future:
    def __init__(self, response=None):
        self.response = response

    def done(self):
        return True

    def result(self):
        return self.response


class Client:
    """Answers immediately and records calls; never touches DDS."""

    def __init__(self, responses=(), on_call=None):
        self.responses = list(responses)
        self.calls = 0
        self.on_call = on_call

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls += 1
        if self.on_call:
            self.on_call(self.calls)
        return Future(self.responses.pop(0))


def paused(value):
    return types.SimpleNamespace(values=[types.SimpleNamespace(type=1, bool_value=value)])


def odom(child_frame, frame='odom'):
    return types.SimpleNamespace(header=types.SimpleNamespace(frame_id=frame),
                                 child_frame_id=child_frame)


def scan(stamp_sec):
    return types.SimpleNamespace(header=types.SimpleNamespace(
        stamp=TimeMessage(sec=stamp_sec, nanosec=0)))


def monitor(*, odometry_mode, links=(), resume_rtab=True, stage='wait-inputs'):
    node = object.__new__(msm.MappingMonitor)
    node.odometry_mode = odometry_mode
    node.odom_child_frame = ('base_footprint' if odometry_mode == 'oracle'
                             else 'ackermann_rear_axle')
    node.session = '/static-only/session.json'
    node.resume_rtab = resume_rtab
    node.received = {}
    node.scan = None
    node.scan_history = deque(maxlen=8)
    node.grid = None
    node.safety = None
    node.controllers = True
    node.controller_future = None
    node.controller_client = types.SimpleNamespace(service_is_ready=lambda: False)
    node.facts = {}
    node.last_panel = None
    node.buffer = Buffer(links)
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=NOW_NS))
    node.boot_stage = stage
    node.boot_future = None
    node.boot_deadline = 0.
    node.boot_rpc_attempts = 0
    node.boot_error = None
    node.boot_complete = False
    node.good_input_samples = 0
    node.last_input_stamp = None
    node.rtab_params = Client([paused(True), paused(False)])
    node.rtab_resume = Client([None])
    return node


def feed(node, stamp_sec):
    """One scan sample plus the sensor inputs every run publishes."""
    node.scan = scan(stamp_sec)
    node.scan_history.append((time.monotonic(), TimeMessage(sec=stamp_sec, nanosec=0)))
    node.mark('scan')
    for name in ('clock', 'joints', 'safety'):
        node.mark(name)
    node.safety = types.SimpleNamespace(emergency_stop_active=False)
    node.odom_cb(odom(node.odom_child_frame))


def accepted(node, message):
    before = node.received.get('odom')
    msm.MappingMonitor.odom_cb(node, message)
    return node.received.get('odom') != before

WHEEL_LINKS = {('ackermann_rear_axle', 'base_footprint'): transform(.30),
               ('odom', 'ackermann_rear_axle'): transform(),
               ('odom', 'base_scan'): transform()}
ORACLE_LINKS = {('odom', 'base_footprint'): transform(),
                ('odom', 'base_scan'): transform()}




@pytest.mark.parametrize('odometry_mode,accepted_child,rejected_child', [
    ('oracle', 'base_footprint', 'ackermann_rear_axle'),  # Route A: RTAB + Oracle
    ('wheel', 'ackermann_rear_axle', 'base_footprint'),   # historical disk-map RTAB
])
def test_the_selected_odometry_source_defines_the_accepted_odom_frame(
        odometry_mode, accepted_child, rejected_child):
    node = monitor(odometry_mode=odometry_mode)
    assert accepted(node, odom(accepted_child))
    assert not accepted(node, odom(rejected_child))
    assert not accepted(node, odom(accepted_child, frame='map'))


def test_the_odom_contract_is_read_from_the_recorded_odometry_source():
    tree = ast.parse((HERE / 'mapping_status_monitor.py').read_text())
    klass = next(node for node in tree.body
                 if isinstance(node, ast.ClassDef) and node.name == 'MappingMonitor')
    init = next(node for node in klass.body
                if isinstance(node, ast.FunctionDef) and node.name == '__init__')
    assigned = {ast.unparse(target): ast.unparse(node.value)
                for node in ast.walk(init) if isinstance(node, ast.Assign)
                for target in node.targets}
    assert assigned['self.odometry_mode'] == (
        "json.loads(Path(session).read_text()).get('odometry_mode', 'wheel')")


@pytest.mark.parametrize('odometry_mode,links', [
    ('oracle', ORACLE_LINKS),
    ('wheel', WHEEL_LINKS),
])
def test_each_mode_passes_its_own_body_frame_barrier(odometry_mode, links):
    node = monitor(odometry_mode=odometry_mode, links=links)
    feed(node, 1)
    msm.MappingMonitor.poll(node)
    assert node.facts['rear_axle_tf'] and node.facts['scan_tf']
    assert node.facts['clock'] and node.facts['vehicle'] and node.facts['scan']
    assert node.facts['odom'] and node.facts['safety']
    assert node.good_input_samples == 1


def test_oracle_mode_needs_no_rear_axle_frame_or_mount():
    # ORACLE_LINKS contains no ackermann_rear_axle link at all.
    node = monitor(odometry_mode='oracle', links=ORACLE_LINKS)
    feed(node, 1)
    msm.MappingMonitor.poll(node)
    assert node.facts['rear_axle_tf'] and not node.boot_error


@pytest.mark.parametrize('links', [
    {('odom', 'ackermann_rear_axle'): transform(), ('odom', 'base_scan'): transform()},
    {('ackermann_rear_axle', 'base_footprint'): transform(.25),
     ('odom', 'ackermann_rear_axle'): transform(), ('odom', 'base_scan'): transform()},
])
def test_wheel_mode_keeps_the_missing_and_shifted_mount_rejections(links):
    node = monitor(odometry_mode='wheel', links=links)
    feed(node, 1)
    msm.MappingMonitor.poll(node)
    assert not node.facts['rear_axle_tf']
    assert node.good_input_samples == 0


def test_oracle_mode_rejects_a_missing_odom_to_footprint_chain():
    node = monitor(odometry_mode='oracle',
                   links={('odom', 'base_scan'): transform()})
    feed(node, 1)
    msm.MappingMonitor.poll(node)
    assert not node.facts['rear_axle_tf'] and node.good_input_samples == 0


@pytest.mark.parametrize('odometry_mode,links', [
    ('oracle', ORACLE_LINKS),
    ('wheel', WHEEL_LINKS),
])
@pytest.mark.parametrize('missing', ['scan', 'scan_tf', 'safety', 'odom'])
def test_no_mode_resumes_without_scan_scan_tf_safety_or_odom(odometry_mode, links, missing):
    node = monitor(odometry_mode=odometry_mode, links=links, stage='wait-inputs')
    node.rtab_resume = Client([None] * 4)
    for stamp_sec in range(1, 5):
        feed(node, stamp_sec)
        if missing == 'scan':
            node.scan = None
            node.scan_history.clear()
            node.received.pop('scan', None)
        elif missing == 'scan_tf':
            node.buffer.links.pop(('odom', 'base_scan'), None)
        elif missing == 'safety':
            node.received.pop('safety', None)
        else:
            node.received.pop('odom', None)
        msm.MappingMonitor.poll(node)
    assert node.rtab_resume.calls == 0
    assert node.good_input_samples == 0 and not node.boot_complete


def test_three_consecutive_good_samples_resume_rtab_exactly_once():
    node = monitor(odometry_mode='oracle', links=ORACLE_LINKS, stage='check-paused')
    samples = []
    node.rtab_resume = Client([None], on_call=lambda _: samples.append(node.good_input_samples))
    for stamp_sec in range(1, 6):
        feed(node, stamp_sec)
        msm.MappingMonitor.poll(node)
    assert node.boot_complete and node.boot_error is None
    assert node.rtab_resume.calls == 1 and samples == [3]
    assert node.rtab_params.calls == 2
