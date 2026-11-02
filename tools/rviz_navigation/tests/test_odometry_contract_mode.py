"""The /odom child frame belongs to the odometry source, not to the localizer.

Only the shipped ``LocalizationMonitor._odom``, ``_mark`` and ``_fresh`` run.
The node is built with ``object.__new__``, so no node, subscription, timer,
service, launch, Lease or authorization is created.
"""
from collections import deque
from pathlib import Path
import ast
import math
import sys
import types

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import localization_status_monitor as lsm

HERE = Path(__file__).resolve().parents[1]


def odom(child_frame, *, frame='odom', quaternion=(0., 0., 0., 1.),
         position=(0., 0., 0.)):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id=frame), child_frame_id=child_frame,
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=position[0], y=position[1], z=position[2]),
            orientation=types.SimpleNamespace(x=quaternion[0], y=quaternion[1],
                                              z=quaternion[2], w=quaternion[3]))))


def monitor(*, odometry_mode, amcl_mode=False):
    node = object.__new__(lsm.LocalizationMonitor)
    node.odometry_mode = odometry_mode
    node.amcl_mode = amcl_mode
    node.received = {}
    node.odom_pose = None
    node.odom_history = deque(maxlen=300)
    return node


def accepted(node, message):
    """Dispatch one sample and report whether it re-marked the odom input."""
    before = node.received.get('odom')
    lsm.LocalizationMonitor._odom(node, message)
    return node.received.get('odom') != before


@pytest.mark.parametrize('amcl_mode,odometry_mode,child', [
    (False, 'oracle', 'base_footprint'),       # Route A: RTAB-Map with Oracle
    (False, 'wheel', 'ackermann_rear_axle'),   # disk-map RTAB-Map with wheel
    (True, 'oracle', 'base_footprint'),        # audited AMCL with Oracle
    (True, 'wheel', 'ackermann_rear_axle'),
])
def test_the_selected_odometry_source_defines_the_accepted_odom_frame(
        amcl_mode, odometry_mode, child):
    node = monitor(odometry_mode=odometry_mode, amcl_mode=amcl_mode)
    assert accepted(node, odom(child))
    assert node.odom_pose == (0., 0., 0.) and len(node.odom_history) == 1


@pytest.mark.parametrize('odometry_mode,wrong_child', [
    ('oracle', 'ackermann_rear_axle'),
    ('wheel', 'base_footprint'),
])
def test_the_other_sources_frame_is_rejected_in_each_mode(odometry_mode, wrong_child):
    node = monitor(odometry_mode=odometry_mode)
    assert not accepted(node, odom(wrong_child))
    assert node.odom_pose is None and not node.odom_history


def test_localization_kind_never_selects_the_odom_contract():
    # The same odometry source must produce the same verdict for both localizers.
    for amcl_mode in (False, True):
        for mode, child in (('oracle', 'base_footprint'),
                            ('wheel', 'ackermann_rear_axle')):
            assert accepted(monitor(odometry_mode=mode, amcl_mode=amcl_mode), odom(child))
            other = 'ackermann_rear_axle' if child == 'base_footprint' else 'base_footprint'
            assert not accepted(monitor(odometry_mode=mode, amcl_mode=amcl_mode), odom(other))


@pytest.mark.parametrize('message', [
    odom('base_footprint', frame='map'),
    odom('base_footprint', quaternion=(0., 0., 0., 0.)),
    odom('base_footprint', quaternion=(0., 0., 0., math.nan)),
    odom('base_footprint', position=(math.inf, 0., 0.)),
    odom('base_footprint', position=(0., 0., math.nan)),
])
def test_invalid_odometry_samples_stay_unaccepted(message):
    node = monitor(odometry_mode='oracle')
    assert not accepted(node, message)
    assert node.odom_pose is None and not node.odom_history


def test_a_rejected_sample_never_moves_the_odometry_anchor():
    node = monitor(odometry_mode='oracle')
    assert accepted(node, odom('base_footprint', position=(1., 2., 0.)))
    assert node.odom_pose == (1., 2., 0.)
    assert not accepted(node, odom('ackermann_rear_axle', position=(9., 9., 0.)))
    assert node.odom_pose == (1., 2., 0.) and len(node.odom_history) == 1


def test_the_contract_is_read_from_the_recorded_odometry_source():
    tree = ast.parse((HERE / 'localization_status_monitor.py').read_text())
    klass = next(node for node in tree.body
                 if isinstance(node, ast.ClassDef) and node.name == 'LocalizationMonitor')
    init = next(node for node in klass.body
                if isinstance(node, ast.FunctionDef) and node.name == '__init__')
    assigned = {ast.unparse(target): ast.unparse(node.value)
                for node in ast.walk(init) if isinstance(node, ast.Assign)
                for target in node.targets}
    assert assigned['self.odometry_mode'] == "self.session.get('odometry_mode', 'wheel')"
    body = ast.unparse(next(node for node in klass.body
                            if isinstance(node, ast.FunctionDef) and node.name == '_odom'))
    assert 'amcl_mode' not in body
    assert "self.odometry_mode == 'oracle'" in body
