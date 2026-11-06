import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import PoseStamped, Point32
from nav_msgs.msg import Path as RosPath
from cleannav_interfaces.action import NavigateWithinPolygon
from cleannav_interfaces.msg import NavigationBoundaryState
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from polygon_constraint import validate_boundary, validate_swept_path, contains_rectangle, contains_pose
from boundary_runtime import PolygonNavigationMixin
from navigation_facade_node import NavigationFacadeRuntime
from test_navigation_facade import (executor, FakeOuterGoal, FakeGoalHandle, FakeActionClient,
    _goal, _path, _wrapped, _execute, GoalStatus)

SQUARE=((-2.,-2.),(2.,-2.),(2.,2.),(-2.,2.))

def path(*poses):
    p=RosPath();p.header.frame_id='map'
    for x,y,yaw in poses:
        q=PoseStamped();q.header.frame_id='map';q.pose.position.x=float(x);q.pose.position.y=float(y)
        q.pose.orientation.z=math.sin(yaw/2);q.pose.orientation.w=math.cos(yaw/2);p.poses.append(q)
    return p

def request():
    g=NavigateWithinPolygon.Goal();g.pose.header.frame_id='map';g.pose.pose.orientation.w=1.
    g.boundary.header.frame_id='map';g.boundary.polygon.points=[Point32(x=x,y=y) for x,y in SQUARE]
    g.map_id='map-a';g.map_revision='r1';g.map_sha256='a'*64
    return g

def test_swept_geometry_inside_and_footprint_outside_despite_center_inside():
    assert validate_swept_path(path((0,0,0),(1,0,0)),SQUARE)['boundary_checks']>1
    assert not contains_pose((1.6,0.,0.),SQUARE,margin=.15)
    with pytest.raises(ValueError,match='footprint'):
        validate_swept_path(path((0,0,0),(1.6,0,0)),SQUARE)

def test_concave_notch_cannot_be_missed_by_footprint_corners():
    u=validate_boundary(((0,0),(4,0),(4,4),(2.5,4),(2.5,1.5),(1.5,1.5),(1.5,4),(0,4)))
    assert not contains_rectangle([(1,1),(3,1),(3,3),(1,3)],u)
    with pytest.raises(ValueError):
        validate_swept_path(path((.75,3,0),(3.25,3,0)),u,bounds=(-.1,.1,.1))

def test_rotating_sweep_must_fit_even_when_both_end_poses_fit():
    narrow=validate_boundary(((-1,-.6),(1,-.6),(1,.6),(-1,.6)))
    assert contains_pose((0,0,0),narrow,margin=.15)
    assert contains_pose((0,0,math.pi),narrow,margin=.15)
    with pytest.raises(ValueError):
        validate_swept_path(path((0,0,0),(0,0,math.pi)),narrow)

@pytest.mark.parametrize('points',[
    ((0,0),(1,1),(0,1),(1,0)),((0,0),(0,0),(1,1)),
    ((0,0),(1,0),(float('nan'),1)),((0,0),(1,0),(2,0))])
def test_invalid_boundary_rejected(points):
    with pytest.raises(ValueError):validate_boundary(points)

def test_constraint_map_and_full_goal_footprint_checked_before_reserving_owner():
    n=PolygonNavigationMixin();values=dict(constraint_map_id='map-a',constraint_map_revision='r1',constraint_map_sha256='a'*64,boundary_margin_m=.15)
    n.get_parameter=lambda k:SimpleNamespace(value=values[k])
    assert n._check_constraint_request(request())==SQUARE
    bad=request();bad.map_sha256='b'*64
    with pytest.raises(ValueError,match='identity'):n._check_constraint_request(bad)
    bad=request();bad.pose.pose.position.x=1.6
    with pytest.raises(ValueError,match='footprint'):n._check_constraint_request(bad)

def test_both_costmap_tokens_and_active_state_must_be_acknowledged():
    n=PolygonNavigationMixin();n._boundary=SQUARE;n._boundary_token='new';n._boundary_states={}
    for name in ('/global_costmap/global_costmap','/local_costmap/local_costmap'):
        n._boundary_states[name]=(NavigationBoundaryState(token='new',active=True,ready=True),time.monotonic())
    assert n._boundary_ready()
    n._boundary_states['/local_costmap/local_costmap']=(NavigationBoundaryState(token='old',active=True,ready=True),time.monotonic())
    assert not n._boundary_ready()
    n._boundary_states['/local_costmap/local_costmap']=(NavigationBoundaryState(token='new',active=True,ready=False),time.monotonic())
    assert not n._boundary_ready()

def test_constrained_and_regular_actions_share_one_owner_and_typed_terminal(executor):
    ph=FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    fh=FakeGoalHandle(result=_wrapped(GoalStatus.STATUS_SUCCEEDED))
    r=NavigationFacadeRuntime(FakeActionClient(ph),FakeActionClient(fh))
    outer=FakeOuterGoal();outer.request=request()
    assert r.accept_goal(outer.request)
    assert not r.accept_goal(_goal())
    task=executor.create_task(r.execute(outer))
    deadline=time.monotonic()+2
    while not task.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
    assert isinstance(task.result(),NavigateWithinPolygon.Result)
    assert outer.terminals==['SUCCEEDED']
    assert not r.busy

def test_failed_boundary_barrier_never_sends_inner_actions(executor):
    async def failure(_handle):raise ValueError('no local ACK')
    planner=FakeActionClient();follower=FakeActionClient()
    r=NavigationFacadeRuntime(planner,follower,session_initializer=failure)
    outer=FakeOuterGoal();outer.request=request()
    _execute(r,outer,executor)
    assert outer.terminals==['ABORTED'] and not r.busy
    assert not planner.send_calls and not follower.send_calls

@pytest.mark.parametrize('active_still_valid',[True,False])
def test_rejected_replan_revalidates_active_path_before_retaining_owner(executor,active_still_valid):
    from test_navigation_facade import _start,_spin_until,_spin_until_done
    current=_path(1.);replacement=_path(2.)
    planner=FakeActionClient(goal_handles=[FakeGoalHandle(result=_wrapped(4,current)),FakeGoalHandle(result=_wrapped(4,replacement))])
    follow_handle=FakeGoalHandle()
    original_cancel=follow_handle.cancel_goal_async
    def cancel_terminal():
        future=original_cancel()
        if not follow_handle.result_future.done():follow_handle.result_future.set_result(_wrapped(5))
        return future
    follow_handle.cancel_goal_async=cancel_terminal
    follow=FakeActionClient(follow_handle);events=[];calls=[]
    async def admission(p,g):
        calls.append(p)
        if p.poses[-1].pose.position.x==2.:raise ValueError('replacement outside polygon')
        if len(calls)>2 and not active_still_valid:raise ValueError('new obstacle on current path')
        return dict(boundary_checks=3)
    runtime=NavigationFacadeRuntime(planner,follow,path_validator=admission,diagnostic=events.append)
    outer=FakeOuterGoal();outer.request=request();task=_start(runtime,outer,executor)
    _spin_until(executor,lambda:len(follow.send_calls)==1)
    assert runtime.request_replan()
    if active_still_valid:
        _spin_until(executor,lambda:any(e['event']=='REPLAN_REJECTED_ACTIVE_PATH_REVALIDATED' for e in events))
        assert len(follow.send_calls)==1 and not follow_handle.cancel_calls
        follow_handle.result_future.set_result(_wrapped(4))
        _spin_until_done(executor,task)
        assert outer.terminals==['SUCCEEDED']
    else:
        _spin_until_done(executor,task)
        assert outer.terminals==['ABORTED'] and follow_handle.cancel_calls==1
    assert len(calls)==3
