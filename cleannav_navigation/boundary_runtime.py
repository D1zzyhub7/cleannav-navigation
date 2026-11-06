"""Action-bound geometry restriction shared by both costmaps and admission."""
import copy
import math
import time
from cleannav_interfaces.action import NavigateWithinPolygon
from cleannav_interfaces.msg import NavigationBoundary, NavigationBoundaryState
from rclpy.action import GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from polygon_constraint import validate_boundary, validate_swept_path, contains_pose

CONSTRAINED_ACTION_NAME = '/cleannav/navigate_within_polygon'


class PolygonNavigationMixin:
    def _setup_polygon_navigation(self):
        for key in ('map_id','map_revision','map_sha256'):
            self.declare_parameter('constraint_'+key,'')
        self.declare_parameter('boundary_margin_m',.15)
        self._boundary = None
        self._boundary_token = ''
        self._boundary_states = {}
        self._boundary_ever_activated = False
        self._boundary_armed = False
        self._boundary_pub = self.create_publisher(NavigationBoundary,
            '/cleannav/navigation/boundary',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._boundary_sub = self.create_subscription(NavigationBoundaryState,
            '/cleannav/navigation/boundary_state',self._on_boundary_state,10)
        self._boundary_timer = self.create_timer(.1,self._watch_boundary)

    def _on_boundary_state(self,message):
        self._boundary_states[message.costmap_id]=(message,time.monotonic())

    def _boundary_ready(self):
        now=time.monotonic()
        for name in ('/global_costmap/global_costmap','/local_costmap/local_costmap'):
            state,received=self._boundary_states.get(name,(None,0.))
            if (state is None or now-received>1.5 or state.token!=self._boundary_token
                    or not state.ready or state.active!=(self._boundary is not None)):
                return False
        return True

    def _check_constraint_request(self,request):
        if request.boundary.header.frame_id!=request.pose.header.frame_id or request.pose.header.frame_id!='map':
            raise ValueError('constraint and goal must use map frame')
        for key in ('map_id','map_revision','map_sha256'):
            expected=self.get_parameter('constraint_'+key).value
            if not expected or getattr(request,key)!=expected:
                raise ValueError('constraint map identity mismatch: '+key)
        boundary=validate_boundary([(p.x,p.y) for p in request.boundary.polygon.points])
        q=request.pose.pose.orientation
        pose=(request.pose.pose.position.x,request.pose.pose.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))
        margin=float(self.get_parameter('boundary_margin_m').value)
        if margin<.1 or not contains_pose(pose,boundary,margin=margin):
            raise ValueError('goal footprint and margin outside task polygon')
        return boundary

    async def _initialize_boundary(self,handle):
        self._boundary_armed=False
        constrained=isinstance(handle.request,NavigateWithinPolygon.Goal)
        if not constrained and not self._boundary_ever_activated:
            self._boundary=None
            return  # An ordinary M0/GOTO does not require the optional layer.
        boundary=self._check_constraint_request(handle.request) if constrained else None
        self._boundary=boundary
        self._boundary_token=bytes(handle.goal_id.uuid).hex()
        message=NavigationBoundary(token=self._boundary_token,active=constrained,
            clearance_m=0.)  # Rasterize the physical boundary; swept admission retains 0.15m.
        if constrained:
            message.boundary=copy.deepcopy(handle.request.boundary)
            self._boundary_ever_activated=True
        self._boundary_pub.publish(message)
        wake=Future();deadline=time.monotonic()+6.
        def poll():
            if wake.done(): return
            if self._boundary_ready(): wake.set_result(True)
            elif time.monotonic()>=deadline or self._runtime._is_cancel_requested(self._runtime._session):
                wake.set_result(False)
        timer=self.create_timer(.05,poll,clock=Clock(clock_type=ClockType.STEADY_TIME))
        try:
            if not await wake:
                raise ValueError('both costmaps did not acknowledge immutable task boundary')
        finally:
            self.destroy_timer(timer)
        self._boundary_armed=constrained
        self._runtime._report(dict(event='BOUNDARY_APPLIED',token=self._boundary_token,
            active=constrained,vertices=len(boundary or ()),costmaps=list(self._boundary_states)))

    def _watch_boundary(self):
        if not self._boundary_armed or not self._runtime.busy:
            return
        if not self._boundary_ready():
            session=self._runtime._session
            if session is not None and not session.cancel_requested:
                self._runtime._report(dict(event='BOUNDARY_STATE_LOST',token=self._boundary_token))
                self._runtime.request_cancel(session.outer_goal_handle)

    def _constrained_goal_callback(self,request):
        try:
            self._check_constraint_request(request)
        except (ValueError,TypeError,AttributeError) as error:
            self.get_logger().error('CONSTRAINT_REJECTED '+str(error))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT if self._runtime.accept_goal(request) else GoalResponse.REJECT

    def _validate_boundary_path(self,path):
        if self._boundary is None:
            return {}
        if not self._boundary_ready():
            raise ValueError('task boundary costmap acknowledgement stale')
        proof=validate_swept_path(path,self._boundary,
            margin=float(self.get_parameter('boundary_margin_m').value))
        return dict(proof,boundary_token=self._boundary_token)
