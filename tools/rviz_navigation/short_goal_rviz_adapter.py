#!/usr/bin/env python3
"""Run-bound adapter for exactly one bounded, real RViz GUI goal."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from cleannav_interfaces.msg import TaskStatus
from cleannav_interfaces.srv import SubmitNavigationGoal
from std_msgs.msg import Bool, String
from cleannav_interfaces.msg import SafetyStatus
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener

from short_goal_policy import (LEASE_RELEASE_CONFIRMATION_S, MAX_GOAL_RUNTIME_S,
                               close_lifecycle, update_lifecycle, update_scope,
                               validate_authority_files, validate_forward_goal,
                               validate_lifecycle, validate_motion_admission,
                               write_revocation_receipt, sha256)

INPUT = "/cleannav/rviz/goal_pose"
SERVICE = "/cleannav/mission/submit_navigation_goal"
MAP = "/cleannav/localization/map"
EVENTS = "/cleannav/simulation/rviz_goal_events"
EXPECTED_RVIZ_NODE = "rviz"


def _yaw(q) -> float:
    return math.atan2(2.0*(q.w*q.z + q.x*q.y),
                      1.0 - 2.0*(q.y*q.y + q.z*q.z))


def _known_free_neighborhood(grid: OccupancyGrid | None, x: float, y: float,
                             radius_m: float = 0.22) -> bool:
    if grid is None or grid.header.frame_id != "map" or grid.info.resolution <= 0:
        return False
    origin = grid.info.origin
    yaw = _yaw(origin.orientation)
    dx, dy = x-origin.position.x, y-origin.position.y
    local_x = math.cos(yaw)*dx + math.sin(yaw)*dy
    local_y = -math.sin(yaw)*dx + math.cos(yaw)*dy
    resolution = grid.info.resolution
    cx, cy = math.floor(local_x/resolution), math.floor(local_y/resolution)
    cells = math.ceil(radius_m/resolution)
    for gy in range(cy-cells, cy+cells+1):
        for gx in range(cx-cells, cx+cells+1):
            if (gx-cx)**2 + (gy-cy)**2 > cells*cells:
                continue
            if gx < 0 or gy < 0 or gx >= grid.info.width or gy >= grid.info.height:
                return False
            value = int(grid.data[gy*grid.info.width+gx])
            if value < 0 or value >= 65:
                return False
    return True


class ShortGoalAdapter(Node):
    def __init__(self, args):
        super().__init__("cleannav_short_goal_rviz_adapter")
        self.args = args
        self.admission_path = Path(args.admission)
        self.client = self.create_client(SubmitNavigationGoal, SERVICE)
        self.events = self.create_publisher(String, EVENTS, 10)
        self.estop = self.create_publisher(Bool, "/cleannav/safety/emergency_stop", 10)
        self.cancel = self.create_client(CancelGoal, "/cleannav/navigate_to_pose/_action/cancel_goal")
        self.create_subscription(PoseStamped, INPUT, self.on_goal, 10)
        self.create_subscription(TaskStatus, "/cleannav/task_status", self.on_status, 10)
        self.create_subscription(SafetyStatus, "/cleannav/safety_status", self.on_safety, 10)
        self.create_subscription(OccupancyGrid, MAP, self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.goal_seen = False
        self.service_ready_reported = False
        self.safety = None
        self.safety_owner_history = []
        self.lease_seen = False
        self.lease_released = False
        self.terminal_state = None
        self.result_ok = False
        self.stop_wait_deadline = None
        self.terminal_received_monotonic = None
        self.future = None
        self.execution_id = ""
        self.task_id = None
        self.command_id = ""
        self.deadline = None
        self.terminal = False
        self.map = None
        self.timer = self.create_timer(0.1, self.poll,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.emit("ADAPTER_STARTED", input=INPUT, service=SERVICE,
                  max_goal_distance_m=2.0, reverse_allowed=False,
                  authorization_file=self.args.authorization,
                  scope_file=self.args.scope, lifecycle_file=self.args.lifecycle)

    def emit(self, event, **fields):
        payload = dict(event=event, run_id=self.args.run_id,
                       monotonic_ns=time.monotonic_ns(), wall_time_ns=time.time_ns(), **fields)
        text = json.dumps(payload, allow_nan=False, sort_keys=True)
        self.get_logger().info("[RVIZ_SHORT_GOAL] " + text)
        self.events.publish(String(data=text))
        path = Path(self.args.events_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
        try:
            os.write(descriptor, (text + "\n").encode("utf-8"))
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def on_map(self, msg):
        if (msg.header.frame_id == "map" and msg.info.resolution > 0
                and msg.info.width > 0 and msg.info.height > 0
                and len(msg.data) == msg.info.width*msg.info.height):
            self.map = msg

    def on_goal(self, msg):
        if self.goal_seen:
            self.emit("REJECTED", reason="single-goal limit already consumed")
            self.stop_active_task("second RViz goal rejected by single-goal Scope")
            return
        self.goal_seen = True
        endpoints = self.get_publishers_info_by_topic(INPUT)
        publisher_records = [{
            "node_name": e.node_name, "node_namespace": e.node_namespace,
            "topic_type": e.topic_type, "endpoint_gid": list(e.endpoint_gid),
        } for e in endpoints]
        self.command_id = "rviz-short-" + uuid.uuid4().hex
        p, q = msg.pose.position, msg.pose.orientation
        self.emit("CLICK", command_id=self.command_id, frame=msg.header.frame_id,
                  stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                  position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w],
                  publisher_endpoints=publisher_records)
        try:
            if not self.service_ready_reported:
                raise ValueError("Motion Admission or Mission service is not READY")
            validate_motion_admission(run_id=self.args.run_id,
                admission_path=self.admission_path,
                authorization_path=Path(self.args.authorization),
                scope_path=Path(self.args.scope), now_monotonic_ns=time.monotonic_ns())
            auth, scope = validate_authority_files(
                run_id=self.args.run_id,
                authorization_path=Path(self.args.authorization),
                scope_path=Path(self.args.scope), now_monotonic_ns=time.monotonic_ns())
            lifecycle = validate_lifecycle(
                run_id=self.args.run_id, lifecycle_path=Path(self.args.lifecycle),
                authorization_path=Path(self.args.authorization),
                scope_path=Path(self.args.scope), now_monotonic_ns=time.monotonic_ns())
            if scope.get("mission_execution_id") != self.args.execution_id:
                raise ValueError("scope Mission execution identity mismatch")
            if lifecycle.get("accepted_goal_count") != 0 or lifecycle.get("status") != "AUTHORIZED":
                raise ValueError("single-goal Scope has already been consumed or closed")
            if (len(publisher_records) != 1
                    or publisher_records[0]["node_name"] != EXPECTED_RVIZ_NODE
                    or publisher_records[0]["node_namespace"] != "/"
                    or publisher_records[0]["topic_type"] != "geometry_msgs/msg/PoseStamped"
                    or not publisher_records[0]["endpoint_gid"]):
                raise ValueError("goal publisher is not the unique identified RViz GUI endpoint")
            if msg.header.frame_id != "map":
                raise ValueError("RViz goal must be stamped in map frame")
            safety_age = (time.monotonic_ns()-self.safety.get("monotonic_ns", 0)
                          if isinstance(self.safety, dict) else None)
            if (self.safety is None or safety_age is None
                    or not 0 <= safety_age <= 1_000_000_000
                    or self.safety.get("block_all") is not False
                    or self.safety.get("estop") is not False
                    or self.safety.get("enabled") is not False
                    or self.safety.get("owner") not in (None, "")):
                raise ValueError("Safety is not in the admitted, unleased pre-goal state")
            transform = self.buffer.lookup_transform("map", "base_footprint", Time())
            transform_stamp = (transform.header.stamp.sec * 10**9
                               + transform.header.stamp.nanosec)
            now_ros_ns = self.get_clock().now().nanoseconds
            if (transform_stamp <= 0 or now_ros_ns <= 0
                    or now_ros_ns - transform_stamp > 500_000_000):
                raise ValueError("current map to base_footprint transform is stale")
            base = transform.transform.translation
            bound = validate_forward_goal(
                goal_x=p.x, goal_y=p.y, goal_yaw=_yaw(q),
                base_x=base.x, base_y=base.y, base_yaw=_yaw(transform.transform.rotation),
                map_free=_known_free_neighborhood(self.map, p.x, p.y))
            self.emit("GOAL_BOUNDS_PASS", command_id=self.command_id, **bound,
                      authorization_status=auth.get("status"), scope_status=scope.get("status"))
            if not self.client.service_is_ready():
                raise ValueError("Mission Manager submission service is not ready")
        except Exception as error:
            self.emit("REJECTED", command_id=self.command_id,
                      reason=f"{type(error).__name__}: {error}")
            self.terminal_state = TaskStatus.STATE_FAILED
            self.terminal_received_monotonic = time.monotonic()
            self.result_ok = False
            return
        request = SubmitNavigationGoal.Request()
        request.command_id = self.command_id
        request.source = request.SOURCE_RVIZ
        request.goal = msg
        request.valid_for.sec = 30
        request.busy_policy = request.REJECT_IF_BUSY
        self.future = self.client.call_async(request)
        self.deadline = time.monotonic() + 4.0
        self.emit("SUBMITTED", command_id=self.command_id, service=SERVICE,
                  mission_execution_id=self.args.execution_id,
                  maximum_goal_runtime_s=MAX_GOAL_RUNTIME_S)

    def on_status(self, msg):
        if not self.command_id or msg.command_id != self.command_id:
            return
        if self.task_id is not None and int(msg.task_id) != self.task_id:
            self.emit("IDENTITY_FAILURE", expected_task_id=self.task_id,
                      observed_task_id=int(msg.task_id), command_id=msg.command_id)
            self.terminal_state = TaskStatus.STATE_FAILED
            self.terminal_received_monotonic = time.monotonic()
            self.result_ok = False
            self.stop_active_task("Mission TaskStatus task identity mismatch")
            return
        self.emit("TASK_STATUS", command_id=msg.command_id, task_id=msg.task_id,
                  execution_id=msg.execution_id, scope=int(msg.status_scope),
                  state=int(msg.state), reason_code=int(msg.reason_code),
                  progress=float(msg.progress), message=msg.message)
        if msg.execution_id and msg.execution_id != self.args.execution_id:
            self.emit("IDENTITY_FAILURE", expected_execution_id=self.args.execution_id,
                      observed_execution_id=msg.execution_id)
            self.terminal_state = TaskStatus.STATE_FAILED
            self.terminal_received_monotonic = time.monotonic()
            self.result_ok = False
        elif msg.state in (TaskStatus.STATE_SUCCEEDED, TaskStatus.STATE_CANCELED,
                           TaskStatus.STATE_FAILED, TaskStatus.STATE_EMERGENCY_STOPPED):
            self.terminal_state = msg.state
            self.terminal_received_monotonic = time.monotonic()
            self.result_ok = msg.state == TaskStatus.STATE_SUCCEEDED
        self.maybe_finish()

    def on_safety(self, msg):
        owner = msg.lease_owner_execution_id
        previous_owner = self.safety.get("owner") if isinstance(self.safety, dict) else None
        self.safety = dict(block_all=msg.block_all, estop=msg.emergency_stop_active,
                           enabled=msg.autonomous_enabled, owner=owner,
                           monotonic_ns=time.monotonic_ns())
        if previous_owner != owner:
            # History keeps the original capture time. emit() owns the event envelope.
            transition = dict(run_id=self.args.run_id, source="/cleannav/safety_status",
                previous_owner=previous_owner, owner=owner,
                monotonic_ns=self.safety["monotonic_ns"],
                block_all=msg.block_all, estop=msg.emergency_stop_active,
                autonomous_enabled=msg.autonomous_enabled)
            self.safety_owner_history.append(transition)
            self.emit("SAFETY_LEASE_OWNER_TRANSITION", source=transition["source"],
                previous_owner=transition["previous_owner"], owner=transition["owner"],
                block_all=transition["block_all"], estop=transition["estop"],
                autonomous_enabled=transition["autonomous_enabled"])
        if owner and owner != self.args.execution_id:
            self.emit("LEASE_OWNER_MISMATCH", expected=self.args.execution_id,
                      observed=owner, safety=self.safety)
            self.stop_active_task("unexpected Safety Lease owner")
            return
        if msg.emergency_stop_active and not self.terminal:
            self.emit("SAFETY_ESTOP_ACTIVE", safety=self.safety)
            self.result_ok = False
            if self.command_id:
                self.stop_active_task("Safety ESTOP became active")
        scope_path = Path(self.args.scope)
        lifecycle_path = Path(self.args.lifecycle)
        if owner == self.args.execution_id and not self.lease_seen:
            if not self.command_id:
                self.emit("LEASE_OWNER_MISMATCH", expected="no-owner-before-goal",
                          observed=owner, safety=self.safety)
                self.stop_active_task("Mission Safety Lease appeared before the one RViz goal")
                return
            self.lease_seen = True
            self.emit("LEASE_ACQUIRED", execution_id=owner, command_id=self.command_id,
                      task_id=self.task_id, safety=self.safety)
            try:
                now = time.monotonic_ns()
                update_scope(scope_path, expected_run_id=self.args.run_id,
                    expected_profile_id="rviz_short_goal_015", updates={
                        "status": "LEASE_ACTIVE", "lease_owner_execution_id": owner,
                        "lease_acquired_monotonic_ns": now,
                        "accepted_goal_count": 1, "accepted_command_id": self.command_id,
                        "accepted_task_id": self.task_id,
                    })
                update_lifecycle(lifecycle_path, expected_run_id=self.args.run_id,
                    expected_profile_id="rviz_short_goal_015", updates={
                        "status": "LEASE_ACTIVE", "lease_owner_execution_id": owner,
                        "lease_identity": owner,
                        "lease_id": None,
                        "lease_id_status": "SafetyLease interface exposes execution_id, not a unique lease token",
                        "lease_acquired_monotonic_ns": now,
                        "scope_sha256": sha256(scope_path),
                        "accepted_goal_count": 1, "command_id": self.command_id,
                        "task_id": self.task_id,
                    })
            except Exception as error:
                self.emit("RECEIPT_UPDATE_FAILED", reason=f"{type(error).__name__}: {error}")
                self.stop_active_task("could not persist the actual Mission Lease")
        if self.lease_seen and not owner and self.terminal_state is not None and not self.terminal:
            now = time.monotonic_ns()
            self.lease_released = True
            self.emit("LEASE_RELEASED", execution_id=self.args.execution_id,
                      release_reason="Mission task terminal and Safety owner cleared",
                      safety=self.safety)
            try:
                update_scope(scope_path, expected_run_id=self.args.run_id,
                    expected_profile_id="rviz_short_goal_015", updates={
                        "status": "CLOSED", "lease_owner_execution_id": None,
                        "lease_released_monotonic_ns": now,
                        "closed_monotonic_ns": now,
                        "release_reason": "Mission task terminal and Safety owner cleared",
                    })
                update_lifecycle(lifecycle_path, expected_run_id=self.args.run_id,
                    expected_profile_id="rviz_short_goal_015", updates={
                        "status": "LEASE_RELEASED", "lease_owner_execution_id": None,
                        "lease_released_monotonic_ns": now,
                        "release_reason": "Mission task terminal and Safety owner cleared",
                        "scope_sha256": sha256(scope_path),
                    })
                close_lifecycle(lifecycle_path, expected_run_id=self.args.run_id,
                    expected_profile_id="rviz_short_goal_015", monotonic_ns=now,
                    release_reason="Mission task terminal and Safety owner cleared",
                    lease_released=True)
                write_revocation_receipt(run_id=self.args.run_id,
                    profile_id="rviz_short_goal_015", run_dir=Path(self.args.events_path).parent,
                    authorization_path=Path(self.args.authorization), scope_path=scope_path,
                    lifecycle_path=lifecycle_path, monotonic_ns=now,
                    release_reason="Mission task terminal and Safety owner cleared",
                    lease_acquired=True, lease_released=True)
            except Exception as error:
                self.emit("RECEIPT_UPDATE_FAILED", reason=f"{type(error).__name__}: {error}")
                self.result_ok = False
            self.terminal = True
        self.maybe_finish()

    def maybe_finish(self):
        """Close receipts only after observing actual Lease release or no-Lease failure."""
        if self.terminal or self.terminal_state is None or self.safety is None:
            return
        if self.safety.get("owner") not in (None, ""):
            return
        if self.lease_seen and not self.lease_released:
            return
        if not self.lease_seen:
            if (self.terminal_received_monotonic is None
                    or time.monotonic()-self.terminal_received_monotonic < 2.0):
                return
            # Mission completion without an observed Safety Lease acquisition
            # cannot qualify as successful motion, even if TaskStatus says
            # SUCCEEDED. This grace interval lets the 2 Hz SafetyStatus stream
            # expose a short acquisition/release before finalization.
            self.result_ok = False
            self.emit("LEASE_ACQUISITION_NOT_CONFIRMED", reason="no Safety owner observed during grace window")
        now = time.monotonic_ns()
        try:
            scope_path = Path(self.args.scope)
            scope = json.loads(scope_path.read_text(encoding="utf-8"))
            if scope.get("status") != "CLOSED":
                update_scope(scope_path, expected_run_id=self.args.run_id,
                    expected_profile_id="rviz_short_goal_015", updates={
                        "status": "CLOSED", "lease_owner_execution_id": None,
                        "closed_monotonic_ns": now,
                        "release_reason": ("Mission Lease released" if self.lease_seen
                            else "Mission ended before Safety Lease acquisition"),
                    })
            update_lifecycle(Path(self.args.lifecycle), expected_run_id=self.args.run_id,
                expected_profile_id="rviz_short_goal_015", updates={
                    "scope_sha256": sha256(scope_path),
                })
            lifecycle = close_lifecycle(Path(self.args.lifecycle), expected_run_id=self.args.run_id,
                expected_profile_id="rviz_short_goal_015", monotonic_ns=now,
                release_reason=("Mission Lease released" if self.lease_seen
                                else "Mission ended before Safety Lease acquisition"),
                lease_released=True)
            write_revocation_receipt(run_id=self.args.run_id,
                profile_id="rviz_short_goal_015", run_dir=Path(self.args.events_path).parent,
                authorization_path=Path(self.args.authorization), scope_path=scope_path,
                lifecycle_path=Path(self.args.lifecycle), monotonic_ns=now,
                release_reason=lifecycle.get("release_reason", "Mission terminal"),
                lease_acquired=self.lease_seen, lease_released=self.lease_released)
        except Exception as error:
            self.emit("RECEIPT_UPDATE_FAILED", reason=f"{type(error).__name__}: {error}")
            self.result_ok = False
            return
        self.terminal = True

    def stop_active_task(self, reason):
        """Use the existing Safety ESTOP and Nav2 cancel path on adapter faults."""
        self.emit("SUPERVISED_STOP_REQUESTED", reason=reason,
                  execution_id=self.execution_id or self.args.execution_id)
        self.estop.publish(Bool(data=True))
        if self.cancel.service_is_ready():
            try:
                self.cancel.call_async(CancelGoal.Request())
            except Exception as error:
                self.emit("NAV2_CANCEL_REQUEST_FAILED",
                          reason=f"{type(error).__name__}: {error}")

    def poll(self):
        if not self.service_ready_reported:
            try:
                validate_motion_admission(run_id=self.args.run_id,
                    admission_path=self.admission_path,
                    authorization_path=Path(self.args.authorization),
                    scope_path=Path(self.args.scope), now_monotonic_ns=time.monotonic_ns())
            except ValueError:
                pass
            else:
                safety_age = (time.monotonic_ns()-self.safety.get("monotonic_ns", 0)
                              if isinstance(self.safety, dict) else None)
                safety_pre_goal_ready=(self.safety is not None and safety_age is not None
                    and 0 <= safety_age <= 1_000_000_000
                    and self.safety.get("block_all") is False
                    and self.safety.get("estop") is False
                    and self.safety.get("enabled") is False
                    and self.safety.get("owner") in (None,""))
                if self.client.service_is_ready() and safety_pre_goal_ready:
                    self.service_ready_reported = True
                    self.emit("SERVICE_READY", input=INPUT, service=SERVICE,
                              execution_id=self.args.execution_id)
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            try:
                response = future.result()
                self.execution_id = response.execution_id
                if not response.accepted or response.execution_id != self.args.execution_id:
                    self.emit("REJECTED", command_id=self.command_id, task_id=response.task_id,
                              execution_id=response.execution_id, reason=response.reason,
                              reason_code=response.reason_code,
                              mission_service_accepted=bool(response.accepted))
                    self.emit("IDENTITY_FAILURE", expected_execution_id=self.args.execution_id,
                              observed_execution_id=response.execution_id)
                    self.terminal_state = TaskStatus.STATE_FAILED
                    self.terminal_received_monotonic = time.monotonic()
                    self.result_ok = False
                else:
                    try:
                        self.task_id = int(response.task_id)
                        if self.task_id <= 0 or self.task_id > (2**63 - 1):
                            raise ValueError("Mission returned an invalid task_id")
                        lifecycle_before = json.loads(Path(self.args.lifecycle).read_text())
                        has_lease = (self.lease_seen or (self.safety is not None
                            and self.safety.get("owner") == self.args.execution_id))
                        active_status = "LEASE_ACTIVE" if has_lease else "GOAL_ACCEPTED"
                        update_scope(Path(self.args.scope), expected_run_id=self.args.run_id,
                            expected_profile_id="rviz_short_goal_015", updates={
                                "status": active_status, "accepted_goal_count": 1,
                                "accepted_command_id": self.command_id,
                                "accepted_task_id": self.task_id,
                            })
                        update_lifecycle(Path(self.args.lifecycle), expected_run_id=self.args.run_id,
                            expected_profile_id="rviz_short_goal_015", updates={
                                "status": active_status, "accepted_goal_count": 1,
                                "command_id": self.command_id,
                                "task_id": self.task_id,
                                "lease_owner_execution_id": (self.args.execution_id
                                    if has_lease else lifecycle_before.get(
                                        "lease_owner_execution_id")),
                                "scope_sha256": sha256(Path(self.args.scope)),
                            })
                        self.emit("ACCEPTED", command_id=self.command_id,
                                  task_id=response.task_id,
                                  execution_id=response.execution_id,
                                  reason=response.reason, reason_code=response.reason_code)
                    except Exception as error:
                        self.emit("RECEIPT_UPDATE_FAILED",
                                  reason=f"{type(error).__name__}: {error}")
                        self.stop_active_task("could not bind Mission task to the active Scope")
                    self.deadline = time.monotonic() + MAX_GOAL_RUNTIME_S
            except Exception as error:
                self.emit("SERVICE_ERROR", command_id=self.command_id,
                          reason=f"{type(error).__name__}: {error}")
                self.terminal_state = TaskStatus.STATE_FAILED
                self.terminal_received_monotonic = time.monotonic()
                self.result_ok = False
        if self.deadline is not None and time.monotonic() >= self.deadline and not self.terminal:
            self.emit("GOAL_TIMEOUT", command_id=self.command_id,
                      execution_id=self.execution_id or self.args.execution_id)
            self.stop_active_task("bounded short-goal runtime expired")
            self.deadline = None
            self.stop_wait_deadline = time.monotonic() + LEASE_RELEASE_CONFIRMATION_S
            self.terminal_state = TaskStatus.STATE_FAILED
            self.terminal_received_monotonic = time.monotonic()
        if (self.stop_wait_deadline is not None and time.monotonic() >= self.stop_wait_deadline
                and not self.terminal):
            self.emit("STOP_UNCONFIRMED", command_id=self.command_id,
                      execution_id=self.execution_id or self.args.execution_id,
                      safety=self.safety)
            self.result_ok = False
            self.terminal = True
        self.maybe_finish()
        if self.terminal:
            self.emit("MISSION_TERMINAL", execution_id=self.execution_id,
                      command_id=self.command_id, task_id=self.task_id,
                      terminal_state=self.terminal_state, succeeded=self.result_ok,
                      lease_seen=self.lease_seen, lease_released=self.lease_released,
                      lifecycle_closed=(json.loads(Path(self.args.lifecycle).read_text()).get("status")
                                        == "CLOSED"), safety=self.safety)
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--authorization", required=True)
    parser.add_argument("--scope", required=True)
    parser.add_argument("--admission", required=True)
    parser.add_argument("--lifecycle", required=True)
    parser.add_argument("--execution-id", required=True)
    parser.add_argument("--events-path", required=True)
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = ShortGoalAdapter(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
