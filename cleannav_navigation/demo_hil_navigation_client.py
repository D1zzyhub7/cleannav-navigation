#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import (
    ProxyHandler,
    Request,
    build_opener,
)

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


DEFAULT_SERVER = "http://192.168.8.10:18081"
DEFAULT_POLL_INTERVAL = 0.25
DEFAULT_HTTP_TIMEOUT = 1.5

ACTION_NAME = "/cleannav/navigate_to_pose"

EVENT_GOAL_ACCEPTED = "GOAL_ACCEPTED"
EVENT_GOAL_REJECTED = "GOAL_REJECTED"
EVENT_SUCCEEDED = "SUCCEEDED"
EVENT_FAILED = "FAILED"
EVENT_CANCEL_CONFIRMED = "CANCEL_CONFIRMED"
EVENT_CANCEL_FAILED = "CANCEL_FAILED"

TERMINAL_EVENTS = {
    EVENT_GOAL_REJECTED,
    EVENT_SUCCEEDED,
    EVENT_FAILED,
    EVENT_CANCEL_CONFIRMED,
    EVENT_CANCEL_FAILED,
}


class HttpStatusError(RuntimeError):

    def __init__(
        self,
        status: int,
        body: str,
    ) -> None:
        super().__init__(
            f"HTTP {status}: {body}"
        )
        self.status = status
        self.body = body


class HilHttpTransport:
    """Small HTTP transport with environment proxies explicitly disabled."""

    def __init__(
        self,
        base_url: str,
        timeout: float,
    ) -> None:
        self._base_url = base_url.rstrip("/")
        self._timeout = timeout

        # Important for the WSL/J6M HIL link:
        # never inherit VPN/proxy environment variables.
        self._opener = build_opener(
            ProxyHandler({})
        )

    def health(self) -> dict[str, Any]:
        return self._request_json(
            "GET",
            "/health",
        )

    def get_events(
        self,
        after_event_id: int,
    ) -> list[dict[str, Any]]:
        query = urlencode(
            {
                "after": after_event_id,
            }
        )

        response = self._request_json(
            "GET",
            f"/nav/events?{query}",
        )

        events = response.get(
            "events",
            [],
        )

        if not isinstance(events, list):
            raise ValueError(
                "J6M response field 'events' is not a list"
            )

        return [
            event
            for event in events
            if isinstance(event, dict)
        ]

    def post_result(
        self,
        body: dict[str, Any],
    ) -> dict[str, Any]:
        return self._request_json(
            "POST",
            "/nav/result",
            body,
        )

    def _request_json(
        self,
        method: str,
        path: str,
        body: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        data = None

        headers = {
            "Accept": "application/json",
        }

        if body is not None:
            data = json.dumps(
                body,
                separators=(",", ":"),
            ).encode("utf-8")

            headers["Content-Type"] = (
                "application/json"
            )

        request = Request(
            self._base_url + path,
            data=data,
            headers=headers,
            method=method,
        )

        try:
            with self._opener.open(
                request,
                timeout=self._timeout,
            ) as response:
                raw = response.read().decode(
                    "utf-8"
                )

        except HTTPError as exc:
            raw = exc.read().decode(
                "utf-8",
                errors="replace",
            )

            raise HttpStatusError(
                exc.code,
                raw,
            ) from exc

        if not raw:
            return {}

        parsed = json.loads(raw)

        if not isinstance(parsed, dict):
            raise ValueError(
                "HTTP response is not a JSON object"
            )

        return parsed


@dataclass
class NavigationRequestState:
    nav_request_id: str
    goal_handle: Any = None

    cancel_requested: bool = False
    cancel_in_flight: bool = False
    cancel_accepted: bool = False

    terminal: bool = False

    emitted_events: set[str] = field(
        default_factory=set
    )


class DemoHilNavigationClient(Node):
    """Bridge J6M HIL navigation HTTP events to the local ROS action."""

    def __init__(
        self,
        *,
        server: str,
        poll_interval: float,
        http_timeout: float,
        after_event_id: int,
        execute: bool,
    ) -> None:
        super().__init__(
            "cleannav_demo_hil_navigation_client"
        )

        self._execute = execute

        self._transport = HilHttpTransport(
            server,
            http_timeout,
        )

        self._poll_interval = poll_interval
        self._initial_after_event_id = (
            after_event_id
        )

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            ACTION_NAME,
        )

        self._states: dict[
            str,
            NavigationRequestState,
        ] = {}

        self._incoming_events: queue.Queue[
            dict[str, Any]
        ] = queue.Queue()

        self._outgoing_results: queue.Queue[
            dict[str, Any]
        ] = queue.Queue()

        self._stop_event = threading.Event()

        self._timer = self.create_timer(
            0.02,
            self._drain_events,
        )

        self.get_logger().info(
            "Demo HIL Navigation Client starting"
        )
        self.get_logger().info(
            f"J6M server: {server}"
        )
        self.get_logger().info(
            f"ROS action: {ACTION_NAME}"
        )
        self.get_logger().info(
            "mode: "
            + (
                "EXECUTE"
                if execute
                else "OBSERVE_ONLY"
            )
        )
        self.get_logger().info(
            "initial after_event_id: "
            f"{after_event_id}"
        )

        health = self._transport.health()

        self.get_logger().info(
            "J6M HIL navigation health: "
            + json.dumps(
                health,
                ensure_ascii=False,
                separators=(",", ":"),
            )
        )

        if self._execute:
            self.get_logger().info(
                "Waiting for local Navigation "
                "Facade action server..."
            )

            if not self._action_client.wait_for_server(
                timeout_sec=5.0
            ):
                raise RuntimeError(
                    "Navigation Facade action server "
                    f"{ACTION_NAME} is unavailable"
                )

            self.get_logger().info(
                "Navigation Facade action server READY"
            )

        self._http_thread = threading.Thread(
            target=self._http_loop,
            name="cleannav-demo-hil-http-client",
            daemon=True,
        )

        self._http_thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()

        if self._http_thread.is_alive():
            self._http_thread.join(
                timeout=2.0
            )

        self._action_client.destroy()

    def _http_loop(self) -> None:
        after_event_id = (
            self._initial_after_event_id
        )

        pending_results: list[
            dict[str, Any]
        ] = []

        while not self._stop_event.is_set():

            while True:
                try:
                    pending_results.append(
                        self._outgoing_results
                        .get_nowait()
                    )
                except queue.Empty:
                    break

            remaining_results: list[
                dict[str, Any]
            ] = []

            for result in pending_results:
                try:
                    response = (
                        self._transport.post_result(
                            result
                        )
                    )

                    self.get_logger().info(
                        "J6M result accepted: "
                        + json.dumps(
                            response,
                            ensure_ascii=False,
                            separators=(",", ":"),
                        )
                    )

                except HttpStatusError as exc:
                    # 4xx means the request reached J6M
                    # but violates the HIL contract.
                    # Repeating forever would only spam
                    # the server, so drop it and expose
                    # the error loudly.
                    if 400 <= exc.status < 500:
                        self.get_logger().error(
                            "J6M rejected navigation "
                            f"result permanently: {exc}"
                        )
                    else:
                        remaining_results.append(
                            result
                        )

                except (
                    URLError,
                    TimeoutError,
                    OSError,
                    ValueError,
                    json.JSONDecodeError,
                ) as exc:
                    self.get_logger().warning(
                        "Result POST failed; "
                        f"will retry: {exc}"
                    )

                    remaining_results.append(
                        result
                    )

            pending_results = remaining_results

            try:
                events = self._transport.get_events(
                    after_event_id
                )

                events.sort(
                    key=lambda item: int(
                        item.get(
                            "event_id",
                            0,
                        )
                    )
                )

                for event in events:
                    try:
                        event_id = int(
                            event["event_id"]
                        )
                    except (
                        KeyError,
                        TypeError,
                        ValueError,
                    ):
                        self.get_logger().error(
                            "Ignoring malformed HIL "
                            f"event: {event!r}"
                        )
                        continue

                    if event_id <= after_event_id:
                        continue

                    after_event_id = event_id

                    if self._execute:
                        self._incoming_events.put(
                            event
                        )
                    else:
                        self.get_logger().info(
                            "[OBSERVE] "
                            + json.dumps(
                                event,
                                ensure_ascii=False,
                                separators=(",", ":"),
                            )
                        )

            except (
                HttpStatusError,
                URLError,
                TimeoutError,
                OSError,
                ValueError,
                json.JSONDecodeError,
            ) as exc:
                self.get_logger().warning(
                    "HIL event poll failed: "
                    f"{exc}"
                )

            self._stop_event.wait(
                self._poll_interval
            )

    def _drain_events(self) -> None:
        while True:
            try:
                event = (
                    self._incoming_events
                    .get_nowait()
                )
            except queue.Empty:
                return

            try:
                self._handle_event(
                    event
                )
            except Exception as exc:
                self.get_logger().error(
                    "Unhandled HIL event error: "
                    f"{exc}; event={event!r}"
                )

    def _handle_event(
        self,
        event: dict[str, Any],
    ) -> None:
        operation = event.get(
            "operation"
        )

        nav_request_id = event.get(
            "nav_request_id"
        )

        if (
            not isinstance(
                nav_request_id,
                str,
            )
            or not nav_request_id
        ):
            raise ValueError(
                "event has invalid nav_request_id"
            )

        event_id = event.get(
            "event_id"
        )

        self.get_logger().info(
            "Processing HIL event "
            f"id={event_id} "
            f"operation={operation} "
            f"request={nav_request_id}"
        )

        if operation == "SUBMIT_GOAL":
            self._handle_submit_goal(
                nav_request_id,
                event,
            )
            return

        if operation == "CANCEL_GOAL":
            self._handle_cancel_goal(
                nav_request_id
            )
            return

        self.get_logger().warning(
            "Ignoring unsupported HIL "
            f"operation: {operation!r}"
        )

    def _handle_submit_goal(
        self,
        nav_request_id: str,
        event: dict[str, Any],
    ) -> None:
        if nav_request_id in self._states:
            self.get_logger().info(
                "Duplicate SUBMIT_GOAL ignored: "
                f"{nav_request_id}"
            )
            return

        state = NavigationRequestState(
            nav_request_id=nav_request_id
        )

        self._states[
            nav_request_id
        ] = state

        goal_data = event.get(
            "goal"
        )

        try:
            ros_goal = self._build_ros_goal(
                goal_data
            )

        except (
            TypeError,
            ValueError,
            KeyError,
        ) as exc:
            self._emit_result(
                state,
                EVENT_GOAL_REJECTED,
                {
                    "reason": (
                        "invalid_goal_payload"
                    ),
                    "detail": str(exc),
                },
            )
            return

        future = (
            self._action_client
            .send_goal_async(
                ros_goal
            )
        )

        future.add_done_callback(
            lambda done_future,
            request_id=nav_request_id:
            self._on_goal_response(
                request_id,
                done_future,
            )
        )

    def _handle_cancel_goal(
        self,
        nav_request_id: str,
    ) -> None:
        state = self._states.get(
            nav_request_id
        )

        if state is None:
            # This should not normally occur because
            # J6M event history is ordered, but expose
            # the contract failure if it does.
            self.get_logger().error(
                "CANCEL_GOAL received before "
                "SUBMIT_GOAL was known: "
                f"{nav_request_id}"
            )
            return

        if state.terminal:
            self.get_logger().info(
                "Ignoring CANCEL_GOAL for "
                "terminal request "
                f"{nav_request_id}"
            )
            return

        state.cancel_requested = True

        if state.goal_handle is not None:
            self._request_cancel(
                state
            )
        else:
            self.get_logger().info(
                "Cancellation queued until "
                "ROS goal acceptance: "
                f"{nav_request_id}"
            )

    def _on_goal_response(
        self,
        nav_request_id: str,
        future: Any,
    ) -> None:
        state = self._states.get(
            nav_request_id
        )

        if (
            state is None
            or state.terminal
        ):
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._emit_result(
                state,
                EVENT_GOAL_REJECTED,
                {
                    "reason": (
                        "action_send_exception"
                    ),
                    "detail": str(exc),
                },
            )
            return

        if not goal_handle.accepted:
            self._emit_result(
                state,
                EVENT_GOAL_REJECTED,
                {
                    "reason": (
                        "ros_action_goal_rejected"
                    ),
                },
            )
            return

        state.goal_handle = goal_handle

        self._emit_result(
            state,
            EVENT_GOAL_ACCEPTED,
            {
                "action": ACTION_NAME,
            },
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            lambda done_future,
            request_id=nav_request_id:
            self._on_goal_result(
                request_id,
                done_future,
            )
        )

        if state.cancel_requested:
            self._request_cancel(
                state
            )

    def _request_cancel(
        self,
        state: NavigationRequestState,
    ) -> None:
        if (
            state.terminal
            or state.cancel_in_flight
            or state.goal_handle is None
        ):
            return

        state.cancel_in_flight = True

        future = (
            state.goal_handle
            .cancel_goal_async()
        )

        future.add_done_callback(
            lambda done_future,
            request_id=state.nav_request_id:
            self._on_cancel_response(
                request_id,
                done_future,
            )
        )

    def _on_cancel_response(
        self,
        nav_request_id: str,
        future: Any,
    ) -> None:
        state = self._states.get(
            nav_request_id
        )

        if (
            state is None
            or state.terminal
        ):
            return

        state.cancel_in_flight = False

        try:
            response = future.result()

            return_code = int(
                getattr(
                    response,
                    "return_code",
                    -1,
                )
            )

            goals_canceling = list(
                getattr(
                    response,
                    "goals_canceling",
                    [],
                )
            )

            accepted = (
                return_code == 0
                and len(
                    goals_canceling
                ) > 0
            )

        except Exception as exc:
            self._emit_result(
                state,
                EVENT_CANCEL_FAILED,
                {
                    "reason": (
                        "cancel_request_exception"
                    ),
                    "detail": str(exc),
                },
            )
            return

        if not accepted:
            self._emit_result(
                state,
                EVENT_CANCEL_FAILED,
                {
                    "reason": (
                        "ros_action_cancel_rejected"
                    ),
                    "return_code": return_code,
                },
            )
            return

        state.cancel_accepted = True

        self.get_logger().info(
            "ROS cancel accepted; waiting "
            "for actual CANCELED result: "
            f"{nav_request_id}"
        )

    def _on_goal_result(
        self,
        nav_request_id: str,
        future: Any,
    ) -> None:
        state = self._states.get(
            nav_request_id
        )

        if (
            state is None
            or state.terminal
        ):
            return

        try:
            wrapped_result = future.result()
            status = int(
                wrapped_result.status
            )

        except Exception as exc:
            self._emit_result(
                state,
                EVENT_FAILED,
                {
                    "reason": (
                        "action_result_exception"
                    ),
                    "detail": str(exc),
                },
            )
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            if state.cancel_requested:
                self._emit_result(
                    state,
                    EVENT_CANCEL_FAILED,
                    {
                        "reason": (
                            "goal_finished_before_cancel"
                        ),
                        "ros_status": status,
                    },
                )
            else:
                self._emit_result(
                    state,
                    EVENT_SUCCEEDED,
                    {
                        "ros_status": status,
                    },
                )
            return

        if status == GoalStatus.STATUS_CANCELED:
            if (
                state.cancel_requested
                and state.cancel_accepted
            ):
                self._emit_result(
                    state,
                    EVENT_CANCEL_CONFIRMED,
                    {
                        "ros_status": status,
                    },
                )
            elif state.cancel_requested:
                self._emit_result(
                    state,
                    EVENT_CANCEL_FAILED,
                    {
                        "reason": (
                            "canceled_without_valid_"
                            "cancel_response"
                        ),
                        "ros_status": status,
                    },
                )
            else:
                self._emit_result(
                    state,
                    EVENT_FAILED,
                    {
                        "reason": (
                            "unexpected_ros_cancel"
                        ),
                        "ros_status": status,
                    },
                )
            return

        self._emit_result(
            state,
            EVENT_FAILED,
            {
                "reason": (
                    "ros_action_not_succeeded"
                ),
                "ros_status": status,
            },
        )

    def _emit_result(
        self,
        state: NavigationRequestState,
        event_name: str,
        payload: object | None,
    ) -> None:
        if event_name in state.emitted_events:
            return

        if state.terminal:
            return

        state.emitted_events.add(
            event_name
        )

        if event_name in TERMINAL_EVENTS:
            state.terminal = True

        body = {
            "nav_request_id": (
                state.nav_request_id
            ),
            "event": event_name,
            "payload": payload,
        }

        self.get_logger().info(
            "Queueing J6M navigation result: "
            + json.dumps(
                body,
                ensure_ascii=False,
                separators=(",", ":"),
            )
        )

        self._outgoing_results.put(
            body
        )

    def _build_ros_goal(
        self,
        goal_data: object,
    ) -> NavigateToPose.Goal:
        if not isinstance(
            goal_data,
            dict,
        ):
            raise ValueError(
                "goal must be a JSON object"
            )

        if goal_data.get(
            "kind"
        ) != "pose_stamped":
            raise ValueError(
                "only pose_stamped HIL goals "
                "are supported"
            )

        frame_id = goal_data.get(
            "frame_id"
        )

        if (
            not isinstance(
                frame_id,
                str,
            )
            or not frame_id
        ):
            raise ValueError(
                "goal frame_id is invalid"
            )

        position = goal_data.get(
            "position"
        )

        orientation = goal_data.get(
            "orientation"
        )

        if not isinstance(
            position,
            dict,
        ):
            raise ValueError(
                "goal position is invalid"
            )

        if not isinstance(
            orientation,
            dict,
        ):
            raise ValueError(
                "goal orientation is invalid"
            )

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = (
            frame_id
        )

        goal.pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal.pose.pose.position.x = float(
            position["x"]
        )
        goal.pose.pose.position.y = float(
            position["y"]
        )
        goal.pose.pose.position.z = float(
            position.get(
                "z",
                0.0,
            )
        )

        goal.pose.pose.orientation.x = float(
            orientation.get(
                "x",
                0.0,
            )
        )
        goal.pose.pose.orientation.y = float(
            orientation.get(
                "y",
                0.0,
            )
        )
        goal.pose.pose.orientation.z = float(
            orientation.get(
                "z",
                0.0,
            )
        )
        goal.pose.pose.orientation.w = float(
            orientation.get(
                "w",
                1.0,
            )
        )

        return goal


def parse_args(
    argv: list[str],
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "CleanNav PC-side Demo HIL Navigation "
            "HTTP-to-ROS2 bridge"
        )
    )

    parser.add_argument(
        "--server",
        default=DEFAULT_SERVER,
        help=(
            "J6M HIL navigation server base URL "
            f"(default: {DEFAULT_SERVER})"
        ),
    )

    parser.add_argument(
        "--poll-interval",
        type=float,
        default=DEFAULT_POLL_INTERVAL,
        help=(
            "HTTP event polling interval in seconds"
        ),
    )

    parser.add_argument(
        "--http-timeout",
        type=float,
        default=DEFAULT_HTTP_TIMEOUT,
        help="HTTP timeout in seconds",
    )

    parser.add_argument(
        "--after-event-id",
        type=int,
        default=0,
        help=(
            "Start polling strictly after this "
            "J6M event id"
        ),
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "Actually execute J6M navigation events. "
            "Without this flag the client is "
            "observe-only."
        ),
    )

    options = parser.parse_args(
        remove_ros_args(
            args=argv
        )[1:]
    )

    if options.after_event_id < 0:
        parser.error(
            "--after-event-id must be >= 0"
        )

    if options.poll_interval <= 0:
        parser.error(
            "--poll-interval must be > 0"
        )

    if options.http_timeout <= 0:
        parser.error(
            "--http-timeout must be > 0"
        )

    return options


def main(
    args: list[str] | None = None,
) -> None:
    argv = (
        sys.argv
        if args is None
        else args
    )

    options = parse_args(
        argv
    )

    rclpy.init(
        args=argv
    )

    node: (
        DemoHilNavigationClient
        | None
    ) = None

    try:
        node = DemoHilNavigationClient(
            server=options.server,
            poll_interval=(
                options.poll_interval
            ),
            http_timeout=(
                options.http_timeout
            ),
            after_event_id=(
                options.after_event_id
            ),
            execute=options.execute,
        )

        rclpy.spin(
            node
        )

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
