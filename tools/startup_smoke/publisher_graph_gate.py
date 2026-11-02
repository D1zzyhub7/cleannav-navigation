"""Bounded, fail-closed convergence check for the /cmd_vel publisher graph."""
from __future__ import annotations

import time
from typing import Any, Callable, Iterable


EXPECTED_NODE = "cleannav_safety_supervisor"
EXPECTED_NAMESPACE = "/"
EXPECTED_TOPIC_TYPE = "geometry_msgs/msg/Twist"
UNKNOWN_NODE_NAMES = {"", "_NODE_NAME_UNKNOWN_", "UNKNOWN"}
UNKNOWN_NAMESPACES = {None, "", "_NODE_NAMESPACE_UNKNOWN_", "UNKNOWN"}
UNKNOWN_TOPIC_TYPES = {None, "", "_TOPIC_TYPE_UNKNOWN_", "UNKNOWN"}
UNKNOWN_ENDPOINT_GIDS = {None, "", "_ENDPOINT_GID_UNKNOWN_", "UNKNOWN"}


class PublisherGraphGateError(RuntimeError):
    def __init__(self, message: str, result: dict[str, Any]):
        super().__init__(message)
        self.result = result


def _json_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    try:
        return [_json_value(item) for item in list(value)]
    except (TypeError, ValueError):
        return str(value)


def _qos_info(qos: Any) -> dict[str, Any] | None:
    if qos is None:
        return None
    fields = (
        "history", "depth", "reliability", "durability", "lifespan",
        "deadline", "liveliness", "liveliness_lease_duration",
        "avoid_ros_namespace_conventions",
    )
    return {field: _json_value(getattr(qos, field)) for field in fields
            if hasattr(qos, field)}


def endpoint_record(endpoint: Any) -> dict[str, Any]:
    """Return JSON-safe identity and QoS details from ROS or test endpoint data."""
    if isinstance(endpoint, dict):
        record = dict(endpoint)
        record.setdefault("node_name", None)
        record.setdefault("node_namespace", None)
        record.setdefault("topic_type", None)
        record.setdefault("endpoint_gid", None)
        record["endpoint_gid"] = _json_value(record["endpoint_gid"])
        if "qos_profile" in record:
            record["qos_profile"] = _json_value(record["qos_profile"])
        return record

    gid = getattr(endpoint, "endpoint_gid", None)
    if gid is None:
        gid = getattr(endpoint, "gid", None)
    return {
        "node_name": getattr(endpoint, "node_name", None),
        "node_namespace": getattr(endpoint, "node_namespace", None),
        "topic_type": getattr(endpoint, "topic_type", None),
        "endpoint_gid": _json_value(gid),
        "endpoint_type": _json_value(getattr(endpoint, "endpoint_type", None)),
        "qos_profile": _qos_info(getattr(endpoint, "qos_profile", None)),
    }


def endpoint_key(record: dict[str, Any]) -> tuple[Any, ...]:
    """Endpoint identity used for consecutive-sample stability."""
    return (
        record.get("node_name"), record.get("node_namespace"),
        record.get("topic_type"), record.get("endpoint_gid"),
    )


def publisher_snapshot_error(records: list[dict[str, Any]]) -> str | None:
    """Validate a post-convergence snapshot; empty and unknown sets fail closed."""
    if not records:
        return "publisher_set_empty"
    if len(records) != 1:
        return "publisher_count_not_one"
    endpoint = records[0]
    if _unknown_identity_fields(endpoint):
        return "publisher_identity_unknown"
    if endpoint.get("node_name") != EXPECTED_NODE:
        return "unexpected_publisher_node"
    if endpoint.get("node_namespace") != EXPECTED_NAMESPACE:
        return "unexpected_publisher_namespace"
    if endpoint.get("topic_type") != EXPECTED_TOPIC_TYPE:
        return "publisher_topic_type_unknown_or_unexpected"
    return None


def _unknown_identity_fields(endpoint: dict[str, Any]) -> list[str]:
    """Return identity fields that ROS graph discovery has not resolved yet."""
    unknown = []
    if endpoint.get("node_name") in UNKNOWN_NODE_NAMES:
        unknown.append("node_name")
    if endpoint.get("node_namespace") in UNKNOWN_NAMESPACES:
        unknown.append("node_namespace")
    if endpoint.get("topic_type") in UNKNOWN_TOPIC_TYPES:
        unknown.append("topic_type")
    gid = endpoint.get("endpoint_gid")
    if (gid is None or gid == [] or
            (isinstance(gid, str) and gid in UNKNOWN_ENDPOINT_GIDS)):
        unknown.append("endpoint_gid")
    return unknown


class PublisherGraphConvergence:
    """Tracks a single expected endpoint across a bounded discovery window."""

    def __init__(self, start_monotonic_ns: int, *, timeout_ns: int = 5_000_000_000,
                 minimum_samples: int = 5, minimum_stable_ns: int = 1_000_000_000):
        self.start_ns = int(start_monotonic_ns)
        self.deadline_ns = self.start_ns + int(timeout_ns)
        self.minimum_samples = int(minimum_samples)
        self.minimum_stable_ns = int(minimum_stable_ns)
        self.sample_count = 0
        self.stable_count = 0
        self.stable_since_ns: int | None = None
        self.last_key: tuple[Any, ...] | None = None
        self.last_endpoints: list[dict[str, Any]] = []
        self.failure_reason: str | None = None
        self.converged = False
        self.convergence_achieved = False
        self.converged_endpoint_key: tuple[Any, ...] | None = None
        self.samples: list[dict[str, Any]] = []

    def sample_query(
        self, query: Callable[[], Iterable[Any]], monotonic_ns: Any,
    ) -> dict[str, Any]:
        """Sample a ROS graph query and retain query failures as fail-closed evidence."""
        try:
            endpoints = query()
        except Exception as error:
            now = monotonic_ns() if callable(monotonic_ns) else int(monotonic_ns)
            return self.sample(
                [], now,
                query_error=f"publisher_graph_query_failed:{type(error).__name__}",
            )
        now = monotonic_ns() if callable(monotonic_ns) else int(monotonic_ns)
        return self.sample(endpoints, now)

    def sample(
        self, endpoints: Iterable[Any], monotonic_ns: int, *,
        query_error: str | None = None,
    ) -> dict[str, Any]:
        now = int(monotonic_ns)
        try:
            records = [endpoint_record(endpoint) for endpoint in endpoints]
        except Exception as error:
            records = []
            query_error = query_error or f"publisher_graph_record_failed:{type(error).__name__}"
        self.sample_count += 1
        self.last_endpoints = records
        reason = None
        observation_reason = None
        identity_unknown_fields = (
            _unknown_identity_fields(records[0]) if len(records) == 1 else []
        )

        if self.failure_reason is not None:
            reason = self.failure_reason
        elif query_error is not None:
            reason = query_error
        elif self.convergence_achieved:
            # The initial discovery deadline no longer applies. Keep checking
            # the full endpoint identity on every later sample and fail closed
            # if the established publisher disappears, changes, or is joined.
            reason = publisher_snapshot_error(records)
            if reason is None and endpoint_key(records[0]) != self.converged_endpoint_key:
                reason = "publisher_identity_changed"
        elif len(records) > 1:
            reason = "publisher_count_not_one"
        elif records:
            if identity_unknown_fields:
                reason = "publisher_identity_unknown"
            else:
                reason = publisher_snapshot_error(records)
            if reason is None and now > self.deadline_ns:
                reason = "publisher_convergence_timeout"
            if reason is None:
                key = endpoint_key(records[0])
                if key != self.last_key:
                    self.last_key = key
                    self.stable_since_ns = now
                    self.stable_count = 1
                else:
                    self.stable_count += 1
                stable_elapsed_ns = now - int(self.stable_since_ns)
                if (self.stable_count >= self.minimum_samples
                        and stable_elapsed_ns >= self.minimum_stable_ns):
                    self.converged = True
                    self.convergence_achieved = True
                    self.converged_endpoint_key = key
            if reason is None and not self.converged and now >= self.deadline_ns:
                reason = "publisher_convergence_timeout"
        else:
            self.last_key = None
            self.stable_since_ns = None
            self.stable_count = 0
            if now >= self.deadline_ns:
                reason = "publisher_convergence_timeout"

        if reason is not None:
            self.failure_reason = reason
            self.converged = False
        elif self.convergence_achieved:
            self.converged = True

        stable_elapsed_ns = (now - int(self.stable_since_ns)
                             if self.stable_since_ns is not None else 0)
        record = {
            "sample_index": self.sample_count,
            "monotonic_ns": now,
            "endpoints": records,
            "endpoint_count": len(records),
            "observation_reason": observation_reason,
            "identity_unknown_fields": identity_unknown_fields,
            "stable_count": self.stable_count,
            "stable_elapsed_ns": stable_elapsed_ns,
            "converged": self.converged,
            "convergence_achieved": self.convergence_achieved,
            "failure_reason": self.failure_reason,
        }
        self.samples.append(record)
        return record

    def result(self) -> dict[str, Any]:
        return {
            "status": "PASS" if self.converged and self.failure_reason is None else "FAIL",
            "converged": self.converged and self.failure_reason is None,
            "sample_count": self.sample_count,
            "minimum_samples": self.minimum_samples,
            "minimum_stable_seconds": self.minimum_stable_ns / 1e9,
            "timeout_seconds": (self.deadline_ns - self.start_ns) / 1e9,
            "failure_reason": self.failure_reason,
            "convergence_achieved": self.convergence_achieved,
            "stable_count": self.stable_count,
            "endpoints": self.last_endpoints,
            "samples": list(self.samples),
        }


def converge_publisher_graph(
    query: Callable[[], Iterable[Any]],
    *,
    on_sample: Callable[[dict[str, Any]], None] | None = None,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    timeout_s: float = 5.0,
    poll_s: float = 0.1,
) -> dict[str, Any]:
    """Poll every 100 ms until the exact publisher is stable or the gate fails."""
    if timeout_s <= 0 or poll_s <= 0:
        raise ValueError("publisher graph gate timing must be positive")
    start = monotonic_ns()
    gate = PublisherGraphConvergence(
        start, timeout_ns=int(timeout_s * 1e9), minimum_samples=5,
        minimum_stable_ns=1_000_000_000,
    )
    next_poll_ns = start
    while True:
        now = monotonic_ns()
        sample = gate.sample_query(query, monotonic_ns)
        if on_sample:
            on_sample(sample)
        if sample["failure_reason"] or sample["converged"]:
            break
        now = monotonic_ns()
        if now >= gate.deadline_ns:
            gate.failure_reason = "publisher_convergence_timeout"
            gate.samples[-1]["failure_reason"] = gate.failure_reason
            break
        next_poll_ns += int(poll_s * 1e9)
        sleep_ns = min(max(0, next_poll_ns - now), gate.deadline_ns - now)
        if sleep_ns:
            sleep(sleep_ns / 1e9)

    result = gate.result()
    if not result["converged"]:
        raise PublisherGraphGateError(
            f"/cmd_vel publisher graph did not converge: {result['failure_reason']}",
            result,
        )
    return result
