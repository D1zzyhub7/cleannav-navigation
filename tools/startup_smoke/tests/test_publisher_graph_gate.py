"""Regression tests for bounded /cmd_vel graph discovery and identity checks."""
import pytest

from publisher_graph_gate import PublisherGraphGateError, converge_publisher_graph


EXPECTED = {
    "node_name": "cleannav_safety_supervisor",
    "node_namespace": "/",
    "topic_type": "geometry_msgs/msg/Twist",
    "endpoint_gid": "01:02:03",
}


class FakeClock:
    def __init__(self):
        self.now_ns = 0

    def monotonic_ns(self):
        return self.now_ns

    def sleep(self, seconds):
        self.now_ns += int(seconds * 1e9)


def run_with_snapshots(snapshots, *, timeout_s=5.0):
    clock = FakeClock()
    index = 0

    def query():
        nonlocal index
        value = snapshots(index, clock) if callable(snapshots) else (
            snapshots[min(index, len(snapshots) - 1)] if snapshots else [])
        index += 1
        return value

    result = converge_publisher_graph(
        query, monotonic_ns=clock.monotonic_ns, sleep=clock.sleep,
        timeout_s=timeout_s, poll_s=0.1,
    )
    return result, clock, index


def test_empty_graph_then_unique_expected_publisher_converges():
    result, _clock, _index = run_with_snapshots(
        lambda index, _clock: [] if index < 2 else [EXPECTED])
    assert result["status"] == "PASS"
    assert result["sample_count"] >= 5
    assert result["samples"][0]["endpoint_count"] == 0
    assert result["samples"][-1]["stable_elapsed_ns"] >= 1_000_000_000
    assert result["endpoints"] == [EXPECTED]


def test_publisher_never_appears_times_out_fail_closed():
    clock = FakeClock()
    samples = []
    with pytest.raises(PublisherGraphGateError) as exc:
        converge_publisher_graph(lambda: [], on_sample=samples.append,
                                 monotonic_ns=clock.monotonic_ns, sleep=clock.sleep)
    assert exc.value.result["failure_reason"] == "publisher_convergence_timeout"
    assert samples[-1]["monotonic_ns"] <= 5_000_000_000
    assert exc.value.result["status"] == "FAIL"


def test_unknown_identity_then_expected_publisher_converges_within_bound():
    unknown = {**EXPECTED, "node_name": "_NODE_NAME_UNKNOWN_"}
    result, _clock, _index = run_with_snapshots(
        lambda index, _clock: [unknown] if index < 2 else [EXPECTED])
    assert result["status"] == "PASS"
    assert result["samples"][0]["observation_reason"] == "publisher_identity_unknown"
    assert result["samples"][0]["identity_unknown_fields"] == ["node_name"]
    assert result["samples"][-1]["stable_elapsed_ns"] >= 1_000_000_000


def test_unknown_publisher_identity_at_deadline_fails_closed():
    unknown = {**EXPECTED, "node_name": "_NODE_NAME_UNKNOWN_"}
    clock = FakeClock()
    samples = []
    with pytest.raises(PublisherGraphGateError) as exc:
        converge_publisher_graph(lambda: [unknown], on_sample=samples.append,
                                 monotonic_ns=clock.monotonic_ns, sleep=clock.sleep)
    assert exc.value.result["failure_reason"] == "publisher_identity_unknown"
    assert samples[-1]["monotonic_ns"] == 5_000_000_000
    assert samples[-1]["identity_unknown_fields"] == ["node_name"]
    assert exc.value.result["status"] == "FAIL"


def test_unknown_namespace_is_transient_but_never_accepted():
    unknown = {**EXPECTED, "node_namespace": None}
    result, _clock, _index = run_with_snapshots(
        lambda index, _clock: [unknown] if index < 1 else [EXPECTED])
    assert result["status"] == "PASS"
    assert result["samples"][0]["identity_unknown_fields"] == ["node_namespace"]


def test_multiple_publishers_fail_closed():
    second = {**EXPECTED, "node_name": "intruder", "endpoint_gid": "04:05:06"}
    with pytest.raises(PublisherGraphGateError) as exc:
        run_with_snapshots([[EXPECTED, second]])
    assert exc.value.result["failure_reason"] == "publisher_count_not_one"


def test_endpoint_change_resets_stability_and_fails_if_no_convergence():
    def alternating(index, _clock):
        return [{**EXPECTED, "endpoint_gid": f"gid-{index % 2}"}]

    with pytest.raises(PublisherGraphGateError) as exc:
        run_with_snapshots(alternating)
    assert exc.value.result["failure_reason"] == "publisher_convergence_timeout"
    assert exc.value.result["sample_count"] > 5
    assert exc.value.result["stable_count"] == 1


def test_deadline_overrun_fails_even_if_expected_publisher_appears():
    clock = FakeClock()

    def late_query():
        clock.now_ns = 5_100_000_000
        return [EXPECTED]

    with pytest.raises(PublisherGraphGateError) as exc:
        converge_publisher_graph(late_query, monotonic_ns=clock.monotonic_ns,
                                 sleep=clock.sleep)
    assert exc.value.result["failure_reason"] == "publisher_convergence_timeout"
