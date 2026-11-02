"""Regression tests for bounded /cmd_vel graph discovery and live identity checks."""
import pytest

from publisher_graph_gate import (
    PublisherGraphConvergence,
    PublisherGraphGateError,
    converge_publisher_graph,
)


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


def converged_gate():
    gate = PublisherGraphConvergence(0)
    for now_ns in (0, 250_000_000, 500_000_000, 750_000_000, 1_000_000_000):
        sample = gate.sample([EXPECTED], now_ns)
    assert sample["converged"] is True
    assert gate.convergence_achieved is True
    return gate


def test_empty_graph_then_unique_expected_publisher_converges():
    result, _clock, _index = run_with_snapshots(
        lambda index, _clock: [] if index < 2 else [EXPECTED])
    assert result["status"] == "PASS"
    assert result["sample_count"] >= 7
    assert result["samples"][0]["endpoint_count"] == 0
    assert result["samples"][-1]["stable_elapsed_ns"] >= 1_000_000_000
    assert result["endpoints"] == [EXPECTED]


def test_no_publisher_before_deadline_remains_waiting():
    gate = PublisherGraphConvergence(0)
    sample = gate.sample([], 4_900_000_000)
    assert sample["failure_reason"] is None
    assert sample["converged"] is False
    assert gate.failure_reason is None


def test_publisher_never_appears_times_out_fail_closed():
    clock = FakeClock()
    samples = []
    with pytest.raises(PublisherGraphGateError) as exc:
        converge_publisher_graph(lambda: [], on_sample=samples.append,
                                 monotonic_ns=clock.monotonic_ns, sleep=clock.sleep)
    assert exc.value.result["failure_reason"] == "publisher_convergence_timeout"
    assert samples[-1]["monotonic_ns"] <= 5_000_000_000
    assert exc.value.result["status"] == "FAIL"


def test_deadline_overrun_fails_even_if_expected_publisher_appears():
    clock = FakeClock()

    def late_query():
        clock.now_ns = 5_100_000_000
        return [EXPECTED]

    with pytest.raises(PublisherGraphGateError) as exc:
        converge_publisher_graph(late_query, monotonic_ns=clock.monotonic_ns,
                                 sleep=clock.sleep)
    assert exc.value.result["failure_reason"] == "publisher_convergence_timeout"
    assert exc.value.result["status"] == "FAIL"


@pytest.mark.parametrize("bad_endpoint, expected_reason", [
    ({**EXPECTED, "node_name": "_NODE_NAME_UNKNOWN_"}, "publisher_identity_unknown"),
    ({**EXPECTED, "node_namespace": None}, "publisher_identity_unknown"),
    ({**EXPECTED, "topic_type": None}, "publisher_identity_unknown"),
    ({**EXPECTED, "endpoint_gid": None}, "publisher_identity_unknown"),
    ({**EXPECTED, "node_name": "untrusted_node"}, "unexpected_publisher_node"),
])
def test_unknown_or_illegal_initial_identity_fails_closed(bad_endpoint, expected_reason):
    gate = PublisherGraphConvergence(0)
    sample = gate.sample([bad_endpoint], 1)
    assert sample["failure_reason"] == expected_reason
    assert sample["converged"] is False
    assert gate.result()["status"] == "FAIL"


def test_multiple_publishers_fail_closed_before_convergence():
    second = {**EXPECTED, "node_name": "intruder", "endpoint_gid": "04:05:06"}
    with pytest.raises(PublisherGraphGateError) as exc:
        run_with_snapshots([[EXPECTED, second]])
    assert exc.value.result["failure_reason"] == "publisher_count_not_one"


def test_endpoint_change_before_convergence_resets_stability_until_timeout():
    def alternating(index, _clock):
        return [{**EXPECTED, "endpoint_gid": f"gid-{index % 2}"}]

    with pytest.raises(PublisherGraphGateError) as exc:
        run_with_snapshots(alternating)
    assert exc.value.result["failure_reason"] == "publisher_convergence_timeout"
    assert exc.value.result["sample_count"] > 5
    assert exc.value.result["stable_count"] == 1
    assert exc.value.result["convergence_achieved"] is False


def test_converged_publisher_remains_healthy_after_initial_deadline():
    gate = converged_gate()
    sample = gate.sample([EXPECTED], 30_000_000_000)
    assert sample["converged"] is True
    assert sample["failure_reason"] is None
    assert gate.result()["status"] == "PASS"


def test_convergence_does_not_become_initial_timeout_as_time_advances():
    gate = converged_gate()
    for now_ns in (6_000_000_000, 10_000_000_000, 60_000_000_000):
        sample = gate.sample([EXPECTED], now_ns)
        assert sample["failure_reason"] is None
        assert sample["converged"] is True
    assert gate.result()["status"] == "PASS"
    assert gate.result()["failure_reason"] is None


@pytest.mark.parametrize("replacement, expected_reason", [
    ([], "publisher_set_empty"),
    ([EXPECTED, {**EXPECTED, "endpoint_gid": "second-gid"}], "publisher_count_not_one"),
    ([{**EXPECTED, "node_name": "UNKNOWN"}], "publisher_identity_unknown"),
    ([{**EXPECTED, "endpoint_gid": "replacement-gid"}], "publisher_identity_changed"),
    ([{**EXPECTED, "node_name": "intruder"}], "unexpected_publisher_node"),
])
def test_converged_publisher_loss_or_identity_change_fails_closed(
    replacement, expected_reason,
):
    gate = converged_gate()
    sample = gate.sample(replacement, 6_000_000_000)
    assert sample["convergence_achieved"] is True
    assert sample["converged"] is False
    assert sample["failure_reason"] == expected_reason
    assert gate.result()["status"] == "FAIL"


def test_graph_query_exception_fails_closed_before_convergence():
    gate = PublisherGraphConvergence(0)

    def broken_query():
        raise RuntimeError("graph service unavailable")

    sample = gate.sample_query(broken_query, 100_000_000)
    assert sample["failure_reason"] == "publisher_graph_query_failed:RuntimeError"
    assert sample["converged"] is False
    assert gate.result()["status"] == "FAIL"


def test_graph_query_exception_fails_closed_after_convergence():
    gate = converged_gate()

    def broken_query():
        raise RuntimeError("graph service unavailable")

    sample = gate.sample_query(broken_query, 6_000_000_000)
    assert sample["convergence_achieved"] is True
    assert sample["failure_reason"] == "publisher_graph_query_failed:RuntimeError"
    assert sample["converged"] is False
    assert gate.result()["status"] == "FAIL"
