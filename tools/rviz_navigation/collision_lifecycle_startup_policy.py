"""Pure fail-closed policy for Collision Monitor lifecycle startup."""

UNCONFIGURED = 1
INACTIVE = 2
ACTIVE = 3


def startup_decision(*, state_id, stable_state_samples, services_ready,
                     services_stable_s, discovery_settle_s, valid_scan_messages,
                     startup_sent):
    """Allow exactly one manager startup after endpoints and initial state settle."""
    if startup_sent:
        return 'WAIT_RECONCILE'
    if not services_ready:
        return 'WAIT_SERVICES'
    if state_id is None:
        return 'WAIT_STATE'
    if state_id == ACTIVE:
        return 'FAIL_UNMANAGED_ACTIVE'
    if state_id != UNCONFIGURED:
        if state_id in (10, 11, 12):
            return 'WAIT_TRANSITION'
        return 'FAIL_UNEXPECTED_INITIAL_STATE'
    if valid_scan_messages < 3:
        return 'WAIT_SCAN'
    if stable_state_samples < 3 or services_stable_s < discovery_settle_s:
        return 'WAIT_STABLE'
    return 'REQUEST_MANAGER_STARTUP'


def completion_decision(*, manager_active, monitor_state_id, valid_scan_count,
                        startup_response, startup_timed_out, deadline_expired):
    """Pass only when the manager owns an ACTIVE monitor with real scan input."""
    if manager_active is True and monitor_state_id == ACTIVE and valid_scan_count >= 3:
        return 'PASS_RECONCILED' if startup_timed_out else 'PASS'
    if startup_response is False:
        return 'FAIL_MANAGER_REJECTED'
    if deadline_expired:
        if manager_active is True and monitor_state_id != ACTIVE:
            return 'FAIL_MANAGER_ACTIVE_MONITOR_INACTIVE'
        if monitor_state_id == ACTIVE and manager_active is not True:
            return 'FAIL_ACTIVE_WITHOUT_MANAGER_BOND'
        if manager_active is None or monitor_state_id is None:
            return 'FAIL_STATE_UNCONFIRMED'
        if valid_scan_count < 3:
            return 'FAIL_SCAN_UNCONFIRMED'
        return 'FAIL_NOT_ACTIVE'
    return 'WAIT'
