#!/usr/bin/env python3
"""Start Collision Monitor once through Nav2 manager after DDS discovery settles.

This process never calls the monitor's change_state service. Nav2's lifecycle
manager remains the sole transition owner and establishes the bond required for
runtime supervision. If a manager response is lost, state is reconciled through
the manager's is_active service plus the monitor's GetState service; ambiguous
or unmanaged states fail closed.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from collision_lifecycle_startup_policy import (
    ACTIVE, UNCONFIGURED, completion_decision, startup_decision,
)


def _service_contract(services, node_name, manager_name):
    expected = {
        node_name.rstrip('/') + '/get_state': 'lifecycle_msgs/srv/GetState',
        node_name.rstrip('/') + '/change_state': 'lifecycle_msgs/srv/ChangeState',
        manager_name.rstrip('/') + '/manage_nodes': 'nav2_msgs/srv/ManageLifecycleNodes',
        manager_name.rstrip('/') + '/is_active': 'std_srvs/srv/Trigger',
    }
    checks = {}
    for name, type_name in expected.items():
        actual = services.get(name)
        checks[name] = {
            'status': 'WAIT' if actual is None else
                      'READY' if type_name in actual else 'INVALID',
            'expected': type_name,
            'actual': list(actual) if actual is not None else None,
        }
    statuses = {item['status'] for item in checks.values()}
    status = 'INVALID' if 'INVALID' in statuses else 'READY' if statuses == {'READY'} else 'WAIT'
    return {'status': status, 'services': checks}


class StartupCoordinator(Node):
    def __init__(self, node_name, manager_name):
        super().__init__('mapping_collision_lifecycle_startup_coordinator')
        self.node_name = node_name.rstrip('/')
        self.manager_name = manager_name.rstrip('/')
        self.state_client = self.create_client(GetState, self.node_name + '/get_state')
        self.manager_client = self.create_client(
            ManageLifecycleNodes, self.manager_name + '/manage_nodes')
        self.active_client = self.create_client(Trigger, self.manager_name + '/is_active')
        self.scans = 0
        self.valid_scan_messages = 0
        self.valid_scan_samples = 0
        self.invalid_scan_samples = 0
        self.frames = Counter()
        self.scan_parameters = Counter()
        self.events = []
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        self.scans += 1
        self.frames[msg.header.frame_id] += 1
        self.scan_parameters[json.dumps({
            'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment, 'range_min': msg.range_min,
            'range_max': msg.range_max, 'beam_count': len(msg.ranges),
        }, sort_keys=True)] += 1
        valid = sum(1 for value in msg.ranges if math.isfinite(value)
                    and msg.range_min <= value <= msg.range_max)
        self.valid_scan_samples += valid
        self.invalid_scan_samples += len(msg.ranges) - valid
        if msg.header.frame_id == 'base_scan' and valid > 0:
            self.valid_scan_messages += 1

    def _spin_future(self, future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(.05, max(.005, deadline-time.monotonic())))
        if future.done():
            return future.result(), False
        return None, True

    def _query(self, client, request, timeout_s=.75):
        if not client.service_is_ready():
            return None, 'SERVICE_NOT_READY'
        future = client.call_async(request)
        response, timed_out = self._spin_future(future, timeout_s)
        if timed_out:
            client.remove_pending_request(future)
            return None, 'RESPONSE_TIMEOUT'
        return response, None

    def run(self, *, timeout_s, discovery_settle_s, startup_response_timeout_s):
        started_utc = datetime.now(timezone.utc).isoformat()
        start = time.monotonic()
        deadline = start + timeout_s
        services_ready_since = None
        last_poll = 0.0
        stable_state_id = None
        stable_state_samples = 0
        manager_active = None
        monitor_state_id = None
        startup_response = None
        startup_timed_out = False
        startup_sent = False
        startup_requested_at = None
        transitions = []
        state_queries = []
        manager_queries = []
        service_checks = []
        error = None
        result = 'WAIT_SERVICES'

        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            services = {name: types for name, types in self.get_service_names_and_types()}
            contract = _service_contract(services, self.node_name, self.manager_name)
            if not service_checks or contract['status'] != service_checks[-1]['status']:
                service_checks.append({
                    'elapsed_s': round(now-start, 6), 'status': contract['status'],
                    'services': contract['services'],
                })
            if contract['status'] == 'INVALID':
                result = 'FAIL_SERVICE_CONTRACT'
                error = 'lifecycle manager or monitor service type mismatch'
                break
            if contract['status'] == 'READY':
                if services_ready_since is None:
                    services_ready_since = now
            else:
                services_ready_since = None

            if now >= last_poll:
                state_request_at = time.monotonic()
                state_response, state_error = self._query(self.state_client, GetState.Request())
                if state_response is not None and state_response.current_state is not None:
                    monitor_state_id = int(state_response.current_state.id)
                    if monitor_state_id == stable_state_id:
                        stable_state_samples += 1
                    else:
                        stable_state_id = monitor_state_id
                        stable_state_samples = 1
                    state_queries.append({
                        'requested_elapsed_s': round(state_request_at-start, 6),
                        'response_elapsed_s': round(time.monotonic()-start, 6),
                        'state_id': monitor_state_id,
                        'state_label': str(state_response.current_state.label),
                    })
                elif state_error:
                    state_queries.append({
                        'requested_elapsed_s': round(state_request_at-start, 6),
                        'error': state_error,
                    })

                manager_request_at = time.monotonic()
                active_response, active_error = self._query(self.active_client, Trigger.Request())
                if active_response is not None:
                    manager_active = bool(active_response.success)
                    manager_queries.append({
                        'requested_elapsed_s': round(manager_request_at-start, 6),
                        'response_elapsed_s': round(time.monotonic()-start, 6),
                        'active': manager_active,
                        'message': str(active_response.message),
                    })
                elif active_error:
                    manager_queries.append({
                        'requested_elapsed_s': round(manager_request_at-start, 6),
                        'error': active_error,
                    })

                now = time.monotonic()
                stable_for = (now-services_ready_since if services_ready_since is not None else 0.0)
                decision = startup_decision(
                    state_id=monitor_state_id,
                    stable_state_samples=stable_state_samples,
                    services_ready=(contract['status'] == 'READY'),
                    services_stable_s=stable_for,
                    discovery_settle_s=discovery_settle_s,
                    valid_scan_messages=self.valid_scan_messages,
                    startup_sent=startup_sent,
                )
                if decision == 'FAIL_UNMANAGED_ACTIVE':
                    result = decision
                    error = 'monitor is active before this lifecycle manager owns startup/bond'
                    break
                if decision == 'FAIL_UNEXPECTED_INITIAL_STATE':
                    result = decision
                    error = f'unexpected monitor lifecycle state before startup: {monitor_state_id}'
                    break
                if decision == 'REQUEST_MANAGER_STARTUP':
                    request = ManageLifecycleNodes.Request()
                    request.command = ManageLifecycleNodes.Request.STARTUP
                    requested = time.monotonic()
                    future = self.manager_client.call_async(request)
                    startup_sent = True
                    startup_requested_at = requested
                    transitions.append({
                        'kind': 'manager_startup_request',
                        'requested_elapsed_s': round(requested-start, 6),
                        'manager': self.manager_name,
                        'request_count': 1,
                    })
                    response, timed_out = self._spin_future(
                        future, min(startup_response_timeout_s, max(.05, deadline-time.monotonic())))
                    if timed_out:
                        self.manager_client.remove_pending_request(future)
                        startup_timed_out = True
                        error = 'manager STARTUP response timed out; reconciling manager and monitor states'
                        transitions.append({
                            'kind': 'manager_startup_response_timeout',
                            'elapsed_s': round(time.monotonic()-start, 6),
                        })
                    elif response is not None:
                        startup_response = bool(response.success)
                        transitions.append({
                            'kind': 'manager_startup_response',
                            'elapsed_s': round(time.monotonic()-start, 6),
                            'success': startup_response,
                        })
                        if startup_response is False:
                            result = 'FAIL_MANAGER_REJECTED'
                            error = 'lifecycle manager rejected STARTUP'
                            break
                    else:
                        result = 'FAIL_MANAGER_EMPTY_RESPONSE'
                        error = 'lifecycle manager returned no STARTUP response'
                        break
                last_poll = time.monotonic() + .25

            decision = completion_decision(
                manager_active=manager_active,
                monitor_state_id=monitor_state_id,
                valid_scan_count=self.valid_scan_samples,
                startup_response=startup_response,
                startup_timed_out=startup_timed_out,
                deadline_expired=False,
            )
            if decision.startswith('PASS'):
                result = decision
                error = None
                break
            if (startup_timed_out and startup_response is None
                    and time.monotonic()-startup_requested_at >= 3.0
                    and manager_active is not True):
                # The manager still owns an unresolved transition request. Do
                # not retry or issue a competing transition to the node.
                result = 'FAIL_STARTUP_UNRESOLVED'
                break
            rclpy.spin_once(self, timeout_sec=.05)

        if result.startswith('WAIT') or result == 'FAIL_STARTUP_UNRESOLVED':
            final = completion_decision(
                manager_active=manager_active,
                monitor_state_id=monitor_state_id,
                valid_scan_count=self.valid_scan_samples,
                startup_response=startup_response,
                startup_timed_out=startup_timed_out,
                deadline_expired=True,
            )
            if final.startswith('FAIL'):
                result = final
            elif result == 'FAIL_STARTUP_UNRESOLVED':
                result = 'FAIL_STARTUP_UNRESOLVED'
            else:
                result = 'FAIL_TIMEOUT'
            if error is None:
                error = result

        record = {
            'schema_version': 1,
            'started_utc': started_utc,
            'node': self.node_name,
            'manager': self.manager_name,
            'manager_owns_transitions': True,
            'coordinator_called_change_state_directly': False,
            'startup_request_count': sum(1 for item in transitions
                                         if item['kind'] == 'manager_startup_request'),
            'startup_response': startup_response,
            'startup_response_timed_out': startup_timed_out,
            'initial_state_id': (state_queries[0].get('state_id')
                                 if state_queries and 'state_id' in state_queries[0] else None),
            'last_state_id': monitor_state_id,
            'manager_active': manager_active,
            'elapsed_s': round(time.monotonic()-start, 6),
            'discovery_settle_s': discovery_settle_s,
            'service_checks': service_checks,
            'state_observations': state_queries,
            'manager_observations': manager_queries,
            'transitions': transitions,
            'scan': {
                'topic': '/scan', 'count': self.scans,
                'valid_messages': self.valid_scan_messages,
                'valid_samples': self.valid_scan_samples,
                'invalid_samples': self.invalid_scan_samples,
                'frames': dict(self.frames),
                'parameter_sets': [json.loads(value) for value in self.scan_parameters],
            },
            'decision': result,
            'passed': result.startswith('PASS'),
            'error': error,
        }
        return record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node', default='/collision_monitor')
    parser.add_argument('--manager', default='/mapping_collision_lifecycle_manager')
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--discovery-settle', type=float, default=2.0)
    parser.add_argument('--startup-response-timeout', type=float, default=12.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if min(args.timeout, args.startup_response_timeout) <= 0 or args.discovery_settle < 0:
        parser.error('timeouts must be positive and discovery settle nonnegative')
    node = None
    record = None
    try:
        rclpy.init(args=None)
        node = StartupCoordinator(args.node, args.manager)
        record = node.run(
            timeout_s=args.timeout,
            discovery_settle_s=args.discovery_settle,
            startup_response_timeout_s=args.startup_response_timeout,
        )
    except Exception as exc:
        record = {
            'schema_version': 1, 'node': args.node, 'manager': args.manager,
            'manager_owns_transitions': True,
            'coordinator_called_change_state_directly': False,
            'decision': 'FAIL_EXCEPTION', 'passed': False,
            'error': f'{type(exc).__name__}: {exc}',
        }
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({
        'passed': record.get('passed'), 'decision': record.get('decision'),
        'state_id': record.get('last_state_id'),
        'manager_active': record.get('manager_active'),
        'scan_count': record.get('scan', {}).get('count'),
        'error': record.get('error'),
    }, sort_keys=True), flush=True)
    return 0 if record.get('passed') is True else 1


if __name__ == '__main__':
    raise SystemExit(main())
