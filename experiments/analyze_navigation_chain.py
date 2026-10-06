#!/usr/bin/env python3
"""Check that one run captured every stage of the navigation command chain."""

import csv
import json
import sys
from collections import Counter


REQUIRED_EVENTS = (
    'scan', 'local_costmap', 'cmd_vel_candidate', 'safety_status',
    'cmd_vel', 'odom', 'obstacle_world',
)


def summarize(path):
    counts = Counter()
    first = {}
    last = {}
    invalid_costmaps = 0
    costmap_status_counts = Counter()
    front_return_samples = 0
    no_front_return_samples = 0
    aligned_clock_samples = 0
    mismatched_clock_samples = 0
    with open(path, newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            event = row['event']
            counts[event] += 1
            first.setdefault(event, row['elapsed_sec'])
            last[event] = row['elapsed_sec']
            if event == 'local_costmap':
                costmap_status_counts[row['status']] += 1
                if row['status'] != 'ok':
                    invalid_costmaps += 1
            if event == 'scan':
                if int(row['scan_valid_count']) > 0:
                    front_return_samples += 1
                else:
                    no_front_return_samples += 1
            if event in ('scan', 'local_costmap'):
                ros_time = int(row['ros_time_ns'])
                source_time = int(row['source_stamp_ns'])
                if ros_time > 0 and source_time > 0:
                    if abs(ros_time - source_time) <= 5_000_000_000:
                        aligned_clock_samples += 1
                    else:
                        mismatched_clock_samples += 1
    missing = [event for event in REQUIRED_EVENTS if not counts[event]]
    clock_samples = aligned_clock_samples + mismatched_clock_samples
    return {
        'complete': (
            not missing
            and counts['local_costmap'] > invalid_costmaps
            and aligned_clock_samples > 0
            and aligned_clock_samples / clock_samples >= 0.95
        ),
        'missing_events': missing,
        'invalid_costmap_samples': invalid_costmaps,
        'costmap_status_counts': dict(costmap_status_counts),
        'front_return_scan_samples': front_return_samples,
        'no_front_return_scan_samples': no_front_return_samples,
        'aligned_clock_samples': aligned_clock_samples,
        'mismatched_clock_samples': mismatched_clock_samples,
        'event_counts': {event: counts[event] for event in REQUIRED_EVENTS},
        'first_elapsed_sec': first,
        'last_elapsed_sec': last,
    }


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: analyze_navigation_chain.py CHAIN.csv')
    result = summarize(sys.argv[1])
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['complete'] else 1)


if __name__ == '__main__':
    main()
