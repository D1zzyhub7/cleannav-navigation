# Dynamic obstacle stress batch — 2026-10-05

## Configuration

- Headless Gazebo, AMCL, map server, project navigation launch.
- MPPI controller frequency: 20 Hz for all extended cases.
- Vehicle `vx_max`: 0.55 m/s.
- Dynamic obstacle speeds: 1.50, 1.80, and 2.10 m/s.
- Dynamic obstacle path ranges: 13, 15, and 17 m.
- Costmap obstacle/raytrace ranges: 3.5/4.0, 4.0/4.5, and 4.5/5.0 m.
- Random static obstacle seeds: 404, 505, and 606.

## Complete three-case run

Output directory:

`C:\Users\sangy\Documents\ChatGPT\智能清扫小车\batch-headless-cruise-20261005-011558`

| Case | Pedestrian speed | Path range | Navigation | Stop/resume | Result |
| --- | ---: | ---: | --- | --- | --- |
| fast_person_04 | 1.50 m/s | 13 m | Timed out and canceled | Stop 49.236 s, release 50.840 s, resumed 51.440 s | CRUISE_FAIL |
| fast_person_05 | 1.80 m/s | 15 m | Succeeded | No measured stop/resume | NO_STOP_RESUME |
| fast_person_06 | 2.10 m/s | 17 m | Not started; AMCL did not become active | Not measured | STARTUP_FAIL |

`fast_person_04` traveled 4.3477 m for a goal less than 1 m away. It recovered its velocity after stopping, but MPPI did not return to the short goal before the 90 s timeout.

`fast_person_05` reached the goal with 0.7167 m odometry displacement. The real collision obstacle occupied the path, but the command stream had no sustained zero-speed interval before goal completion.

## Isolated highest-stress run

Output directory:

`C:\Users\sangy\Documents\ChatGPT\智能清扫小车\batch-headless-cruise-20261005-005659`

The isolated 2.10 m/s case succeeded with a two-second initial obstacle hold:

- first motion: 8.633 s
- stop: 11.740 s
- obstacle release/end of stop: 33.171 s
- resumed command: 33.471 s
- goal status: `SUCCEEDED`
- displacement: 0.5609 m
- MPPI optimizer failures during blockage: 8

This proves the retry path can close the loop, but the complete batch shows that it is not yet repeatable.

## Additional controlled check

Output directory:

`C:\Users\sangy\Documents\ChatGPT\智能清扫小车\batch-headless-cruise-20261005-012550`

The 1.80 m/s case was repeated with a five-second initial obstacle hold. The goal succeeded, but `/cmd_vel` still had no sustained stop interval. The obstacle controller log and Gazebo pose samples confirm that the collision model was at `(-1.25, 0.415)` while the vehicle approached.

## Conclusion

The expanded batch does not establish reliable dynamic avoidance. Two separate defects remain visible:

1. A detected blockage can produce MPPI optimizer failures and recovery, but the recovered trajectory may continue away from the short goal until timeout.
2. In other repetitions, the collision obstacle occupies the route without the safety output producing a measurable zero-speed command.

The next investigation should trace one timestamp across `/scan`, the local costmap, raw controller velocity, the safety supervisor input/output, and final `/cmd_vel`. The batch should not be used for parameter ranking until that command-chain defect is fixed.
