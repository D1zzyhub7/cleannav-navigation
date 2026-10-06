# Navigation chain logging

`batch_headless_cruise.sh` now saves `chain.csv` beside the existing
`motion.csv` for each case. The monitor subscribes to `/scan`,
`/local_costmap/costmap`, `/cleannav/cmd_vel_candidate`,
`/cleannav/safety_supervisor_status`, `/cmd_vel`, `/odom`, and Gazebo
`/model_states`. `chain_summary.json` reports sample counts, missing stages,
frame mismatches, and simulation clock alignment. The existing `motion.csv`
columns remain unchanged for the stop/resume analyzer.

All rows are timestamped on receipt by the **same monitor process**:

- `elapsed_sec` and `monotonic_ns`: one monotonic clock; use these to order
  all events and measure delays within a run.
- `ros_time_ns`: monitor's ROS clock, set to Gazebo simulation time by the
  batch script.
- `source_stamp_ns`: sensor/odometry/costmap header timestamp when present.
- `wall_time_ns`: host wall time for correlation with text logs. It is not a
  substitute for simulation time.

`scan_front_min_m` is the nearest valid laser return within 30 degrees of the
sensor's forward axis. A blank value and `scan_valid_count=0` mean that this
sector has no valid return, not that it is guaranteed obstacle-free.
`scan_total_valid_count` distinguishes a blank front sector from an entirely
empty scan. The scan is in `base_scan`, so its distance is measured from the
laser, 0.50 m ahead of `base_link` in the demo vehicle.

Costmap distances are calculated from `OccupancyGrid` cells with occupancy
exactly 100 (lethal). Lower values can be inflation costs.
`costmap_front_lethal_m` is the nearest lethal cell ahead of the robot
within 0.75 m laterally. It uses the most recently received `/odom` pose only
when the odometry and costmap frame IDs match; `odom_age_sec` records the age
of that pose. The full grid is not saved in this CSV. The costmap topic is
published periodically, so `source_stamp_ns` also helps distinguish an old
map from a fresh observation. `obstacle_world` and `robot_world` coordinates
are Gazebo world coordinates and must not be compared directly with odometry
coordinates.

The first instrumented run on 2026-10-06 is in
`batch-headless-cruise-20261006-133654/fast_person_05`. It captured all seven
stages and aligned 988 of 988 nonzero sensor/costmap header stamps with the
monitor's ROS clock. Its 698 scan samples had **no valid front-sector return**.
The costmap nevertheless reported high occupancy roughly 0.25 m from the
robot. This is evidence that the old scenario geometry must be checked before
ranking navigation parameters: the car starts at world X=-1.875 and the
pedestrian is placed at X=-1.25, only 0.625 m between centers. The vehicle's
front collision extent is 0.45 m and the pedestrian's half extent is 0.325 m,
so their projected collision volumes overlap by about 0.15 m at that pose.

The revised experiment starts the pedestrian at world X=-0.40 and sends the
car toward X=0.50. At the vehicle's initial pose, the front collision boxes
have a 0.70 m gap and the laser has a 0.65 m gap to the pedestrian surface.
The pedestrian remains on the route for 5 seconds after activation. The
scenario log records these values for every case.
