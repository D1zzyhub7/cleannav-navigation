# Dynamic obstacle stop-chain diagnosis (2026-10-06)

## Scenario correction

`batch_headless_cruise.sh` now places the pedestrian at world X=-0.40 and
sends the car from X=-1.875 toward X=0.50. At the initial pose, the collision
boxes have 0.70 m longitudinal clearance; the forward laser has 0.65 m to
the pedestrian surface. The old pedestrian X=-1.25 overlapped the car's
projected collision box by about 0.15 m. The pedestrian is held on the route
for 5 seconds after activation. `robot_world` and `obstacle_world` in
`chain.csv` let us check the actual Gazebo poses, not just requested spawns.

## Evidence from the corrected scenario

The data directories are siblings of this repository. These are diagnostic
runs, **not successful navigation results**.

| Run | Observation | Result |
| --- | --- | --- |
| `batch-headless-cruise-20261006-135426/fast_person_05` | 58/577 scan samples had valid front returns. At elapsed 7 s the scan read 0.463 m, a lethal costmap cell was 0.534 m ahead, and both candidate and final commands were about 0.002 m/s. At 15 s the pedestrian was at world Y=11.75 and the front scan was empty, but a lethal costmap cell remained about 0.524 m ahead; the candidate and final command were still about 0.001 m/s. | The stop command reached `/cmd_vel`, but the vehicle failed to finish. The controller later reported repeated `Optimizer fail to compute path` and exhausted 20 recovery attempts. |
| `batch-headless-cruise-20261006-140033/fast_person_05` | The actual vehicle pose initially matched the requested world spawn (X about -1.863). While the pedestrian remained at world Y=0.415, the nearest front scan return reached 0.196 m at elapsed 9.47 s. The candidate was about 0.089 m/s and the final command matched. Across elapsed 6–35 s, 554/554 final commands with a candidate no more than 0.15 s old matched it within 0.001 m/s. | This run did **not** issue a stop while the close obstacle was detected; the goal timed out after 35 s. |

The same monitor recorded all stages with the Gazebo clock aligned: 818/818
sensor/costmap stamp checks in the first run and 385/385 in the second. Thus
the observed difference is not explained by mixing wall and simulation time.

## Location of the failure

There is no evidence of a stop command disappearing between
`/cleannav/cmd_vel_candidate`, `safety_supervisor`, and `/cmd_vel`. When MPPI
requests a stop, the supervisor forwards it. When MPPI requests forward
motion near the pedestrian, the supervisor forwards that too. The supervisor
currently handles lease, emergency stop and velocity limits; it has no
independent scan/costmap hazard check. The unsafe no-stop case therefore
starts at the controller decision stage. The stuck-after-stop case also
starts upstream: lethal local-costmap cells can remain after the pedestrian
leaves the laser's forward sector, and the controller then emits near-zero
commands or fails optimization. The present CSV does not identify which
costmap layer retained those cells; that requires layer-specific tracing.

Do not rank navigation parameter sets using these runs as successful dynamic
avoidance. First make the close-obstacle stop deterministic and establish
costmap clearing after the pedestrian departs, then repeat the batch.

## Fixed-scenario resolution

The fixed baseline now disables random static obstacles, keeps dynamic scan
returns in the local costmap, uses an independent front-laser stop gate, and
moves the pedestrian across the route only once. The global plan is a 2.40 m
straight path with start, middle and end yaw all equal to zero.

The original MPPI experiment used `vx_std=0.02` with
`regenerate_noises=false`. It repeatedly selected a biased steering command
and drove away from the straight path. The fixed experiment uses
`vx_std=0.15`, `batch_size=2000`, regenerated noise, and disables path pose
orientation scoring. Run `batch-headless-cruise-20261006-224631` then stopped
at 3.13 s, resumed at 7.64 s and completed the goal. Run
`batch-headless-cruise-20261006-225942` also completed. Run
`batch-headless-cruise-20261006-230300` reached the goal according to the
controller and final world pose, but FastDDS lost the outer action response;
the batch script now restricts DDS to local UDP and independently recovers
this result only when the controller log and Gazebo pose agree.
