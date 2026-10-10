# PC Simulation V2: managed RViz pose navigation

This PC integration adds a dedicated input path. It does not change TaskCommand V1,
Task 30–33, the MPPI/Smac parameters, or the Safety output policy.

## Contract and ownership

RViz SetGoal publishes `geometry_msgs/PoseStamped` only to
`/cleannav/rviz/goal_pose`. `rviz_goal_adapter` calls
`/cleannav/mission/submit_navigation_goal` (`cleannav_interfaces/srv/SubmitNavigationGoal`).
It never calls a navigation action, requests a lease, or publishes velocity.
Task 40, NAVIGATE_TO_POSE, is allocated from the catalog's reserved range.
Source 5 is dedicated to the pose service; TaskCommand V1 is unchanged and the HMI
subscriber rejects Task 40/source 5.

Mission Manager's existing Core owns command deduplication, execution_id,
generation, pause/resume, cancellation and lease effects. RealNavigationAdapter
submits the frozen map pose to the single Navigation Facade. The Facade runs
ComputePathToPose and FollowPath. MPPI candidates continue through Safety to the
Ackermann controller. Service acceptance means admission, never task success.

The service accepts finite positions and nondegenerate finite quaternions in map
or odom. This is planar navigation: roll/pitch project to yaw, with z retained in
the Nav2 payload. Odom requires an exact-time, planar map transform, at most 0.5
ROS seconds old; no latest-TF fallback. Request stamps must be nonzero, no more
than 0.25 ROS seconds into the future, with validity >0 and <=30 ROS seconds.
The frozen goal survives pause/resume; resume increments generation.

Same command_id and original frame/position/normalized yaw/task/policy/validity
replay the prior result without a new execution; differing semantics conflict.
Active or queued tasks reject new IDs as POSE_BUSY. REPLACE is unsupported.
Fresh readiness and Safety status are mandatory. Motion-disabled, block_all,
ESTOP, missing dependencies and stale observations reject new requests before
registration. Callbacks share the node's mutually exclusive callback group.

Adapter requests expire after five ROS seconds and report a three-wall-second
service availability/response timeout. A sent request is never retried. A response
timeout means acceptance is uncertain; late response and TaskStatus remain
correlated by command_id. The adapter is an input transport, not an authorization
or recovery controller.

## Run the validated isolated overlay

```bash
source /opt/ros/humble/setup.bash
source /tmp/cleannav-step2-1/install/local_setup.bash
source /tmp/cleannav-step3/install/local_setup.bash
ros2 launch cleannav_simulation cleannav_full_sim.launch.py \
  gui:=false start_rviz:=true enable_rviz_goals:=true \
  enable_motion:=true ros_domain_id:=232 gazebo_port:=11463
```

Wait for SYSTEM READY and RVIZ_GOAL SERVICE_READY. Use 2D Goal Pose to click and
drag to specify orientation. The dedicated view is TopDownOrtho. Service response and TaskStatus
are printed in the main terminal. Another click during execution is rejected;
use the existing TaskCommand controls to pause (2), resume (3), stop (4), or ESTOP
(6). ESTOP reset (7) retains its existing confirmation requirements.

Omit enable_motion:=true to inspect the GUI safely: clicks return
POSE_MOTION_DISABLED. Omit enable_rviz_goals:=true to retain the original
visualization-only default with no goal adapter/service. No task auto-starts.

## Rebuild without depending on temporary overlays

Use a new build/install/log directory, source only ROS Humble, and build the eight
navigation simulation packages from the integrated repositories:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/cleannav-step3-fresh/log build \
  --base-paths cleannav-system/src \
  --build-base /tmp/cleannav-step3-fresh/build \
  --install-base /tmp/cleannav-step3-fresh/install \
  --packages-select cleannav_interfaces cleannav_global_planner \
    cleannav_path_executor cleannav_navigation cleannav_safety_supervisor \
    cleannav_simulation cleannav_rtabmap cleannav_mission_manager
source /tmp/cleannav-step3-fresh/install/local_setup.bash
```

The original MCity assets remain required at the full launch's default asset path.
No vehicle verification is implied by PC test results. Runtime evidence is reported
separately from mock UNIT/INTEGRATION evidence; arrival does not mean coverage
cleaning completion.
