#!/usr/bin/env bash

# Headless stress batch for recovery, dynamic avoidance and cruise mode.
# The launch uses the repository's local obstacle SDFs and an empty Gazebo
# world, so it does not depend on the optional external MCity asset pack.

source /opt/ros/humble/setup.bash
source /home/hyn/cleannav_ws/install/setup.bash

# Every node in this experiment runs inside one WSL instance. Restrict DDS to
# localhost and UDP to avoid FastDDS shared-memory port locks corrupting action
# replies during repeated launches.
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# ROS setup scripts reference optional environment variables. Enable strict
# mode only after both setup files have been sourced.
set -uo pipefail

OUT="/mnt/c/Users/sangy/Documents/ChatGPT/智能清扫小车/batch-headless-cruise-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
BASE="/home/hyn/cleannav_ws/install/cleannav_navigation/share/cleannav_navigation/config/nav2_params_ackermann.yaml"
EMPTY_MAP_YAML="/tmp/cleannav_headless_empty_map.yaml"
python3 "$PWD/experiments/create_headless_empty_map.py" "$EMPTY_MAP_YAML"
printf 'scenario,seed,controller_frequency,vx_max,inflation_radius,movement_time_allowance,person_speed,person_range,obstacle_scan_range,raytrace_scan_range,person_hold_sec,person_direction,person_x,multiple_dynamic,status,error_lines\n' > "$OUT/summary.csv"
# Keep each batch away from domains used by earlier interrupted runs. The
# shell PID changes on every invocation, reducing collisions with stale DDS
# participants while staying within the ROS 2 domain range.
DOMAIN_BASE=$((40 + ($$ % 40)))
ACTIVE_DOMAIN=""

cleanup_group() {
  local pid="$1"
  kill -INT -- -"$pid" 2>/dev/null || true
  sleep 3
  kill -TERM -- -"$pid" 2>/dev/null || true
  sleep 2
  kill -KILL -- -"$pid" 2>/dev/null || true
}

domain_pids() {
  local domain="$1" env_file pid
  for env_file in /proc/[0-9]*/environ; do
    if grep -zqx "ROS_DOMAIN_ID=$domain" "$env_file" 2>/dev/null; then
      pid="${env_file#/proc/}"
      pid="${pid%/environ}"
      printf '%s\n' "$pid"
    fi
  done
}

cleanup_domain() {
  local domain="$1" pid
  while read -r pid; do
    [ -n "$pid" ] && kill -TERM "$pid" 2>/dev/null || true
  done < <(domain_pids "$domain")
  sleep 3
  while read -r pid; do
    [ -n "$pid" ] && kill -KILL "$pid" 2>/dev/null || true
  done < <(domain_pids "$domain")
}

cleanup_active_domain() {
  if [ -n "$ACTIVE_DOMAIN" ]; then
    cleanup_domain "$ACTIVE_DOMAIN"
  fi
}
trap cleanup_active_domain EXIT

wait_lifecycle() {
  local domain="$1" node="$2"
  for _ in $(seq 1 100); do
    if ROS_DOMAIN_ID="$domain" timeout 2s ros2 lifecycle get "$node" 2>/dev/null | grep -q '^active'; then return 0; fi
    sleep 1
  done
  return 1
}

wait_action() {
  local domain="$1"
  for _ in $(seq 1 100); do
    if ROS_DOMAIN_ID="$domain" ros2 action info /cleannav/navigate_to_pose 2>/dev/null | grep -q 'Action servers: 1'; then return 0; fi
    sleep 1
  done
  return 1
}

wait_single_action() {
  local domain="$1" action="$2"
  for _ in $(seq 1 100); do
    if ROS_DOMAIN_ID="$domain" ros2 action info "$action" 2>/dev/null | grep -q 'Action servers: 1'; then return 0; fi
    sleep 1
  done
  return 1
}

wait_node() {
  local domain="$1" node="$2"
  for _ in $(seq 1 120); do
    if ROS_DOMAIN_ID="$domain" ros2 node list --no-daemon --spin-time 1 2>/dev/null \
        | grep -Fxq "$node"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_topic_once() {
  local domain="$1" topic="$2"
  for _ in $(seq 1 120); do
    if ROS_DOMAIN_ID="$domain" timeout 3s ros2 topic echo \
        --no-daemon --spin-time 1 --once "$topic" \
        >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_transform() {
  local domain="$1"
  for _ in $(seq 1 120); do
    # tf2_echo normally exits with timeout status 124 after printing a valid
    # transform. Capture first so pipefail does not turn that into a false
    # readiness failure.
    local tf_output
    tf_output=$(ROS_DOMAIN_ID="$domain" timeout 4s ros2 run tf2_ros tf2_echo map base_link 2>/dev/null || true)
    if printf '%s\n' "$tf_output" | grep -q 'Translation'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

model_exists() {
  local domain="$1" model_name="$2"
  ROS_DOMAIN_ID="$domain" timeout 5s ros2 topic echo \
    --no-daemon --spin-time 1 --once /model_states \
    2>/dev/null | grep -Fq "$model_name"
}

spawn_second_obstacle() {
  local domain="$1" case_dir="$2" attempt
  : > "$case_dir/obstacle2_spawn.log"
  for attempt in 1 2 3; do
    if model_exists "$domain" cleannav_demo_dynamic_obstacle_2; then
      return 0
    fi
    printf 'spawn attempt=%s\n' "$attempt" >> "$case_dir/obstacle2_spawn.log"
    ROS_DOMAIN_ID="$domain" timeout 20s ros2 run gazebo_ros spawn_entity.py \
      -entity cleannav_demo_dynamic_obstacle_2 \
      -file "$PWD/cleannav_simulation/models/cleannav_demo_dynamic_obstacle.sdf" \
      -x 0.10 -y -8.0 -z 0.40 >> "$case_dir/obstacle2_spawn.log" 2>&1 || true
    if grep -q -E 'Successfully spawned entity|already exists' \
        "$case_dir/obstacle2_spawn.log"; then
      return 0
    fi
    sleep 2
  done
  model_exists "$domain" cleannav_demo_dynamic_obstacle_2
}

run_case() {
  local name="$1" seed="$2" frequency="$3" vx="$4" inflation="$5" allowance="$6" person_speed="$7" person_range="$8" obstacle_scan_range="$9" raytrace_scan_range="${10}"
  local pedestrian_hold_sec="${11:-5.0}" pedestrian_direction="${12:-forward}" pedestrian_x="${13:--0.40}" multiple_dynamic="${14:-false}"
  local domain=$((DOMAIN_BASE + seed % 20))
  ACTIVE_DOMAIN="$domain"
  local case_dir="$OUT/$name"
  mkdir -p "$case_dir"
  # Keep the pedestrian's collision box clear of both the car nose and the
  # forward lidar at the instant it enters the route. The previous x=-1.25
  # overlapped the car starting at x=-1.875 (0.45 m nose, 0.325 m half box).
  local goal_x=0.50
  local goal_timeout_sec="${CLEANNAV_GOAL_TIMEOUT_SEC:-90}"

  # An interrupted ros2 launch can orphan grandchildren outside the original
  # process group. Clean only this case's isolated DDS domain.
  cleanup_domain "$domain"

  # Deterministic random static obstacles. The seed is recorded with every run.
  local block_x block_y cylinder_x cylinder_y obstacle_b_y static_enabled
  static_enabled="${CLEANNAV_RANDOM_STATIC:-false}"
  block_x=$(awk -v s="$seed" 'BEGIN{srand(s); printf "%.2f", -0.8 + rand()*2.8}')
  block_y=$(awk -v s="$seed" 'BEGIN{srand(s+17); printf "%.2f", 1.5 + rand()}')
  cylinder_x=$(awk -v s="$seed" 'BEGIN{srand(s+31); printf "%.2f", 0.2 + rand()*2.8}')
  cylinder_y=$(awk -v s="$seed" 'BEGIN{srand(s+47); printf "%.2f", -2.5 + rand()}')
  if [ "$pedestrian_direction" = reverse ]; then
    obstacle_b_y=$(awk -v r="$person_range" 'BEGIN{printf "%.2f", -4.00-r}')
  else
    obstacle_b_y=$(awk -v r="$person_range" 'BEGIN{printf "%.2f", -4.00+r}')
  fi
  printf 'seed=%s static_enabled=%s block=(%s,%s) cylinder=(%s,%s) dynamic_speed=%s range=%s obstacle_scan=%s raytrace_scan=%s pedestrian_x=%s direction=%s multiple_dynamic=%s goal_x=%s initial_hold_sec=%s\n' \
    "$seed" "$static_enabled" "$block_x" "$block_y" "$cylinder_x" "$cylinder_y" "$person_speed" "$person_range" "$obstacle_scan_range" "$raytrace_scan_range" "$pedestrian_x" "$pedestrian_direction" "$multiple_dynamic" "$goal_x" "$pedestrian_hold_sec" > "$case_dir/scenario.log"

  local param="/tmp/cleannav_${name}.yaml"
  cp "$BASE" "$param"
  sed -i "s#/rtabmap/map#/map#g; s/use_sim_time: False/use_sim_time: True/g; s#yaml_filename: \"map.yaml\"#yaml_filename: \"${EMPTY_MAP_YAML}\"#; s/controller_frequency: [0-9.]*/controller_frequency: ${frequency}/; s/vx_max: [0-9.]*/vx_max: ${vx}/; s/inflation_radius: [0-9.]*/inflation_radius: ${inflation}/g; s/movement_time_allowance: [0-9.]*/movement_time_allowance: ${allowance}/" "$param"
  sed -i "s/raytrace_max_range: [0-9.]*/raytrace_max_range: ${raytrace_scan_range}/g; s/obstacle_max_range: [0-9.]*/obstacle_max_range: ${obstacle_scan_range}/g" "$param"
  # A planar LaserScan needs one 2D obstacle layer. Keep the voxel plugin
  # definition available in the base config, but exclude it from this fixed
  # experiment so we can isolate stale marks from duplicate scan layers.
  sed -i 's/plugins: \["obstacle_layer", "voxel_layer", "inflation_layer"\]/plugins: ["obstacle_layer", "inflation_layer"]/' "$param"
  if [ "$static_enabled" = false ]; then
    # The fixed baseline uses a known empty map. Dynamic returns belong in
    # the local costmap; feeding them to the global planner produced a 5.55 m
    # loop for a 2.45 m straight goal before the pedestrian even appeared.
    sed -i 's/plugins: \["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"\]/plugins: ["static_layer", "inflation_layer"]/' "$param"
  fi
  # This experiment verifies the requested stop-then-resume behavior. Keep
  # MPPI forward-only so a temporary pedestrian blockage cannot become an
  # indefinite reverse escape trajectory. Smac retains Reeds-Shepp because
  # the mapped short corridor cannot produce a valid Dubin path.
  sed -i 's/vx_min: -0.15/vx_min: 0.0/' "$param"
  sed -i 's/failure_tolerance: 5.0/failure_tolerance: 15.0/; s/batch_size: 750/batch_size: 2000/; s/vx_std: 0.02/vx_std: 0.15/; s/regenerate_noises: false/regenerate_noises: true/; s/use_path_orientations: true/use_path_orientations: false/' "$param"
  sed -i '/    scan_topic: scan/a\    set_initial_pose: True\n    initial_pose:\n      x: -1.875\n      y: 0.415\n      yaw: 0.0' "$param"
  # The headless world has no static landmarks. Keep AMCL as the map->odom
  # provider but prevent dynamic, unmapped test obstacles from dragging the
  # localization estimate away from the deterministic initial pose.
  sed -i 's/update_min_a: 0.2/update_min_a: 1000.0/; s/update_min_d: 0.25/update_min_d: 1000.0/' "$param"

  setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 launch cleannav_simulation ackermann_mcity_obstacles_demo.launch.py gui:=false world:=/usr/share/gazebo-11/worlds/empty.world model_path:=/usr/share/gazebo-11/models spawn_x:=-1.875 spawn_y:=0.415 enable_static_obstacles:=$static_enabled static_block_x:=$block_x static_block_y:=$block_y static_cylinder_x:=$cylinder_x static_cylinder_y:=$cylinder_y enable_dynamic_obstacle:=true dynamic_obstacle_speed:=$person_speed dynamic_obstacle_a_x:=$pedestrian_x dynamic_obstacle_a_y:=-4.00 dynamic_obstacle_b_x:=$pedestrian_x dynamic_obstacle_b_y:=$obstacle_b_y" > "$case_dir/sim.log" 2>&1 &
  local sim_pid=$!
  setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 launch nav2_bringup localization_launch.py map:=$EMPTY_MAP_YAML params_file:=$param use_sim_time:=True autostart:=True" > "$case_dir/localization.log" 2>&1 &
  local loc_pid=$!
  if ! wait_lifecycle "$domain" /amcl; then
    echo 'amcl not active' > "$case_dir/readiness.log"
    cleanup_group "$loc_pid"; cleanup_group "$sim_pid"
    cleanup_domain "$domain"
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,STARTUP_FAIL,0\n' "$name" "$seed" "$frequency" "$vx" "$inflation" "$allowance" "$person_speed" "$person_range" "$obstacle_scan_range" "$raytrace_scan_range" "$pedestrian_hold_sec" "$pedestrian_direction" "$pedestrian_x" "$multiple_dynamic" >> "$OUT/summary.csv"
    ACTIVE_DOMAIN=""
    return 0
  fi
  ROS_DOMAIN_ID=$domain python3 "$PWD/experiments/set_initial_pose_experiment.py" > "$case_dir/initialpose.log" 2>&1 || true
  if ! wait_for_transform "$domain"; then
    echo 'map->base_link transform did not become available' > "$case_dir/readiness.log"
    cleanup_group "$loc_pid"; cleanup_group "$sim_pid"
    cleanup_domain "$domain"
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,STARTUP_FAIL,0\n' "$name" "$seed" "$frequency" "$vx" "$inflation" "$allowance" "$person_speed" "$person_range" "$obstacle_scan_range" "$raytrace_scan_range" "$pedestrian_hold_sec" "$pedestrian_direction" "$pedestrian_x" "$multiple_dynamic" >> "$OUT/summary.csv"
    ACTIVE_DOMAIN=""
    return 0
  fi
  setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 launch cleannav_navigation cleannav_ackermann_navigation.launch.py params_file:=$param use_sim_time:=true autostart:=true replan_period_sec:=1.0 safety_block_all:=true safety_front_stop_enabled:=true" > "$case_dir/navigation.log" 2>&1 &
  local nav_pid=$!
  local status=STARTUP_FAIL cruise_ok=1
  local readiness_error=""
  if ! wait_lifecycle "$domain" /planner_server; then
    readiness_error='planner_server not active'
  elif ! wait_lifecycle "$domain" /controller_server; then
    readiness_error='controller_server not active'
  elif ! wait_action "$domain"; then
    readiness_error='outer navigation action server count is not one'
  elif ! wait_single_action "$domain" /compute_path_to_pose; then
    readiness_error='planner action server count is not one'
  elif ! wait_single_action "$domain" /follow_path; then
    readiness_error='controller action server count is not one'
  elif ! wait_node "$domain" /cleannav_dynamic_obstacle_controller; then
    readiness_error='dynamic obstacle controller node not discovered'
  elif ! wait_topic_once "$domain" /odom; then
    readiness_error='odom topic produced no message'
  elif ! wait_topic_once "$domain" /scan; then
    readiness_error='scan topic produced no message'
  fi
  if [ -z "$readiness_error" ]; then
    if ! ROS_DOMAIN_ID=$domain python3 \
        "$PWD/experiments/reset_gazebo_vehicle.py" \
        --x -1.875 --y 0.415 --yaw 0.0 \
        > "$case_dir/vehicle_reset.log" 2>&1; then
      readiness_error='vehicle Gazebo pose reset could not be verified'
    else
      ROS_DOMAIN_ID=$domain python3 \
        "$PWD/experiments/set_initial_pose_experiment.py" \
        > "$case_dir/initialpose_after_reset.log" 2>&1 || true
      if ! wait_for_transform "$domain"; then
        readiness_error='map->base_link transform unavailable after vehicle reset'
      fi
    fi
  fi
  if [ -z "$readiness_error" ]; then
    # Restart only the demo obstacle controller immediately before the goal.
    # Launch/readiness time otherwise changes its ping-pong phase, so some
    # runs never put the pedestrian in front of the moving vehicle.
    local old_obstacle_pid obstacle_pid="" obstacle2_pid="" reset_obstacle_pid="" restart_obstacle_b_y
    while read -r old_obstacle_pid; do
      if tr '\0' ' ' < "/proc/$old_obstacle_pid/cmdline" 2>/dev/null \
          | grep -q 'dynamic_obstacle_controller'; then
        kill -TERM "$old_obstacle_pid" 2>/dev/null || true
      fi
    done < <(domain_pids "$domain")
    sleep 1
    setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 run cleannav_simulation dynamic_obstacle_controller --ros-args -p obstacle_name:=cleannav_demo_dynamic_obstacle -p a_x:=-1.10 -p a_y:=-8.0 -p b_x:=-1.10 -p b_y:=-7.0 -p speed:=0.10 -p update_rate:=20.0" > "$case_dir/obstacle_reset.log" 2>&1 &
    reset_obstacle_pid=$!
    if wait_node "$domain" /cleannav_dynamic_obstacle_controller; then
      sleep 1
      cleanup_group "$reset_obstacle_pid"
    else
      readiness_error='temporary obstacle reset controller not discovered'
    fi
    if [ -z "$readiness_error" ] && [ "$multiple_dynamic" = true ]; then
      spawn_second_obstacle "$domain" "$case_dir" || \
        readiness_error='second dynamic obstacle failed to spawn after 3 attempts'
    fi
  fi
  if [ -z "$readiness_error" ]; then
    ROS_DOMAIN_ID=$domain python3 "$PWD/experiments/dynamic_recovery_monitor.py" \
      "$case_dir/motion.csv" "$case_dir/chain.csv" \
      --ros-args -p use_sim_time:=true \
      > "$case_dir/monitor.log" 2>&1 & local monitor_pid=$!
    ROS_DOMAIN_ID=$domain timeout 10s ros2 service call /cleannav/safety/acquire_lease cleannav_interfaces/srv/SafetyLease "{execution_id: cruise_$name}" > "$case_dir/lease.log" 2>&1 || true
    : > "$case_dir/goal.log"
    # This goal crosses the moving obstacle's path and must both succeed and
    # show a real stop/resume sequence in the independent motion log.
    for waypoint in "$goal_x,0.415"; do
      wx="${waypoint%,*}"; wy="${waypoint#*,}"
      ROS_DOMAIN_ID=$domain timeout -k 5s --signal=INT "${goal_timeout_sec}s" ros2 action send_goal /cleannav/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: $wx, y: $wy, z: 0.0}, orientation: {w: 1.0}}}}" --feedback >> "$case_dir/goal.log" 2>&1 &
      local goal_pid=$! motion_started=0
      # Start the pedestrian after the controller has issued a real motion
      # command. Hold it on the route long enough to observe the controller's
      # reaction before the faster pedestrian moves away.
      for _ in $(seq 1 300); do
        if awk -F, '$2 == "cmd" && ($3 > 0.03 || $3 < -0.03) {found=1} END {exit !found}' "$case_dir/motion.csv" 2>/dev/null; then
          motion_started=1
          break
        fi
        sleep 0.1
      done
      if [ "$motion_started" -eq 1 ]; then
        if [ "$pedestrian_direction" = reverse ]; then
          restart_obstacle_b_y=$(awk -v r="$person_range" 'BEGIN{printf "%.2f", 0.415-r}')
        else
          restart_obstacle_b_y=$(awk -v r="$person_range" 'BEGIN{printf "%.2f", 0.415+r}')
        fi
        setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 run cleannav_simulation dynamic_obstacle_controller --ros-args -p obstacle_name:=cleannav_demo_dynamic_obstacle -p a_x:=$pedestrian_x -p a_y:=0.415 -p b_x:=$pedestrian_x -p b_y:=$restart_obstacle_b_y -p speed:=$person_speed -p update_rate:=20.0 -p initial_hold_sec:=$pedestrian_hold_sec -p ping_pong:=false" > "$case_dir/controlled_obstacle.log" 2>&1 &
        obstacle_pid=$!
        if [ "$multiple_dynamic" = true ]; then
          local second_speed second_end_y second_hold
          second_speed=$(awk -v s="$person_speed" 'BEGIN{printf "%.2f", s*0.85}')
          second_end_y=$(awk -v r="$person_range" 'BEGIN{printf "%.2f", 0.415-r}')
          second_hold=$(awk -v h="$pedestrian_hold_sec" 'BEGIN{printf "%.2f", h+2.0}')
          setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 run cleannav_simulation dynamic_obstacle_controller --ros-args -r __node:=cleannav_dynamic_obstacle_controller_2 -p obstacle_name:=cleannav_demo_dynamic_obstacle_2 -p a_x:=0.10 -p a_y:=0.415 -p b_x:=0.10 -p b_y:=$second_end_y -p speed:=$second_speed -p update_rate:=20.0 -p initial_hold_sec:=$second_hold -p ping_pong:=false" > "$case_dir/controlled_obstacle_2.log" 2>&1 &
          obstacle2_pid=$!
        fi
      fi
      wait "$goal_pid"
      goal_rc=$?
      goal_status=$(grep 'Goal finished with status:' "$case_dir/goal.log" | tail -1 | awk '{print $5}')
      # FastDDS can lose the outer action response even though the controller
      # reached and stopped at the goal. Recover only when both the controller
      # log and the final Gazebo world pose independently confirm arrival.
      if [ -z "$goal_status" ] \
          && grep -q 'Reached the goal!' "$case_dir/navigation.log" \
          && awk -F, -v gx="$wx" -v gy="$wy" '
               $6 == "robot_world" {x=$10; y=$11; found=1}
               END {if (!found) exit 1; dx=x-gx; dy=y-gy;
                    exit !((dx*dx + dy*dy) <= 0.1225)}' \
             "$case_dir/chain.csv"; then
        goal_status=SUCCEEDED
        goal_rc=0
        echo "RESULT_RECOVERED_FROM_CONTROLLER_AND_WORLD_POSE" >> "$case_dir/goal.log"
      fi
      local world_goal_ok=0
      if awk -F, -v gx="$wx" -v gy="$wy" '
           $6 == "robot_world" {x=$10; y=$11; found=1}
           END {if (!found) exit 1; dx=x-gx; dy=y-gy;
                exit !((dx*dx + dy*dy) <= 0.1225)}' \
         "$case_dir/chain.csv"; then
        world_goal_ok=1
      fi
      if [ "$motion_started" -ne 1 ] || [ "$goal_rc" -ne 0 ] || [ "$goal_status" != "SUCCEEDED" ]; then
        cruise_ok=0
        echo "WAYPOINT_FAILED,$wx,$wy,status=${goal_status:-NO_RESULT},rc=$goal_rc,motion_started=$motion_started" >> "$case_dir/goal.log"
        break
      fi
      if [ "$world_goal_ok" -ne 1 ]; then
        cruise_ok=0
        echo "WORLD_GOAL_MISS,$wx,$wy,tolerance_m=0.35" >> "$case_dir/goal.log"
        break
      fi
    done
    kill -INT "$monitor_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$monitor_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM "$monitor_pid" 2>/dev/null || true
    sleep 0.2
    kill -KILL "$monitor_pid" 2>/dev/null || true
    wait "$monitor_pid" 2>/dev/null || true
    python3 "$PWD/experiments/analyze_navigation_chain.py" \
      "$case_dir/chain.csv" > "$case_dir/chain_summary.json" 2>&1 || true
    if [ -n "$obstacle_pid" ]; then
      cleanup_group "$obstacle_pid"
    fi
    if [ -n "$obstacle2_pid" ]; then
      cleanup_group "$obstacle2_pid"
    fi
    if python3 "$PWD/experiments/analyze_dynamic_recovery.py" \
        "$case_dir/motion.csv" > "$case_dir/recovery.json" 2>&1; then
      recovery_ok=1
    else
      recovery_ok=0
    fi
    if [ "$cruise_ok" -eq 1 ] && [ "$recovery_ok" -eq 1 ]; then
      status=SUCCEEDED
    elif [ "$cruise_ok" -eq 1 ]; then
      status=NO_STOP_RESUME
    else
      status=CRUISE_FAIL
    fi
  else
    printf '%s\n' "$readiness_error" > "$case_dir/readiness.log"
  fi
  cleanup_group "$nav_pid"; cleanup_group "$loc_pid"; cleanup_group "$sim_pid"
  cleanup_domain "$domain"
  ACTIVE_DOMAIN=""
  local errors
  errors=$(grep -h '\[ERROR\]' "$case_dir"/*.log 2>/dev/null | grep -v -E 'context is invalid|rcl_shutdown already called|process has died' | wc -l | tr -d ' ')
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$name" "$seed" "$frequency" "$vx" "$inflation" "$allowance" "$person_speed" "$person_range" "$obstacle_scan_range" "$raytrace_scan_range" "$pedestrian_hold_sec" "$pedestrian_direction" "$pedestrian_x" "$multiple_dynamic" "$status" "$errors" >> "$OUT/summary.csv"
}

if [ -z "${CLEANNAV_CASE_FILTER:-}" ] || [ "$CLEANNAV_CASE_FILTER" = fast_person_01 ]; then
  run_case fast_person_01 101 20.0 0.50 0.65 60.0 0.60 7.0 2.5 3.0 || true
fi
if [ -z "${CLEANNAV_CASE_FILTER:-}" ] || [ "$CLEANNAV_CASE_FILTER" = fast_person_02 ]; then
  run_case fast_person_02 202 20.0 0.50 0.65 90.0 0.90 9.0 2.5 3.0 || true
fi
if [ -z "${CLEANNAV_CASE_FILTER:-}" ] || [ "$CLEANNAV_CASE_FILTER" = fast_person_03 ]; then
  run_case fast_person_03 303 25.0 0.55 0.60 120.0 1.20 11.0 2.5 3.0 || true
fi
if [ -z "${CLEANNAV_CASE_FILTER:-}" ] || [ "$CLEANNAV_CASE_FILTER" = extended ] || [ "$CLEANNAV_CASE_FILTER" = fast_person_04 ]; then
  run_case fast_person_04 404 20.0 0.55 0.60 120.0 1.50 13.0 3.5 4.0 || true
fi
if [ -z "${CLEANNAV_CASE_FILTER:-}" ] || [ "$CLEANNAV_CASE_FILTER" = extended ] || [ "$CLEANNAV_CASE_FILTER" = fast_person_05 ]; then
  run_case fast_person_05 505 20.0 0.55 0.60 120.0 1.80 15.0 4.0 4.5 || true
fi
if [ -z "${CLEANNAV_CASE_FILTER:-}" ] || [ "$CLEANNAV_CASE_FILTER" = extended ] || [ "$CLEANNAV_CASE_FILTER" = fast_person_06 ]; then
  run_case fast_person_06 606 20.0 0.55 0.60 120.0 2.10 17.0 4.5 5.0 || true
fi
stress_repeats="${CLEANNAV_REPEAT_COUNT:-5}"
if [ "${CLEANNAV_CASE_FILTER:-}" = stress ] || [ "${CLEANNAV_CASE_FILTER:-}" = stress_a ]; then
  for repeat in $(seq 1 "$stress_repeats"); do
    run_case "stress_a_r${repeat}" "$((7100 + repeat))" 20.0 0.55 0.60 120.0 2.40 20.0 5.0 5.5 3.0 forward -0.55 false || true
  done
fi
if [ "${CLEANNAV_CASE_FILTER:-}" = stress ] || [ "${CLEANNAV_CASE_FILTER:-}" = stress_b ]; then
  for repeat in $(seq 1 "$stress_repeats"); do
    run_case "stress_b_r${repeat}" "$((7200 + repeat))" 20.0 0.55 0.60 120.0 2.70 22.0 6.0 6.5 5.0 reverse -0.30 false || true
  done
fi
if [ "${CLEANNAV_CASE_FILTER:-}" = stress ] || [ "${CLEANNAV_CASE_FILTER:-}" = stress_c ]; then
  for repeat in $(seq 1 "$stress_repeats"); do
    run_case "stress_c_r${repeat}" "$((7300 + repeat))" 20.0 0.55 0.60 120.0 3.00 24.0 7.0 7.5 7.0 forward -0.55 true || true
  done
fi
if [ "${CLEANNAV_CASE_FILTER:-}" = stress_a_retry ]; then
  run_case stress_a_retry_7105 7105 20.0 0.55 0.60 120.0 2.40 20.0 5.0 5.5 3.0 forward -0.55 false || true
fi
if [ "${CLEANNAV_CASE_FILTER:-}" = stress_b_retry ]; then
  run_case stress_b_retry_7204 7204 20.0 0.55 0.60 120.0 2.70 22.0 6.0 6.5 7.0 reverse -0.30 false || true
fi
echo "$OUT"
