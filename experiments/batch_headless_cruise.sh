#!/usr/bin/env bash

# Headless stress batch for recovery, dynamic avoidance and cruise mode.
# The launch uses the repository's local obstacle SDFs and an empty Gazebo
# world, so it does not depend on the optional external MCity asset pack.

source /opt/ros/humble/setup.bash
source /home/hyn/cleannav_ws/install/setup.bash

# ROS setup scripts reference optional environment variables. Enable strict
# mode only after both setup files have been sourced.
set -uo pipefail

OUT="/mnt/c/Users/sangy/Documents/ChatGPT/智能清扫小车/batch-headless-cruise-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
BASE="/home/hyn/cleannav_ws/install/cleannav_navigation/share/cleannav_navigation/config/nav2_params_ackermann.yaml"
EMPTY_MAP_YAML="/tmp/cleannav_headless_empty_map.yaml"
python3 "$PWD/experiments/create_headless_empty_map.py" "$EMPTY_MAP_YAML"
printf 'scenario,seed,controller_frequency,vx_max,inflation_radius,movement_time_allowance,person_speed,person_range,obstacle_scan_range,raytrace_scan_range,status,error_lines\n' > "$OUT/summary.csv"
# Keep each batch away from domains used by earlier interrupted runs. The
# shell PID changes on every invocation, reducing collisions with stale DDS
# participants while staying within the ROS 2 domain range.
DOMAIN_BASE=$((200 + ($$ % 30)))
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

run_case() {
  local name="$1" seed="$2" frequency="$3" vx="$4" inflation="$5" allowance="$6" person_speed="$7" person_range="$8" obstacle_scan_range="$9" raytrace_scan_range="${10}"
  local domain=$((DOMAIN_BASE + seed % 20))
  ACTIVE_DOMAIN="$domain"
  local case_dir="$OUT/$name"
  mkdir -p "$case_dir"
  # Keep the pedestrian's collision box clear of both the car nose and the
  # forward lidar at the instant it enters the route. The previous x=-1.25
  # overlapped the car starting at x=-1.875 (0.45 m nose, 0.325 m half box).
  local pedestrian_x=-0.40 goal_x=0.50 pedestrian_hold_sec=5.0
  local goal_timeout_sec="${CLEANNAV_GOAL_TIMEOUT_SEC:-90}"

  # An interrupted ros2 launch can orphan grandchildren outside the original
  # process group. Clean only this case's isolated DDS domain.
  cleanup_domain "$domain"

  # Deterministic random static obstacles. The seed is recorded with every run.
  local block_x block_y cylinder_x cylinder_y obstacle_b_y
  block_x=$(awk -v s="$seed" 'BEGIN{srand(s); printf "%.2f", -0.8 + rand()*2.8}')
  block_y=$(awk -v s="$seed" 'BEGIN{srand(s+17); printf "%.2f", 1.5 + rand()}')
  cylinder_x=$(awk -v s="$seed" 'BEGIN{srand(s+31); printf "%.2f", 0.2 + rand()*2.8}')
  cylinder_y=$(awk -v s="$seed" 'BEGIN{srand(s+47); printf "%.2f", -2.5 + rand()}')
  obstacle_b_y=$(awk -v r="$person_range" 'BEGIN{printf "%.2f", -4.00 + r}')
  printf 'seed=%s block=(%s,%s) cylinder=(%s,%s) dynamic_speed=%s range=%s obstacle_scan=%s raytrace_scan=%s pedestrian_x=%s goal_x=%s initial_hold_sec=%s initial_front_clearance=0.70m initial_lidar_clearance=0.65m\n' \
    "$seed" "$block_x" "$block_y" "$cylinder_x" "$cylinder_y" "$person_speed" "$person_range" "$obstacle_scan_range" "$raytrace_scan_range" "$pedestrian_x" "$goal_x" "$pedestrian_hold_sec" > "$case_dir/scenario.log"

  local param="/tmp/cleannav_${name}.yaml"
  cp "$BASE" "$param"
  sed -i "s#/rtabmap/map#/map#g; s/use_sim_time: False/use_sim_time: True/g; s#yaml_filename: \"map.yaml\"#yaml_filename: \"${EMPTY_MAP_YAML}\"#; s/controller_frequency: [0-9.]*/controller_frequency: ${frequency}/; s/vx_max: [0-9.]*/vx_max: ${vx}/; s/inflation_radius: [0-9.]*/inflation_radius: ${inflation}/g; s/movement_time_allowance: [0-9.]*/movement_time_allowance: ${allowance}/" "$param"
  sed -i "s/raytrace_max_range: [0-9.]*/raytrace_max_range: ${raytrace_scan_range}/g; s/obstacle_max_range: [0-9.]*/obstacle_max_range: ${obstacle_scan_range}/g" "$param"
  # This experiment verifies the requested stop-then-resume behavior. Keep
  # MPPI forward-only so a temporary pedestrian blockage cannot become an
  # indefinite reverse escape trajectory. Smac retains Reeds-Shepp because
  # the mapped short corridor cannot produce a valid Dubin path.
  sed -i 's/vx_min: -0.15/vx_min: 0.0/' "$param"
  sed -i 's/failure_tolerance: 5.0/failure_tolerance: 15.0/; s/wz_max: 0.60/wz_max: 0.05/; s/cost_weight: 14.0/cost_weight: 40.0/' "$param"
  sed -i '/    scan_topic: scan/a\    set_initial_pose: True\n    initial_pose:\n      x: -1.875\n      y: 0.415\n      yaw: 0.0' "$param"
  # The headless world has no static landmarks. Keep AMCL as the map->odom
  # provider but prevent dynamic, unmapped test obstacles from dragging the
  # localization estimate away from the deterministic initial pose.
  sed -i 's/update_min_a: 0.2/update_min_a: 1000.0/; s/update_min_d: 0.25/update_min_d: 1000.0/' "$param"

  setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 launch cleannav_simulation ackermann_mcity_obstacles_demo.launch.py gui:=false world:=/usr/share/gazebo-11/worlds/empty.world model_path:=/usr/share/gazebo-11/models spawn_x:=-1.875 spawn_y:=0.415 enable_static_obstacles:=true static_block_x:=$block_x static_block_y:=$block_y static_cylinder_x:=$cylinder_x static_cylinder_y:=$cylinder_y enable_dynamic_obstacle:=true dynamic_obstacle_speed:=$person_speed dynamic_obstacle_a_x:=$pedestrian_x dynamic_obstacle_a_y:=-4.00 dynamic_obstacle_b_x:=$pedestrian_x dynamic_obstacle_b_y:=$obstacle_b_y" > "$case_dir/sim.log" 2>&1 &
  local sim_pid=$!
  setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 launch nav2_bringup localization_launch.py map:=$EMPTY_MAP_YAML params_file:=$param use_sim_time:=True autostart:=True" > "$case_dir/localization.log" 2>&1 &
  local loc_pid=$!
  if ! wait_lifecycle "$domain" /amcl; then
    echo 'amcl not active' > "$case_dir/readiness.log"
    cleanup_group "$loc_pid"; cleanup_group "$sim_pid"
    cleanup_domain "$domain"
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,STARTUP_FAIL,0\n' "$name" "$seed" "$frequency" "$vx" "$inflation" "$allowance" "$person_speed" "$person_range" "$obstacle_scan_range" "$raytrace_scan_range" >> "$OUT/summary.csv"
    ACTIVE_DOMAIN=""
    return 0
  fi
  ROS_DOMAIN_ID=$domain python3 "$PWD/experiments/set_initial_pose_experiment.py" > "$case_dir/initialpose.log" 2>&1 || true
  if ! wait_for_transform "$domain"; then
    echo 'map->base_link transform did not become available' > "$case_dir/readiness.log"
    cleanup_group "$loc_pid"; cleanup_group "$sim_pid"
    cleanup_domain "$domain"
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,STARTUP_FAIL,0\n' "$name" "$seed" "$frequency" "$vx" "$inflation" "$allowance" "$person_speed" "$person_range" "$obstacle_scan_range" "$raytrace_scan_range" >> "$OUT/summary.csv"
    ACTIVE_DOMAIN=""
    return 0
  fi
  setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 launch cleannav_navigation cleannav_ackermann_navigation.launch.py params_file:=$param use_sim_time:=true autostart:=true replan_period_sec:=1.0 safety_block_all:=true" > "$case_dir/navigation.log" 2>&1 &
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
    # Restart only the demo obstacle controller immediately before the goal.
    # Launch/readiness time otherwise changes its ping-pong phase, so some
    # runs never put the pedestrian in front of the moving vehicle.
    local old_obstacle_pid obstacle_pid="" reset_obstacle_pid="" restart_obstacle_b_y
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
        restart_obstacle_b_y=$(awk -v r="$person_range" 'BEGIN{printf "%.2f", 0.415 + r}')
        setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 run cleannav_simulation dynamic_obstacle_controller --ros-args -p obstacle_name:=cleannav_demo_dynamic_obstacle -p a_x:=$pedestrian_x -p a_y:=0.415 -p b_x:=$pedestrian_x -p b_y:=$restart_obstacle_b_y -p speed:=$person_speed -p update_rate:=20.0 -p initial_hold_sec:=$pedestrian_hold_sec" > "$case_dir/controlled_obstacle.log" 2>&1 &
        obstacle_pid=$!
      fi
      wait "$goal_pid"
      goal_rc=$?
      goal_status=$(grep 'Goal finished with status:' "$case_dir/goal.log" | tail -1 | awk '{print $5}')
      if [ "$motion_started" -ne 1 ] || [ "$goal_rc" -ne 0 ] || [ "$goal_status" != "SUCCEEDED" ]; then
        cruise_ok=0
        echo "WAYPOINT_FAILED,$wx,$wy,status=${goal_status:-NO_RESULT},rc=$goal_rc,motion_started=$motion_started" >> "$case_dir/goal.log"
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
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$name" "$seed" "$frequency" "$vx" "$inflation" "$allowance" "$person_speed" "$person_range" "$obstacle_scan_range" "$raytrace_scan_range" "$status" "$errors" >> "$OUT/summary.csv"
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
echo "$OUT"
