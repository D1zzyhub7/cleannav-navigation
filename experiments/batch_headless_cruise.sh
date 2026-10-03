#!/usr/bin/env bash
set -euo pipefail

# Headless stress batch for recovery, dynamic avoidance and cruise mode.
# The launch uses the repository's local obstacle SDFs and an empty Gazebo
# world, so it does not depend on the optional external MCity asset pack.

source /opt/ros/humble/setup.bash
source /home/hyn/cleannav_ws/install/setup.bash

OUT="/mnt/c/Users/sangy/Documents/ChatGPT/智能清扫小车/batch-headless-cruise-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
BASE="/home/hyn/cleannav_ws/install/cleannav_navigation/share/cleannav_navigation/config/nav2_params_ackermann.yaml"
printf 'scenario,seed,controller_frequency,vx_max,inflation_radius,movement_time_allowance,person_speed,person_range,status,error_lines\n' > "$OUT/summary.csv"
DOMAIN_BASE=$((140 + ($(date +%s) % 80)))

cleanup_group() {
  local pid="$1"
  kill -INT -- -"$pid" 2>/dev/null || true
  sleep 3
  kill -TERM -- -"$pid" 2>/dev/null || true
  sleep 2
  kill -KILL -- -"$pid" 2>/dev/null || true
}

wait_lifecycle() {
  local domain="$1" node="$2"
  for _ in $(seq 1 100); do
    if ROS_DOMAIN_ID="$domain" timeout 5s ros2 lifecycle get "$node" 2>/dev/null | grep -q '^active'; then return 0; fi
    sleep 1
  done
  return 1
}

wait_action() {
  local domain="$1"
  for _ in $(seq 1 100); do
    if ROS_DOMAIN_ID="$domain" ros2 action info /cleannav/navigate_to_pose 2>/dev/null | grep -q 'Action servers: [1-9]'; then return 0; fi
    sleep 1
  done
  return 1
}

run_case() {
  local name="$1" seed="$2" frequency="$3" vx="$4" inflation="$5" allowance="$6" person_speed="$7" person_range="$8"
  local domain=$((DOMAIN_BASE + seed % 20))
  local case_dir="$OUT/$name"
  mkdir -p "$case_dir"

  # Deterministic random static obstacles. The seed is recorded with every run.
  local block_x block_y cylinder_x cylinder_y obstacle_b_y
  block_x=$(awk -v s="$seed" 'BEGIN{srand(s); printf "%.2f", -0.8 + rand()*2.8}')
  block_y=$(awk -v s="$seed" 'BEGIN{srand(s+17); printf "%.2f", -1.0 + rand()*2.3}')
  cylinder_x=$(awk -v s="$seed" 'BEGIN{srand(s+31); printf "%.2f", 0.2 + rand()*2.8}')
  cylinder_y=$(awk -v s="$seed" 'BEGIN{srand(s+47); printf "%.2f", -1.0 + rand()*2.3}')
  obstacle_b_y=$(awk -v r="$person_range" 'BEGIN{printf "%.2f", 0.415 + r/2}')
  printf 'seed=%s block=(%s,%s) cylinder=(%s,%s) dynamic_speed=%s range=%s\n' \
    "$seed" "$block_x" "$block_y" "$cylinder_x" "$cylinder_y" "$person_speed" "$person_range" > "$case_dir/scenario.log"

  local param="/tmp/cleannav_${name}.yaml"
  cp "$BASE" "$param"
  sed -i "s#/rtabmap/map#/map#g; s/use_sim_time: False/use_sim_time: True/g; s#yaml_filename: \"map.yaml\"#yaml_filename: \"/home/hyn/cleannav_ws/install/cleannav_navigation/share/cleannav_navigation/maps/cleannav_first_map.yaml\"#; s/controller_frequency: [0-9.]*/controller_frequency: ${frequency}/; s/vx_max: [0-9.]*/vx_max: ${vx}/; s/inflation_radius: [0-9.]*/inflation_radius: ${inflation}/g; s/movement_time_allowance: [0-9.]*/movement_time_allowance: ${allowance}/" "$param"

  setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 launch cleannav_simulation ackermann_mcity_obstacles_demo.launch.py gui:=false world:=/usr/share/gazebo-11/worlds/empty.world model_path:=/usr/share/gazebo-11/models spawn_x:=-1.875 spawn_y:=0.415 enable_static_obstacles:=true static_block_x:=$block_x static_block_y:=$block_y static_cylinder_x:=$cylinder_x static_cylinder_y:=$cylinder_y enable_dynamic_obstacle:=true dynamic_obstacle_speed:=$person_speed dynamic_obstacle_a_x:=-1.675 dynamic_obstacle_a_y:=0.415 dynamic_obstacle_b_x:=-1.675 dynamic_obstacle_b_y:=$obstacle_b_y" > "$case_dir/sim.log" 2>&1 &
  local sim_pid=$!
  setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 launch nav2_bringup localization_launch.py map:=/home/hyn/cleannav_ws/install/cleannav_navigation/share/cleannav_navigation/maps/cleannav_first_map.yaml params_file:=$param use_sim_time:=True autostart:=True" > "$case_dir/localization.log" 2>&1 &
  local loc_pid=$!
  wait_lifecycle "$domain" /amcl || echo 'amcl not active' > "$case_dir/readiness.log"
  ROS_DOMAIN_ID=$domain python3 /home/hyn/set_initial_pose_experiment.py > "$case_dir/initialpose.log" 2>&1 || true
  setsid bash -c "export ROS_DOMAIN_ID=$domain; exec ros2 launch cleannav_navigation cleannav_ackermann_navigation.launch.py params_file:=$param use_sim_time:=true autostart:=true replan_period_sec:=1.0 safety_block_all:=true" > "$case_dir/navigation.log" 2>&1 &
  local nav_pid=$!
  local status=STARTUP_FAIL cruise_ok=1
  if wait_lifecycle "$domain" /planner_server && wait_lifecycle "$domain" /controller_server && wait_action "$domain"; then
    sleep 75
    ROS_DOMAIN_ID=$domain timeout 300s ros2 topic echo /cmd_vel > "$case_dir/cmd_vel.log" 2>&1 & local cmd_pid=$!
    ROS_DOMAIN_ID=$domain timeout 300s ros2 topic echo /odom > "$case_dir/odom.log" 2>&1 & local odom_pid=$!
    ROS_DOMAIN_ID=$domain timeout 10s ros2 service call /cleannav/safety/acquire_lease cleannav_interfaces/srv/SafetyLease "{execution_id: cruise_$name}" > "$case_dir/lease.log" 2>&1 || true
    : > "$case_dir/goal.log"
    # Cruise mode: four sequential NavigateToPose legs in a repeatable loop.
    for waypoint in '-1.20,0.415' '-0.20,1.20' '1.20,0.415' '-0.20,-0.35'; do
      wx="${waypoint%,*}"; wy="${waypoint#*,}"
      if ! ROS_DOMAIN_ID=$domain timeout -k 5s --signal=INT 90s ros2 action send_goal /cleannav/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: $wx, y: $wy, z: 0.0}, orientation: {w: 1.0}}}}" --feedback >> "$case_dir/goal.log" 2>&1; then
        cruise_ok=0
        echo "WAYPOINT_FAILED,$wx,$wy" >> "$case_dir/goal.log"
        break
      fi
    done
    kill "$cmd_pid" "$odom_pid" 2>/dev/null || true
    if [ "$cruise_ok" -eq 1 ]; then status=SUCCEEDED; else status=CRUISE_FAIL; fi
  fi
  cleanup_group "$nav_pid"; cleanup_group "$loc_pid"; cleanup_group "$sim_pid"
  local errors
  errors=$(grep -h '\[ERROR\]' "$case_dir"/*.log 2>/dev/null | grep -v -E 'context is invalid|rcl_shutdown already called|process has died' | wc -l | tr -d ' ')
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$name" "$seed" "$frequency" "$vx" "$inflation" "$allowance" "$person_speed" "$person_range" "$status" "$errors" >> "$OUT/summary.csv"
}

run_case fast_person_01 101 20.0 0.50 0.65 60.0 0.60 2.5
run_case fast_person_02 202 20.0 0.50 0.65 90.0 0.90 3.5
run_case fast_person_03 303 25.0 0.55 0.60 120.0 1.20 4.5
echo "$OUT"
