#!/usr/bin/env bash
# Sole public caller for a source-bound, supervised RViz navigation session.
# Run as a child process; do not use `exec` from an interactive shell.
set -eo pipefail
ENTRY_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
NAV_ROOT=$(cd -- "$ENTRY_DIR/../.." && pwd)
WORKSPACE_ROOT=$NAV_ROOT
while [[ ! -f "$WORKSPACE_ROOT/cleannav-build/locks/build.lock" || ! -f "$WORKSPACE_ROOT/cleannav-build/locks/runtime.lock" ]]; do
  [[ "$WORKSPACE_ROOT" != / ]] || { printf 'CleanNav workspace root not found from Navigation worktree\n' >&2; exit 2; }
  WORKSPACE_ROOT=$(dirname -- "$WORKSPACE_ROOT")
done

MAP_DIRECTORY=
WORLD_FILE=
MAP_PROFILE=
TIMING_PROFILE=standard
MOTION_PROFILE=none
DRY_RUN=false
ODOMETRY_MODE=wheel
ODOMETRY_MODE_SET=false
while (($#)); do
  case "$1" in
    --timing-profile)
      (($# >= 2)) || { printf "%s\n" "--timing-profile needs a value" >&2; exit 2; }
      TIMING_PROFILE=$2; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    --patrol)
      [[ "$MOTION_PROFILE" == none ]] || { printf '%s\n' 'conflicting motion selection' >&2; exit 2; }
      MOTION_PROFILE=patrol; shift ;;
    --enable-motion)
      [[ "$MOTION_PROFILE" == none ]] || { printf '%s\n' 'duplicate or conflicting motion selection' >&2; exit 2; }
      MOTION_PROFILE=historical; shift ;;
    --motion-profile)
      (($# >= 2)) || { printf '%s\n' '--motion-profile needs a value' >&2; exit 2; }
      [[ "$MOTION_PROFILE" == none ]] || { printf '%s\n' 'duplicate --motion-profile' >&2; exit 2; }
      MOTION_PROFILE=$2; shift 2 ;;
    --odometry-mode)
      (($# >= 2)) || { printf '%s\n' '--odometry-mode needs a value' >&2; exit 2; }
      [[ "$ODOMETRY_MODE_SET" == false ]] || { printf '%s\n' 'duplicate --odometry-mode' >&2; exit 2; }
      ODOMETRY_MODE=$2; ODOMETRY_MODE_SET=true; shift 2 ;;
    --map-profile)
      (($# >= 2)) || { printf '%s\n' '--map-profile needs a profile ID' >&2; exit 2; }
      [[ -z "$MAP_PROFILE" ]] || { printf '%s\n' 'duplicate --map-profile' >&2; exit 2; }
      MAP_PROFILE=$2; shift 2 ;;
    --map-directory)
      (($# >= 2)) || { printf '%s\n' '--map-directory needs a path' >&2; exit 2; }
      [[ -z "$MAP_DIRECTORY" ]] || { printf '%s\n' 'duplicate --map-directory' >&2; exit 2; }
      MAP_DIRECTORY=$2; shift 2 ;;
    --world)
      (($# >= 2)) || { printf '%s\n' '--world needs a path' >&2; exit 2; }
      [[ -z "$WORLD_FILE" ]] || { printf '%s\n' 'duplicate --world' >&2; exit 2; }
      WORLD_FILE=$2; shift 2 ;;
    *) printf 'unsupported argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done
[[ -n "$MAP_PROFILE" && -n "$MAP_DIRECTORY" && -n "$WORLD_FILE" ]] || {
  printf '%s\n' 'usage: run_rviz_navigation.sh [--odometry-mode wheel|oracle] --map-profile roof-lidar-360|mcity-localization07|mcity-localization07-canonical-debug|simple-four-wall-nohit-v1|simple-four-wall-nohit-v1-oracle-short-goal --map-directory ABSOLUTE_MAP_DIR --world ABSOLUTE_WORLD_FILE' >&2
  exit 2
}
case "$ODOMETRY_MODE" in wheel|oracle) ;; *)
  printf 'unsupported odometry mode: %s\n' "$ODOMETRY_MODE" >&2; exit 2;; esac
case "$MAP_PROFILE" in roof-lidar-360|mcity-localization07|mcity-localization07-canonical-debug|patrol-a-v4|simple-four-wall-nohit-v1|simple-four-wall-nohit-v1-oracle-short-goal) ;; *)
  printf 'unsupported map profile: %s\n' "$MAP_PROFILE" >&2; exit 2;; esac

case "$MOTION_PROFILE" in none|historical|patrol) ;;
  short-goal) [[ "$MAP_PROFILE" == simple-four-wall-nohit-v1-oracle-short-goal && "$ODOMETRY_MODE" == oracle ]] || { printf '%s\n' 'short-goal requires the reviewed simple-map Oracle-AMCL profile' >&2; exit 2; };; *)
  printf 'unsupported motion profile: %s\n' "$MOTION_PROFILE" >&2; exit 2;; esac
case "$TIMING_PROFILE" in standard|localization-diagnostic) ;; *)
  printf 'unsupported timing profile: %s\n' "$TIMING_PROFILE" >&2; exit 2;; esac
if [[ "$MOTION_PROFILE" == short-goal ]]; then
  TIMING_PROFILE=short-goal-motion
elif [[ "$MOTION_PROFILE" == historical || "$MOTION_PROFILE" == patrol ]]; then
  TIMING_PROFILE=historical-motion
fi
if [[ "$TIMING_PROFILE" == localization-diagnostic && "$MAP_PROFILE" != mcity-localization07-canonical-debug ]]; then
  printf 'localization diagnostic requires the canonical localization07 profile\n' >&2; exit 2
fi
export CLEANNAV_RVIZ_TIMING_PROFILE=$TIMING_PROFILE
export CLEANNAV_RVIZ_MOTION_PROFILE=$MOTION_PROFILE
if [[ "$MOTION_PROFILE" == patrol && ( "$MAP_PROFILE" != patrol-a-v4 || "$ODOMETRY_MODE" != wheel ) ]]; then
  printf '%s\n' 'patrol requires patrol-a-v4 with formal wheel odometry' >&2; exit 2
fi
AMCL_SETUP=
if [[ "$MAP_PROFILE" == simple-four-wall-nohit-v1-oracle-short-goal ]]; then
  [[ "$ODOMETRY_MODE" == oracle && ( "$MOTION_PROFILE" == none || "$MOTION_PROFILE" == short-goal || "$MOTION_PROFILE" == historical ) ]] || { printf '%s\n' 'Oracle-AMCL simple map supports only none, short-goal, or historical motion' >&2; exit 2; }
  AMCL_SETUP=$(PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$ENTRY_DIR" python3 - "$NAV_ROOT/tools/startup_smoke/amcl_private_buffer_diagnostic_profile.json" <<'PYSETUP'
import json,sys
from pathlib import Path
import integration
integration.verify_source_manifest()
profile=Path(sys.argv[1])
contract=json.loads(integration.contract_path_for_profile('simple-four-wall-nohit-v1-oracle-short-goal').read_text())
if integration.digest(profile)!=contract['diagnostic_profile_sha256']:
    raise ValueError('AMCL setup profile identity mismatch')
print(json.loads(profile.read_text())['selection']['startup_overlay_setup'])
PYSETUP
  )
fi
if [[ "$DRY_RUN" == true ]]; then
  if [[ -n "$AMCL_SETUP" ]]; then
    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
    source "$AMCL_SETUP" >&2
  fi
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$ENTRY_DIR${PYTHONPATH:+:$PYTHONPATH}" python3 - "$MAP_PROFILE" "$MAP_DIRECTORY" "$WORLD_FILE" "$MOTION_PROFILE" "$ODOMETRY_MODE" <<'PYDRY'
import json,sys
from pathlib import Path
import integration, navigation_supervisor as supervisor
from supervisor_policy import MAX_LAUNCH_RUNTIME_S
integration.verify_source_manifest()
selected=integration.verify_map(sys.argv[2],sys.argv[3],profile=sys.argv[1],odometry_mode=sys.argv[5])
if selected.get('localization_runtime'):
    import overlay_identity
    overlay_identity.verify_environment(__import__('os').environ['CLEANNAV_RVIZ_MACHINE_CONFIG'])
    integration.verify_amcl_navigation_runtime(selected)
print(json.dumps(dict(dynamic_entry_calls=0,argv=supervisor.bounded_launch_argv(world=Path(sys.argv[3]),map_directory=Path(sys.argv[2]),map_profile=sys.argv[1],output_dir=Path('/not-created'),runtime_timeout_s=MAX_LAUNCH_RUNTIME_S,motion_profile=sys.argv[4],odometry_mode=sys.argv[5])),indent=2))
PYDRY
  exit $?
fi
# Pure validation before Run ID/output creation; shared by supervisor and watchdog.
if ! CLEANNAV_RVIZ_GUI_GOAL_WAIT_S=$(PYTHONPATH="$ENTRY_DIR" python3 -c \
  'import supervisor_policy as p; print(p.HISTORICAL_GUI_GOAL_WAIT_S)'); then
  printf '%s\n' 'invalid CLEANNAV_RVIZ_GUI_GOAL_WAIT_S' >&2
  exit 2
fi
export CLEANNAV_RVIZ_GUI_GOAL_WAIT_S
RUN_ID="rviz-nav-$(date -u +%Y%m%dT%H%M%SZ)-$(python3 -c 'import uuid; print(uuid.uuid4().hex[:8])')"
T0_NS=$(python3 -c 'import time; print(time.monotonic_ns())')
T0_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
OUTPUT_ROOT_REQUESTED=${CLEANNAV_RVIZ_OUTPUT_ROOT:-$WORKSPACE_ROOT/cleannav-deliverables/rviz-navigation-runs}
DELIVERABLES_ROOT=$(realpath -m -- "$WORKSPACE_ROOT/cleannav-deliverables")
OUTPUT_ROOT=$(realpath -m -- "$OUTPUT_ROOT_REQUESTED")
case "$OUTPUT_ROOT/" in
  "$DELIVERABLES_ROOT"/*/) ;;
  *) printf 'RViz run output must stay below %s\n' "$DELIVERABLES_ROOT" >&2; exit 2 ;;
esac
mkdir -p -- "$OUTPUT_ROOT"
OUTPUT_ROOT=$(cd -- "$OUTPUT_ROOT" && pwd -P)
export CLEANNAV_RVIZ_OUTPUT_ROOT=$OUTPUT_ROOT
RUN_DIR=$OUTPUT_ROOT/$RUN_ID
mkdir -- "$RUN_DIR"
export CLEANNAV_WORKSPACE_ROOT=$WORKSPACE_ROOT
export CLEANNAV_RVIZ_RUN_DIR=$RUN_DIR

OVERLAY_SETUP=${AMCL_SETUP:-${CLEANNAV_RVIZ_OVERLAY_SETUP:-}}
write_wrapper_record() {
  python3 - "$RUN_DIR/wrapper.json" "$RUN_ID" "$T0_NS" "$NAV_ROOT" \
    "$WORKSPACE_ROOT" "$MAP_PROFILE" "$MAP_DIRECTORY" "$WORLD_FILE" "$OVERLAY_SETUP" "$T0_UTC" \
    "$MOTION_PROFILE" "$ODOMETRY_MODE" \
    "${ROS_DOMAIN_ID:-<unset>}" "${ROS_LOCALHOST_ONLY:-<unset>}" <<'PY'
import json,sys
from pathlib import Path
path=Path(sys.argv[1])
path.write_text(json.dumps({
  'run_id':sys.argv[2], 't0_monotonic_ns':int(sys.argv[3]),
  'timing_profile':__import__('os').environ['CLEANNAV_RVIZ_TIMING_PROFILE'],
  'navigation_worktree':sys.argv[4], 'workspace_root':sys.argv[5],
  'map_profile':sys.argv[6], 'map_directory_argument':sys.argv[7], 'world_argument':sys.argv[8],
  'overlay_setup_argument':sys.argv[9], 't0_utc':sys.argv[10],
  'motion_profile':sys.argv[11], 'odometry_mode':sys.argv[12],
  'gui_goal_wait_s':float(__import__('os').environ['CLEANNAV_RVIZ_GUI_GOAL_WAIT_S']),
  'domain_before_wrapper_default':sys.argv[13],
  'localhost_only_before_wrapper_default':sys.argv[14],
},indent=2)+'\n')
PY
}
record_setup_failure() {
  local reason=$1
  set +e
  python3 "$ENTRY_DIR/navigation_supervisor.py" \
    --record-wrapper-failure --run-id "$RUN_ID" --run-dir "$RUN_DIR" \
    --t0-ns "$T0_NS" --reason "$reason" >/dev/null 2>&1
  local receipt_rc=$?
  set -e
  if ((receipt_rc != 0)); then
    printf 'setup failed and failure receipt could not be written; run_dir=%s\n' "$RUN_DIR" >&2
  fi
  printf 'rviz_navigation run_id=%s run_dir=%s setup_exit_code=2\n' "$RUN_ID" "$RUN_DIR" >&2
  exit 2
}

write_wrapper_record
[[ -n "$OVERLAY_SETUP" && -f "$OVERLAY_SETUP" ]] || record_setup_failure 'CLEANNAV_RVIZ_OVERLAY_SETUP must point to the reviewed overlay setup file'
OVERLAY_SETUP=$(realpath -- "$OVERLAY_SETUP")
OVERLAY_BEFORE_SHA=$(python3 -c 'import hashlib,sys; print(hashlib.sha256(open(sys.argv[1],"rb").read()).hexdigest())' "$OVERLAY_SETUP")
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
set +e
source /opt/ros/humble/setup.bash >>"$RUN_DIR/environment-setup.log" 2>&1
setup_rc=$?
set -e
((setup_rc == 0)) || record_setup_failure "ROS Humble setup exited $setup_rc"
command -v ros2 >>"$RUN_DIR/environment-setup.log" 2>&1 || record_setup_failure 'ros2 is unavailable after ROS Humble setup'
command -v xacro >>"$RUN_DIR/environment-setup.log" 2>&1 || record_setup_failure 'xacro is unavailable after ROS Humble setup'
set +e
source "$OVERLAY_SETUP" >>"$RUN_DIR/environment-setup.log" 2>&1
setup_rc=$?
set -e
((setup_rc == 0)) || record_setup_failure "approved overlay setup exited $setup_rc"
[[ "${CLEANNAV_RVIZ_TIMING_PROFILE:-}" == "$TIMING_PROFILE" ]] || record_setup_failure 'overlay changed the explicit timing profile'
OVERLAY_AFTER_SHA=$(python3 -c 'import hashlib,sys; print(hashlib.sha256(open(sys.argv[1],"rb").read()).hexdigest())' "$OVERLAY_SETUP")
[[ "$OVERLAY_BEFORE_SHA" == "$OVERLAY_AFTER_SHA" ]] || record_setup_failure 'overlay setup changed while being sourced'
[[ -z "${ROS_DISTRO:-}" || "$ROS_DISTRO" == humble ]] || record_setup_failure "ROS_DISTRO mismatch after setup: ${ROS_DISTRO}"
[[ -z "${ROS_DOMAIN_ID:-}" || "$ROS_DOMAIN_ID" == 204 ]] || record_setup_failure "ROS_DOMAIN_ID conflict after setup: ${ROS_DOMAIN_ID}"
[[ -z "${ROS_LOCALHOST_ONLY:-}" || "$ROS_LOCALHOST_ONLY" == 1 ]] || record_setup_failure "ROS_LOCALHOST_ONLY conflict after setup: ${ROS_LOCALHOST_ONLY}"
export ROS_DOMAIN_ID=204 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11704 GAZEBO_IP=127.0.0.1 GAZEBO_MODEL_DATABASE_URI=''
export PYTHONDONTWRITEBYTECODE=1

set +e
WRAPPER_START_TICKS=$(python3 -c 'import pathlib,sys; s=(pathlib.Path("/proc")/sys.argv[1]/"stat").read_text(); e=s.rfind(")"); print(int(s[e+2:].split()[19]))' "$$")
identity_rc=$?
set -e
((identity_rc == 0)) || record_setup_failure 'could not read wrapper PID/start-time identity for watchdog'
python3 "$ENTRY_DIR/deadline_watchdog.py" \
  --run-dir "$RUN_DIR" --run-id "$RUN_ID" --t0-ns "$T0_NS" \
  --wrapper-pid "$$" --wrapper-start-ticks "$WRAPPER_START_TICKS" \
  >"$RUN_DIR/watchdog.log" 2>&1 &
watchdog_pid=$!
watchdog_ready=false
for _ in {1..200}; do
  if [[ -f "$RUN_DIR/watchdog_ready.json" ]]; then
    watchdog_ready=true
    break
  fi
  if ! kill -0 "$watchdog_pid" 2>/dev/null; then
    break
  fi
  sleep 0.01
done
if [[ "$watchdog_ready" != true ]]; then
  kill -TERM "$watchdog_pid" 2>/dev/null || true
  wait "$watchdog_pid" 2>/dev/null || true
  record_setup_failure 'independent deadline watchdog did not become ready'
fi

set +e
python3 "$ENTRY_DIR/navigation_supervisor.py" \
  --run-id "$RUN_ID" --run-dir "$RUN_DIR" --t0-ns "$T0_NS" \
  --map-profile "$MAP_PROFILE" --map-directory "$MAP_DIRECTORY" --world "$WORLD_FILE" \
  --overlay-setup "$OVERLAY_SETUP" \
  --overlay-before-sha256 "$OVERLAY_BEFORE_SHA" \
  --overlay-after-sha256 "$OVERLAY_AFTER_SHA" \
  --motion-profile "$MOTION_PROFILE" \
  --odometry-mode "$ODOMETRY_MODE" \
  >"$RUN_DIR/supervisor.log" 2>&1 &
supervisor_pid=$!
python3 - "$RUN_DIR/supervisor_state.json" "$RUN_ID" "$T0_NS" "$supervisor_pid" <<'PY'
import json,sys,time
from pathlib import Path
path=Path(sys.argv[1]); run_id=sys.argv[2]; t0=int(sys.argv[3]); pid=int(sys.argv[4])
deadline=time.monotonic()+1.0
start_ticks=None
while time.monotonic()<deadline:
    try:
        stat=(Path('/proc')/str(pid)/'stat').read_text()
        end=stat.rfind(')'); start_ticks=int(stat[end+2:].split()[19]); break
    except (OSError,ValueError,IndexError):
        time.sleep(0.01)
if start_ticks is None:
    raise SystemExit('could not bind supervisor PID/start-time identity')
record={'run_id':run_id,'timing_profile':__import__('os').environ['CLEANNAV_RVIZ_TIMING_PROFILE'],'supervisor_pid':pid,'supervisor_start_ticks':start_ticks,
        'launch_pid':None,'launch_pgid':None,'launch_start_ticks':None,
        'readiness_all_passed':False,'readiness_phases':{},'t0_monotonic_ns':t0}
tmp=path.with_suffix(path.suffix+'.wrapper.tmp')
tmp.write_text(json.dumps(record,indent=2)+'\n'); tmp.replace(path)
PY
binding_rc=$?
if ((binding_rc != 0)); then
  kill -KILL "$supervisor_pid" 2>/dev/null || true
  wait "$supervisor_pid" 2>/dev/null || true
  kill -TERM "$watchdog_pid" 2>/dev/null || true
  wait "$watchdog_pid" 2>/dev/null || true
  record_setup_failure 'could not bind supervisor PID/start-time identity for watchdog'
fi
rc=125
watchdog_rc=125
remaining_pids=("$supervisor_pid" "$watchdog_pid")
while ((${#remaining_pids[@]})); do
  completed_pid=
  wait -n -p completed_pid "${remaining_pids[@]}"
  child_rc=$?
  if [[ "$completed_pid" == "$supervisor_pid" ]]; then
    rc=$child_rc
  elif [[ "$completed_pid" == "$watchdog_pid" ]]; then
    watchdog_rc=$child_rc
  else
    rc=125
    watchdog_rc=125
    break
  fi
  next_pids=()
  for child_pid in "${remaining_pids[@]}"; do
    [[ "$child_pid" == "$completed_pid" ]] || next_pids+=("$child_pid")
  done
  remaining_pids=("${next_pids[@]}")
  if [[ "$completed_pid" == "$watchdog_pid" && "$watchdog_rc" != 0 ]]; then
    # The helper records its failure before returning. Stop the other child so
    # the wrapper cannot hang forever if the helper itself failed.
    python3 - "$RUN_DIR" "$supervisor_pid" <<'PY'
import json,os,signal,sys
from pathlib import Path
run_dir=Path(sys.argv[1]); supervisor_pid=int(sys.argv[2])
try:
    state=json.loads((run_dir/'supervisor_state.json').read_text())
except Exception:
    state={}
try:
    launch_pid=state.get('launch_pid')
    if isinstance(launch_pid,int):
        stat=(Path('/proc')/str(launch_pid)/'stat').read_text()
        end=stat.rfind(')'); fields=stat[end+2:].split()
        if int(fields[19])==state.get('launch_start_ticks') and int(fields[2])==launch_pid:
            os.killpg(launch_pid,signal.SIGKILL)
except Exception:
    pass
try:
    os.kill(supervisor_pid,signal.SIGKILL)
except ProcessLookupError:
    pass
PY
  fi
done
set -e
supervisor_rc=$rc
if [[ -f "$RUN_DIR/hard_deadline.json" || -f "$RUN_DIR/watchdog_failure.json" ]]; then
  set +e
  python3 "$ENTRY_DIR/navigation_supervisor.py" \
    --record-watchdog-failure --run-id "$RUN_ID" --run-dir "$RUN_DIR" \
    --t0-ns "$T0_NS"
  receipt_rc=$?
  set -e
  if ((receipt_rc != 124)); then
    printf 'watchdog fired but terminal failure receipt could not be finalized; run_dir=%s finalizer_rc=%s\n' \
      "$RUN_DIR" "$receipt_rc" >&2
    rc=125
  else
    rc=124
  fi
else
  set +e
  python3 "$ENTRY_DIR/navigation_supervisor.py" \
    --record-watchdog-exit --run-id "$RUN_ID" --run-dir "$RUN_DIR" \
    --t0-ns "$T0_NS" --watchdog-pid "$watchdog_pid" \
    --watchdog-exit-code "$watchdog_rc" --supervisor-exit-code "$rc"
  receipt_rc=$?
  set -e
  if ((receipt_rc != 0)); then
    printf 'watchdog exit receipt could not be finalized; run_dir=%s finalizer_rc=%s\n' \
      "$RUN_DIR" "$receipt_rc" >&2
    rc=125
  fi
fi
printf 'watchdog_pid=%s watchdog_exit_code=%s supervisor_pid=%s supervisor_exit_code=%s\n' \
  "$watchdog_pid" "$watchdog_rc" "$supervisor_pid" "$supervisor_rc"
printf 'rviz_navigation run_id=%s run_dir=%s exit_code=%s\n' "$RUN_ID" "$RUN_DIR" "$rc"
exit "$rc"
