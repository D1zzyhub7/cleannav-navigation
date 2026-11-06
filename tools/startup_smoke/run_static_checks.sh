#!/usr/bin/env bash
# Static-only validation; this script never starts ROS nodes or Gazebo.
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
NAV_ROOT=$(cd -- "$HERE/../.." && pwd)
cd -- "$NAV_ROOT"
WORKSPACE_ROOT=$NAV_ROOT
while [[ ! -f "$WORKSPACE_ROOT/cleannav-build/locks/build.lock" || ! -f "$WORKSPACE_ROOT/cleannav-build/locks/runtime.lock" ]]; do
  [[ "$WORKSPACE_ROOT" != / ]] || { printf 'CleanNav workspace root not found from Navigation checkout\n' >&2; exit 2; }
  WORKSPACE_ROOT=$(dirname -- "$WORKSPACE_ROOT")
done
export CLEANNAV_WORKSPACE_ROOT="$WORKSPACE_ROOT"
MODEL_PATH="$WORKSPACE_ROOT/../cleannav_demo_assets/osrf_car_demo/car_demo/models"
[[ -d "$MODEL_PATH" ]] || { printf 'external Gazebo model assets are missing: %s\n' "$MODEL_PATH" >&2; exit 2; }
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/humble/setup.bash
command -v ros2 >/dev/null
command -v xacro >/dev/null

TMP_DIR=$(mktemp -d "${TMPDIR:-/tmp}/cleannav-startup-static.XXXXXX")
trap 'rm -rf -- "$TMP_DIR"' EXIT
export PYTHONPYCACHEPREFIX="$TMP_DIR/pycache"

while IFS= read -r script; do bash -n "$script"; done < <(printf '%s\n' "$HERE/startup_smoke.sh" "$HERE/run_static_checks.sh")
python3 -m py_compile \
  "$HERE/startup_supervisor.py" \
  "$HERE/startup_smoke.launch.py" \
  "$HERE/mapping_mode_retry.py" \
  "$HERE/publisher_graph_gate.py" \
  "$HERE/roof_lidar_runtime.py" \
  "$HERE/tests/test_publisher_graph_gate.py" \
  "$HERE/tests/test_startup_smoke_contract.py" \
  "$NAV_ROOT/cleannav_simulation/launch/startup_smoke_simulation.launch.py" \
  "$NAV_ROOT/cleannav_rtabmap/launch/startup_smoke_rtabmap.launch.py"

xacro "$NAV_ROOT/cleannav_simulation/urdf/cleannav_ackermann_startup_smoke.urdf.xacro" \
  "controllers_file:=$NAV_ROOT/cleannav_simulation/config/ackermann_controllers.yaml" \
  cmd_vel_topic:=/cmd_vel sim_lidar_samples:=181 sim_lidar_rate:=8 \
  > "$TMP_DIR/startup_smoke.urdf"
python3 - "$TMP_DIR/startup_smoke.urdf" <<'PY'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
for reference, expected_samples, expected_frame in (
    ("base_scan", "181", "base_scan"),
    ("mapping_lidar_link", "360", "mapping_lidar_link"),
):
    gazebos = root.findall(f"gazebo[@reference='{reference}']")
    assert gazebos, f"missing Gazebo sensor block for {reference}"
    sensor = next((block.find("sensor") for block in gazebos
                   if block.find("sensor") is not None), None)
    assert sensor is not None and sensor.attrib.get("type") == "ray"
    assert sensor.findtext("update_rate") == "8"
    assert sensor.findtext("ray/scan/horizontal/samples") == expected_samples
    plugin = sensor.find("plugin")
    assert plugin is not None
    assert plugin.findtext("frame_name") == expected_frame
    if reference == "base_scan":
        remaps = [value.text for value in sensor.findall("plugin/ros/remapping")]
        assert "~/out:=scan_raw" in remaps and "~/out:=scan" in remaps
    else:
        remaps = [value.text for value in sensor.findall("plugin/ros/remapping")]
        assert "~/out:=/mapping/scan_raw" in remaps
        assert "~/out:=/mapping/scan" in remaps
if root.find("link[@name='base_scan']") is None:
    raise AssertionError("low lidar link missing")
if root.find("link[@name='mapping_lidar_link']") is None:
    raise AssertionError("roof lidar link missing")
PY

PYTHONPATH="$HERE${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m pytest -q -p no:cacheprovider "$HERE/tests"
printf 'STATIC_CHECKS_PASS\n'
