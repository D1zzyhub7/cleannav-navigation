#!/usr/bin/env bash
# Only syntax, real LaunchDescription construction and pure policy tests.
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
NAV=$(cd -- "$HERE/../.." && pwd)
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$HERE:$NAV/tools/startup_smoke${PYTHONPATH:+:$PYTHONPATH}"
export ROS_LOG_DIR="${CLEANNAV_STATIC_LOG_ROOT:-$(mktemp -d /tmp/cleannav-rviz-static-log.XXXXXX)}"
bash -n "$HERE/run_static_checks.sh"
bash -n "$HERE/run_rviz_navigation.sh"
python3 - "$HERE" <<'PY'
from pathlib import Path
import ast,sys,xml.etree.ElementTree as ET,yaml
root=Path(sys.argv[1]);repo=root.parents[1]
for p in root.rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
for p in root.glob('*.yaml'):yaml.safe_load(p.read_text())
yaml.safe_load((root/'navigation.rviz').read_text())
ET.parse(repo/'cleannav_simulation/urdf/cleannav_ackermann_startup_smoke.urdf.xacro')
print('SYNTAX_CHECKS_PASS')
PY
if [[ -z "${CLEANNAV_STATIC_PREFIXES_JSON:-}" || ! -f "$CLEANNAV_STATIC_PREFIXES_JSON" ]]; then
  echo 'CLEANNAV_STATIC_PREFIXES_JSON must identify the audited installed prefixes' >&2
  exit 2
fi
if [[ -z "${CLEANNAV_STATIC_MAP_PROFILE:-}" || -z "${CLEANNAV_STATIC_MAP_DIRECTORY:-}" \
      || -z "${CLEANNAV_STATIC_WORLD_FILE:-}" ]]; then
  echo 'set CLEANNAV_STATIC_MAP_PROFILE, CLEANNAV_STATIC_MAP_DIRECTORY, and CLEANNAV_STATIC_WORLD_FILE explicitly' >&2
  exit 2
fi
SIM_CONTROLLERS=$(python3 - "$CLEANNAV_STATIC_PREFIXES_JSON" <<'PY'
import json,sys
from pathlib import Path
p=json.loads(Path(sys.argv[1]).read_text())['cleannav_simulation']
print(Path(p)/'share/cleannav_simulation/config/ackermann_controllers.yaml')
PY
)
test -f "$SIM_CONTROLLERS"
xacro "$NAV/cleannav_simulation/urdf/cleannav_ackermann_startup_smoke.urdf.xacro" \
  "controllers_file:=$SIM_CONTROLLERS" \
  cmd_vel_topic:=/cmd_vel sim_lidar_samples:=181 sim_lidar_rate:=8 \
  > "${CLEANNAV_STATIC_URDF_OUTPUT:-/tmp/cleannav-dual-lidar-static.urdf}"
python3 - "${CLEANNAV_STATIC_URDF_OUTPUT:-/tmp/cleannav-dual-lidar-static.urdf}" "$SIM_CONTROLLERS" <<'PY'
from pathlib import Path
import sys,xml.etree.ElementTree as ET,yaml
urdf=ET.parse(sys.argv[1]).getroot(); controller=Path(sys.argv[2]).resolve()
by_reference={x.attrib.get('reference'):x for x in urdf.findall('gazebo')}
def sensor(reference):
    node=by_reference[reference].find('sensor')
    assert node is not None
    return node
low=sensor('base_scan'); roof=sensor('mapping_lidar_link')
assert low.findtext('update_rate')=='8'
assert low.findtext('./ray/scan/horizontal/samples')=='181'
assert abs(float(low.findtext('./ray/scan/horizontal/min_angle'))+1.5707963267948966)<1e-9
assert abs(float(low.findtext('./ray/scan/horizontal/max_angle'))-1.5707963267948966)<1e-9
assert roof.findtext('update_rate')=='8'
assert roof.findtext('./ray/scan/horizontal/samples')=='360'
assert float(roof.findtext('./ray/scan/horizontal/min_angle'))==-3.141592653589793
assert abs(float(roof.findtext('./ray/scan/horizontal/max_angle'))-3.12413936106985)<1e-9
assert roof.findtext('./plugin[@name="mapping_roof_raw"]/ros/remapping')=='~/out:=/mapping/scan_raw'
assert roof.findtext('./plugin[@name="mapping_roof_raw"]/frame_name')=='mapping_lidar_link'
assert roof.findtext('./plugin[@name="mapping_roof_self_filter"]/ros/remapping')=='~/out:=/mapping/scan'
assert roof.findtext('./plugin[@name="mapping_roof_self_filter"]/frame_name')=='mapping_lidar_link'
assert low.findtext('./plugin[@name="cleannav_laser_self_filter"]/ros/remapping')=='~/out:=scan'
assert low.findtext('./plugin[@name="cleannav_laser_self_filter"]/frame_name')=='base_scan'
roof_joint=urdf.find('./joint[@name="mapping_lidar_mount"]')
assert roof_joint is not None
assert abs(float(roof_joint.find('origin').attrib['xyz'].split()[2])-.80)<1e-9
params=by_reference[None].find('.//plugin[@name="gazebo_ros2_control"]/parameters')
assert params is not None and Path(params.text).resolve()==controller
cfg=yaml.safe_load(controller.read_text())['ackermann_controller']['ros__parameters']
assert cfg['base_frame_id']=='ackermann_rear_axle' and cfg['odom_frame_id']=='odom'
print('DUAL_LIDAR_AND_REAR_AXLE_CONTROLLER_CONTRACT_PASS')
PY
python3 -m pytest -q -p no:cacheprovider \
  "$HERE/tests" \
  "$NAV/tools/startup_smoke/tests/test_publisher_graph_gate.py" \
  "$NAV/cleannav_navigation/test/test_execution_mode.py" \
  "$NAV/cleannav_navigation/test/test_ackermann_reverse_config.py"
git -C "$NAV" diff --check
