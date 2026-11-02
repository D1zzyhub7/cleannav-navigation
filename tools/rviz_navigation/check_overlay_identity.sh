#!/usr/bin/env bash
# Configuration-only subprocess: no runner, Run ID, ROS nodes, locks or sockets.
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
[[ $# == 1 || $# == 3 ]] || { printf 'usage: check_overlay_identity.sh SETUP [--report NEW_JSON_FILE]\n' >&2; exit 2; }
SETUP=$1
REPORT=
if [[ $# == 3 ]]; then
  [[ $2 == --report && -n $3 && ! -e $3 ]] || { printf 'report must be a new JSON file\n' >&2; exit 2; }
  REPORT=$3
fi
[[ -f "$SETUP" ]] || { printf 'reviewed setup file does not exist\n' >&2; exit 2; }
export PYTHONDONTWRITEBYTECODE=1
export CLEANNAV_RVIZ_OVERLAY_SETUP
CLEANNAV_RVIZ_OVERLAY_SETUP=$(realpath -- "$SETUP")
source "$CLEANNAV_RVIZ_OVERLAY_SETUP"
[[ -n ${CLEANNAV_RVIZ_MACHINE_CONFIG:-} ]] || { printf 'setup did not identify its machine configuration\n' >&2; exit 2; }
ARGS=(check-environment --config "$CLEANNAV_RVIZ_MACHINE_CONFIG")
[[ -z "$REPORT" ]] || ARGS+=(--report "$REPORT")
python3 "$HERE/overlay_identity.py" "${ARGS[@]}"
