#!/usr/bin/env bash
# 统一入口：资源预检、独占锁、实际工具约束由独立 Python supervisor 管理。
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$script_dir/safe_build.py" "$@"
