#!/usr/bin/env python3
"""Regenerate the startup entry's repository-relative source identity manifest."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path


NAV = Path(__file__).resolve().parents[2]
MANIFEST = NAV / "STARTUP_MANIFEST.json"
FILES = (
    "tools/startup_smoke/startup_smoke.sh",
    "tools/startup_smoke/startup_supervisor.py",
    "tools/startup_smoke/startup_smoke.launch.py",
    "tools/startup_smoke/mapping_mode_retry.py",
    "tools/startup_smoke/publisher_graph_gate.py",
    "tools/startup_smoke/roof_lidar_runtime.py",
    "tools/startup_smoke/run_static_checks.sh",
    "tools/startup_smoke/update_manifest.py",
    "tools/startup_smoke/tests/test_publisher_graph_gate.py",
    "tools/startup_smoke/tests/test_startup_smoke_contract.py",
    "cleannav_simulation/launch/startup_smoke_simulation.launch.py",
    "cleannav_simulation/urdf/cleannav_ackermann_startup_smoke.urdf.xacro",
    "cleannav_simulation/worlds/startup_smoke_mcity.world",
    "cleannav_rtabmap/launch/startup_smoke_rtabmap.launch.py",
)
PROVENANCE = {
    "git_parent_commit": "403639cafbc5b19d8a7ffc0699c0677831ba0fc6",
    "historical_candidate_sha256": "17effab2ee69b967789f1aa486f6e1224832053fbcfca19b2e4287f089765f96",
    "verified_reference_entry_sha256": "bd07e9a48940e92660a9a349fca95c951d30418eb92dba1c5d35509356f73da8",
    "historical_simulation_launch_sha256": "76ce38574c5bfaeaa07d1e6b9eaae25ca9b47902d89b652d7067aee62f98c9a0",
    "historical_roof_xacro_sha256": "59036f677ae1dd873157132583f41fd7ad0cd722bf7bc790734b379343287a3a",
    "historical_rtabmap_launch_sha256": "07bfe6304239b0568d2717162ea3e457ffb953fb86a7c4c9a14e38556fb2a8d0",
    "historical_world_sha256": "c7c38234503b91c06a4f743c714b3448b22d21ebd65732f2b75beae0e3862637",
}


def main() -> int:
    files = {}
    for relative in FILES:
        path = NAV / relative
        if not path.is_file():
            raise FileNotFoundError(f"required startup source is missing: {relative}")
        files[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    data = {"schema_version": 1, "operation": "startup_smoke", "files": files,
            "provenance": PROVENANCE}
    MANIFEST.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {MANIFEST} ({len(files)} repository-relative files)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
