# Repository Startup Smoke Contract

This is the canonical operations specification for the repository-owned
`startup_smoke` entry. The project workspace's `docs/OPERATIONS_RUNBOOK.md`
points here for the implementation contract; historical run records remain
evidence and are not executable recipes.

## Scope and evidence

`startup_smoke` is a bounded **no-motion startup check**. It starts the
Gazebo/Ackermann model, low and roof lidars, ROS clock/TF/odometry, the existing
Safety Supervisor, Collision Monitor lifecycle, and RTAB-Map input path. It
does not start a route, mission, motion Lease, RViz goal, or movement
authorization. It requires a new `startup-smoke-*` run ID and refuses an
existing output directory.

The archived workcopy entry at
`cleannav-deliverables/startup-and-git-recovery-20261021/workcopy/tools/startup_smoke.sh`
has SHA-256
`bd07e9a48940e92660a9a349fca95c951d30418eb92dba1c5d35509356f73da8`. It
passed one dynamic no-motion run (`startup-smoke-20261021T111500Z-01`, rc 0,
30 s effective observation, 90.904 s total) with cgroup status
`UNKNOWN / WAIVED`. The repository-owned copy is a path/ownership
consolidation of that entry. **This copy has static validation only; it has not
been rerun against Gazebo.** The prior runtime result must not be represented
as a runtime pass for this relocated copy.

## Source and environment selection

- `tools/startup_smoke/startup_smoke.sh` is the only supported startup-smoke
  entry in this repository.
- Its launch adapter, supervisor, mapping composition, cleanup/resource
  helpers, graph identity gate, model Xacro, simulation launcher, RTAB wrapper,
  world, and static contract tests are repository-owned and bound by
  `STARTUP_MANIFEST.json`.
- The shell finds the workspace by the existing build/runtime lock files; it
  does not guess from the caller's current directory. It then changes to that
  workspace root. The Gazebo demo assets are expected in the existing sibling
  directory `../cleannav_demo_assets`, resolved relative to that workspace;
  startup preflight rejects a missing models directory.
- ROS Humble is sourced from `/opt/ros/humble/setup.bash` after initializing
  `AMENT_TRACE_SETUP_FILES`. The validated local overlay chain is supplied by
  the operator through `CLEANNAV_STARTUP_OVERLAY_SETUP`; no historical
  Candidate path or install prefix is embedded in this repository entry.
- The selected vehicle Xacro, simulation launch, RTAB wrapper, and world are
  passed by repository-relative source paths resolved at runtime. Runtime
  packages, controller configuration, Safety configuration, and Gazebo/ROS
  plugins still come from the explicitly sourced overlay. The supervisor
  records package prefixes and rejects missing package shares. The local
  overlay setup must export `CLEANNAV_EXPECTED_SIMULATION_PREFIX` and
  `CLEANNAV_EXPECTED_RTABMAP_PREFIX`; the supervisor compares the resolved
  ament prefixes to these explicit operator-bound values and fails closed if
  either is absent or different. The Safety package share must also exist.
- Domain `204`, localhost-only DDS, and Gazebo port `11704` are fixed to the
  previously validated startup profile. The supervisor checks for domain,
  process, port, lock, memory, swap, PSI, cgroup-waiver, and cleanup conflicts;
  do not override these values by sourcing another profile afterward.

The repository cannot manufacture or select an installed overlay. Before any
separately authorized runtime task, the operator must provide a local setup
script that reproduces the previously validated Humble/overlay order and sets
the expected Simulation and RTAB-Map prefix values. Never replace that stack
by guessing a different install tree. The source/overlay relation is part of
each run's identity and evidence.

For historical comparison only, the successful reference run reported
Simulation at
`/home/d1zzy/code/cleannav_modularization/cleannav-build/step3-2-f/review-fixes-20261019T041000Z-mapping-050/install/cleannav_simulation`
and RTAB-Map at
`/home/d1zzy/code/cleannav_modularization/cleannav-build/pc-sim-v2/install/cleannav_rtabmap`.
These are recorded observations, not defaults embedded in the launcher. The
local overlay setup must set the expected prefix variables to the approved
current paths; if the historical paths are no longer the approved install,
stop and verify the replacement stack before runtime use.

## Invocation contract

Use a child process so the caller's interactive shell remains available:

```bash
CLEANNAV_STARTUP_OVERLAY_SETUP=/path/to/approved-local-overlay.bash \
  bash /path/to/cleannav-navigation/tools/startup_smoke/startup_smoke.sh \
  --mode startup_smoke --run-id startup-smoke-<unique-id>
```

The overlay path is a local machine setting, not a repository default. The
entry requires an explicit run ID, permits only `--mode startup_smoke`, and
does not provide motion, route, RViz, map-continuation, or Lease options. It
creates output below
`$CLEANNAV_STARTUP_OUTPUT_ROOT/<run-id>` when that variable is supplied, or
under the workspace's `cleannav-deliverables/startup-smoke-runs/<run-id>` by
default. The target must not already exist.

This document does not grant runtime authorization. A task must independently
authorize a dynamic call and satisfy all live preflight gates. Never invoke
`ros2 launch` directly to bypass the supervisor.

## Readiness and stop rules

The supervisor starts T0 before identity, lock, resource, process, port, and
ROS-domain checks. It requires the active Ackermann and joint-state
controllers; live `/clock`, `/odom`, joint states and required TF; valid
181-beam `/scan` plus 360-beam `/mapping/scan_raw` and `/mapping/scan`; RTAB
input; Collision Monitor lifecycle state `ACTIVE`; Safety `block_all`; no
autonomous enable, no Lease owner, no raw command publisher; and exactly one
stable `/cmd_vel` publisher with all observed commands zero.

All readiness must be achieved by the observation admission cutoff and remain
true for at least 30 s. T0 is monotonic: the supervisor starts cleanup no
later than T0+145 s, grants bounded SIGINT/SIGTERM/SIGKILL cleanup intervals,
and enforces a T0+180 s final cutoff. Cleanup passes only with a complete
process-group scan, no running members, no zombies, and a reaped launch parent.
Failed or unknown gates fail closed and are preserved in the run record. No
automatic retry is allowed.

The resource exception remains explicit: cgroup headroom may be
`UNKNOWN / WAIVED` only when the existing WSL exception and companion resource
stability checks pass. This is not a full cgroup-headroom PASS.

## Static verification

Run the static-only checks before considering any later runtime task:

```bash
bash tools/startup_smoke/run_static_checks.sh
```

After a reviewed source or contract change, regenerate the identity file with
`python3 tools/startup_smoke/update_manifest.py`, then rerun this static check
and review the complete Git diff. The runtime does not regenerate or waive its
own manifest.

This checks shell and Python syntax, repository-owned launch argument
contracts, the no-motion policy, manifest hashes, publisher-identity
convergence tests, and Xacro expansion/structure. It does not start ROS nodes,
Gazebo, RTAB-Map, sensors, or a vehicle. Static success is not runtime
readiness.

## Historical inputs

The repository base is Git commit `403639cafbc5b19d8a7ffc0699c0677831ba0fc6`.
The copied startup/sensor assets were derived from the successful
`roof-lidar-g3-20261020T030538Z` workcopy (historical Candidate SHA
`17effab2ee69b967789f1aa486f6e1224832053fbcfca19b2e4287f089765f96`). The
verified original entry, its source hashes, and the runtime result are
provenance only; they are not path inputs or authority to reuse old run IDs or
authorizations.
