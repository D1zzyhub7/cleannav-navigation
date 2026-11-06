# Development and reproducible source build

This release restores existing runtime source; it does not optimize or revalidate the vehicle. Read [Architecture](README_ARCHITECTURE.md), [Operations](OPERATIONS_RUNBOOK.md), [Safety](SAFETY_AND_RECOVERY.md), and [source provenance](../provenance/R8_A4_SOURCE_MAP.json).

## Environment and dependencies

The historical platform is Ubuntu22.04 / ROS2 Humble / Python3.10 on WSL2, GCC11-era system dependencies. Package manifests declare dependencies; the recovered Nav2 snapshots retain their original versions/licenses. Install the ROS/Nav2/ROSIDL/pluginlib/tf2/geometry/navigation dependencies plus xtensor/xsimd and development tools in the normal Humble workspace. The package manifests, not an unverified apt command list, are authoritative. This task did not reinstall system packages or modify /opt/ros.

`vendor/nav2_mppi_controller` is R7-RC with its original controller, critics, curvature profile, header ABI, benchmark definition and Bounded source. `vendor/nav2_costmap_2d` is A2, including exact-time snapshot/filter/publication changes. They override their system packages together; do not select unmodified upstream sources or A3 libraries. `vendor/cleannav_interfaces` restores all15 installed source message/service/action definitions; ROSIDL generates C/Python headers locally. The generated artifacts are not checked in.

## Checkout layout

The established build manager accepts a unique experiment's `src` directory and writes only a `review-fixes-*` destination. Preserve that convention instead of inventing another build path. With `CLEANNAV_WORKSPACE_ROOT` set to your workspace directory, a fresh layout is:

```text
workspace/
  cleannav-experiments/r8-a4-source/src/
    cleannav-navigation/             # this Git checkout
    nav2_costmap_2d -> cleannav-navigation/vendor/nav2_costmap_2d
    nav2_mppi_controller -> cleannav-navigation/vendor/nav2_mppi_controller
    cleannav-interfaces -> cleannav-navigation/vendor/cleannav_interfaces
  cleannav-experiments/step3-2-f-20261011/tools/
    safe_colcon_build.sh
    safe_build.py
  cleannav-build/locks/build.lock
  cleannav-build/step3-2-f/review-fixes-r8-a4-source/
```

These source aliases are workspace conveniences, outside Git and without shared writable install files. This release itself used an isolated Git worktree in that `src/cleannav-navigation` position. An ordinary clone of the release tag provides the same source tree. Never put the frozen candidate install/build/bag/DB into Git, and never overwrite another checkout's dirty files.

The existing build manager is shipped under `tools/safe_build/`; restore those two files into the established workspace tool location shown above **only if that location is absent**. In an existing CleanNav workspace review its version/diff rather than overwriting shared tools blindly. `CLEANNAV_WORKSPACE_ROOT` explicitly identifies the workspace when the tool is used from the checkout. The default old path and all previous targets remain available. The release adds `--source-only-release` solely to omit historical CleanNav binary underlays and use Humble plus `--candidate-underlay`; it does not alter resource policy, lock, compiler guard, concurrency, ownership or cleanup.

## Build

No Gazebo/runtime may overlap compilation. The existing supervisor samples memory/swap/PSI, acquires shared build.lock, uses MAKEFLAGS='-j1 -l1', clears GNUMAKEFLAGS, sets CMAKE_BUILD_PARALLEL_LEVEL=1, sequential executor/workers1 and checks actual make/compiler argv. Do not raise concurrency or auto-retry resource rejection.

Use a clean shell; do not enable `set -u` before ROS setup. For builds the supervisor sources Humble, then only the explicit successful release underlay; it exports the compiler limits after setup. The runtime runner has its own different source/export ordering and is not a build helper.

The following variables name your fresh layout; they are not credentials or a runtime environment file:

```bash
release_src="$CLEANNAV_WORKSPACE_ROOT/cleannav-experiments/r8-a4-source/src"
release_build="$CLEANNAV_WORKSPACE_ROOT/cleannav-build/step3-2-f/review-fixes-r8-a4-source"
build_entry="$CLEANNAV_WORKSPACE_ROOT/cleannav-experiments/step3-2-f-20261011/tools/safe_colcon_build.sh"

bash "$build_entry" nav2-costmap --source-only-release   --candidate-source-root "$release_src" --candidate-root "$release_build"

bash "$build_entry" mppi --source-only-release   --candidate-source-root "$release_src" --candidate-root "$release_build"   --candidate-underlay "$release_build"

bash "$build_entry" mission-sequence-interfaces --source-only-release   --candidate-source-root "$release_src" --candidate-root "$release_build"   --candidate-underlay "$release_build"

bash "$build_entry" navigation-planner --source-only-release \
  --candidate-source-root "$release_src" --candidate-root "$release_build"

bash "$build_entry" navigation-executor --source-only-release \
  --candidate-source-root "$release_src" --candidate-root "$release_build" \
  --candidate-underlay "$release_build"

bash "$build_entry" navigation-b-facade --source-only-release   --candidate-source-root "$release_src" --candidate-root "$release_build"   --candidate-underlay "$release_build"
```

Run each only after the previous rc0/artifact/ament-index/installation check and success marker. Preserve each safe-build evidence directory: the shared destination's marker describes the latest phase. These package target names and source/destination conventions are from the existing manager. A2's source-only build already used just Humble; the release source-only mode removes obsolete additional overlays for the other targets. Planner/Executor must precede Navigation because colcon requires their local package setup. The old Safety source copy has COLCON_IGNORE: it is not the frozen externally deployed Safety and must not be rebuilt as R8-A4. No Safety algorithm is changed. Build evidence and exact commands for this release are in [source release validation](validation/R8_A4_SOURCE_RELEASE.md).

Do not invoke unrestricted colcon build. BUILD_TESTING=OFF is the manager's existing package-build policy; A2 snapshot executables and the RC native offline regression use their explicit original CMake switches. Compilation success does not mean all upstream CTest/lint suites ran.

## Tests

From the Git checkout, independently of ROS or installed Navigation:

```bash
python3 -m pytest -q tests/patrol
```

This is the original166-case suite, including132 phase/direction cases,9 optimized paths, reverse751, forward932, tangent/chord counterexample, fallback, exceptions, duplicate/zero segments and open paths. Only import/data locators changed. `tests/patrol/fixtures/PROVENANCE.json` records original input SHA, extraction and fixture SHA; numerical values/order are unchanged. The legacy implementation is a test-only reference fixture. It is never installed.

Package unit tests in `cleannav_navigation/test`, Planner and Executor use ROS messages/mocked action handles where applicable. Source directories must precede dependency paths; do not accidentally import installed Facade. They are development tests, not an authorization to start real navigation or advertise a dynamic PASS. Review helpers for node/service/Lease side effects before running tests outside the documented set. Legacy assertions can have known incompatibilities with later Facade behavior; retain and report failures rather than changing core assertions to force a green result.

The original RC native `r7_offline_regression` is a local executable in `install/nav2_mppi_controller/lib/nav2_mppi_controller`; A2 snapshot executables and invocation details come from their existing test CMake/source. Their offline feasibility/ideal-plant results are distinct from actual vehicle motion. See the release validation receipt for executed argv, scope and rc.

## Source and binary identity

The four key Python modules are verbatim with the requested SHA. Boundary C++ is traced to Candidate43's source manifest and matching binary; RC MPPI/Bounded and A2 Costmap to their original build plans/receipts/source snapshots. The source map records every recovered file and install rule. CMake/ROSIDL/install registrations are packaging recovery, not algorithm changes. Recompiled ELF can differ due to paths/build IDs/compiler flags/system ABI; never substitute Git commit SHA for candidate identity or call rebuilt artifacts newly dynamically validated.

The recorded historical dynamic env chose A4 Navigation, RC MPPI, A2 Costmap and the A2 external components. New build output is isolated and does not replace them. Deployment requires the frozen external map/world/model/route/Registry and exact MM/Gateway/Safety/localization components; those large/runtime artifacts are not recreated by unit tests or bundled in source.

## Contribution and freeze

Use an isolated worktree, preserve existing dirty/untracked files, review paths/secrets/sizes/links and stage explicit files. No git add . / -A, reset --hard, clean -fd, force push or main overwrite. Source/build and documentation commits are separate. R8-A4's tag records this source recovery and its provenance; future R9 geometry work must have a separate change and validation scope.

## R9 offline editor development

Use `tools/map-editor/requirements.txt` in a Python3.10 environment. No third-party
backend install or C++ rebuild is required for this Python-only change. From this
checkout run:

```bash
python3 -B -m pytest -q -p no:cacheprovider tests/patrol tools/map-editor/tests
```

The166 source A4 tests remain unchanged. Twelve editor tests use explicitly bound
portable fixtures (including loopback editor HTTP tests, not Gateway tasks); six
new real-Pillar tests cover geometry, complete sweep, source-generated C, entry,
Registry, fixed-start split and the old Campus unknown-cell encoding regression.
No ROS init, simulator, Safety Lease or velocity publisher is used by this suite.
An independent checkout outside the local workspace is tested separately.

Exact gzip captures have decompressed source SHA in
`tools/map-editor/tests/fixtures/SOURCES.json`; fixtures do not contain a runtime
RTAB DB and are explicitly test-only. `support/SOURCES.json` records verbatim
existing map validator, Registry loader and schema provenance. The actual vehicle
YAML already in this repository is byte-identical to the frozen RC reference; it
is read, not tuned. Do not regenerate frozen real maps to satisfy unit tests.

The existing PlannerServer-only static replay is a separate environment-dependent
check using the historical exact final costmap, not the unit suite and not a lap.
The current result records action statuses, actual parameters, input/readback SHA,
coordinator rc and original cleanup receipt separately. A prior receipt-path typo
was fixed only in the independent diagnostic copy; no runtime runner was changed.

Future irregular-site work is only the isolated typed contract. It must not be
imported into core/UI or registered in ROS without a new integration task.
