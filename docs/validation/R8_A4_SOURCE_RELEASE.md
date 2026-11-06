# R8-A4 source recovery and build validation

2026-11-06. **Source recovery/build/regression PASS. No new dynamic patrol was run.** Original candidate identity `6785d5a3b98447606d52b2cdb9b4600d5fb2b7416f852f3dcef37cd8033e68f5` and its [complete single-lap PASS](R8_A4_DYNAMIC_RESULT.md) remain unchanged. This report concerns the recoverable Git source and isolated rebuilt products.

Source/test recovery commit: `996beee479166287045932308a354038d2b786ce`. Its tree matches the independently tested archive tree.

## Source closure

The four runtime Python files are verbatim with the requested SHA: Facade c45132dbd5881a58df0087203418bb4b6b983d271436fd383a45da56ab12e948; splitter870e5de1e5e75facdfea3d901b872b7ed40e434422d68ba65e3969a3511803bd; boundary_runtime b58261544fbd54147fd2cd740a28952efbc01c7bcbbb0da5c46f12fea8103467; polygon_constraint cdabd056221ac78a6ccf1d8027a78d77b9dd99ef53fedb4e895144eb89d21a8d. Fresh installed-process inspection confirms the same source/installed hashes and forward932/reverse751.

165 recovered C++/header files are byte-identical to historical originals. The RC seal verifies79source entries, A2 seal108, zero differences. Boundary source2a2c3686… matches Candidate43 source manifest5b75f456… and its cb97daa6… runtime library. RC/A2 original build plans/receipts identify exact source trees and runtime libraries. Installed headers were compared individually. [Source map](../../provenance/R8_A4_SOURCE_MAP.json) records source path/hash, original location/hash, dynamic output/hash and CMake/setup install rule.

Navigation CMake is recovered from Candidate43 and registers A4, both plugins, headers/XML and source tests. The RC Bounded implementation is restored verbatim in Navigation and its original vendor package. Interfaces use15verbatim installed source definitions; ROSIDL packaging is regenerated without schema changes. Planner/Executor Python modules match running installs. The old Safety source copy is excluded via COLCON_IGNORE, preserving the frozen external Safety dependency instead of substituting an older implementation. No Safety code, MPPI algorithm/parameter, A4 algorithm, map/Registry/route or external interface was changed.

## Build outcomes

All builds use the existing safe_colcon_build.sh supervisor, isolated source/destination, shared lock, serial package order and compiler guard. The release source-only option removes historical binary underlays; resource/PSI/swap limits, MAKEFLAGS -j1 -l1, clearedGNUMAKEFLAGS, CMAKE_BUILD_PARALLEL_LEVEL1, workers1 and ownership/cleanup remain. Six packages built: A2 Costmap, RC MPPI, interfaces, Planner, Executor, Navigation. 9 restored libraries resolve with no missing dependencies/A3 paths.

| Phase | rc | max compilers | elapsed seconds |
|---|---:|---:|---:|
| costmap | 0 | 1 | 281.507 |
| mppi | 0 | 1 | 248.304 |
| mppi-source-only | 0 | 0 | 15.255 |
| interfaces | 0 | 1 | 66.168 |
| planner | 0 | 0 | 2.141 |
| executor | 0 | 0 | 2.143 |
| navigation | 0 | 0 | 14.658 |
| navigation-first-dependency-failure | 1 | 0 | 1.12 |
| navigation-link-failure | 2 | 1 | 32.081 |

The first Navigation attempt was refused by colcon before compilation because local Planner/Executor/Safety setups were missing. Planner/Executor were then built, and the stale Safety copy excluded as documented. A second attempt compiled plugins but failed at the redundant unqualified -lmppi_controller link introduced during packaging recovery; that redundant entry was removed, preserving ament's dependency linking. Final Navigation build succeeded; no algorithm was changed to resolve either packaging issue. All first-error logs/receipts remain in the local release evidence. The final relink reports0compilers because compilation had already completed in the preserved prior attempt; initial C++ builds and the plugin compile phase observed at most1. No resource-pressure retry occurred.

[Exact commands/results/artifact receipts](evidence/source-build-receipts.json). Build/runtime were never concurrent. No Gazebo, actual GOTO/PATROL, POST, Lease request or final velocity publication occurred. Existing mocked ROS-message tests and native in-process fixture nodes are offline tests, not a dynamic system validation.

## Regressions

- A4 source-only suite: **166 passed**, including132start/direction cases and9optimized routes; no ROS dependency, install import or absolute data locator. All core assertions preserved; [JUnit](evidence/patrol-source-tests.xml).
- Existing Navigation/Planner/Executor source units: **197 passed**, including Facade asynchronous lifecycle, polygon/costmap and configuration tests; [JUnit](evidence/package-unit-tests.xml). No skipped failures concealed.
- Existing RC native unit fixture:7PASS; Bounded/PathHandler binding2PASS. [Units](evidence/native--units.txt), [binding](evidence/native--binding.txt).
- A2 original/candidate ABI executables rc0; publisher size240→272bytes, remaining listed base sizes consistent. This is an ABI record, not a whole-upstream CTest claim.
- Fresh Python process resolves all6built package prefixes to the isolated release install, imports splitter from installed Facade directory, matches all4source hashes, computes932/751, and ldd resolves9libraries. [Installation result](evidence/restored-install-check.json).

The builder retains BUILD_TESTING=OFF; its explicit A2 snapshot/native-regression targets were built. The complete upstream Nav2 CTest/lint suite was not executed. Numerical/source tests do not authorize or revalidate rebuilt artifacts dynamically. Recompiled binary hashes differ with path/build ID/packaging/compiler environment and are separately recorded; the historical dynamic baseline remains the frozen candidate.

## Checkout independence and review

A git archive of the exact staged source tree, in a separate directory with PYTHONPATH empty, also passed all166 tests. The committed source tree is checked against that archive tree SHA before publication. [Checkout test receipt](evidence/checkout-source-tests.json). The original17user files/status and392frozen runtime product entries remain unchanged; [preservation receipt](evidence/final-preservation.json). Git whitespace review reports inherited trailing spaces/EOF blank lines in copied vendor documentation/XML and a test file; these are preserved instead of changing sealed source bytes. No runtime algorithm is affected.

## Limits and release boundary

The Git checkout provides Navigation/Planner/Executor, exact RC/A2 dependency source snapshots, message definitions and portable regression fixtures. Deploying the historical complete system additionally requires the separately maintained MM/Gateway/Safety/localization/simulation components and frozen external map/world/Registry/route assets. Their selected original prefixes/hashes remain recorded as dependencies; this release does not pretend to bundle large runtime artifacts or freshly validate those external source trees.

R8-A4 remains one full simulated lapPASS, with GOTOstateful-stop precision and unknown foreign-process/cgroup visibility explicitly retained. Multi-lap statistics, real-vehicle validation and R9 geometry optimization remain outstanding. [Machine source-closure receipt](evidence/source-closure-receipt.json).
