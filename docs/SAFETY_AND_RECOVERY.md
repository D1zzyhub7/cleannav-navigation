# Safety and recovery

The [Operations Runbook](OPERATIONS_RUNBOOK.md) is the current PATROL workflow. This document preserves its safety contract; it does not add permission, runtime checks or an alternative startup layer.

## Permission and readiness

A source build or unit-test PASS grants no motion permission. A future dynamic run needs its own explicit authorization, candidate identity, fresh session IDs and independent resources. Do not replay the successful historical session. Keep original Collision Monitor/scan, localization, Keepout/Unknown, Safety and Lease predicates. Unknown safety state remains fail-closed; unreadable process environments are UNKNOWN rather than proof of an idle Domain.

## Command and Lease lifecycle

Mission Manager acquires the real Safety Lease for execution and releases it at terminal completion/failure. Navigation/controller output is only a candidate command. Safety owns arbitration and final trusted command publication. Do not bypass it with direct Twist, RViz or API calls, set approved flags, fabricate Lease ownership or relax speed/radius constraints.

At successful completion require zero final commands (all six Twist components), stationary physical evidence, reliable Safety controlled-stop acknowledgement and empty Lease owner. A stopped vehicle with FAILED sequence is not a successful lap. A sequence success without terminal cleanup evidence is insufficient.

## Failure handling

Preserve the first causal error, bag, action UUID transitions, task events, Safety/Lease timeline and cleanup receipt. Separate startup transients, task faults and shutdown SIGINT errors. Optimizer fail, LOST, Keepout-core253 and ESTOP have distinct causes. Do not repeatedly run a deterministic failure or loosen criteria to obtain PASS. Existing Localization/Security failure latches require a newly validated session rather than in-session reset.

Use the original runner/supervisor cleanup contract and registered PID/PGID/SID ownership. Do not use broad pkill/killall, terminate unknown processes, close protected terminals, or equate closing a shell with child-process exit. The original stopping/cleanup logic preserves foreign processes. Verify owned children/groups gone, ports released, readable Domain holders absent, and lock released; retain unreadable holders as UNKNOWN.

## Build resource safety

Use the recovered existing resource-supervised build manager, only one package phase at a time, MAKEFLAGS=-j1 -l1, empty GNUMAKEFLAGS, CMAKE_BUILD_PARALLEL_LEVEL=1, sequential executor/workers1 and the shared build.lock. Actual compiler count and argv matter. Stop on memory/swap/PSI pressure; do not increase concurrency, auto-retry resource rejection or overlap Gazebo/runtime with compilation. cgroup UNKNOWN/WAIVED is not complete resource verification.

## Known observations

A4 first lap had no Optimizer failure/LOST/ESTOP/core253 and completed both segments. GOTO's stateful checker returned business success with stop distance about0.505m; this is recorded, not repaired here. Four runtime-generated pyc caches do not change source identity but prevent claiming the entire candidate directory never changed. Raw action goal payloads were not captured. See [validation](validation/R8_A4_DYNAMIC_RESULT.md).
