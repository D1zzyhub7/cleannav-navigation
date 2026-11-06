# Navigation Operations Runbook — R8-A4

Status: R8-A4 completed one full simulated lap on2026-11-06; R9 C later completed one authorized fixed-start/ccw lap with that unchanged installation. See [R9 final result](validation/R9_DYNAMIC_RESULT.md). It does not grant a new runtime authorization. [Final dynamic result](validation/R8_A4_DYNAMIC_RESULT.md) supersedes the historical PREPARED reports. The [verbatim workspace Runbook](experiment-archive/r8-a4/WORKSPACE_OPERATIONS_RUNBOOK.txt) and [report source index](experiment-archive/r8-a4/SOURCES.json) retain earlier successes/failures without treating their consumed sessions as current commands.

## 1. Workflows and scope

| Workflow | Entry / evidence | Permission |
|---|---|---|
| Source development | pytest and supervised package build in [Development Guide](DEVELOPMENT_GUIDE.md) | No motion/Lease/API submission |
| Formal PATROL | Fresh independent session, `bash run_patrol_lap.sh` | Separate explicit authorization and fixed identity |
| Historical experiment | Frozen runner/env/manifest/log/bag | Read-only evidence, never replay consumed IDs |
| Legacy no-motion smoke / RViz | Existing specialized documents under docs/operations | Separate profile/readiness contract; not PATROL proof |

Do not use evidence/preflight-command.sh, evidence/preflight.py or another helper as a formal entry. Do not introduce a replacement startup wrapper, observer or runtime Gate. This release creates none.

## 2. Environment initialization and ROS source order

Use the reviewed Ubuntu22.04/Humble environment and the exact candidate-run/env.sh for the selected candidate. Runtime env composes package prefixes, not source trees. The historical A4 env sources A2's environment, then prepends A4 Navigation to AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH/LD_LIBRARY_PATH/PYTHONPATH. Existing missing setup-hook warnings remain recorded; do not silently repair or hide them.

The proven runner owns ordering: process/port/Domain checks first; source Humble and candidate env; export target Domain; verify selected prefixes/identity/resources/map; establish FD9/SID; launch; wait for health/Security/clock; exactly one POST. **Do not source overlay or export ROS_DOMAIN_ID outside the runner.** For build-only source order see Development Guide; it is not a runtime recipe.

## 3. Package and install prefix selection

Historical A4 chose new Navigation prefix containing only the split change, R7-RC MPPI, R7-A2 Costmap, A2's existing MM/Simulation/Gateway/Safety/Planner/Executor/RTAB components and custom interfaces. The [recorded component map](validation/evidence/runtime-loaded-components.json) gives actual process argv and loaded libraries; [source map](../provenance/R8_A4_SOURCE_MAP.json) maps restored sources.

Do not rebuild old startup/pc-sim Facade and assume PATROL survives: those sources lacked the running PATROL and later lifecycle fixes. Never mix A3 MPPI/Bounded ABI. Python launched as an installed Facade script searches its own directory first; PYTHONPATH alone does not prove the module selected. Verify installed module file/hash and actual process/library selection using the existing evidence path, without adding runtime machinery.

## 4. Session binding

Each future authorized attempt needs fresh session directory, full candidate/parent identity, unique Domain/Gazebo/Gateway ports, sequence_id, command_id, command/sequence files and idempotency DB. Do not reuse a prior DB or executed session. Preserve exact runner/Observer logic; substitute only required binding values as the prior PREP did. Never run two Gazebo/patrol tasks in parallel.

The successful historical session was `session-patrol-lap-1-r8-a4-20261106T025507Z`, Domain228/Gazebo11844/Gateway18144. It is consumed and cannot be run again. Its absolute path is an evidence locator, not a portable launch command for another developer.

## 5. Formal startup

Only after a fresh session is correctly bound and separately authorized, enter that session as cwd and run:

```bash
bash run_patrol_lap.sh
```

Do not call from a different directory, prepend source/export, invoke helper preflight, or reconstruct ros2 launch arguments from memory. Runner source/export/check order and cleanup remain authoritative. SYSTEM READY is necessary, not proof of localization/navigation/lap completion.

## 6. Resources, locks, FD9 and SID

The existing shared build.lock excludes compilation and large runtime. The runner opens/inherits FD9 to the correct inode and exports its owned SID according to the existing Campus lock contract. Installed launch checks the inherited descriptor/inode/SID; a shell variable naming a lock is insufficient. Preserve FD9 across the intended process chain, actual SID and process-group ownership, set-m/wait-n and final cleanup. Do not pre-acquire a conflicting lock outside the runner or add a second runtime lock.

Retain memory/swap/PSI and runtime resource checks. cgroup UNKNOWN/WAIVED is not full resourcePASS. PREP had27 unreadable process environments; final sampling26 remained UNKNOWN. Do not use sudo, change /proc visibility, whitelist unknown PIDs or claim Domain fully idle based only on readable processes.

## 7. Localization initialization

Use the exact map ID/revision/hash, frozen database, initial pose, sensor/model/TF and localization parameters from the selected candidate. No map/Registry/EntryPose modification is part of this release. In the successful runtime, localization thresholds were the existing .02m/.03rad update settings inherited through the candidate chain; this release does not tune them.

Require fresh localization, proper map→odom→vehicle frames, sensor stamps and TF consistency. An initial pose is only a prior. Localization LOST after readiness is a real failure; shutdown-only LOST/errors must be separated by timeline. Existing failure latches cannot be reset in-session to resume.

## 8. Mission Manager and stable clock

Require actual service/action readiness, real adapters, valid Security and stable clocks before task submission. Preserve the original gate: nonzero advancing /clock, latest signed MM clock lag within250ms,3s continuous stability and distinct recent MM stamps. /clock QoS is the existing BEST_EFFORT/VOLATILE choice; default RELIABLE previously received zero samples. Do not loosen clock windows or infer MM time from Gateway time.

Historical A4 gate stabilized3.020s,7 distinct MM samples, no backward jump. This is evidence for that run, not a waiver for future sessions.

## 9. Gateway single submission and idempotency

The existing runner/Observer sends the sole authorized POST after readiness. Keep the exact API/sequence schema and idempotency semantics; no alternative app control chain. Require HTTP202, accepted=true/replayed=false, one command/sequence binding and a real task event. A repeated/replayed acceptance is not a new successful execution. Do not manually POST or add retries during this release task.

## 10. Safety Lease

A real Lease is acquired for GOTO and PATROL by the existing execution path, not simulated by test helpers. Historical owners were ros-execution-1/2. MM terminal state, Safety owner and final stop acknowledgement must agree. Release clears owner and resources; task admission alone grants no right to publish velocity.

## 11. GOTO

GOTO brings the vehicle to the route entry through Facade/planner/controller and can replan under existing lifecycle behavior. Require actual action and MM step success. Record path/time/replan and endpoint separately. A4 GOTO was SUCCEEDED but took193.528s/41.283m and stopped about0.5048m from entry. Existing stateful checker may latch XY earlier; do not write this as instantaneous .30m accuracy or silently change goal tolerances.

## 12. PATROL and two FollowPath segments

The recorded admitted map path had1649 poses identical to the frozen route. `closed_loop_splits` computes `((0,933),(932,1649))`; the shared Pose is original index932. Facade must finish the first real inner FollowPath, then the second, before outer success. Check full goal UUIDs and accepted ROS stamps; late statuses from prior GOTO can cross task-step boundaries.

The original Observer reported3 successes due to a delayed GOTO result. Independent bag UUID/accepted-stamp classification establishes exactly2 PATROL goals, bothSUCCEEDED. Never change the Observer raw count to hide this interpretation issue. Index932 is deterministic reconstruction from pinned source+recorded input, not a directly logged action goal payload.

## 13. Completion conditions

The original patrol checker XY=.10m/yaw=.35rad remains. Historical nearest preceding reconstructed terminal samples were .097167m/.141618rad and .090793m/.009656rad, both jointly within bounds, with additional valid samples in each window. Reconstructed odom+prior map←odom is not exact controller TF Buffer/checker latch telemetry.

Full lap acceptance requires both inner successes, outer FollowPath SUCCEEDED, Mission Manager/APP SUCCEEDED, completed_steps2/2 and complete trajectory progression. GOTO alone, partial PATROL, current_lap1/1 or stopped vehicle is insufficient.

## 14. MPPI and Ackermann

Use the exact recovered R7-RC source, header ABI, critics and numerical configuration. Preserve batch750/horizon50, vx/wz/noise/std/radius settings from the effective YAML. Existing minimum radius1.12m and output constraints remain. Do not use this publication to tune critics, widen tolerances or increase sampling. The native benchmark is computeVelocityCommands integer-ms timing, not total node latency or continuous scheduling proof.

## 15. Keepout and A2 Costmap

A2 grid/header/update/service snapshots share timestamp semantics, with exact-time TF and rollback/refusal on missing TF. All matching A2 libraries/headers must travel together. Keepout/Unknown/Footprint/padding assets and strict core254 predicate remain frozen;253 must not be accepted as near-lethal success. Runtime recorded local656/global201 unique verdicts and raw local646/global198 strict replays all passed with core253=0. Offline TF interpolation/replay is distinct from direct filter-internal telemetry.

## 16. Logs, bag and metrics

Save actual cwd/argv/rc, identities, prefix/library maps, launch.log, original ROS logs, bag, task/action/Safety/Lease timelines, trajectory, first-error classification, cleanup and file SHA. Read bag offline; do not start ROS graph merely to inspect evidence. Keep parser/env/schema failures and corrected analysis invocations.

Historical MPPI3747 samples: P50/P95/P99/max8/16/18/27ms, >100ms0; total frequency9.843Hz with gaps at handoff/stopping, not continuous10Hz proof. cmd_vel~10.003Hz/max gap.1043s. No added CPU sampler was used. PATROL187.305s/40.281m vs41.086m route; nearest-arc seam projection exceeding100% is not precise coverage proof.

## 17. Stop, failure and recovery

Use the existing runner/Safety cancellation/controlled-stop path; never publish direct velocity, reset latches, force-complete a sequence or bypass Safety. Preserve first causal task error; distinguish startup entity-not-spawned transients and teardown SIGINT/KeyboardInterrupt logs. Do not label every shutdown traceback as a patrol failure or claim logs contain no errors.

Require all6 final Twist components zero, physical stop evidence, reliable Safety controlled-stop acknowledgement, owner empty and resource release. Historical Safety log explicitly acknowledged controlled stop and lease cleared/block_all=true. The older security timeline lacked stop_confirmed; do not invent that field.

## 18. Cleanup

Track owned PID/start ticks/PGID/SID. Follow original registered process-group cleanup, preserve foreign processes, wait for exits and save cleanup.json/group_after. Verify target ports, readable Domain holders, shared lock reacquire-and-release and map DB SHA before/after. A collector counting itself once does not mean a leftover runtime. Do not terminate unknown processes or use broad kill commands. Tool/UI terminal closure is not process cleanup proof.

## 19. Common faults

| Symptom | Evidence / handling |
|---|---|
| Ownership rejection before startup | Check actual FD9 inode/SID inheritance and original source order; no alternate wrapper |
| Clock gate sees zero samples | Compare offered /clock QoS, MM stamps and bag; do not increase timeout blindly |
| HTTP422 timestamp future | MM clock handoff may lag Gateway; preserve first response, do not retry submission |
| GOTO succeeds but entry error large | Stateful checker/trajectory timeline, separate business success from instantaneous precision |
| Optimizer failure in bend | Inspect admitted split, terminal geometry, path progress and original optimizer logs; no automatic MPPI tuning |
| Keepout core253 / Security LOST | Inspect exact snapshot stamp/grid/TF; do not relax strict predicate or erase latch |
| Import wrong split module | Inspect installed Facade script directory and source SHA, not PYTHONPATH alone |
| Apparent3 PATROL successes | Classify full UUID accepted stamps to exclude delayed GOTO status |
| Shutdown traceback | Separate teardown from first task error; preserve logs and wait/cleanup outcomes |

## 20. History, limits and next stage

[Validation history](VALIDATION_HISTORY.md) retains R6 failures, R7-RC253, A2 later-turn failure, A3 offline noneligibility and R8-A4 correction. [Verbatim implementation](experiment-archive/r8-a4/R8_A4_IMPLEMENTATION_RESULT.txt) and [PREP](experiment-archive/r8-a4/R8_A4_DYNAMIC_PREP.txt) are dated snapshots; final dynamicPASS is authoritative.

No multi-lap statistics or real-vehicle validation has passed. Full foreign-process/cgroup visibility remains unknown. General continuous-curve fallback/endpoints and GOTO precision observations remain. At the R8-A4 source-release point R9 was not implemented; the later R9 result below supersedes that historical status. Further changes and dynamics require their own scope and evidence.

## R9 candidate operations boundary — historical offline stage, superseded below

R9 is an offline regular-site candidate, not the current dynamic baseline. Its C
entry moves0.1904m west, heading unchanged; use the newly generated Registry/route
together. Do not deploy either into a frozen A4 session or reuse an old task DB.
Only fixed production start and forward route are accepted for this candidate;
reverse/rephasing scope is not expanded by the unchanged A4 helper.

No R9 dynamic session or runnable environment has been prepared by this PR. After
user review and a separate dynamic authorization, bind a fresh independent session
through the established A2/A4 runner and Observer contract, preserving component
prefixes, new IDs/ports/domain, clock/readiness, Lease/stop and cleanup. The sole
formal command remains `bash run_patrol_lap.sh` in that fresh session’s cwd; no
outer source/export or evidence/preflight entry. No new runtime Gate is added.

The irregular-site file is NOT_INTEGRATED and cannot be selected in the editor or
runner. The offline editor/demo is unrelated to Gateway task submission.

## R9 dynamic freeze — current limited baseline

R9 path identity25239cb9dec7c29e2939a7a6cc745f0a7b6daa355d030c43265dcd8f07404c4c
was run with A4 install identity6785d5a3b98447606d52b2cdb9b4600d5fb2b7416f852f3dcef37cd8033e68f5.
No R9 ROS package exists; offline generator source is not a runtime overlay.
Use the original A4 Navigation/RC MPPI/A2 Costmap/MM/Safety/Gateway/environment
unchanged with checksum-bound R9 C route and matching Registry. The final
[composition](validation/r9-dynamic/R9_FROZEN_COMPOSITION.json) pins all prefixes,
world/map/profile and identities. Keep the old profile's historical mask-tube
route metadata as provenance, not a current path selector.

Session session-patrol-lap-1-r9-20261106T112441Z (224/11845/18145) is consumed1/1,
rc0 and fully cleaned. Do not rerun its bash script or reuse its sequence/command/
idempotency DB. Any future dynamic work requires new authorization and a fresh
independent binding through the original formal runner/Observer. Source/export,
resource checks,FD9/SID,3s/250ms clock gate,one POST,Lease/stop/cleanup stay in the
runner; no outer preflight/source/export, no new Gate.

This freeze covers fixed production path start,forward ccw,regular site,C1.344m
and the A4 execution chain only. It does not validate arbitrary starts/reverse,
multi-lap,real vehicle or irregular-site backend. Current V1 API status was read
by the Observer, not a Flutter phone UI run. GOTO stateful endpoint behavior
(~.549m final offset) remains recorded. Safety confirms reliable zero-command
delivery; measured tiny final odom/Oracle jitter is listed, not replaced with0.

Other modes remain at the [read-only audit](V1_MODE_ARCHITECTURE_AUDIT.md) stage.
Current Registry has no charger. Do not open disabled tasks, reuse R9 motion
authorization for target cleaning, change APIs or attach the future irregular
planner. Pause/Resume/STOP/ESTOP/Reset remain global controls.
