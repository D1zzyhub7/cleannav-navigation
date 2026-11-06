# R8-A4 final dynamic validation — one complete simulated lap PASS

Date2026-11-06. Authoritative original: [R8_A4_THREE_ATTEMPT_DYNAMIC_RESULT](../experiment-archive/r8-a4/R8_A4_THREE_ATTEMPT_DYNAMIC_RESULT.txt), with [full attempt1 result](../experiment-archive/r8-a4/ATTEMPT1_RESULT.txt). Prior implementation/PREP NOT RUN statements describe earlier time points, not the final outcome.

Candidate `6785d5a3b98447606d52b2cdb9b4600d5fb2b7416f852f3dcef37cd8033e68f5`; parentA2 `d705384e023a3c545f90d95fb7062c7bcedac7a47903507868128b56fc61e8f2`. Latest actual dynamic baseline is A4. User authorized up to3 independent attempts; attempt1 passed, attempts2/3 NOT_RUN_STOPPED_AFTER_FIRST_SUCCESS. The consumed session cannot be reused.

| Check | Recorded outcome |
|---|---|
| SYSTEM READY / clock | PASS, stable3.020s,7 distinctMM samples, no rollback |
| Sole POST | 202/accepted/replayedfalse, one final idempotency record |
| Real Lease / GOTO | Owners ros-execution-1/2; GOTO action success |
| First inner FollowPath | adffa66c3e1a43b2a5c15cb2531fda70, accepted245.2sim s, SUCCEEDED |
| Second inner FollowPath | 6fe32f6f28c1482b9b080ccf09c470c3, accepted351.4sim s, SUCCEEDED |
| Outer FollowPath | e9237688e4bf416ebb617003b2ebc388, accepted245.0sim s, SUCCEEDED |
| MM / APP | SUCCEEDED, completed_steps2/2 |
| Complete PATROL | PASS,187.305s,40.281m odom; original route41.086m |
| Optimizer / Localization LOST / Security LOST / protective ESTOP | All0 in applicable task/after-first-good windows |
| Keepout core253 | 0; recorded local656/global201 unique verdictsPASS; raw646/198 strict offline replayPASS |
| Final safe stop | All6 finalTwist components0, Safety reliable acknowledgement, Leaseownerempty |
| Cleanup | OwnPIDs/groups gone, ports11844/18144free, lockreleased, mapSHAunchanged |

The admitted map-frame1649-pose path exactly matches original XY, wrapped yaw difference≤2.45e-16. Actual Facade executable came from A4 Navigation; live library maps selected RC MPPI and A2 Costmap. The fixed module deterministically yields split932; raw FollowPath goal payload/in-process module.__file__ were not captured. Do not recast reconstruction as direct action payload observation.

Terminal nearest preceding bagpose errors: first .097167m/.141618rad, second .090793m/.009656rad, jointly within original .10m/.35rad. Each window has4 jointly-valid samples. Poses use odom plus latest prior received map←odom/static transform and precede status by59/36ms; they are not exact controller Buffer/latch telemetry. Original Observer raw success3 includes delayedGOTO; complete UUID/acceptance stamps establish actual2PATROL successes.

GOTO was193.528s/41.283m with191PATH_REPLACED and about.5048m entry distance at stable stop. Business success is not instantaneous .30m precisionPASS. This source publication does not change checker behavior.

MPPI3747 benchmarks P50/P95/P99/max8/16/18/27ms, no>100ms. Callback gaps include handoff/stopped intervals; no claim of continuous10Hz. Final commands zero tail2.600s, last1s odom drift.000201m, Safety reliable-stop log confirms parking/release. MapDB before/after `e16e672a46ebd237a607bf8c34fb968f3f0479ea00fd0de6b40ed5a1d5df9bbd`. Frozen392product entries and96candidate-manifest entries unchanged;4generatedpyc caches listed separately.

Foreign unreadable process environments remain UNKNOWN (PREP27/end26), cgroupUNKNOWN/WAIVED; CPU not additionally sampled. Startup warnings and teardown SIGINT errors remain in original logs. Original bag/native logs are local archive artifacts, not shipped in Git. One simulated lap is validated; multi-lap robustness, real vehicle and R9 are not.

Evidence: [machine receipt](evidence/dynamic-validation-receipt.json), [two-goal/pose evidence](evidence/terminal-evidence.json), [loaded components](evidence/runtime-loaded-components.json), [cleanup](evidence/postrun-cleanup-verification.json), [identity](evidence/postrun-identity.json), [clock](evidence/clock-gate.json), [idempotency](evidence/gateway-idempotency-evidence.json), [trajectory](evidence/trajectory-and-velocity.png). [Archive sources and hashes](../experiment-archive/r8-a4/SOURCES.json).
