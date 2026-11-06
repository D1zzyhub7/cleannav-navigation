# Validation history and remaining work

| Date / version | Verified result | Meaning |
|---|---|---|
| 2026-11-04 R6 | Full lap failed; adapter/localization/Optimizer failures in separate attempts | Preserve failure history; no generic patrol PASS |
| 2026-11-05 R7-RC attempt1 | Keepout core8 cells at253 caused Security LOST during GOTO | PATROL not started; strict policy unchanged |
| 2026-11-05 R7-A2 | GOTO/first turn passed; later PATROL Optimizer failure at original split876 | About55% route progress, not a lap |
| 2026-11-05 R7-A3 | Offline partial failure, dynamic NOT RUN | Do not mix its MPPI/Bounded ABI into A4 |
| 2026-11-06 R8 independent review | Terminal-tangent criterion corrected original chord counterexample | Analysis, not dynamic evidence |
| 2026-11-06 A4 implementation | 166 offline testsPASS incl132 phase/direction; installed loading verified | Then PREPARED/NOT RUN; historical state only |
| 2026-11-06 A4 dynamic | First full lapPASS,2 inner+outer+MM/APP success,2/2 | Used1/3; second/third not run after success |
| 2026-11-06 source release | Exact runtime source recovery, source tests/build recorded separately | Does not revalidate dynamics of rebuilt artifacts |

Formal [dynamic result](validation/R8_A4_DYNAMIC_RESULT.md) supersedes old PREPARED/latest-A2 wording. Verbatim originals, failure analyses, implementation and preparation are retained as .txt historical archives; [SOURCES.json](experiment-archive/r8-a4/SOURCES.json) records source paths/dates/SHA. Their absolute paths are evidence locators, not runnable instructions or claims that raw bags are in Git.

Remaining: multi-lap statistical robustness, real vehicle validation, full foreign-process visibility/cgroup verification, GOTO stateful endpoint precision/long detour observation, general continuous-curve endpoint reachability. R9 route-generation geometry/turn-radius/curvature optimization has not been implemented. No Safety criterion or map/Registry/route is relaxed for publication.
