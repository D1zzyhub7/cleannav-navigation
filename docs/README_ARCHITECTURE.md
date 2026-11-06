# Architecture — frozen R8-A4

This describes the R8-A4 runtime recovered in this repository, not the older startup-only worktree. See [README](../README.md) and [source map](../provenance/R8_A4_SOURCE_MAP.json).

## Frames, localization and planning

Localization publishes map→odom; vehicle odometry provides odom→base_footprint/rear axle and the static vehicle transforms. Consistent map identity, source timestamps and fresh TF are prerequisites. The successful PATROL used the existing RTAB localization/Oracle simulation chain, not a newly introduced localization algorithm. Localization health and Security readiness are independent evidence; either can block execution.

The Global Planner/Hybrid-A* bridge returns map-frame paths. Paths are checked for finite full geometry, map identity and costmap safety before admission. The Path Executor adapter is separate from the Facade's direct Nav2 action-client lifecycle. Hybrid-A* feasibility is not proof that MPPI will finish a finite endpoint.

## Facade and controller

`cleannav_navigation/navigation_facade_node.py` is recovered verbatim from the running installation, SHA c45132dbd5881a58df0087203418bb4b6b983d271436fd383a45da56ab12e948. It mediates application-facing navigation/FollowPath/polygon actions, validates paths, performs existing replanning, tracks inner goals and publishes terminal results. The recovered asynchronous lifecycle retains protection against stale Future callbacks; no new state machine is added.

PATROL's admitted closed path goes through `closed_loop_splits`. The two slices share the original split Pose; the Facade sends successive finite FollowPath goals and only returns outer success after both succeed. Open paths remain one segment. Boundary geometry helpers and the Boundary costmap plugin constrain task geometry in addition to Keepout/Unknown/Safety; they do not replace those constraints.

R7-RC MPPI contains the actual curvature reference/critic/output implementation used by the dynamic baseline. Its Bounded plugin calls the real MPPI then enforces the existing Ackermann output contract. The recovered source and numerical constraints are frozen; no R8-A4 sampling/critic change is made. Effective runtime values include N=750, horizon50, vx_max=.30m/s and minimum turning radius1.12m; exact YAML, rather than rounded prose, governs every parameter. Neither pure Ackermann feasibility nor stopping alone constitutes task success.

## Costmap and Keepout

A2 Costmap makes the grid snapshot, filter TF and publication/service timestamp coherent. Exact-time TF failure does not fall back to latest/identity. Its costmap-filter ABI changes require the matching package libraries and compatible consumers. Vendored A2 headers and implementation are retained together. Keepout core cells must be lethal254; observed253 is not accepted. Runtime Security uses readiness/failure latches; later good data cannot erase a latched failure in the same session.

## Mission Manager, Gateway and Safety

Gateway accepts one idempotent task-sequence submission. Mission Manager drives GOTO then PATROL and owns step/sequence completion. It obtains real Safety Leases; application admission is not motion permission. Safety Supervisor arbitrates candidate commands, monitors health/Lease, publishes final commands, handles controlled stop/ESTOP and releases ownership. There must be a trusted unique final command publisher.

The Navigation repository declares these external dependencies; restoring navigation source does not alter Mission Manager/Gateway/Safety interfaces. Historical launch and consumed-run environments refer to local assets and other package prefixes; they are not a portable auto-start script.

## Evidence and scope

See [dynamic result](validation/R8_A4_DYNAMIC_RESULT.md) for action UUIDs and endpoint samples, and [history](VALIDATION_HISTORY.md) for A2 failure and the A4 decision. One original-route simulated lap is validated. Multi-lap statistics, real vehicle dynamics and R9 geometry has offline evidence on its own branch, with no dynamic validation.

## Offline regular-site generation and future boundary

`tools/map-editor/core.py` owns offline perimeter geometry; it is distinct from the
frozen Facade splitter. The map editor produces map-identity-bound route/Registry
assets, and the existing Mission/Facade chain consumes them. R9 raises only the
design radius factor1.03→1.20; footprint/minR/buffer/.15m safety margin and existing
rounding/discretization/closure remain. Vehicle values now come from the identical
frozen YAML in this checkout; read-only map/Registry helper snapshots remove old
local-worktree import dependencies, without modifying any runtime package.

Fixed original entry/forward C-route compatibility is the product acceptance
scope. Wider reverse/rephasing cases are documented limitations of the unchanged
near-straight predicate, not a new runtime Gate or an A4 redesign.

The separate future irregular-site contract has no arrows into core, UI, launch or
ROS. It names possible Fields2Cover research stages only; it does not add reverse
execution, continuous-curvature geometry or coverage planning to CleanNav.
