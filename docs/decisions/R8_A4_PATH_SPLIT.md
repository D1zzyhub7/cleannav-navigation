# Decision: freeze R8-A4 path segmentation

Date: 2026-11-06. Status: existing runtime algorithm frozen; source ownership restored. [Dynamic baseline](../validation/R8_A4_DYNAMIC_RESULT.md).

The former farthest-from-start split selected original-route index876 inside the third bend. A finite FollowPath endpoint began convergence before the bend finished; A2 eventually failed later in that turn. Changing the split was isolated from changing the controller or route geometry.

R8-A4 adds private `_straight_before(points,index)`: walk backwards until at least1m path arc length is covered; use the last nonzero terminal segment as the terminal tangent; require every checked point's perpendicular deviation≤.01m. A zero-length final segment is rejected. The original chord criterion has reverse/rephasing counterexamples and is not used.

Keep the public `closed_loop_splits(points, *, closure_m=0.05, minimum_far_m=1.0)` signature. Among candidates satisfying the original distance lower bound and incoming-straight predicate, select the farthest. If none qualify, use the original farthest candidate, preserving all original exception checks/text. No inserted points, coordinate/yaw edits, public parameters, interface changes or changes to open-path behavior. Slices share the exact original Pose.

Original1649-point route computes `((0,933),(932,1649))`; reverse computes index751. These are computed acceptance results, never product constants. 166 regressions include132 start/direction cases and9 optimized route fixtures. Continuous curves may use legacy fallback, so this is not universal finite-endpoint feasibility proof.

The implementation deliberately preserves R7-RC MPPI, A2 Costmap, Safety, Mission Manager, Gateway, map/Registry/route/Footprint/Keepout and the mature runner. The installed Navigation copy had no canonical owner; this release recovers its exact modules, original C++ sources and explicit build/install rules in the existing ROS package layout. It does not push an install prefix or substitute older Facade/upstream Nav2.

R8-A4 is the latest actual dynamic baseline after one complete simulated lap. R9 may separately study route turn radius/curvature continuity later; that work is not implemented or authorized by this release task. Keep its future geometry changes distinct from this frozen baseline.
