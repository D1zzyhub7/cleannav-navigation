# Irregular site planning — NOT_INTEGRATED / FUTURE_EXTENSION

`irregular_site_planner.py` defines an offline input/output contract and always
raises `NotImplementedError`. It cannot plan non-regular sites today. No source
from the upstream project has been copied; no Fields2Cover, GDAL, GEOS or
OR-Tools package has been installed for this work. `core.py`, UI, launch and
runtime never import or register this module.

## References

- [Official Fields2Cover repository](https://github.com/Fields2Cover/Fields2Cover).
- [BSD-3-Clause license](https://github.com/Fields2Cover/Fields2Cover/blob/main/LICENSE),
  copyright Wageningen University; a future integration must preserve applicable
  notices and assess dependencies separately. This interface is CleanNav code.
- Mier, Valente and de Bruin, *Fields2Cover: An Open-Source Coverage Path Planning
  Library for Unmanned Agricultural Vehicles*, IEEE Robotics and Automation
  Letters 8(4), 2166–2172 (2023),
  [DOI 10.1109/LRA.2023.3248439](https://doi.org/10.1109/LRA.2023.3248439).

## Feature mapping for future research

| CleanNav need | Upstream reference | Remaining CleanNav work |
|---|---|---|
| Non-convex free region, internal holes | [Cell decomposition](https://fields2cover.github.io/source/tutorials/decomposition.html) | Occupancy/unknown/Keepout to polygon conversion with traceable error |
| Trapezoidal / boustrophedon cells | [Upstream feature list](https://github.com/Fields2Cover/Fields2Cover#core-features) | Select decomposition; retain obstacle and boundary margins |
| Connect cells | [Decomposition and route planner](https://fields2cover.github.io/source/tutorials/decomposition.html#decomposition-and-route-planner) | Check corridor connectivity with complete vehicle footprint |
| Kinematic connections | [Dubins / Reeds-Shepp](https://fields2cover.github.io/source/tutorials/path_planning.html) | Explicit forward/reverse policy; current PATROL does not gain reverse support |
| Continuous-curvature turns | [Path planning tutorial](https://fields2cover.github.io/source/tutorials/path_planning.html#dubins-curves-with-continuous-curvature) | Define and validate curvature-rate/steering constraints |

Reviewed official documentation on 2026-11-06 (site labels itself v2.1.1).
This maps research options, not claimed CleanNav capabilities. Coverage swaths
are not equivalent to a closed perimeter patrol. A future implementation needs
separate evaluation and explicit integration authorization.

## Contract

Input includes checksum-bound map identity, OccupancyGrid geometry and cell
values, a boundary and Keepout geometry (holes supported), full vehicle
Footprint, minimum radius, design margin and safety margin. All map poses and
footprints must use the existing route vehicle reference convention.

Output reserves existing `route.json` fields: map, route_id, dense_path_map,
waypoints, mission_checkpoints and validation report. No fake route is emitted.
SMAC and motion flags cannot become true from this interface; a later adapter
would preserve current Registry/schema and require independent verification.
No new ROS parameters, interfaces or runtime Gate are defined.
