"""NOT_INTEGRATED / FUTURE_EXTENSION — interface only, no planning implementation.

Reference (no third-party source copied):
https://github.com/Fields2Cover/Fields2Cover — BSD-3-Clause
G. Mier, J. Valente, S. de Bruin (2023), Fields2Cover: An Open-Source
Coverage Path Planning Library for Unmanned Agricultural Vehicles,
IEEE RA-L 8(4), 2166–2172. https://doi.org/10.1109/LRA.2023.3248439

Potential mappings, not implemented here:
* non-convex boundaries/internal obstacles -> cell geometry + decomposition;
* trapezoidal/boustrophedon cells -> decomposition module;
* inter-cell connections -> route planning/headland corridors;
* bounded-radius connections -> Dubins (forward) / Reeds-Shepp (reverse);
* bounded curvature change -> continuous-curvature variants.
See README.md for official tutorial links and integration limitations.

All positions are metres in the map frame; yaw is radians at the same vehicle
reference point as the existing route.json. Occupancy data is bottom-row-first,
row-major {-1,0..100}; unknown is not free. Footprint coordinates use the
existing vehicle reference frame. The request carries the original map
identity; an implementation must never infer motion approval from geometry.
This file imports only the standard library and is not imported by core/UI/ROS.
"""
from typing import Literal, TypedDict

STATUS = "NOT_INTEGRATED / FUTURE_EXTENSION"


class MapIdentity(TypedDict):
    map_id: str
    revision: str
    sha256: str
    frame_id: str


class OccupancyGrid(TypedDict):
    frame_id: str
    width: int
    height: int
    resolution: float
    origin: list[float]  # [x, y, yaw]
    data: list[int]      # width * height cells


class PlannerInput(TypedDict):
    map: MapIdentity
    occupancy_grid: OccupancyGrid
    boundary_map: list[list[float]]  # non-convex outer boundary, [x,y]
    keepout_geojson: dict            # Polygon/MultiPolygon, including holes
    footprint: list[list[float]]     # full polygon, not only a disk radius
    min_turning_radius_m: float
    safety_margin_m: float
    design_radius_margin: float     # e.g. .20; does not alter vehicle constraint
    allow_reverse: bool             # future RS only; current PATROL is forward


class Pose(TypedDict):
    x: float
    y: float
    yaw: float


class RouteDocument(TypedDict):
    # Compatible geometry fields reserved from the current route.json.
    # A future adapter must also preserve its provenance/metadata and Registry.
    map: MapIdentity
    route_id: str
    dense_path_map: list[Pose]
    waypoints: list[list[float]]
    mission_checkpoints: list[Pose]
    geometry_validated: bool
    smac_validated: Literal[False]
    motion_validated: Literal[False]
    report: dict


def plan_irregular_site(request: PlannerInput) -> RouteDocument:
    """Future contract only. Never return a fake route or implicit validation.

    A backend would need to check full padded footprint sweeps, Keepout,
    closure, heading, curvature/rate, entry reachability and execution support.
    Decomposition/coverage output is not automatically a single closed patrol
    loop; reverse/cusp/multi-region semantics need separate review. No backend,
    third-party dependency, file output, ROS endpoint or registration exists.
    """
    raise NotImplementedError(STATUS)
