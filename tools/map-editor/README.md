# CleanNav map editor — regular-site R9 candidate

Offline map-bound region editing and perimeter generation. No ROS node, vehicle
command publisher, Mission/Gateway client or movement authorization exists here.
The original UI and API are retained. R8-A4 remains the only dynamically validated
baseline; R9 is a fixed-start, forward-route **offline candidate**.

## Source and local use

The former non-Git `cleannav-tools/map-editor` is restored into this repository's
`tools/map-editor`. Python dependencies are listed in `requirements.txt`; this
change installed none. Use Ubuntu22.04 / Python3.10 with NumPy, Pillow, PyYAML,
Shapely2.0.7 and jsonschema4.23.0. Existing validation helpers are bundled verbatim
in `support`, with original source SHA in `support/SOURCES.json`; these are not
new runtime packages or altered Mission interfaces.

From this Git checkout:

```bash
python3 tools/map-editor/server.py --map demo-fixture
python3 -m pytest -q tools/map-editor/tests tests/patrol
```

The local editor binds loopback at `http://localhost:8765`. This is an offline
editor API, not the vehicle Gateway. Ctrl+C stops it. The demo is explicitly
synthetic/test-only. A production saved-map bundle still needs its original
manifest/database/OccupancyGrid/YAML/Profile/Keepout assets; these are not replaced
by derived test fixtures. In the established CleanNav workspace, saved maps are
discovered by the unchanged existing discovery flow. The old default Campus
export is known to fail current image/Occupancy validation; do not overwrite that
frozen map or treat it as a current valid production input.

## Regular-site geometry and constraints

The existing pipeline retains free-region extraction, largest interior rectangle,
simplification, inward buffer, tangent rounded loops, .025m sampling and closure.
Only the design radius factor changes from1.03 to1.20. Vehicle geometry is read
from the existing frozen `cleannav_navigation/config/nav2_local_recovery.yaml`:
minimum turn radius1.12m, full Footprint1.10×.68m. The design radius is1.344m
(20% margin), maximum circle curvature .74404762m⁻¹.

The nominal buffer remains .8166065264130885m = .6466065264130885m vehicle
circumscribed radius + .15m safety margin + .02m existing allowance. Complete
oriented Footprint and +.15m swept geometry are checked separately. Straight
support lines remain unchanged; circular/straight joins are G1 tangent-continuous,
not G2 curvature-continuous. Larger radius is not automatically a better patrol.

R9 C has1637 poses; existing A4 source computes `((0,939),(938,1637))` from the
fixed production start/forward route. Neither938 nor the radius is hard-coded
into A4. Arbitrary start/reverse routes are outside this candidate's product
scope; the prior near-straight/exact-straight counterexamples remain recorded.
No A4, MPPI, Goal Checker, speed, Safety, Keepout or public interface changes.

## Export and validation scope

Draw/finish a region, generate a loop, place a charger staging pose and export the
existing RegionRegistry v1. The owner export uses existing tangent checkpoints;
the reviewed PATROL asset then uses the existing arc-length checkpoints and
`path_file` dense-route binding. R8-A4 Mission Manager performs GOTO then admits
the dense PATROL path through Facade into two finite FollowPath segments; the
older independent-checkpoint execution note in the legacy validator is not a
claim about this current runtime chain.

[Current result](../../docs/validation/R9_REGULAR_SITE_RESULT.md) records real-map
sweeps, new entry, Registry and actual SMAC static replay. Geometry/static planning
PASS is not tracking/full-lap PASS. `motion_validated=false` remains. No startup
runner, authorization, runtime Gate or dynamic task was added.

## Reproducible fixtures and known history

`tests/fixtures/SOURCES.json` records SHA256 of every exact gzip capture from the
real Pillar map/Keepout, legacy Campus export and projection reference. No RTAB DB
is bundled; a test-only model uses decoded captured grids/masks. It never claims
to validate a production map DB or bypasses production MapModel loading. Synthetic
demo tests remain explicitly labelled separately.

The10 historical setup errors came from eager loading the old Campus default:
52550 unknown cells used gray205, interpreted as free at free_thresh=.25;24677
free cells used254 instead of the validator's255, yielding77227 exact-image
mismatches. Tests now isolate inputs, reproduce that failure from exact captured
bytes, and exercise the real current site without mutating historical assets.
Original failure logs/SHAs remain in the validation archive. Geometry assertions
were retained; only fixture binding/provenance classification changed.

## Future Extensions

[Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) (BSD-3-Clause) is a
research reference for non-convex areas, obstacle-aware decomposition, region
connections and feasible/continuous-curvature turns. See the [future contract and
feature mapping](future/README.md). `future/irregular_site_planner.py` is
**NOT_INTEGRATED / FUTURE_EXTENSION**: an input/output convention, no algorithm.
It always raises NotImplementedError and is never imported by core/UI/ROS. No
Fields2Cover/GDAL/GEOS/OR-Tools dependency was installed or integrated.
