"""Offline map-bound perimeter planning; no ROS node or movement interface."""
from __future__ import annotations
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile

TOOL = Path(__file__).resolve().parent
ROOT = next((parent for parent in TOOL.parents if (parent / "PROJECT_MASTER.md").is_file()),
            TOOL.parents[1])
sys.path.insert(0, str(TOOL / '.deps'))
import numpy as np
from PIL import Image
import yaml
from map_codec import decode_trinary
from shapely import affinity
from shapely.geometry import Polygon, LineString, Point, box
from shapely.ops import unary_union

REPO = TOOL.parents[1]
SRC = ROOT / 'cleannav-worktrees/pc-sim-v2-mapping-localization/src'
SIM = SRC / 'cleannav-navigation/cleannav_simulation'
DEFAULT_MAP = ROOT / 'cleannav-experiments/campus-curb-safety-20261012/maps/campus_curb_scan_20261012_083555_fd1b1b'
DEFAULT_PROFILE = SIM / 'config/campus/security-profile.json'
DEMO_MAP = TOOL / 'fixtures/perimeter_demo'
from support.region_registry import RegionRegistry


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vehicle():
    path = REPO / 'cleannav_navigation/config/nav2_local_recovery.yaml'
    params = yaml.safe_load(path.read_text())
    footprint = json.loads(params['global_costmap']['global_costmap']['ros__parameters']['footprint'])
    radius = params['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']
    return dict(footprint=footprint, turning_radius=float(radius),
                disk_radius=max(math.hypot(*xy) for xy in footprint),
                source=str(path.relative_to(REPO)), source_sha256=digest(path))


def grid_geometry(mask, resolution, origin):
    """Union complete cell squares, not point samples; row zero is bottom."""
    cells = []
    for y, row in enumerate(mask):
        transitions = np.diff(np.r_[False, row, False].astype(np.int8))
        for x0, x1 in zip(np.flatnonzero(transitions == 1), np.flatnonzero(transitions == -1)):
            cells.append(box(x0*resolution, y*resolution, x1*resolution, (y+1)*resolution))
    geometry = unary_union(cells)
    return affinity.translate(affinity.rotate(geometry, origin[2], origin=(0, 0), use_radians=True), origin[0], origin[1])


class MapModel:
    def __init__(self, directory=DEFAULT_MAP, profile=None):
        directory = Path(directory).resolve()
        # Gazebo scene bundles may carry an occupancy projection generated
        # directly from the SDF collision geometry. Keep this provenance
        # separate from both RTAB saved maps and the editor-only demo fixture.
        # It is useful for offline planner validation, but is never described
        # as sensor-observed localization data.
        if (directory / 'manifest.json').is_file():
            candidate = json.loads((directory / 'manifest.json').read_text())
            if candidate.get('source_type') == 'gazebo_world_collision_projection':
                self._load_world_projection(directory, candidate)
                return
        # The bundled demo is a synthetic OccupancyGrid fixture, never a saved
        # RTAB-Map map. Keep the branch explicit so it cannot be mistaken for
        # a production map by the editor or its generated registry.
        if (directory / 'manifest.json').is_file():
            candidate = json.loads((directory / 'manifest.json').read_text())
            if candidate.get('editor_fixture') is True:
                self._load_fixture(directory, candidate)
                return
        spec = importlib.util.spec_from_file_location('existing_map_assets', TOOL/'support/mapping_assets.py')
        assets = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(assets)
        assets.validate_map(directory)
        m = json.loads((directory/'manifest.json').read_text())
        g = json.loads((directory/'occupancy_grid.json').read_text())
        self.directory = directory
        self.identity = dict(map_id=m['name'], revision=m['created_at'],
                             sha256=m['sha256']['occupancy_grid.json'], frame_id='map')
        self.width, self.height = g['width'], g['height']
        self.resolution, self.origin = float(g['resolution']), list(g['origin'])
        self.data = np.array(g['data'], dtype=np.int16).reshape(self.height, self.width)
        self.free = grid_geometry((self.data >= 0) & (self.data <= 25), self.resolution, self.origin)
        self.keepout = Polygon()
        self.profile_sha256 = None
        if directory == DEFAULT_MAP.resolve() and profile is None:
            profile = DEFAULT_PROFILE
        if profile is None:
            # Newly recorded maps may carry a separate, checksum-bound formal
            # Keepout profile. Apply it only when the profile names this exact
            # map directory; never select a profile by filename alone.
            for candidate in (SIM/'config').rglob('*profile*.json'):
                try:
                    record = json.loads(candidate.read_text())
                    if Path(record.get('map_directory', '')).resolve() == directory:
                        profile = candidate
                        break
                except (OSError, json.JSONDecodeError, TypeError):
                    continue
        if profile:
            profile = Path(profile)
            p = json.loads(profile.read_text())
            if not p['verified'] or Path(p['map_directory']).resolve() != directory:
                raise ValueError('Keepout profile does not match the selected map')
            for name, expected in p['map_hashes'].items():
                if digest(directory/name) != expected:
                    raise ValueError(f'Keepout map checksum mismatch: {name}')
            mask_yaml = profile.parent/p['mask_yaml']
            meta = yaml.safe_load(mask_yaml.read_text())
            image = mask_yaml.parent/meta['image']
            for path in (mask_yaml, image):
                if digest(path) != p['keepout_hashes'][path.name]:
                    raise ValueError(f'Keepout checksum mismatch: {path.name}')
            pixels = np.flipud(np.array(Image.open(image).convert('L')))
            occupied = (1-pixels.astype(float)/255) > meta['occupied_thresh']
            self.keepout = grid_geometry(occupied, meta['resolution'], meta['origin'])
            self.free = self.free.difference(self.keepout)
            self.profile_sha256 = digest(profile)
        self.vehicle = vehicle()
        self.test_only = False
        self.provenance = 'RTAB-Map saved OccupancyGrid'

    def _load_world_projection(self, directory, manifest):
        """Validate a collision-geometry projection bound to its source SDF."""
        required = ('map.yaml', 'map.pgm', 'occupancy_grid.json')
        assets = manifest.get('sha256', {})
        for name in required:
            path = directory / name
            if not path.is_file() or path.is_symlink() or digest(path) != assets.get(name):
                raise ValueError(f'Gazebo projection asset missing or checksum mismatch: {name}')
        world_rel = manifest.get('source_world')
        if not isinstance(world_rel, str) or not world_rel:
            raise ValueError('Gazebo projection is missing its source SDF path')
        world_path = (directory / world_rel).resolve(strict=True)
        if digest(world_path) != manifest.get('source_world_sha256'):
            raise ValueError('Gazebo projection source SDF checksum mismatch')
        g = json.loads((directory / 'occupancy_grid.json').read_text())
        meta = yaml.safe_load((directory / 'map.yaml').read_text())
        width, height = int(g['width']), int(g['height'])
        if (width < 2 or height < 2 or len(g['data']) != width * height
                or g.get('frame_id') != 'map' or float(g['resolution']) <= 0
                or len(g.get('origin', [])) != 3):
            raise ValueError('Gazebo projection OccupancyGrid metadata is invalid')
        values = np.asarray(g['data'], dtype=np.int16)
        if (meta.get('image') != 'map.pgm' or meta.get('mode') != 'trinary'
                or int(meta.get('negate', 1)) != 0
                or not math.isclose(float(meta.get('occupied_thresh', -1)), 0.65, abs_tol=1e-9)
                or not math.isclose(float(meta.get('free_thresh', -1)), 0.25, abs_tol=1e-9)
                or not math.isclose(float(meta['resolution']), float(g['resolution']), abs_tol=1e-8)
                or len(meta.get('origin', [])) != 3
                or any(not math.isclose(float(a), float(b), abs_tol=1e-8)
                       for a, b in zip(meta['origin'], g['origin']))):
            raise ValueError('Gazebo projection ROS map YAML differs from OccupancyGrid')
        image = np.array(Image.open(directory / 'map.pgm').convert('L'))
        if image.shape != (height, width):
            raise ValueError('Gazebo projection PGM dimensions differ from OccupancyGrid')
        decoded = decode_trinary(image, free_thresh=meta['free_thresh'],
                                 occupied_thresh=meta['occupied_thresh'])
        if not np.array_equal(decoded, values.reshape(height, width)):
            raise ValueError('Gazebo projection PGM/YAML loses occupancy or unknown cells; regenerate an independent corrected bundle (gazebo-patrol-v2), leaving historical assets unchanged')
        identity = manifest.get('identity', {})
        expected_sha = digest(directory / 'occupancy_grid.json')
        if (identity.get('sha256') != expected_sha or identity.get('frame_id') != 'map'
                or not identity.get('map_id') or not identity.get('revision')):
            raise ValueError('Gazebo projection map identity is incomplete or stale')
        if np.any(~np.isin(values, [-1, 0, 100])):
            raise ValueError('Gazebo projection contains occupancy values outside {-1,0,100}')
        self.directory = directory
        self.identity = dict(map_id=identity['map_id'], revision=identity['revision'],
                             sha256=identity['sha256'], frame_id='map')
        self.width, self.height = width, height
        self.resolution, self.origin = float(g['resolution']), list(g['origin'])
        self.data = values.reshape(height, width)
        self.free = grid_geometry(self.data == 0, self.resolution, self.origin)
        self.keepout = Polygon()
        self.profile_sha256 = None
        self.vehicle = vehicle()
        self.test_only = False
        self.provenance = 'Gazebo SDF collision-geometry projection (not RTAB sensor map)'

    def _load_fixture(self, directory, manifest):
        gpath = directory/'occupancy_grid.json'
        if digest(gpath) != manifest.get('sha256', {}).get('occupancy_grid.json'):
            raise ValueError('Demo fixture OccupancyGrid checksum mismatch')
        g = json.loads(gpath.read_text())
        width, height = int(g['width']), int(g['height'])
        if width < 2 or height < 2 or len(g['data']) != width*height:
            raise ValueError('Demo fixture OccupancyGrid dimensions are invalid')
        self.directory = directory
        self.identity = dict(map_id=manifest['name'], revision=manifest['created_at'],
                             sha256=manifest['sha256']['occupancy_grid.json'], frame_id='map')
        self.width, self.height = width, height
        self.resolution, self.origin = float(g['resolution']), list(g['origin'])
        self.data = np.array(g['data'], dtype=np.int16).reshape(height, width)
        self.free = grid_geometry((self.data >= 0) & (self.data <= 25), self.resolution, self.origin)
        self.keepout = Polygon()
        self.profile_sha256 = None
        self.vehicle = vehicle()
        self.test_only = True

    def pixel_to_map(self, u, v):
        """Canvas image-edge coordinates; v increases down, origin is cell edge."""
        x, y = u*self.resolution, (self.height-v)*self.resolution
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return [self.origin[0]+c*x-s*y, self.origin[1]+s*x+c*y]

    def map_to_pixel(self, x, y):
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        x, y = x-self.origin[0], y-self.origin[1]
        return [(c*x+s*y)/self.resolution, self.height-(-s*x+c*y)/self.resolution]

    def image(self):
        free = (self.data >= 0) & (self.data <= 25)
        occupied = self.data >= 65
        pixels = np.where(free, 245, np.where(occupied, 35, 150)).astype('uint8')
        out = io.BytesIO()
        Image.fromarray(np.flipud(pixels)).save(out, format='PNG')
        return out.getvalue()

    def info(self):
        keepout = []
        geoms = [self.keepout] if self.keepout.geom_type == 'Polygon' else list(self.keepout.geoms)
        for polygon in geoms:
            if not polygon.is_empty:
                keepout.append(list(polygon.exterior.coords))
        return dict(identity=self.identity, width=self.width, height=self.height,
                    resolution=self.resolution, origin=self.origin, vehicle=self.vehicle,
                    keepout=keepout, profile_sha256=self.profile_sha256,
                    defaults=dict(margin=0.15, direction='ccw', laps=3),
                    directory=str(self.directory), observed_free_area_m2=self.free.area,
                    test_only=self.test_only,
                    provenance=getattr(self, 'provenance',
                                        'TEST FIXTURE' if self.test_only else 'RTAB-Map saved OccupancyGrid'),
                    formal_keepout_loaded=self.profile_sha256 is not None,
                    map_label=('TEST FIXTURE — not a real scanned map' if self.test_only else
                               getattr(self, 'provenance', 'Saved OccupancyGrid map')))


def rounded_loop(vertices, radius, direction='ccw', step=0.025):
    """Exact tangent line/circle fillets, including signed concave turns."""
    vertices = np.asarray(vertices, dtype=float)
    if len(vertices) < 3:
        raise ValueError('Too few contour corners')
    n = len(vertices)
    arcs = []
    for i, v in enumerate(vertices):
        incoming = v-vertices[(i-1) % n]
        outgoing = vertices[(i+1) % n]-v
        li, lo = np.linalg.norm(incoming), np.linalg.norm(outgoing)
        if min(li, lo) < 1e-6:
            raise ValueError('Duplicate contour vertices')
        a, b = incoming/li, outgoing/lo
        delta = math.atan2(np.cross(a, b), np.dot(a, b))
        if abs(delta) > math.pi-0.01:
            raise ValueError('Contour has a reversal cusp')
        distance = radius*math.tan(abs(delta)/2)
        tin, tout = v-a*distance, v+b*distance
        sign = 1 if delta >= 0 else -1
        center = tin+sign*radius*np.array([-a[1], a[0]])
        arcs.append((tin, tout, center, delta, distance, a, b))
    for i in range(n):
        length = np.linalg.norm(vertices[(i+1) % n]-vertices[i])
        if arcs[i][4]+arcs[(i+1) % n][4] > length-0.02:
            raise ValueError('Not enough edge length for minimum-radius turns')
    dense, checkpoints = [], []
    for i, (tin, tout, center, delta, _, a, b) in enumerate(arcs):
        previous = arcs[(i-1) % n][1]
        count = max(1, math.ceil(np.linalg.norm(tin-previous)/step))
        yaw = math.atan2(a[1], a[0])
        for t in np.linspace(0, 1, count, endpoint=False):
            p = previous+(tin-previous)*t
            dense.append([float(p[0]), float(p[1]), yaw])
        checkpoints.append([float(tin[0]), float(tin[1]), yaw])
        if abs(delta) > 1e-5:
            angle = math.atan2(tin[1]-center[1], tin[0]-center[0])
            count = max(1, math.ceil(abs(delta)*radius/step))
            for t in np.linspace(0, 1, count, endpoint=False):
                angle_t = angle+delta*t
                p = center+radius*np.array([math.cos(angle_t), math.sin(angle_t)])
                dense.append([float(p[0]), float(p[1]), yaw+delta*t])
            checkpoints.append([float(tout[0]), float(tout[1]), math.atan2(b[1], b[0])])
    # Start on a straight-to-arc tangent: entrance and loop closing pose coincide.
    if direction == 'cw':
        dense = [[x, y, math.atan2(math.sin(yaw+math.pi), math.cos(yaw+math.pi))]
                 for x, y, yaw in reversed(dense)]
        checkpoints = [[x, y, math.atan2(math.sin(yaw+math.pi), math.cos(yaw+math.pi))]
                       for x, y, yaw in reversed(checkpoints)]
    # Rotate dense to the first checkpoint rather than introducing a connector.
    nearest = min(range(len(dense)), key=lambda i: math.dist(dense[i][:2], checkpoints[0][:2]))
    dense = dense[nearest:]+dense[:nearest]
    dense.append(dense[0][:])
    checkpoints.append(checkpoints[0][:])
    return dense, checkpoints


def validate_loop(model, boundary, dense, margin, radius):
    polygon = Polygon(boundary)
    if not polygon.is_valid or polygon.area < 0.01:
        raise ValueError('Invalid or self-intersecting region polygon')
    if len(dense) < 4 or math.dist(dense[0][:2], dense[-1][:2]) > 1e-6:
        raise ValueError('Loop is not closed')
    points = np.asarray(dense)
    distances = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    angles = np.abs(np.arctan2(np.sin(np.diff(points[:, 2])), np.cos(np.diff(points[:, 2]))))
    if np.any(distances <= 1e-9):
        raise ValueError('Zero length path sample')
    # Arc chord/yaw ratio differs slightly from analytic 1/R; check with discretization bound.
    curvature = angles/distances
    if curvature.max() > 1/radius * 1.001:
        raise ValueError('Curvature exceeds vehicle turning capability')
    path = LineString(points[:, :2])
    if not path.is_simple:
        raise ValueError('Loop self-intersects')
    # Circumscribed disk dominates all orientations of the actual footprint.
    # Additional chord sagitta and buffer polygon discretization guard cover intersample motion.
    disk = model.vehicle['disk_radius']
    envelope = path.buffer(disk+margin+0.005, quad_segs=32)
    if not polygon.covers(envelope):
        raise ValueError('Vehicle swept envelope crosses the region boundary')
    if not model.free.covers(envelope):
        raise ValueError('Vehicle swept envelope hits obstacle, Keepout, unknown or map edge')
    return dict(geometry_pass=True, kinematics_pass=True, motion_validated=False,
                planner_segments_validated=False, minimum_radius_m=radius,
                max_sample_curvature=float(curvature.max()), length_m=float(distances.sum()),
                footprint_disk_radius_m=disk, safety_margin_m=margin,
                checked_samples=len(dense), closure_error_m=0,
                execution_note='Mission uses independent NavigateToPose goals and stops/releases Lease at each segment. Smac paths and tracking still require separate validation.')


def plan(model, boundary, margin=0.15, direction='ccw', laps=3):
    if not 0.05 <= float(margin) <= 0.60:
        raise ValueError('Safety margin must be 0.05–0.60m')
    if direction not in ('cw', 'ccw') or type(laps) is not int or not 1 <= laps <= 10:
        raise ValueError('Invalid direction or laps')
    if not isinstance(boundary, list) or not 3 <= len(boundary) <= 256:
        raise ValueError('Draw 3–256 polygon vertices')
    points = np.asarray(boundary, dtype=float)
    if points.shape != (len(boundary), 2) or not np.isfinite(points).all():
        raise ValueError('Region coordinates must be finite pairs')
    region = Polygon(points)
    if not region.is_valid or region.area < 0.01:
        raise ValueError('Polygon is invalid, self-intersecting or degenerate')
    radius = model.vehicle['turning_radius']*1.20
    clearance = model.vehicle['disk_radius']+float(margin)+0.02
    navigable = region.intersection(model.free).buffer(-clearance)
    if navigable.is_empty:
        return dict(feasible=False, reason='No space remains after footprint, margin, obstacles and unknown exclusion')
    failures, candidates = [], []
    # Test increasingly interior contours; the shallowest safe contour wins.
    for inset in (0, 0.15, 0.30, 0.50, 0.75, 1.0):
        interior = navigable.buffer(-inset)
        components = [interior] if interior.geom_type == 'Polygon' else list(getattr(interior, 'geoms', []))
        for component in sorted(components, key=lambda p: p.area, reverse=True)[:6]:
            if component.is_empty or component.geom_type != 'Polygon':
                continue
            for tolerance in (0.03, 0.08, 0.15, 0.25, 0.40):
                contour = component.simplify(tolerance, preserve_topology=True)
                coords = list(contour.exterior.coords)[:-1]
                # Normalize CCW before applying signed fillets.
                if not contour.exterior.is_ccw:
                    coords.reverse()
                try:
                    dense, waypoints = rounded_loop(coords, radius, direction)
                    report = validate_loop(model, boundary, dense, float(margin), radius)
                    expanded = 1 + laps*(len(waypoints)-1) + 2
                    if len(waypoints) > 64 or expanded > 32:
                        raise ValueError(f'Mission 32-goal limit: selected {laps} laps + GOTO + RETURN need {expanded} goals')
                    report.update(checkpoint_count=len(waypoints), sequence_goals_with_goto_return=expanded,
                                  selected_inset_m=inset, direction=direction,
                                  connector='Entry is the first route pose; no separate internal connector',
                                  approach_from_robot_validated=False)
                    candidates.append(dict(feasible=True, waypoints=waypoints, dense=dense,
                                           report=report, entry_pose=waypoints[0],
                                           reason='Geometry and turning radius checked; movement and Smac segment tracking not validated'))
                except ValueError as exc:
                    failures.append(str(exc))
        if candidates:
            return max(candidates, key=lambda item: item['report']['length_m'])
    return dict(feasible=False, reason='No checked loop found: '+('; '.join(dict.fromkeys(failures))[:800] or 'region too narrow/disconnected'),
                searched_candidates=len(failures), not_a_proof_of_nonexistence=True)


def validate_registry(model, data):
    import jsonschema
    schema = json.loads((TOOL/'support/region_registry.schema.json').read_text())
    validator = jsonschema.Draft202012Validator(schema)
    validator.validate(data)
    if data['map'] != model.identity:
        raise ValueError('Registry map id/revision/SHA does not match loaded map')
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', dir='/tmp') as tmp:
        json.dump(data, tmp); tmp.flush()
        registry = RegionRegistry.load(tmp.name)
        registry.require_map(model.identity['map_id'], model.identity['revision'], model.identity['sha256'])
    for charger in data['chargers']:
        p = charger['staging_pose']
        if not model.free.covers(Point(p['x'], p['y']).buffer(model.vehicle['disk_radius']+0.15+0.005)):
            raise ValueError(f"Charger {charger['charger_id']} has insufficient footprint clearance")
    return data


def export_registry(model, project):
    if project.get('map') != model.identity:
        raise ValueError('Project map identity mismatch; explicitly select its original map')
    regions = []
    reports = []
    for region in project['regions']:
        result = plan(model, region['boundary'], **region.get('options', {}))
        if not result['feasible']:
            raise ValueError(f"Region {region['region_id']}: {result['reason']}")
        pose = lambda p: dict(x=p[0], y=p[1], yaw=p[2])
        regions.append(dict(region_id=region['region_id'], display_name=region['display_name'], aliases=[],
                            boundary=region['boundary'], entry_pose=pose(result['entry_pose']),
                            routes=[dict(route_id=region.get('route_id', 'perimeter'),
                                         waypoints=[pose(p) for p in result['waypoints']])]))
        reports.append(dict(region_id=region['region_id'], **result['report']))
    data = dict(schema_version=1, map=model.identity, regions=regions, chargers=project.get('chargers', []))
    return dict(registry=validate_registry(model, data), validation=reports,
                status='OFFLINE_GEOMETRY_ONLY_NOT_MOTION_ACCEPTED')


def _oriented_footprint(pose, footprint, margin):
    """Conservative polygon footprint at an SE(2) pose, expanded by margin."""
    x, y, yaw = map(float, pose[:3])
    c, s = math.cos(yaw), math.sin(yaw)
    vertices = [(x+c*px-s*py, y+s*px+c*py) for px, py in footprint]
    return Polygon(vertices).buffer(margin, quad_segs=8, join_style='mitre')


def _swept_oriented_footprint(poses, model, margin):
    """Cover the oriented rectangle through every returned path segment.

    The convex hull between consecutive expanded footprints covers translation
    between poses. Dense Smac output bounds the remaining rotational sagitta;
    a 3mm pad covers that interpolation and polygonization error.
    """
    samples = [_oriented_footprint(p, model.vehicle['footprint'], margin) for p in poses]
    pieces = list(samples)
    for left, right in zip(samples[:-1], samples[1:]):
        pieces.append(left.union(right).convex_hull.buffer(.003, quad_segs=8))
    return unary_union(pieces)


def _geometric_curvatures(points):
    """Estimate curvature from path geometry, independent of yaw quantization."""
    values = []
    for a, b, c in zip(points[:-2], points[1:-1], points[2:]):
        ab = math.dist(a[:2], b[:2])
        bc = math.dist(b[:2], c[:2])
        ac = math.dist(a[:2], c[:2])
        if min(ab, bc, ac) <= 1e-8:
            raise ValueError('Smac path contains duplicate neighboring positions')
        twice_area = abs((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))
        values.append(2.0*twice_area/(ab*bc*ac))
    return values


def check_planner_path(model, boundary, candidate_dense, actual_poses, corridor=0.25,
                       require_forward=False):
    """Offline adjudication of saved Smac output; never equates waypoints to executed paths."""
    corridor = float(corridor)
    points = np.asarray(actual_poses, dtype=float)
    reference_points = np.asarray(candidate_dense, dtype=float)
    polygon = Polygon(boundary)
    if not 0.05 <= corridor <= 2.0:
        raise ValueError('Planner path corridor must be 0.05–2.0m')
    if not polygon.is_valid or polygon.area < 0.01:
        raise ValueError('Invalid region polygon')
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('Smac path requires at least two finite [x,y,yaw] poses')
    if reference_points.ndim != 2 or reference_points.shape[1] < 2 or len(reference_points) < 4 or not np.isfinite(reference_points).all():
        raise ValueError('Reference loop requires at least four finite poses')
    segment_lengths = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    if np.any(segment_lengths <= 1e-8):
        raise ValueError('Smac path contains duplicate consecutive positions')
    curvature = _geometric_curvatures(points)
    curvature_limit = 1.0/model.vehicle['turning_radius']
    actual = LineString([p[:2] for p in actual_poses])
    reference = LineString([p[:2] for p in candidate_dense])
    envelope = _swept_oriented_footprint(points, model, .15)
    tangent_alignment = []
    for left, right in zip(points[:-1], points[1:]):
        tangent = math.atan2(right[1]-left[1], right[0]-left[0])
        mid_yaw = left[2] + .5*math.atan2(math.sin(right[2]-left[2]),
                                          math.cos(right[2]-left[2]))
        tangent_alignment.append(math.cos(math.atan2(math.sin(mid_yaw-tangent),
                                                       math.cos(mid_yaw-tangent))))
    forward_pass = all(alignment >= 0.0 for alignment in tangent_alignment)
    return dict(obstacle_clear=bool(model.free.covers(envelope)),
                stays_in_region=bool(polygon.covers(envelope)),
                stays_in_route_corridor=bool(reference.buffer(corridor).covers(actual)),
                turning_radius_pass=bool(not curvature or max(curvature) <= curvature_limit*1.05),
                forward_motion_pass=bool(forward_pass),
                reverse_or_sideways_segments=sum(alignment < 0.0 for alignment in tangent_alignment),
                max_sample_curvature=float(max(curvature, default=0.0)),
                curvature_limit=curvature_limit,
                footprint_check='oriented_rectangle_swept_with_0.15m_margin',
                motion_validated=False)
