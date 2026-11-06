"""Map-version-bound regions, reviewed patrol routes and charger staging poses."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Mapping

import yaml


_ID = re.compile(r'^[A-Za-z0-9._:-]{1,64}$')
_REVISION = re.compile(r'^[A-Za-z0-9._:+-]{1,64}$')
_SHA256 = re.compile(r'^[A-Fa-f0-9]{64}$')
_MAX_REGIONS = 128
_MAX_CHARGERS = 32
_MAX_ROUTE_POINTS = 64
_MAX_PATH_POSES = 4096


class RegionRegistryError(ValueError):
    """A registry cannot safely resolve a business navigation target."""


@dataclass(frozen=True)
class MapIdentity:
    """Immutable identity of the map against which all poses were authored."""

    map_id: str
    revision: str
    sha256: str
    frame_id: str = 'map'

    @property
    def configured(self) -> bool:
        """Whether this identity is complete enough for navigation use."""
        return bool(self.map_id and self.revision and _SHA256.fullmatch(self.sha256))


@dataclass(frozen=True)
class RegionRoute:
    """A reviewed closed checkpoint loop, plus the offline path when one exists."""

    route_id: str
    waypoints: tuple[tuple[float, float, float], ...]
    path_poses: tuple[tuple[float, float, float], ...] = ()


@dataclass(frozen=True)
class RegionDefinition:
    """A named region and its explicitly authored navigation data."""

    region_id: str
    display_name: str
    aliases: tuple[str, ...]
    boundary: tuple[tuple[float, float], ...]
    entry_pose: tuple[float, float, float]
    routes: Mapping[str, RegionRoute]


@dataclass(frozen=True)
class ChargerDefinition:
    """A charger navigation staging pose, not a docking/charging claim."""

    charger_id: str
    display_name: str
    staging_pose: tuple[float, float, float]


class RegionRegistry:
    """Validated immutable snapshot of region and charger configuration."""

    def __init__(
        self,
        identity: MapIdentity,
        regions: Mapping[str, RegionDefinition],
        chargers: Mapping[str, ChargerDefinition],
        *,
        source_sha256: str,
    ) -> None:
        self.identity = identity
        self.regions = dict(regions)
        self.chargers = dict(chargers)
        self.source_sha256 = source_sha256

    @classmethod
    def load(cls, path: str | Path) -> 'RegionRegistry':
        """Load and geometrically validate a YAML registry."""
        source = Path(path)
        try:
            raw_bytes = source.read_bytes()
            raw = yaml.safe_load(raw_bytes)
        except OSError as exc:
            raise RegionRegistryError(f'registry unavailable: {exc}') from exc
        except yaml.YAMLError as exc:
            raise RegionRegistryError(f'registry YAML invalid: {exc}') from exc
        if not isinstance(raw, dict) or raw.get('schema_version') != 1:
            raise RegionRegistryError('schema_version must be 1')
        if set(raw) != {'schema_version', 'map', 'regions', 'chargers'}:
            raise RegionRegistryError('registry has missing or unknown top-level fields')
        map_raw = raw.get('map')
        if not isinstance(map_raw, dict) or set(map_raw) != {
            'map_id', 'revision', 'sha256', 'frame_id'
        }:
            raise RegionRegistryError('map identity must be a mapping')
        identity = MapIdentity(
            map_id=_string(map_raw.get('map_id', ''), 'map.map_id', 64),
            revision=_string(map_raw.get('revision', ''), 'map.revision', 64),
            sha256=_string(map_raw.get('sha256', ''), 'map.sha256', 64),
            frame_id=_string(map_raw.get('frame_id', ''), 'map.frame_id', 32),
        )
        if identity.frame_id != 'map':
            raise RegionRegistryError('only frame_id=map is supported')
        if identity.map_id and _ID.fullmatch(identity.map_id) is None:
            raise RegionRegistryError('map.map_id is invalid')
        if identity.revision and _REVISION.fullmatch(identity.revision) is None:
            raise RegionRegistryError('map.revision is invalid')
        if identity.sha256 and _SHA256.fullmatch(identity.sha256) is None:
            raise RegionRegistryError('map.sha256 must be a 64-digit SHA-256')
        region_items = raw.get('regions', [])
        charger_items = raw.get('chargers', [])
        if not isinstance(region_items, list) or len(region_items) > _MAX_REGIONS:
            raise RegionRegistryError('regions must be a bounded list')
        if not isinstance(charger_items, list) or len(charger_items) > _MAX_CHARGERS:
            raise RegionRegistryError('chargers must be a bounded list')
        if region_items or charger_items:
            if not identity.configured:
                raise RegionRegistryError('executable targets require complete map identity')
        regions: dict[str, RegionDefinition] = {}
        for item in region_items:
            region = _parse_region(item, source.parent)
            if region.region_id in regions:
                raise RegionRegistryError(f'duplicate region_id={region.region_id}')
            regions[region.region_id] = region
        chargers: dict[str, ChargerDefinition] = {}
        for item in charger_items:
            charger = _parse_charger(item)
            if charger.charger_id in chargers:
                raise RegionRegistryError(f'duplicate charger_id={charger.charger_id}')
            chargers[charger.charger_id] = charger
        return cls(
            identity,
            regions,
            chargers,
            source_sha256=hashlib.sha256(raw_bytes).hexdigest(),
        )

    @property
    def configured(self) -> bool:
        """Whether this registry is bound to a named, content-addressed map."""
        return self.identity.configured

    def require_map(self, map_id: str, revision: str, sha256: str) -> None:
        """Reject requests that do not name the exact configured map snapshot."""
        if not self.configured:
            raise RegionRegistryError('region registry has no real map binding')
        requested = (map_id, revision, sha256.lower())
        current = (
            self.identity.map_id,
            self.identity.revision,
            self.identity.sha256.lower(),
        )
        if requested != current:
            raise RegionRegistryError('requested map identity differs from registry')

    def region(self, region_id: str) -> RegionDefinition:
        """Return one configured region or reject it as unknown."""
        try:
            return self.regions[region_id]
        except KeyError as exc:
            raise RegionRegistryError(f'unknown region_id={region_id}') from exc

    def charger(self, charger_id: str) -> ChargerDefinition:
        """Return one configured charger or reject it as unknown."""
        try:
            return self.chargers[charger_id]
        except KeyError as exc:
            raise RegionRegistryError(f'unknown charger_id={charger_id}') from exc


def _string(value: object, name: str, maximum: int) -> str:
    if not isinstance(value, str) or len(value) > maximum:
        raise RegionRegistryError(f'{name} must be a string of length <= {maximum}')
    return value


def _identifier(value: object, name: str) -> str:
    if not isinstance(value, str) or _ID.fullmatch(value) is None:
        raise RegionRegistryError(f'{name} is missing or invalid')
    return value


def _pose(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, dict) or set(value) != {'x', 'y', 'yaw'}:
        raise RegionRegistryError(f'{name} must contain x, y and yaw')
    parsed = tuple(_finite_number(value[key], f'{name}.{key}') for key in ('x', 'y', 'yaw'))
    return parsed


def _finite_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RegionRegistryError(f'{name} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise RegionRegistryError(f'{name} must be finite')
    return result


def _inside(
    point: tuple[float, float],
    polygon: tuple[tuple[float, float], ...],
) -> bool:
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        on_edge = (
            min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9
            and min(y1, y2) - 1e-9 <= y <= max(y1, y2) + 1e-9
        )
        if abs(cross) <= 1e-9 and on_edge:
            return True
        if (y1 > y) != (y2 > y):
            crossing = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing:
                inside = not inside
        previous = current
    return inside


def _parse_region(raw: object, registry_dir: Path) -> RegionDefinition:
    if not isinstance(raw, dict):
        raise RegionRegistryError('region entry must be a mapping')
    allowed = {'region_id', 'display_name', 'aliases', 'boundary', 'entry_pose', 'routes'}
    required = {'region_id', 'boundary', 'entry_pose', 'routes'}
    if not required.issubset(raw) or set(raw) - allowed:
        raise RegionRegistryError('region entry has missing or unknown fields')
    region_id = _identifier(raw.get('region_id'), 'region_id')
    name = _string(raw.get('display_name', region_id), 'display_name', 128)
    aliases_raw = raw.get('aliases', [])
    if not isinstance(aliases_raw, list) or len(aliases_raw) > 16:
        raise RegionRegistryError(f'{region_id}.aliases must be a bounded list')
    aliases = tuple(_string(value, 'alias', 128) for value in aliases_raw)
    polygon_raw = raw.get('boundary')
    if not isinstance(polygon_raw, list) or not 3 <= len(polygon_raw) <= 256:
        raise RegionRegistryError(f'{region_id}.boundary must have 3..256 points')
    polygon = tuple(
        (_finite_number(point[0], 'boundary.x'), _finite_number(point[1], 'boundary.y'))
        for point in polygon_raw
        if isinstance(point, (list, tuple)) and len(point) == 2
    )
    if len(polygon) != len(polygon_raw):
        raise RegionRegistryError(f'{region_id}.boundary contains an invalid point')
    twice_area = sum(
        polygon[i][0] * polygon[(i + 1) % len(polygon)][1]
        - polygon[(i + 1) % len(polygon)][0] * polygon[i][1]
        for i in range(len(polygon))
    )
    area = abs(twice_area) / 2.0
    if area <= 1e-6:
        raise RegionRegistryError(f'{region_id}.boundary has zero area')
    entry = _pose(raw.get('entry_pose'), f'{region_id}.entry_pose')
    if not _inside(entry[:2], polygon):
        raise RegionRegistryError(f'{region_id}.entry_pose lies outside its boundary')
    routes_raw = raw.get('routes', [])
    if not isinstance(routes_raw, list) or len(routes_raw) > 32:
        raise RegionRegistryError(f'{region_id}.routes must be a bounded list')
    routes: dict[str, RegionRoute] = {}
    for route_raw in routes_raw:
        if not isinstance(route_raw, dict):
            raise RegionRegistryError(f'{region_id} route must be a mapping')
        if set(route_raw) - {'route_id', 'waypoints', 'path_file'} or not {
            'route_id', 'waypoints'
        }.issubset(route_raw):
            raise RegionRegistryError(f'{region_id} route has missing or unknown fields')
        route_id = _identifier(route_raw.get('route_id'), 'route_id')
        points_raw = route_raw.get('waypoints')
        if (
            not isinstance(points_raw, list)
            or not 3 <= len(points_raw) <= _MAX_ROUTE_POINTS
        ):
            raise RegionRegistryError(
                f'{route_id}.waypoints must have 3..{_MAX_ROUTE_POINTS} points'
            )
        points = tuple(_pose(point, f'{route_id}.waypoint') for point in points_raw)
        if route_id in routes:
            raise RegionRegistryError(f'duplicate route_id={route_id}')
        if math.dist(points[0][:2], entry[:2]) > 0.05:
            raise RegionRegistryError(f'{route_id} must start at its region entry')
        if math.dist(points[0][:2], points[-1][:2]) > 0.05:
            raise RegionRegistryError(f'{route_id} must be a closed loop')
        if any(not _inside(point[:2], polygon) for point in points):
            raise RegionRegistryError(f'{route_id} has a waypoint outside region boundary')
        path_poses = _load_path_file(
            route_raw.get('path_file'), registry_dir, route_id, entry, polygon)
        routes[route_id] = RegionRoute(route_id, points, path_poses)
    return RegionDefinition(region_id, name, aliases, polygon, entry, routes)


def _load_path_file(value, registry_dir: Path, route_id: str, entry, polygon):
    """Load route.json dense_path_map. Absent means the checkpoint route is unchanged."""
    if value is None:
        return ()
    if not isinstance(value, str) or not value or len(value) > 512:
        raise RegionRegistryError(f'{route_id}.path_file is invalid')
    path = Path(value)
    if not path.is_absolute():
        path = registry_dir / path
    try:
        document = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise RegionRegistryError(f'{route_id} path file is unreadable: {exc}') from exc
    if document.get('route_id') != route_id:
        raise RegionRegistryError(f'{route_id} path file route_id does not match')
    raw_points = document.get('dense_path_map')
    if not isinstance(raw_points, list) or not 2 <= len(raw_points) <= _MAX_PATH_POSES:
        raise RegionRegistryError(
            f'{route_id}.dense_path_map must have 2..{_MAX_PATH_POSES} poses')
    poses = []
    for index, point in enumerate(raw_points):
        if not isinstance(point, dict) or set(point) != {'x', 'y', 'yaw'}:
            raise RegionRegistryError(f'{route_id} dense pose {index} is invalid')
        pose = (
            _finite_number(point['x'], 'path.x'),
            _finite_number(point['y'], 'path.y'),
            _finite_number(point['yaw'], 'path.yaw'),
        )
        if not _inside(pose[:2], polygon):
            raise RegionRegistryError(f'{route_id} dense pose {index} is outside the region')
        poses.append(pose)
    if math.dist(poses[0][:2], entry[:2]) > 0.05:
        raise RegionRegistryError(f'{route_id} dense path must start at the region entry')
    if math.dist(poses[0][:2], poses[-1][:2]) > 0.05:
        raise RegionRegistryError(f'{route_id} dense path must be a closed loop')
    return tuple(poses)


def _parse_charger(raw: object) -> ChargerDefinition:
    if not isinstance(raw, dict):
        raise RegionRegistryError('charger entry must be a mapping')
    if not {'charger_id', 'staging_pose'}.issubset(raw) or set(raw) - {
        'charger_id', 'display_name', 'staging_pose'
    }:
        raise RegionRegistryError('charger entry has missing or unknown fields')
    charger_id = _identifier(raw.get('charger_id'), 'charger_id')
    name = _string(raw.get('display_name', charger_id), 'display_name', 128)
    pose = _pose(raw.get('staging_pose'), f'{charger_id}.staging_pose')
    return ChargerDefinition(charger_id, name, pose)


__all__ = [
    'ChargerDefinition', 'MapIdentity', 'RegionDefinition', 'RegionRegistry',
    'RegionRegistryError', 'RegionRoute',
]
