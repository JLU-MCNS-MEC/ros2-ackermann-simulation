"""Persistent, map-bound semantic landmarks with explicit provenance."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import yaml


def map_identity(map_file: str) -> str:
    """Fingerprint both occupancy pixels and their metric coordinate system."""
    path = Path(map_file)
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except yaml.YAMLError as error:
        raise ValueError('Invalid map YAML') from error
    if (
        not isinstance(data, dict)
        or not {'image', 'resolution', 'origin'} <= data.keys()
    ):
        raise ValueError('Map YAML requires image, resolution and origin')
    if not isinstance(data['image'], str) or not data['image'].strip():
        raise ValueError('Map image must be a nonempty path')
    resolution = data['resolution']
    origin = data['origin']
    if (
        isinstance(resolution, bool)
        or not isinstance(resolution, (float, int))
        or not math.isfinite(resolution)
        or resolution <= 0
    ):
        raise ValueError('Map resolution must be finite and positive')
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or any(
            isinstance(value, bool)
            or not isinstance(value, (float, int))
            or not math.isfinite(value)
            for value in origin
        )
    ):
        raise ValueError('Map origin requires finite [x, y, yaw]')
    image = path.parent / data['image']
    geometry = {key: value for key, value in data.items() if key != 'image'}
    return hashlib.sha256(
        image.read_bytes() + json.dumps(geometry, sort_keys=True).encode()
    ).hexdigest()


def validate_landmark(item: dict) -> dict:
    """Require an explicit, finite approach pose and trustworthy source tag."""
    if not isinstance(item, dict):
        raise ValueError('Landmark must be an object')
    if not isinstance(item.get('name'), str) or not item['name'].strip():
        raise ValueError('Landmark name must be nonempty')
    if item.get('source') not in ('manual_pose', 'rgbd_observation'):
        raise ValueError('Landmark source must be explicit')
    aliases = item.get('aliases', [])
    if not isinstance(aliases, list) or any(
        not isinstance(alias, str) or not alias.strip() for alias in aliases
    ):
        raise ValueError('Aliases must be nonempty strings')
    pose = item.get('approach_pose')
    if (
        not isinstance(pose, list)
        or len(pose) != 3
        or any(
            isinstance(value, bool)
            or not isinstance(value, (float, int))
            or not math.isfinite(value)
            for value in pose
        )
    ):
        raise ValueError('Approach pose requires finite [x, y, yaw]')
    return item


def load_landmarks(path: str, identity: str) -> list[dict]:
    """Reject wrong-map databases and duplicate instance names."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('version') != 1:
        raise ValueError('Unsupported semantic map version')
    if data.get('map_id') != identity or data.get('frame_id') != 'map':
        raise ValueError('Semantic map belongs to a different navigation map')
    if not isinstance(data.get('landmarks'), list):
        raise ValueError('Landmarks must be a list')
    items = [validate_landmark(item) for item in data['landmarks']]
    names = [item['name'].strip().casefold() for item in items]
    if len(names) != len(set(names)):
        raise ValueError('Duplicate landmark names')
    return items


def save_landmarks(path: str, identity: str, items: list[dict]) -> None:
    """Atomically persist a validated semantic database."""
    for item in items:
        validate_landmark(item)
    names = [item['name'].strip().casefold() for item in items]
    if len(names) != len(set(names)):
        raise ValueError('Duplicate landmark names')
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {'version': 1, 'map_id': identity, 'frame_id': 'map', 'landmarks': items}
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', dir=target.parent, delete=False
        ) as stream:
            temporary = stream.name
            json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


def resolve_landmark(items: list[dict], query: str) -> dict:
    """Resolve an exact name or alias; never guess an ambiguous destination."""
    query = query.strip().casefold()
    if not query:
        raise ValueError('Goal name is empty')
    matches = [
        item
        for item in items
        if query
        in {
            value.strip().casefold()
            for value in [item['name'], *item.get('aliases', [])]
        }
    ]
    if not matches:
        raise ValueError(f'Unknown goal: {query}')
    if len(matches) > 1:
        raise ValueError(
            'Ambiguous goal; use one of: ' + ', '.join(item['name'] for item in matches)
        )
    return matches[0]
