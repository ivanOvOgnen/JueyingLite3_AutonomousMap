"""Immutable map identity, integrity, explicit review, and occupancy matching."""
import hashlib
import json
import math
from pathlib import Path
import re
import time

import yaml

from lite3_odom_tf_adapter.map_bundle import load_manifest, manifest_path, write_manifest


def _sha(value):
    return hashlib.sha256(value).hexdigest()


def _signature(width, height, resolution, origin, cells):
    # YAML commonly parses a zero map yaw as int 0, while OccupancyGrid
    # reports the same angle as float 0.0. Normalize both representations
    # before hashing so an identical saved and published map keeps its ID.
    geometry = json.dumps([int(width), int(height), round(float(resolution), 6),
                           [round(float(x), 6) for x in origin]], separators=(',', ':'))
    return _sha(geometry.encode() + bytes(value + 1 for value in cells))


def grid_signature(grid):
    q, position = grid.info.origin.orientation, grid.info.origin.position
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    cells = [-1 if value < 0 or 19 < value < 65 else (0 if value <= 19 else 100) for value in grid.data]
    return _signature(int(grid.info.width), int(grid.info.height), float(grid.info.resolution),
                      [position.x, position.y, yaw], cells)


def _pgm(raw):
    tokens, offset = [], 0
    while len(tokens) < 4:
        while offset < len(raw) and raw[offset] in b' \t\r\n':
            offset += 1
        if offset < len(raw) and raw[offset] == 35:
            offset = raw.index(b'\n', offset) + 1
            continue
        start = offset
        while offset < len(raw) and raw[offset] not in b' \t\r\n':
            offset += 1
        if start == offset:
            raise ValueError('Incomplete PGM header')
        tokens.append(raw[start:offset])
    if tokens[0] != b'P5' or tokens[3] != b'255':
        raise ValueError('Expected an 8-bit binary PGM')
    width, height = int(tokens[1]), int(tokens[2])
    separator = 2 if raw[offset:offset+2] == b'\r\n' else 1
    pixels = raw[offset+separator:]
    if width <= 0 or height <= 0 or len(pixels) != width*height:
        raise ValueError('PGM dimensions/payload disagree')
    return width, height, pixels


def read_bundle(prefix, *, require_reviewed=False, allow_legacy=False):
    prefix = Path(prefix)
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', prefix.name):
        raise ValueError('Invalid map bundle name')
    manifest = load_manifest(prefix)
    legacy = manifest is None and allow_legacy and not require_reviewed and not manifest_path(prefix).exists()
    if manifest is None and not legacy:
        raise ValueError('Map lacks an integrity manifest; import/review it explicitly')
    if legacy:
        # A missing sidecar can be displayed and used to diagnose AMCL, but
        # this transient description is never persisted as a map review.
        manifest = {'quality': {'approval_state': 'experimental'},
                    'review': {}, 'legacy_unverified': True}
    files = {}
    for suffix in ('.pgm', '.yaml'):
        path = prefix.with_suffix(suffix)
        if path.is_symlink():
            raise ValueError('Map symlinks are not supported')
        files[path.name] = path.read_bytes()
        if not legacy and manifest.get('files_sha256', {}).get(path.name) != _sha(files[path.name]):
            raise ValueError('Map file checksum mismatch: ' + path.name)
    metadata = yaml.safe_load(files[prefix.name + '.yaml'])
    if not isinstance(metadata, dict) or metadata.get('image') != prefix.name + '.pgm':
        raise ValueError('Map YAML must select its own PGM')
    resolution, origin = float(metadata['resolution']), list(metadata['origin'])
    if (not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3 or
            not all(isinstance(x, (int, float)) and math.isfinite(x) for x in origin)):
        raise ValueError('Invalid map geometry')
    width, height, pixels = _pgm(files[prefix.name + '.pgm'])
    occupied, free = float(metadata['occupied_thresh']), float(metadata['free_thresh'])
    negate = metadata.get('negate', 0)
    if not 0 <= free < occupied <= 1 or negate not in (0, 1) or metadata.get('mode', 'trinary') != 'trinary':
        raise ValueError('Only valid trinary map bundles are supported')
    image_cells = []
    for gray in pixels:
        probability = gray / 255 if negate else 1 - gray / 255
        image_cells.append(100 if probability > occupied else (0 if probability < free else -1))
    cells = [value for row in reversed(range(height)) for value in image_cells[row*width:(row+1)*width]]
    identity = _sha(b''.join(name.encode() + files[name] for name in sorted(files)))
    approval = manifest.get('quality', {}).get('approval_state', 'experimental')
    review = manifest.get('review', {})
    if require_reviewed and manifest.get('synthetic') is True:
        raise ValueError('Synthetic maps cannot authorize real robot navigation')
    if require_reviewed and (approval != 'reviewed' or review.get('map_id') != identity):
        raise ValueError('Map is not explicitly reviewed for this exact checksum')
    return {'name': prefix.name, 'map_id': identity, 'grid_signature': _signature(width, height, resolution, origin, cells),
            'approval_state': approval, 'width': width, 'height': height, 'resolution': resolution,
            'origin': origin, 'cells': cells, 'manifest': manifest,
            '_image': files[prefix.name+'.pgm'], '_yaml': files[prefix.name+'.yaml']}


def review_bundle(prefix, review):
    """Record an explicit operator review; never infer it from a saved map."""
    bundle = read_bundle(prefix)
    if bundle['manifest'].get('synthetic') is True:
        raise ValueError('Synthetic maps cannot receive a physical map review')
    required = ('reviewed_by', 'notes', 'capture_report_sha256', 'config_sha256', 'source_sha256')
    if not isinstance(review, dict) or any(not isinstance(review.get(key), str) or not review[key].strip() for key in required):
        raise ValueError('Review requires reviewer, notes, capture/config/source hashes')
    for key in required[2:]:
        if not re.fullmatch(r'[a-f0-9]{64}', review[key]):
            raise ValueError('Review hashes must be SHA-256 values')
    if review.get('map_id') != bundle['map_id']:
        raise ValueError('Review does not identify the current map checksum')
    region = review.get('operating_region_cells')
    if (not isinstance(region, list) or len(region) != 4 or
            any(not isinstance(x, int) or isinstance(x, bool) for x in region)):
        raise ValueError('Operating region must be [min_x, min_y, max_x_exclusive, max_y_exclusive] cells')
    x0, y0, x1, y1 = region
    if not 0 <= x0 < x1 <= bundle['width'] or not 0 <= y0 < y1 <= bundle['height']:
        raise ValueError('Operating region is outside the map')
    if review.get('geometry_measured') is not True or review.get('timing_reviewed') is not True:
        raise ValueError('Explicit measured-geometry and timing review acknowledgments are required')
    measurements = review.get('measurements', [])
    if not isinstance(measurements, list) or len(measurements) < 2:
        raise ValueError('Record at least two independent measured map distances')
    for measurement in measurements:
        actual, mapped = measurement.get('measured_m'), measurement.get('map_m')
        if (not all(isinstance(v,(int,float)) and math.isfinite(v) and v>0 for v in (actual,mapped)) or
                abs(actual-mapped)>max(0.05,0.02*actual)):
            raise ValueError('Measured map distance exceeds the declared 0.05 m / 2% target')
    angle = review.get('angle_measurement', {})
    actual, mapped = angle.get('measured_degrees'), angle.get('map_degrees')
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (actual,mapped)) or abs(actual-mapped)>5:
        raise ValueError('Record an independent angle measurement within the 5 degree target')
    manifest = bundle['manifest']
    manifest.setdefault('quality', {})['approval_state'] = 'reviewed'
    manifest['review'] = {**review, 'recorded_unix_ns': time.time_ns()}
    write_manifest(prefix, manifest)
    return read_bundle(prefix, require_reviewed=True)
