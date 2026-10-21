"""Versioned, backward-compatible metadata for a saved Lite3 room map.

The occupancy PGM/YAML pair remains the navigation authority.  This sidecar
only records how optional 3D evidence relates to that pair; it never creates
or upgrades a 3D reconstruction from a live camera preview.
"""

import json
import os
from pathlib import Path
import tempfile


FORMAT = 'lite3_saved_map_manifest/v1'


def manifest_path(prefix):
    """Return the stable sidecar name for a map prefix."""
    return Path(prefix).with_suffix('.lite3-map.json')


def build_manifest(prefix, *, width, height, resolution, origin, frame='map'):
    """Build metadata for a newly saved 2D-only map without inventing 3D."""
    prefix = Path(prefix)
    return {
        'format': FORMAT,
        'map_name': prefix.name,
        'coordinate_convention': {
            'frame': str(frame or 'map'),
            'units': 'metres',
            'axes': 'ROS REP-103 map frame: +x forward/east, +y left/north, +z up',
            'origin': [float(origin[0]), float(origin[1]), float(origin[2])],
            'resolution_metres_per_cell': float(resolution),
        },
        'occupancy_grid': {
            'image': prefix.with_suffix('.pgm').name,
            'yaml': prefix.with_suffix('.yaml').name,
            'width_cells': int(width),
            'height_cells': int(height),
            'authoritative_for_navigation': True,
        },
        'reconstruction_3d': {
            'available': False,
            'status': 'not_captured',
            'reason': ('No timestamp-verified, map-frame registered point cloud was saved with this map. '
                       'A live camera preview is not persistent reconstruction evidence.'),
        },
        'quality': {
            'global_registration': 'unverified',
            'localization': 'unverified',
            'notes': ['Saving a map does not validate physical geometry or AMCL alignment.'],
        },
    }


def write_manifest(prefix, manifest):
    """Atomically write a JSON manifest and return its path."""
    destination = manifest_path(prefix)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=destination.name + '.', suffix='.tmp', dir=str(destination.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as output:
            json.dump(manifest, output, indent=2, sort_keys=True)
            output.write('\n')
        os.replace(temporary, destination)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
    return destination


def load_manifest(prefix):
    """Read a valid current-format manifest, otherwise return ``None``."""
    try:
        value = json.loads(manifest_path(prefix).read_text(encoding='utf-8'))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) and value.get('format') == FORMAT else None
