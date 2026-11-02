"""Pure validated map bundle persistence, shared by ROS and offline tools."""
import hashlib
import math
import os
from pathlib import Path
import re
import tempfile
from lite3_odom_tf_adapter.map_bundle import build_manifest, manifest_path, write_manifest

def _quaternion_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def save_grid(message, prefix, provenance=None):
    """Publish a validated, no-overwrite bundle with YAML as commit marker.

    All files are prepared on the same filesystem. The catalog discovers YAML
    last, after PGM and manifest exist. A crash can leave a reserved/orphan name;
    it cannot expose a half-written YAML/PGM or overwrite an earlier map.
    """
    width = int(message.info.width)
    height = int(message.info.height)
    if width <= 0 or height <= 0 or len(message.data) != width * height:
        raise ValueError('OccupancyGrid dimensions do not match its cell data')
    if any(isinstance(value, bool) or not isinstance(value, int) or value < -1 or value > 100
           for value in message.data):
        raise ValueError('OccupancyGrid cells must be integers in [-1, 100]')
    resolution = float(message.info.resolution)
    origin = message.info.origin
    q = origin.orientation
    values = [resolution, origin.position.x, origin.position.y, origin.position.z, q.x, q.y, q.z, q.w]
    if not all(math.isfinite(value) for value in values) or resolution <= 0:
        raise ValueError('Map resolution/origin must be finite and resolution positive')
    if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01 or abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
        raise ValueError('Map origin must have a normalized planar quaternion')
    prefix = Path(prefix).expanduser()
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', prefix.name):
        raise ValueError('Map name must use 1-64 letters, digits, underscore, or hyphen, without a suffix')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    image_path = prefix.with_suffix('.pgm')
    yaml_path = prefix.with_suffix('.yaml')
    sidecar_path = manifest_path(prefix)
    destinations = (image_path, sidecar_path, yaml_path)

    free_limit = 0.196 * 100
    occupied_limit = 0.65 * 100

    def gray(value):
        if value < 0:
            return 205
        if value <= free_limit:
            return 254
        if value >= occupied_limit:
            return 0
        return 205

    yaw = _quaternion_yaw(origin.orientation)
    yaml_text = (
        f'image: {image_path.name}\n'
        'mode: trinary\n'
        f'resolution: {message.info.resolution:.12g}\n'
        f'origin: [{origin.position.x:.12g}, {origin.position.y:.12g}, {yaw:.12g}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n')
    lock_path = prefix.parent / ('.' + prefix.name + '.save-lock')
    # O_EXCL also protects API and CLI saves of the same name from each other.
    with lock_path.open('x'):
        published = []
        try:
            if any(path.exists() or path.is_symlink() for path in destinations):
                raise FileExistsError('Map bundle name already exists; choose a new name')
            with tempfile.TemporaryDirectory(prefix='.' + prefix.name + '.stage-', dir=str(prefix.parent)) as temporary:
                stage = Path(temporary)
                staged_prefix = stage / prefix.name
                staged_image = staged_prefix.with_suffix('.pgm')
                with staged_image.open('xb') as image:
                    image.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
                    for row in reversed(range(height)):
                        start = row * width
                        image.write(bytes(gray(v) for v in message.data[start:start + width]))
                    image.flush()
                    os.fsync(image.fileno())
                staged_yaml = staged_prefix.with_suffix('.yaml')
                with staged_yaml.open('x') as output:
                    output.write(yaml_text)
                    output.flush()
                    os.fsync(output.fileno())
                manifest = build_manifest(
                    prefix, width=width, height=height, resolution=resolution,
                    origin=(origin.position.x, origin.position.y, yaw),
                    frame=message.header.frame_id or 'map')
                manifest['provenance'] = provenance or {'capture_report_sha256': None, 'source_sha256': None, 'config_sha256': None}
                manifest['files_sha256'] = {
                    image_path.name: hashlib.sha256(staged_image.read_bytes()).hexdigest(),
                    yaml_path.name: hashlib.sha256(staged_yaml.read_bytes()).hexdigest(),
                }
                manifest['source_grid_stamp_ns'] = (
                    int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec))
                manifest['bundle_status'] = 'complete'
                staged_sidecar = write_manifest(staged_prefix, manifest)
                with staged_sidecar.open('rb') as sidecar:
                    os.fsync(sidecar.fileno())
                for source, destination in zip((staged_image, staged_sidecar, staged_yaml), destinations):
                    # Hard links publish a complete file atomically and refuse
                    # an existing destination, including concurrent writers.
                    os.link(source, destination)
                    published.append((destination, destination.stat()))
                directory_fd = os.open(str(prefix.parent), os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
        except Exception:
            for destination, identity in reversed(published):
                if destination.exists() and os.path.samestat(destination.stat(), identity):
                    destination.unlink()
            raise
        finally:
            lock_path.unlink()
    return image_path, yaml_path, sidecar_path
