#!/usr/bin/env python3
"""Offline integrity/timing checks for a passive Lite3 capture; no ROS required.

A passing report validates this tool's checks, never physical mapping acceptance.
TF bracketing is reported separately from a real acquisition-time TF lookup.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

FORMAT = 'lite3_mapping_capture/v1'
REQUIRED = ('depth', 'camera_info', 'scan', 'vendor_odom', 'normalized_odom', 'map', 'tf', 'tf_static')
PERIODIC = ('depth', 'camera_info', 'scan', 'vendor_odom', 'normalized_odom')


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def stats(values):
    values = sorted(value for value in values if finite(value))
    if not values:
        return None
    return {'min': values[0], 'median': statistics.median(values),
            'p95': values[min(len(values) - 1, math.ceil(len(values) * 0.95) - 1)], 'max': values[-1]}


def quaternion_valid(value):
    q = [value.get(key) for key in ('x', 'y', 'z', 'w')]
    return all(finite(x) for x in q) and abs(sum(x * x for x in q) - 1.0) <= 0.01


def validate_capture(directory, *, profile='stationary', min_duration=30.0,
                     min_samples=10, max_gap=1.0, max_source_age=2.0):
    root = Path(directory).resolve()
    failures = []
    warnings = []
    counts = Counter()
    timing = defaultdict(list)
    infos = defaultdict(list)
    depths = []
    scan_frames = []
    dynamic_tf = defaultdict(list)
    static_tf = set()
    payload_files = set()
    saved_bytes = 0
    positions = []
    bad = Counter()

    def fail(code, detail):
        failures.append({'code': code, 'detail': detail})

    def read_json(path):
        try:
            result = json.loads(path.read_text())
            if not isinstance(result, dict):
                raise ValueError('expected an object')
            return result
        except (OSError, ValueError) as error:
            fail('unreadable_json', path.name + ': ' + str(error))
            return {}

    metadata = read_json(root / 'metadata.json')
    summary = read_json(root / 'summary.json')
    for name, value in (('metadata', metadata), ('summary', summary)):
        if value.get('format') != FORMAT:
            fail('capture_format', name + ' has no supported format')
    try:
        ledger = (root / 'events.jsonl').open()
    except OSError as error:
        fail('missing_events', str(error))
        ledger = []
    try:
        for line_number, line in enumerate(ledger, 1):
            try:
                row = json.loads(line)
                if not isinstance(row, dict) or not isinstance(row.get('kind'), str):
                    raise ValueError('event has no kind')
                kind = row['kind']
                counts[kind] += 1
                header = row.get('header', {})
                stamp, receipt = header.get('stamp_ns'), row.get('received_unix_ns')
                if not isinstance(receipt, int) or isinstance(receipt, bool) or receipt <= 0:
                    bad['invalid_receipt'] += 1
                if kind in PERIODIC + ('raw_cloud_header',):
                    if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
                        bad[kind + '_invalid_stamp'] += 1
                    elif isinstance(receipt, int):
                        timing[kind].append((receipt, stamp))
                if kind == 'depth':
                    path = root / str(row.get('file', ''))
                    resolved = path.resolve()
                    if resolved.parent != root / 'depth' or path.is_symlink() or resolved in payload_files:
                        raise ValueError('depth path escapes depth directory, is duplicated, or is a symlink')
                    payload_files.add(resolved)
                    width, height, step = row['width'], row['height'], row['step']
                    bpp = {'16UC1': 2, '32FC1': 4}.get(row['encoding'])
                    if (bpp is None or not all(isinstance(x, int) and x > 0 for x in (width, height, step))
                            or step < width * bpp):
                        raise ValueError('unsupported depth encoding or invalid dimensions/stride')
                    size = resolved.stat().st_size
                    if size != height * step or size != row['bytes']:
                        bad['depth_payload_size'] += 1
                    if digest(resolved) != row['sha256']:
                        bad['depth_payload_hash'] += 1
                    saved_bytes += size
                    depths.append((stamp, header.get('frame_id'), width, height))
                elif kind == 'camera_info':
                    k = row.get('k', [])
                    if (len(k) != 9 or not all(finite(x) for x in k) or k[0] <= 0 or k[4] <= 0
                            or row.get('width', 0) <= 0 or row.get('height', 0) <= 0):
                        bad['invalid_camera_info'] += 1
                    else:
                        infos[stamp].append((header.get('frame_id'), row['width'], row['height']))
                elif kind in ('vendor_odom', 'normalized_odom'):
                    pose = row['pose']
                    if not quaternion_valid(pose['orientation']):
                        bad[kind + '_quaternion'] += 1
                    xyz = [pose['position'][axis] for axis in ('x', 'y', 'z')]
                    if not all(finite(x) for x in xyz):
                        bad[kind + '_position'] += 1
                    elif kind == 'normalized_odom':
                        positions.append(xyz)
                elif kind == 'scan':
                    if (not row['ranges'] or not finite(row['angle_increment']) or row['angle_increment'] <= 0
                            or not finite(row['range_min']) or not finite(row['range_max'])
                            or row['range_min'] < 0 or row['range_max'] <= row['range_min']):
                        bad['invalid_scan_geometry'] += 1
                    for value in row['ranges']:
                        if value in ('NaN', 'Infinity', '-Infinity'):
                            continue
                        if not finite(value):
                            # Historical recorder used non-standard JSON Infinity/NaN.
                            bad['legacy_or_invalid_scan_number'] += 1
                    if isinstance(stamp, int):
                        scan_frames.append((stamp, header.get('frame_id')))
                elif kind == 'map':
                    if (row['width'] <= 0 or row['height'] <= 0 or
                            len(row['data']) != row['width'] * row['height'] or
                            not finite(row['resolution']) or row['resolution'] <= 0 or
                            any(not isinstance(x, int) or x < -1 or x > 100 for x in row['data']) or
                            not quaternion_valid(row['origin']['orientation'])):
                        bad['invalid_map'] += 1
                elif kind in ('tf', 'tf_static'):
                    for transform in row['transforms']:
                        edge = (transform['parent'], transform['child'])
                        if (not all(isinstance(x, str) and x for x in edge) or edge[0] == edge[1]
                                or not quaternion_valid(transform['rotation']) or
                                not all(finite(transform['translation'].get(axis)) for axis in ('x', 'y', 'z'))):
                            bad['invalid_tf'] += 1
                            continue
                        if kind == 'tf_static':
                            static_tf.add(edge)
                        elif isinstance(transform['stamp_ns'], int) and transform['stamp_ns'] > 0:
                            dynamic_tf[edge].append(transform['stamp_ns'])
                        else:
                            bad['invalid_dynamic_tf_stamp'] += 1
            except (AttributeError, KeyError, TypeError, ValueError, OSError, OverflowError) as error:
                fail('malformed_event', f'line {line_number}: {error}')
    finally:
        if hasattr(ledger, 'close'):
            ledger.close()
    required = ('depth', 'camera_info') if profile == 'depth-only' else REQUIRED
    for kind in required:
        if not counts[kind]:
            fail('missing_stream', kind)
    for code, count in bad.items():
        if count:
            fail(code, f'{count} occurrences')
    if summary.get('writer_finished') is not True:
        fail('writer_unfinished', 'Writer did not report completion')
    if summary.get('writer_error'):
        fail('writer_error', str(summary['writer_error']))
    if summary.get('queued_depth', 0):
        fail('queued_depth', 'Unwritten depth remains in queue')
    if not isinstance(summary.get('dropped'), dict) or any(summary.get('dropped', {}).values()):
        fail('capture_drops', str(summary.get('dropped')))
    if summary.get('counts') != dict(counts):
        fail('ledger_counts', 'Summary counts differ from complete ledger')
    if summary.get('saved_depth_bytes') != saved_bytes:
        fail('saved_bytes', 'Summary byte total differs from retained depth payloads')
    if summary.get('reserved_depth_bytes', saved_bytes) != saved_bytes:
        fail('reserved_bytes', 'Reserved depth payloads were not all written')
    actual_files = {x.resolve() for x in (root / 'depth').glob('*.bin')}
    if actual_files != payload_files:
        fail('orphan_payloads', f'{len(actual_files.symmetric_difference(payload_files))} unmatched payload paths')
    unpaired = sum((frame, width, height) not in infos[stamp] for stamp, frame, width, height in depths)
    if unpaired:
        fail('unpaired_depth', f'{unpaired} depth frames lack exact stamp/frame/dimension CameraInfo pairing')
    streams = {}
    for kind, samples in timing.items():
        samples.sort()
        receipts = [x[0] for x in samples]
        stamps = [x[1] for x in samples]
        duration = (max(receipts) - min(receipts)) / 1e9
        gaps = [(b - a) / 1e9 for a, b in zip(receipts, receipts[1:])]
        ages = [(receipt - stamp) / 1e9 for receipt, stamp in samples]
        monotonic = all(b > a for a, b in zip(stamps, stamps[1:]))
        streams[kind] = {'samples': len(samples), 'receipt_duration_seconds': duration,
                         'receipt_rate_hz': (len(samples) - 1) / duration if duration > 0 else None,
                         'receipt_gap_seconds': stats(gaps), 'acquisition_age_seconds': stats(ages),
                         'strictly_increasing_acquisition_stamps': monotonic}
        if kind in required and kind in PERIODIC:
            if len(samples) < min_samples or duration < min_duration:
                fail('short_stream', f'{kind}: {len(samples)} samples over {duration:.3f} s')
            if gaps and max(gaps) > max_gap:
                fail('stream_gap', f'{kind}: largest reception gap {max(gaps):.3f} s')
            if not monotonic:
                fail('stamp_order', kind)
            if kind in ('depth', 'camera_info', 'scan') and any(x < -0.1 or x > max_source_age for x in ages):
                fail('sensor_source_age', f'{kind}: outside [-0.1, {max_source_age}] s')
    for edge in dynamic_tf:
        dynamic_tf[edge] = sorted(set(dynamic_tf[edge]))
    if static_tf.intersection(dynamic_tf):
        fail('tf_static_dynamic_conflict', str(sorted(static_tf.intersection(dynamic_tf))))
    parents = defaultdict(set)
    for parent, child in set(dynamic_tf).union(static_tf):
        parents[child].add(parent)
    if any(len(value) > 1 for value in parents.values()):
        fail('tf_multiple_parents', str({key: sorted(value) for key, value in parents.items() if len(value) > 1}))

    def bracketed(target, source, stamp):
        edges = set(static_tf)
        for edge, stamps in dynamic_tf.items():
            index = bisect_left(stamps, stamp)
            if (index < len(stamps) and stamps[index] == stamp) or (
                    0 < index < len(stamps) and (stamps[index] - stamps[index - 1]) / 1e9 <= max_gap):
                edges.add(edge)
        seen, pending = {source}, [source]
        while pending:
            current = pending.pop()
            if current == target:
                return True
            for parent, child in edges:
                neighbor = child if parent == current else parent if child == current else None
                if neighbor and neighbor not in seen:
                    seen.add(neighbor)
                    pending.append(neighbor)
        return False

    coverage = {}
    for kind, observations in (('depth', [(x[0], x[1]) for x in depths]), ('scan', scan_frames)):
        checked = [(stamp, frame) for stamp, frame in observations if isinstance(stamp, int) and frame]
        covered = sum(bracketed('map', frame, stamp) for stamp, frame in checked)
        coverage[kind] = {'observations': len(checked), 'map_chain_bracketed': covered,
                          'fraction': covered / len(checked) if checked else None}
        if profile == 'stationary' and (not checked or covered != len(checked)):
            fail('tf_bracket_coverage', f'{kind}: {covered}/{len(checked)} have a bracketed map chain')
    if not counts['imu']:
        warnings.append('IMU stream absent; availability must be inventoried on the robot.')
    if counts['raw_cloud_header']:
        warnings.append('Raw cloud ages are diagnostic only; raw cloud remains excluded from registration/obstacle acceptance.')
    displacement = None
    if positions:
        first = positions[0]
        displacement = max(math.sqrt(sum((x - y) ** 2 for x, y in zip(point, first))) for point in positions)
    files = {name: digest(root / name) for name in ('metadata.json', 'summary.json', 'events.jsonl') if (root / name).is_file()}
    return {'format': 'lite3_capture_validation/v1', 'profile': profile,
            'result': 'pass' if not failures else 'fail', 'robot_acceptance': 'not_established',
            'thresholds': {'min_duration_seconds': min_duration, 'min_samples': min_samples,
                           'max_gap_seconds': max_gap, 'max_source_age_seconds': max_source_age},
            'counts': dict(counts), 'saved_depth_bytes': saved_bytes, 'input_sha256': files,
            'streams': streams, 'tf_bracket_coverage': coverage,
            'normalized_odom_max_displacement_from_first_metres': displacement,
            'failures': failures, 'warnings': warnings,
            'unverified': ['Actual acquisition-time TF interpolation/composition and authority identity',
                           'Measured camera calibration, scan geometry and physical stationary state',
                           'Room/map accuracy, source/install provenance, motor isolation and stop behavior']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--profile', choices=('stationary', 'depth-only'), default='stationary')
    parser.add_argument('--min-duration', type=float, default=30.0)
    parser.add_argument('--min-samples', type=int, default=10)
    parser.add_argument('--max-gap', type=float, default=1.0)
    parser.add_argument('--max-source-age', type=float, default=2.0)
    parser.add_argument('--output', type=Path, help='New report path; refuses overwrite')
    options = parser.parse_args(argv)
    if any(not math.isfinite(x) or x <= 0 for x in
           (options.min_duration, options.max_gap, options.max_source_age)) or options.min_samples < 2:
        parser.error('duration/gap/source-age must be finite positive; min-samples must be at least 2')
    report = validate_capture(options.capture, profile=options.profile, min_duration=options.min_duration,
                              min_samples=options.min_samples, max_gap=options.max_gap,
                              max_source_age=options.max_source_age)
    output = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n'
    if options.output:
        with options.output.open('x') as stream:
            stream.write(output)
    else:
        print(output, end='')
    return 0 if report['result'] == 'pass' else 1


if __name__ == '__main__':
    sys.exit(main())
