"""Pure rectified-depth projection; original acquisition frame/time is retained."""
import math
import numpy as np


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def project_depth(image, info, *, now_ns, max_age=1.0, stride=4,
                  min_range=0.15, max_range=3.0):
    stamp = stamp_ns(image.header)
    if (stamp <= 0 or stamp != stamp_ns(info.header) or
            not image.header.frame_id or image.header.frame_id != info.header.frame_id or
            (image.width, image.height) != (info.width, info.height)):
        raise ValueError('Depth/CameraInfo must match exact stamp, frame and dimensions')
    age = (now_ns - stamp) / 1e9
    if age < -0.05 or age > max_age:
        raise ValueError('Depth acquisition time is future or stale')
    if stride < 1 or not 0 < min_range < max_range or not math.isfinite(max_range):
        raise ValueError('Invalid projection range/stride')
    width, height, step = int(image.width), int(image.height), int(image.step)
    dtype = {'16UC1': 'u2', '32FC1': 'f4'}.get(image.encoding)
    if not dtype or width <= 0 or height <= 0:
        raise ValueError('Unsupported depth encoding or dimensions')
    dtype = np.dtype(('>' if image.is_bigendian else '<') + dtype)
    if step < width * dtype.itemsize or len(image.data) != height * step:
        raise ValueError('Depth payload/stride mismatch')
    projection = list(info.p)
    if len(projection) != 12 or not all(math.isfinite(x) for x in projection):
        raise ValueError('Rectified CameraInfo projection matrix is invalid')
    fx, fy, cx, cy = projection[0], projection[5], projection[2], projection[6]
    if fx <= 0 or fy <= 0 or projection[3] != 0 or projection[7] != 0:
        raise ValueError('Expected positive rectified focal lengths and zero depth-camera translation')
    depth = np.ndarray((height, width), dtype=dtype, buffer=bytes(image.data), strides=(step, dtype.itemsize))
    z = depth[::stride, ::stride].astype(np.float32)
    if image.encoding == '16UC1':
        z *= 0.001
    v, u = np.mgrid[0:height:stride, 0:width:stride]
    valid = np.isfinite(z) & (z >= min_range) & (z <= max_range)
    if not np.any(valid):
        raise ValueError('No valid depth returns in the configured range')
    points = np.column_stack(((u[valid] - cx) * z[valid] / fx,
                              (v[valid] - cy) * z[valid] / fy, z[valid])).astype('<f4')
    return points, {'stamp_ns': stamp, 'frame_id': image.header.frame_id,
                    'age_seconds': age, 'point_count': len(points),
                    'valid_fraction': float(np.count_nonzero(valid) / valid.size)}
