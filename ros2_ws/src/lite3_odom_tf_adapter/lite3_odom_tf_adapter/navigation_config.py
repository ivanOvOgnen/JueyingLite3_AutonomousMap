"""Reviewable effective Nav2 config; no automatic recovery into unseen space."""
import copy
import json
import math


def merge_config(base, overlay, *, behavior_tree, commissioning=None):
    merged = copy.deepcopy(base)
    def merge(target, values):
        for key, value in values.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                merge(target[key], value)
            else:
                target[key] = copy.deepcopy(value)
    merge(merged, overlay)
    merged.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})['default_bt_xml_filename'] = str(behavior_tree)
    controller = merged.setdefault('controller_server', {}).setdefault('ros__parameters', {})
    plugin = controller.get('FollowPath', {}).get('plugin')
    if plugin not in ('dwb_core::DWBLocalPlanner', 'dwb_core/DWBLocalPlanner'):
        raise ValueError('Vendor controller plugin must be reviewed; this overlay supports DWB only')
    if commissioning is not None:
        footprint = commissioning.get('footprint')
        if not isinstance(footprint, list) or len(footprint) < 3:
            raise ValueError('Measured footprint polygon required')
        for point in footprint:
            if (not isinstance(point, list) or len(point) != 2 or
                    any(not isinstance(v, (int,float)) or not math.isfinite(v) for v in point)):
                raise ValueError('Invalid footprint vertex')
        area = sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(footprint, footprint[1:]+footprint[:1])) / 2
        if abs(area) < 0.01:
            raise ValueError('Footprint polygon is degenerate')
        for key in ('local_costmap', 'global_costmap'):
            params = merged.setdefault(key, {}).setdefault(key, {}).setdefault('ros__parameters', {})
            params.pop('robot_radius', None)
            params['footprint'] = json.dumps(footprint)
    # Unknown saved-map cells remain blocked by the global planner. The local
    # rolling window carries live sensor obstacles and starts with free cells;
    # it must not depend on map->odom or an unobserved static layer at startup.
    for key, track_unknown in (('local_costmap', False), ('global_costmap', True)):
        params = merged.setdefault(key, {}).setdefault(key, {}).setdefault('ros__parameters', {})
        params['track_unknown_space'] = track_unknown
    return merged
