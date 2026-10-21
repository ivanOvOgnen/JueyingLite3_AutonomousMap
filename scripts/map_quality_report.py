#!/usr/bin/env python3
"""Write a cautious, offline quality summary for a ROS trinary PGM map."""

import argparse
from collections import deque
import math
from pathlib import Path
import re
import sys


MAX_CELLS = 16_000_000
MAX_COMPONENT_CELLS = 2_000_000
MAX_PGM_BYTES = 128 * 1024 * 1024
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def read_map_yaml(path):
    text = path.read_text(encoding="utf-8")

    def scalar(key, required=False, default=None):
        match = re.search(rf"^\s*{re.escape(key)}\s*:\s*(.*?)\s*$", text, re.M)
        if not match:
            if required:
                raise ValueError(f"YAML is missing required field '{key}'.")
            return default
        value = match.group(1).split("#", 1)[0].strip().strip("'\"")
        return value

    image = scalar("image", required=True)
    resolution = float(scalar("resolution", required=True))
    origin_text = scalar("origin", required=True)
    origin_match = re.fullmatch(rf"\[\s*({NUMBER})\s*,\s*({NUMBER})\s*,\s*({NUMBER})\s*\]", origin_text)
    if not origin_match:
        raise ValueError("YAML origin must contain [x, y, yaw].")
    origin = tuple(float(value) for value in origin_match.groups())
    negate = int(scalar("negate", default="0"))
    occupied_threshold = float(scalar("occupied_thresh", default="0.65"))
    free_threshold = float(scalar("free_thresh", default="0.25"))
    mode = scalar("mode", default="trinary")

    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("Map resolution must be a positive finite number.")
    if not all(math.isfinite(value) for value in origin):
        raise ValueError("Map origin values must be finite.")
    if negate not in (0, 1):
        raise ValueError("Map negate must be 0 or 1.")
    if not (0 <= free_threshold < occupied_threshold <= 1):
        raise ValueError("Map thresholds must satisfy 0 <= free < occupied <= 1.")
    if mode != "trinary":
        raise ValueError(f"Only trinary maps are supported; YAML mode is '{mode}'.")

    return {
        "image": image,
        "resolution": resolution,
        "origin": origin,
        "negate": negate,
        "occupied_threshold": occupied_threshold,
        "free_threshold": free_threshold,
    }


def read_pgm(path):
    if path.stat().st_size > MAX_PGM_BYTES:
        raise ValueError(f"PGM exceeds the {MAX_PGM_BYTES // (1024 * 1024)} MiB input limit.")
    data = path.read_bytes()
    offset = 0
    whitespace = b" \t\r\n\v\f"

    def token():
        nonlocal offset
        while offset < len(data):
            if data[offset] in whitespace:
                offset += 1
            elif data[offset] == ord("#"):
                while offset < len(data) and data[offset] not in b"\r\n":
                    offset += 1
            else:
                break
        start = offset
        while offset < len(data) and data[offset] not in whitespace and data[offset] != ord("#"):
            offset += 1
        if start == offset:
            raise ValueError("PGM header is incomplete or malformed.")
        return data[start:offset].decode("ascii")

    magic = token()
    if magic not in ("P2", "P5"):
        raise ValueError("Only P2 and P5 PGM images are supported.")
    try:
        width, height, max_value = int(token()), int(token()), int(token())
    except (UnicodeDecodeError, ValueError) as error:
        raise ValueError("PGM dimensions or maximum pixel value are invalid.") from error

    count = width * height
    if width <= 0 or height <= 0 or count > MAX_CELLS:
        raise ValueError(f"PGM dimensions must be positive and at most {MAX_CELLS:,} cells.")
    if not 1 <= max_value <= 255:
        raise ValueError("Only 8-bit PGM maps (maximum sample 1..255) are supported.")

    if magic == "P5":
        if offset >= len(data) or data[offset] not in whitespace:
            raise ValueError("P5 raster separator is missing.")
        separator = data[offset]
        offset += 1
        if separator == 13 and offset < len(data) and data[offset] == 10:
            offset += 1
        pixels = data[offset:offset + count]
        if len(pixels) != count:
            raise ValueError("P5 pixel raster is incomplete.")
        if any(sample > max_value for sample in pixels):
            raise ValueError("P5 contains a pixel above the declared maximum value.")
    else:
        values = bytearray(count)
        try:
            for index in range(count):
                sample = int(token())
                if not 0 <= sample <= max_value:
                    raise ValueError("P2 contains a pixel outside the declared range.")
                values[index] = sample
        except (UnicodeDecodeError, ValueError) as error:
            raise ValueError("P2 raster is malformed or incomplete.") from error
        pixels = bytes(values)

    return width, height, max_value, pixels


def largest_free_component(mask, width, height):
    seen = bytearray(len(mask))
    largest = 0
    component_count = 0
    for start, is_free in enumerate(mask):
        if not is_free or seen[start]:
            continue
        component_count += 1
        seen[start] = 1
        queue = deque([start])
        size = 0
        while queue:
            index = queue.pop()
            size += 1
            x = index % width
            neighbors = []
            if x > 0:
                neighbors.append(index - 1)
            if x + 1 < width:
                neighbors.append(index + 1)
            if index >= width:
                neighbors.append(index - width)
            if index + width < width * height:
                neighbors.append(index + width)
            for neighbor in neighbors:
                if mask[neighbor] and not seen[neighbor]:
                    seen[neighbor] = 1
                    queue.append(neighbor)
        largest = max(largest, size)
    return component_count, largest


def create_report(image_path, yaml_path):
    metadata = read_map_yaml(yaml_path)
    expected_image = (yaml_path.parent / metadata["image"]).resolve()
    if expected_image != image_path.resolve():
        raise ValueError(
            f"YAML references '{metadata['image']}', which resolves to {expected_image}; "
            f"the selected image is {image_path.resolve()}."
        )

    width, height, max_value, pixels = read_pgm(image_path)
    total = width * height
    free_mask = bytearray(total) if total <= MAX_COMPONENT_CELLS else None
    counts = {"free": 0, "occupied": 0, "unknown": 0}
    free_threshold = metadata["free_threshold"]
    occupied_threshold = metadata["occupied_threshold"]
    negate = metadata["negate"]

    for index, sample in enumerate(pixels):
        intensity = sample / max_value
        occupancy = intensity if negate else 1.0 - intensity
        if occupancy > occupied_threshold:
            counts["occupied"] += 1
        elif occupancy < free_threshold:
            counts["free"] += 1
            if free_mask is not None:
                free_mask[index] = 1
        else:
            counts["unknown"] += 1

    percent = lambda value: 100.0 * value / total
    unknown_percent = percent(counts["unknown"])
    area_x = width * metadata["resolution"]
    area_y = height * metadata["resolution"]
    component_info = None
    if free_mask is not None:
        component_count, largest = largest_free_component(free_mask, width, height)
        component_info = (component_count, largest)

    warnings = []
    if unknown_percent > 50:
        warnings.append("More than half the cells are unknown; map coverage is sparse.")
    if counts["occupied"] == 0:
        warnings.append("No cells classified as occupied; inspect the scan and YAML thresholds.")
    if component_info and counts["free"] and component_info[1] / counts["free"] < 0.80:
        warnings.append("Free cells split across multiple regions; inspect for fragmentation or separate rooms.")
    if not warnings:
        warnings.append("No simple coverage warning triggered; this is not a map-quality or safety pass.")

    lines = [
        f"# Map review: {image_path.name}",
        "",
        f"- YAML: `{yaml_path.name}`",
        f"- Grid: {width} x {height} cells at {metadata['resolution']:.5g} m/cell",
        f"- Physical dimensions: {area_x:.2f} m x {area_y:.2f} m",
        f"- Origin: x={metadata['origin'][0]:.3f} m, y={metadata['origin'][1]:.3f} m, yaw={metadata['origin'][2]:.4f} rad",
        f"- Trinary thresholds: free < {free_threshold:g}; occupied > {occupied_threshold:g}; negate={negate}",
        "",
        "## Cell classification",
        "",
        f"- Free: {counts['free']:,} ({percent(counts['free']):.1f}%)",
        f"- Occupied: {counts['occupied']:,} ({percent(counts['occupied']):.1f}%)",
        f"- Unknown: {counts['unknown']:,} ({unknown_percent:.1f}%)",
    ]
    if component_info:
        largest_percent = 100.0 * component_info[1] / max(counts["free"], 1)
        lines.extend([
            f"- 4-connected free regions: {component_info[0]:,}",
            f"- Largest free region: {component_info[1]:,} cells ({largest_percent:.1f}% of free cells)",
        ])
    else:
        lines.append(f"- Free-region connectivity: skipped above {MAX_COMPONENT_CELLS:,} cells")
    lines.extend(["", "## Review notes", ""])
    lines.extend(f"- {warning}" for warning in warnings)
    lines.extend([
        "- Compare the rendered map with the room: check continuous walls, duplicated walls, openings, and return-to-start alignment.",
        "- These image-derived metrics do not measure odometry drift, obstacle clearance, localization quality, or safe navigation.",
        "- Treat the report as a comparison aid; a person must review the map before any navigation trial.",
        "",
    ])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="ROS trinary P2/P5 PGM map image")
    parser.add_argument("yaml", type=Path, help="Matching ROS map YAML")
    parser.add_argument("-o", "--output", type=Path,
                        help="Report path (default: <image>.audit.md)")
    args = parser.parse_args()
    output_path = args.output or args.image.with_suffix(".audit.md")
    try:
        report = create_report(args.image, args.yaml)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(report, encoding="utf-8")
    except (OSError, ValueError, OverflowError) as error:
        parser.exit(2, f"map-quality-report: {error}\n")
    print(report, end="")
    print(f"\nReport saved to: {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
