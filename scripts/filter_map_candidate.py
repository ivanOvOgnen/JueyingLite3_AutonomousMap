#!/usr/bin/env python3
"""Create a conservative, non-overwriting ROS PGM/YAML cleanup candidate.

The filter only changes isolated occupied cells to unknown. It never marks a
cell free, fills unknown space, changes map geometry, or edits its input files.
The output is a review artifact and must not be treated as navigation-safe.
"""

import argparse
from pathlib import Path
import re
import sys


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
MAX_CELLS = 16_000_000


def read_map_yaml(path):
    text = path.read_text(encoding="utf-8")

    def scalar(key, required=False, default=None):
        match = re.search(rf"^\s*{re.escape(key)}\s*:\s*(.*?)\s*$", text, re.M)
        if not match:
            if required:
                raise ValueError(f"YAML is missing required field '{key}'.")
            return default
        return match.group(1).split("#", 1)[0].strip().strip("'\"")

    image = scalar("image", required=True)
    negate = int(scalar("negate", default="0"))
    occupied = float(scalar("occupied_thresh", default="0.65"))
    free = float(scalar("free_thresh", default="0.25"))
    if negate not in (0, 1) or not (0 <= free < occupied <= 1):
        raise ValueError("Map YAML has invalid negate or occupancy thresholds.")
    if scalar("mode", default="trinary") != "trinary":
        raise ValueError("Only trinary occupancy maps are supported.")
    if not image or Path(image).is_absolute() or ".." in Path(image).parts:
        raise ValueError("YAML image must be a local filename beside the YAML.")
    return text, image, negate, occupied, free


def read_pgm(path):
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
            raise ValueError("PGM header or raster is incomplete.")
        return data[start:offset].decode("ascii")

    magic = token()
    if magic not in ("P2", "P5"):
        raise ValueError("Only P2 and P5 PGM images are supported.")
    width, height, max_value = int(token()), int(token()), int(token())
    count = width * height
    if width <= 0 or height <= 0 or count > MAX_CELLS or not 1 <= max_value <= 255:
        raise ValueError("PGM dimensions or maximum sample are unsupported.")
    if magic == "P5":
        if offset >= len(data) or data[offset] not in whitespace:
            raise ValueError("P5 raster separator is missing.")
        separator = data[offset]
        offset += 1
        if separator == 13 and offset < len(data) and data[offset] == 10:
            offset += 1
        pixels = data[offset:offset + count]
        if len(pixels) != count:
            raise ValueError("P5 raster is incomplete.")
        if max_value != 255:
            pixels = bytes(round(value * 255 / max_value) for value in pixels)
    else:
        samples = [int(token()) for _ in range(count)]
        if any(value < 0 or value > max_value for value in samples):
            raise ValueError("P2 raster has a sample outside its declared range.")
        pixels = bytes(round(value * 255 / max_value) for value in samples)
    return width, height, pixels


def filter_map(image_path, yaml_path, output_prefix, min_neighbors=2):
    image_path = image_path.resolve()
    yaml_path = yaml_path.resolve()
    output_prefix = output_prefix.resolve()
    yaml_text, image_name, negate, occupied_threshold, free_threshold = read_map_yaml(yaml_path)
    expected_image = (yaml_path.parent / image_name).resolve()
    if expected_image != image_path:
        raise ValueError(f"YAML references {image_name!r}, not the selected image.")
    output_image = output_prefix.with_suffix(".pgm")
    output_yaml = output_prefix.with_suffix(".yaml")
    if output_image == image_path or output_yaml == yaml_path:
        raise ValueError("Refusing to overwrite source map files; choose a new output prefix.")
    if output_image.exists() or output_yaml.exists():
        raise FileExistsError("Output map candidate already exists; choose a new prefix.")
    if min_neighbors < 0 or min_neighbors > 8:
        raise ValueError("--min-neighbors must be between 0 and 8.")

    width, height, source = read_pgm(image_path)
    output = bytearray(source)

    def is_occupied(value):
        probability = value / 255 if negate else 1 - value / 255
        return probability > occupied_threshold

    changed = 0
    for y in range(height):
        for x in range(width):
            index = y * width + x
            if not is_occupied(source[index]):
                continue
            neighbors = 0
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height and is_occupied(source[ny * width + nx]):
                        neighbors += 1
            if neighbors < min_neighbors:
                output[index] = 205  # ROS trinary unknown; never convert noise to free space.
                changed += 1

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    with output_image.open("xb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(output)
    yaml_out = re.sub(r"^(\s*image\s*:\s*).*$",
                      lambda match: match.group(1) + output_image.name,
                      yaml_text, count=1, flags=re.M)
    with output_yaml.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(yaml_out)
    return output_image, output_yaml, changed, width * height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Source PGM map")
    parser.add_argument("yaml", type=Path, help="Matching source YAML")
    parser.add_argument("output_prefix", type=Path, help="New candidate path without suffixes")
    parser.add_argument("--min-neighbors", type=int, default=2,
                        help="Keep occupied cells with at least this many occupied neighbors (default: 2)")
    args = parser.parse_args()
    try:
        image, yaml_file, changed, total = filter_map(
            args.image, args.yaml, args.output_prefix, args.min_neighbors)
    except (OSError, UnicodeError, ValueError, OverflowError) as error:
        parser.exit(2, f"map-filter: {error}\n")
    print(f"Wrote automatic cleanup candidate: {image} and {yaml_file}")
    print(f"Changed {changed} isolated occupied cells to unknown; {total} total cells.")
    print("Original files are untouched; this candidate is not navigation-approved.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
