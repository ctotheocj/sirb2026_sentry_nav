#!/usr/bin/env python3
"""Generate a Nav2 occupancy map from a CAD-derived PCD in the same frame."""

import argparse
import math
import os
import struct
import sys
from dataclasses import dataclass
from typing import Iterator


@dataclass
class PcdHeader:
    fields: list[str]
    sizes: list[int]
    types: list[str]
    counts: list[int]
    width: int
    height: int
    points: int
    data: str
    data_offset: int


def _split_values(line: str) -> list[str]:
    return line.strip().split()[1:]


def read_pcd_header(path: str) -> PcdHeader:
    fields: list[str] | None = None
    sizes: list[int] | None = None
    types: list[str] | None = None
    counts: list[int] | None = None
    width = 0
    height = 1
    points = 0
    data = ""
    data_offset = 0

    with open(path, "rb") as f:
      while True:
        line = f.readline()
        if not line:
            raise ValueError("PCD header ended before DATA line")
        data_offset = f.tell()
        text = line.decode("ascii", errors="strict").strip()
        if not text or text.startswith("#"):
            continue
        key = text.split(maxsplit=1)[0].upper()
        if key == "FIELDS":
            fields = _split_values(text)
        elif key == "SIZE":
            sizes = [int(v) for v in _split_values(text)]
        elif key == "TYPE":
            types = _split_values(text)
        elif key == "COUNT":
            counts = [int(v) for v in _split_values(text)]
        elif key == "WIDTH":
            width = int(_split_values(text)[0])
        elif key == "HEIGHT":
            height = int(_split_values(text)[0])
        elif key == "POINTS":
            points = int(_split_values(text)[0])
        elif key == "DATA":
            data = _split_values(text)[0].lower()
            break

    if fields is None:
        raise ValueError("PCD is missing FIELDS")
    if sizes is None:
        sizes = [4] * len(fields)
    if types is None:
        types = ["F"] * len(fields)
    if counts is None:
        counts = [1] * len(fields)
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD FIELDS/SIZE/TYPE/COUNT lengths do not match")
    if points <= 0:
        points = width * height
    if points <= 0:
        raise ValueError("PCD POINTS or WIDTH/HEIGHT is invalid")
    if data not in ("ascii", "binary"):
        raise ValueError(f"unsupported PCD DATA mode: {data}")
    for axis in ("x", "y", "z"):
        if axis not in fields:
            raise ValueError(f"PCD is missing field '{axis}'")
    return PcdHeader(fields, sizes, types, counts, width, height, points, data, data_offset)


def _field_offsets(header: PcdHeader) -> tuple[int, int, int, int]:
    offset = 0
    offsets: dict[str, int] = {}
    for field, size, count in zip(header.fields, header.sizes, header.counts):
        offsets[field] = offset
        offset += size * count
    return offsets["x"], offsets["y"], offsets["z"], offset


def iter_pcd_xyz(path: str, header: PcdHeader) -> Iterator[tuple[float, float, float]]:
    x_idx = header.fields.index("x")
    y_idx = header.fields.index("y")
    z_idx = header.fields.index("z")
    if header.data == "ascii":
        with open(path, "rb") as f:
            f.seek(header.data_offset)
            yielded = 0
            for raw in f:
                if yielded >= header.points:
                    break
                if not raw.strip():
                    continue
                values = raw.decode("ascii", errors="ignore").strip().split()
                if len(values) <= max(x_idx, y_idx, z_idx):
                    continue
                try:
                    point = (float(values[x_idx]), float(values[y_idx]), float(values[z_idx]))
                except ValueError:
                    continue
                if all(math.isfinite(v) for v in point):
                    yielded += 1
                    yield point
        return

    x_offset, y_offset, z_offset, point_step = _field_offsets(header)
    x_type = header.types[x_idx].upper()
    y_type = header.types[y_idx].upper()
    z_type = header.types[z_idx].upper()
    x_size = header.sizes[x_idx]
    y_size = header.sizes[y_idx]
    z_size = header.sizes[z_idx]
    if (x_type, y_type, z_type) != ("F", "F", "F") or (x_size, y_size, z_size) not in (
        (4, 4, 4),
        (8, 8, 8),
    ):
        raise ValueError("binary PCD x/y/z fields must be float32 or float64")
    fmt = "<f" if x_size == 4 else "<d"
    with open(path, "rb") as f:
        f.seek(header.data_offset)
        for _ in range(header.points):
            chunk = f.read(point_step)
            if len(chunk) != point_step:
                break
            x = struct.unpack_from(fmt, chunk, x_offset)[0]
            y = struct.unpack_from(fmt, chunk, y_offset)[0]
            z = struct.unpack_from(fmt, chunk, z_offset)[0]
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                yield (x, y, z)


def collect_projected_points(path: str, z_min: float, z_max: float) -> list[tuple[float, float]]:
    header = read_pcd_header(path)
    points: list[tuple[float, float]] = []
    for x, y, z in iter_pcd_xyz(path, header):
        if z_min <= z <= z_max:
            points.append((x, y))
    if not points:
        raise ValueError(
            f"no PCD points remain after z filtering [{z_min}, {z_max}]; "
            "adjust --z-min/--z-max to match the CAD/field units"
        )
    return points


def dilate_cells(cells: set[tuple[int, int]], radius: int, width: int, height: int) -> set[tuple[int, int]]:
    if radius <= 0:
        return cells
    out: set[tuple[int, int]] = set()
    r2 = radius * radius
    offsets = [
        (dx, dy)
        for dy in range(-radius, radius + 1)
        for dx in range(-radius, radius + 1)
        if dx * dx + dy * dy <= r2
    ]
    for cx, cy in cells:
        for dx, dy in offsets:
            nx = cx + dx
            ny = cy + dy
            if 0 <= nx < width and 0 <= ny < height:
                out.add((nx, ny))
    return out


def write_pgm(path: str, pixels: bytearray, width: int, height: int) -> None:
    with open(path, "wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        f.write(pixels)


def write_yaml(path: str, image_name: str, resolution: float, origin_x: float, origin_y: float) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"image: {image_name}\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {resolution:.8g}\n")
        f.write(f"origin: [{origin_x:.8g}, {origin_y:.8g}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")


def generate_map(args: argparse.Namespace) -> None:
    points = collect_projected_points(args.pcd, args.z_min, args.z_max)
    min_x = min(x for x, _ in points) - args.padding
    max_x = max(x for x, _ in points) + args.padding
    min_y = min(y for _, y in points) - args.padding
    max_y = max(y for _, y in points) + args.padding
    width = max(1, int(math.ceil((max_x - min_x) / args.resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / args.resolution)))
    if width * height > args.max_cells:
        raise ValueError(
            f"map would be {width}x{height}={width * height} cells; "
            f"raise --max-cells or use a coarser --resolution"
        )

    occupied: set[tuple[int, int]] = set()
    for x, y in points:
        cx = int(math.floor((x - min_x) / args.resolution))
        cy = int(math.floor((y - min_y) / args.resolution))
        if 0 <= cx < width and 0 <= cy < height:
            occupied.add((cx, cy))
    occupied = dilate_cells(occupied, args.dilate_cells, width, height)

    pixels = bytearray([254] * (width * height))
    for cx, cy in occupied:
        row = height - 1 - cy
        pixels[row * width + cx] = 0

    out_stem = os.path.abspath(args.out_stem)
    out_dir = os.path.dirname(out_stem) or "."
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = out_stem + ".pgm"
    yaml_path = out_stem + ".yaml"
    write_pgm(pgm_path, pixels, width, height)
    write_yaml(yaml_path, os.path.basename(pgm_path), args.resolution, min_x, min_y)
    print(f"wrote {yaml_path}")
    print(f"wrote {pgm_path}")
    print(
        f"origin=[{min_x:.3f}, {min_y:.3f}, 0.0] size={width}x{height} "
        f"resolution={args.resolution:g} occupied_cells={len(occupied)}"
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Project an STL/CAD-derived PCD into a Nav2 occupancy map. "
            "Use the resulting YAML with the same PCD so map and prior cloud share one coordinate frame."
        )
    )
    parser.add_argument("--pcd", required=True, help="Input PCD path from CloudCompare/CAD sampling")
    parser.add_argument("--out-stem", required=True, help="Output path without .yaml/.pgm suffix")
    parser.add_argument("--resolution", type=float, default=0.05, help="Map resolution in meters/cell")
    parser.add_argument("--z-min", type=float, default=0.05, help="Minimum z used as obstacle")
    parser.add_argument("--z-max", type=float, default=1.80, help="Maximum z used as obstacle")
    parser.add_argument("--padding", type=float, default=0.50, help="Free border around projected PCD")
    parser.add_argument("--dilate-cells", type=int, default=2, help="Inflate occupied cells in the output map")
    parser.add_argument("--max-cells", type=int, default=80_000_000, help="Safety limit for generated grid cells")
    args = parser.parse_args(argv)
    if args.resolution <= 0.0:
        parser.error("--resolution must be > 0")
    if args.z_min > args.z_max:
        parser.error("--z-min must be <= --z-max")
    if args.padding < 0.0:
        parser.error("--padding must be >= 0")
    if args.dilate_cells < 0:
        parser.error("--dilate-cells must be >= 0")
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        generate_map(args)
    except Exception as exc:
        print(f"pcd_to_nav2_map: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
