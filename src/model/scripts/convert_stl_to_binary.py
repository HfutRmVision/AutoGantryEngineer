#!/usr/bin/env python3

from __future__ import annotations

import argparse
import struct
from pathlib import Path


def is_probably_binary_stl(path: Path) -> bool:
    header = path.read_bytes()[:512]
    if b"\x00" in header:
        return True
    try:
        header.decode("utf-8")
    except UnicodeDecodeError:
        return True
    return False


def parse_ascii_stl(path: Path) -> tuple[str, list[tuple[tuple[float, float, float], list[tuple[float, float, float]]]]]:
    text = path.read_text(encoding="utf-8", errors="strict")
    lines = text.splitlines()

    solid_name = path.stem
    triangles: list[tuple[tuple[float, float, float], list[tuple[float, float, float]]]] = []

    current_normal: tuple[float, float, float] | None = None
    current_vertices: list[tuple[float, float, float]] = []
    started = False

    for raw_line in lines:
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("solid"):
            if not started:
                started = True
                maybe_name = line[5:].strip()
                if maybe_name:
                    solid_name = maybe_name
            continue
        if not started:
            continue
        if line.startswith("endsolid"):
            break
        if line.startswith("facet normal"):
            parts = line.split()
            current_normal = (float(parts[2]), float(parts[3]), float(parts[4]))
            current_vertices = []
            continue
        if line.startswith("vertex"):
            parts = line.split()
            current_vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            continue
        if line == "endfacet":
            if current_normal is None or len(current_vertices) != 3:
                raise ValueError(f"Malformed facet in {path}")
            triangles.append((current_normal, current_vertices))
            current_normal = None
            current_vertices = []

    if not triangles:
        raise ValueError(f"No ASCII STL triangles found in {path}")

    return solid_name, triangles


def write_binary_stl(
    output_path: Path,
    solid_name: str,
    triangles: list[tuple[tuple[float, float, float], list[tuple[float, float, float]]]],
) -> None:
    header_text = f"Binary STL generated from {solid_name}"
    header = header_text.encode("ascii", errors="ignore")[:80].ljust(80, b" ")

    with output_path.open("wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(triangles)))
        for normal, vertices in triangles:
            f.write(struct.pack("<3f", *normal))
            for vertex in vertices:
                f.write(struct.pack("<3f", *vertex))
            f.write(struct.pack("<H", 0))


def convert_file(path: Path, output_dir: Path | None, in_place: bool) -> str:
    if is_probably_binary_stl(path):
        return f"skip(binary): {path}"

    solid_name, triangles = parse_ascii_stl(path)

    if in_place:
        output_path = path
    else:
        if output_dir is None:
            raise ValueError("output_dir is required when not converting in place")
        output_path = output_dir / path.name
        output_path.parent.mkdir(parents=True, exist_ok=True)

    temp_path = output_path.with_suffix(output_path.suffix + ".tmp")
    write_binary_stl(temp_path, solid_name, triangles)
    temp_path.replace(output_path)
    return f"converted({len(triangles)} triangles): {path} -> {output_path}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert ASCII STL files to binary STL.")
    parser.add_argument(
        "mesh_dir",
        nargs="?",
        default="src/model/meshes",
        help="Directory containing STL files. Default: src/model/meshes",
    )
    parser.add_argument(
        "--output-dir",
        help="Write converted STL files to a separate directory. Defaults to in-place conversion.",
    )
    parser.add_argument(
        "--pattern",
        default="*.STL",
        help="Glob pattern used to select STL files. Default: *.STL",
    )
    args = parser.parse_args()

    mesh_dir = Path(args.mesh_dir)
    if not mesh_dir.is_dir():
        raise SystemExit(f"Mesh directory does not exist: {mesh_dir}")

    output_dir = Path(args.output_dir) if args.output_dir else None
    in_place = output_dir is None

    converted = 0
    skipped = 0
    failed = 0

    for path in sorted(mesh_dir.glob(args.pattern)):
        if not path.is_file():
            continue
        try:
            result = convert_file(path, output_dir, in_place)
            print(result)
            if result.startswith("converted"):
                converted += 1
            else:
                skipped += 1
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"failed: {path}: {exc}")

    print(f"summary: converted={converted} skipped={skipped} failed={failed}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
