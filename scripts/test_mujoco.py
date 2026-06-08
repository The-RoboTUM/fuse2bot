#!/usr/bin/env python3
"""
Load a MuJoCo MJCF XML and open the interactive viewer.

Usage:
    python scripts/test_mujoco.py <path_to.xml> [--decimate-stl] [--convert-stl]
"""

import argparse
import os
import random
import struct
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer

MAX_FACES = 200_000


# ---------------------------------------------------------------------------
# STL utilities
# ---------------------------------------------------------------------------

def is_ascii_stl(filepath):
    """Check if an STL file is ASCII format (starts with 'solid')."""
    try:
        with open(filepath, "rb") as f:
            header = f.read(80)
        return header.lstrip().startswith(b"solid")
    except Exception:
        return False


def get_stl_face_count(filepath):
    """Read the face count from a binary STL header."""
    try:
        with open(filepath, "rb") as f:
            f.read(80)
            return struct.unpack("<I", f.read(4))[0]
    except Exception:
        return 0


def read_binary_stl(filepath):
    """Read a binary STL and return list of (normal, [v0, v1, v2]) tuples."""
    with open(filepath, "rb") as f:
        f.read(80)  # header
        num_faces = struct.unpack("<I", f.read(4))[0]
        triangles = []
        for _ in range(num_faces):
            n = struct.unpack("<3f", f.read(12))
            v0 = struct.unpack("<3f", f.read(12))
            v1 = struct.unpack("<3f", f.read(12))
            v2 = struct.unpack("<3f", f.read(12))
            f.read(2)  # attribute
            triangles.append((n, [v0, v1, v2]))
    return triangles


def write_binary_stl(filepath, triangles):
    """Write a list of (normal, [v0, v1, v2]) tuples as a binary STL."""
    with open(filepath, "wb") as f:
        f.write(b"\x00" * 80)
        f.write(struct.pack("<I", len(triangles)))
        for normal, verts in triangles:
            f.write(struct.pack("<3f", *normal))
            for v in verts:
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))


def convert_ascii_stl_to_binary(filepath):
    """Convert an ASCII STL file to binary STL in-place."""
    triangles = []
    with open(filepath, "r", errors="replace") as f:
        normal = (0.0, 0.0, 0.0)
        verts = []
        for line in f:
            line = line.strip()
            if line.startswith("facet normal"):
                parts = line.split()
                normal = (float(parts[2]), float(parts[3]), float(parts[4]))
                verts = []
            elif line.startswith("vertex"):
                parts = line.split()
                verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith("endfacet"):
                if len(verts) == 3:
                    triangles.append((normal, verts))
    if not triangles:
        return
    write_binary_stl(filepath, triangles)


def decimate_stl(filepath, max_faces=MAX_FACES):
    """Decimate a binary STL in-place via uniform random subsampling."""
    face_count = get_stl_face_count(filepath)
    if face_count <= max_faces:
        return face_count, face_count

    triangles = read_binary_stl(filepath)
    random.seed(42)  # reproducible
    decimated = random.sample(triangles, max_faces)
    write_binary_stl(filepath, decimated)
    return face_count, max_faces


# ---------------------------------------------------------------------------
# Batch processing
# ---------------------------------------------------------------------------

def resolve_mesh_paths(xml_path):
    """Yield (filename, absolute_filepath) for every mesh in the MJCF."""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    xml_dir = os.path.dirname(os.path.abspath(xml_path))

    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""

    for mesh in root.iter("mesh"):
        filename = mesh.get("file", "")
        if not filename:
            continue
        filepath = os.path.join(xml_dir, meshdir, filename)
        if os.path.isfile(filepath):
            yield filename, filepath


def process_stl_files(xml_path, convert=False, decimate=False):
    """Find all STL meshes referenced by an MJCF XML and process them."""
    converted = 0
    decimated_count = 0

    for filename, filepath in resolve_mesh_paths(xml_path):
        if convert and is_ascii_stl(filepath):
            print(f"  Converting ASCII -> binary: {filename}")
            convert_ascii_stl_to_binary(filepath)
            converted += 1

        if decimate:
            face_count = get_stl_face_count(filepath)
            if face_count > MAX_FACES:
                orig, final = decimate_stl(filepath, MAX_FACES)
                print(f"  Decimated {filename}: {orig:,} -> {final:,} faces")
                decimated_count += 1

    if convert:
        print(f"  Converted {converted} ASCII STL file(s).")
    if decimate:
        print(f"  Decimated {decimated_count} oversized STL file(s) to <= {MAX_FACES:,} faces.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Load MJCF and open MuJoCo viewer")
    parser.add_argument("xml", help="Path to the MuJoCo XML file")
    parser.add_argument("--convert-stl", action="store_true",
                        help="Auto-convert ASCII STL files to binary (in-place)")
    parser.add_argument("--decimate-stl", action="store_true",
                        help="Decimate STL files exceeding 200k faces (in-place)")
    args = parser.parse_args()

    if args.convert_stl or args.decimate_stl:
        process_stl_files(args.xml, convert=args.convert_stl, decimate=args.decimate_stl)

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)

    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
