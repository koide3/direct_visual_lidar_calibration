"""Minimal artifact I/O; no geometry or solver dependencies."""
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_yaml(path, value):
    Path(path).write_text(yaml.safe_dump(value, sort_keys=False, allow_unicode=True), encoding="utf-8")


def write_ply(path, points, intensity):
    """The exact binary float32 XYZI representation consumed by the native solver."""
    records = np.column_stack((points, intensity)).astype("<f4")
    if records.ndim != 2 or records.shape[1] != 4 or not np.isfinite(records).all():
        raise ValueError("PLY requires finite Nx3 points and N intensities")
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(points)}\nproperty float x\nproperty float y\n"
              "property float z\nproperty float intensity\nend_header\n")
    with Path(path).open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(records.tobytes())


def read_ply(path):
    """Read the explicit XYZI format exported by this workflow."""
    with Path(path).open("rb") as stream:
        header = []
        for _ in range(32):
            line = stream.readline()
            if not line:
                raise ValueError(f"truncated PLY header: {path}")
            header.append(line.decode("ascii").strip())
            if header[-1] == "end_header":
                break
        if header[:2] != ["ply", "format binary_little_endian 1.0"] or header[-1] != "end_header":
            raise ValueError("expected binary little-endian XYZI PLY")
        if header[3:7] != [f"property float {x}" for x in ("x", "y", "z", "intensity")]:
            raise ValueError("unexpected PLY properties")
        if not header[2].startswith("element vertex "):
            raise ValueError("missing PLY vertex count")
        count = int(header[2].split()[2])
        if count < 1:
            raise ValueError("PLY point count must be positive")
        payload = stream.read()
    if len(payload) != count * 16:
        raise ValueError("PLY payload does not match vertex count")
    records = np.frombuffer(payload, dtype="<f4").reshape(count, 4)
    if not np.isfinite(records).all():
        raise ValueError("PLY contains non-finite data")
    return records[:, :3].astype(np.float64), records[:, 3].astype(np.float64)


def save_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise OSError(f"could not write image: {path}")
