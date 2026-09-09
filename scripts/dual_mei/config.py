"""Validated configuration shared by both input modes."""
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from mei_calibration_geometry import rigid_transform
from static_bag_input import _configuration
from .bag_readers import validate_bag

CAMERAS = ("cam0", "cam1")
VIS_DEFAULTS = dict(point_radius_px=3, alpha=0.8, max_points=0,
                    color_by="range", color_range=None)
POINT_DEFAULTS = dict(voxel_size_m=0.02, split_margin_px=8, min_camera_points=500)
SOLVER_DEFAULTS = dict(max_outer_iterations=10, max_inner_iterations=256,
                       calibrate_executable=None, num_threads=2)


def mapping(value, name, allowed=None):
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    if allowed is not None and set(value) - set(allowed):
        raise ValueError(f"{name}: unknown keys {sorted(set(value) - set(allowed))}")
    return dict(value)


def number(value, name, minimum=None, maximum=None, integer=False):
    if value is None or isinstance(value, bool):
        raise ValueError(f"{name}: fill in a numeric value")
    try:
        value = float(value)
    except (ValueError, TypeError):
        raise ValueError(f"{name}: expected a number") from None
    if not np.isfinite(value) or (minimum is not None and value < minimum) or (maximum is not None and value > maximum):
        raise ValueError(f"{name}: value out of range")
    if integer:
        if value != int(value):
            raise ValueError(f"{name}: expected an integer")
        return int(value)
    return value


def resolve_path(value, base, name, require_file=True):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name}: fill in the path")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = base / path
    path = path.resolve()
    if require_file and not path.is_file():
        raise ValueError(f"{name}: file does not exist: {path}")
    return path


def topic_name(value, name):
    if not isinstance(value, str) or not value.startswith("/") or any(c.isspace() for c in value):
        raise ValueError(f"{name}: expected an absolute ROS topic")
    return value


def visualization_config(value):
    result = dict(VIS_DEFAULTS)
    result.update(mapping(value, "visualization", VIS_DEFAULTS))
    result["point_radius_px"] = number(result["point_radius_px"], "point_radius_px", 1, 32, integer=True)
    result["max_points"] = number(result["max_points"], "max_points", 0, integer=True)
    result["alpha"] = number(result["alpha"], "alpha", 0, 1)
    if result["color_by"] not in ("range", "intensity"):
        raise ValueError("color_by must be range or intensity")
    if result["color_range"] is not None:
        values = result["color_range"]
        if not isinstance(values, (list, tuple)) or len(values) != 2:
            raise ValueError("color_range must be null or [minimum, maximum]")
        result["color_range"] = [number(x, "color_range") for x in values]
        if result["color_range"][0] >= result["color_range"][1]:
            raise ValueError("color_range must be increasing")
    return result


def validate_camera(entry, name, base):
    c = mapping(entry, name, {"model", "width", "height", "fx", "fy", "cx", "cy", "xi",
                             "distortion", "max_theta_deg", "mask_path", "T_cam_lidar"})
    if str(c.get("model", "")).lower() != "mei":
        raise ValueError(f"{name}.model must be mei")
    c["model"] = "mei"
    for key in ("width", "height"):
        c[key] = number(c.get(key), f"{name}.{key}", 8, integer=True)
    for key in ("fx", "fy", "cx", "cy", "xi"):
        c[key] = number(c.get(key), f"{name}.{key}")
    if min(c["fx"], c["fy"]) <= 0 or c["xi"] < 0:
        raise ValueError(f"{name}: fx/fy must be positive and xi nonnegative")
    c["max_theta_deg"] = number(c.get("max_theta_deg", 95), f"{name}.max_theta_deg", 1, 179)
    distortion = mapping(c.get("distortion"), f"{name}.distortion")
    if set(distortion) != {"k1", "k2", "k3", "p1", "p2"}:
        raise ValueError(f"{name}.distortion: supply k1,k2,k3,p1,p2, including explicit zeros")
    c["distortion"] = {key: number(value, f"{name}.{key}") for key, value in distortion.items()}
    transform = rigid_transform(c.pop("T_cam_lidar", None), f"{name}.T_cam_lidar")
    mask = None
    mask_path = c.pop("mask_path", None)
    if mask_path:
        mask_path = resolve_path(mask_path, base, f"{name}.mask_path")
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE | cv2.IMREAD_IGNORE_ORIENTATION)
        if mask is None or mask.shape != (c["height"], c["width"]) or not np.any(mask):
            raise ValueError(f"{name}: mask must be nonempty and match configured image dimensions")
        mask = np.uint8(mask != 0) * 255
    return dict(camera=c, transform=transform, mask=mask)


@dataclass
class CalibrationConfig:
    path: Path
    bag: Path
    input: dict
    static: dict
    cameras: dict
    image_selection: dict
    pointcloud: dict
    solver: dict
    output: Path
    visualization: dict


def upgrade_legacy(raw):
    """Version 1 retains external-image semantics, without hidden path remapping."""
    old = deepcopy(raw)
    allowed = {"schema_version", "bag_path", "output_dir", "static", "pointcloud", "solver", "cameras"}
    mapping(old, "config", allowed)
    cameras = mapping(old.get("cameras", {}), "cameras")
    cameras = {name: mapping(entry, name) for name, entry in cameras.items()}
    inputs = {name: dict(image_path=entry.pop("image_path", None))
              for name, entry in cameras.items()}
    static = mapping(old.get("static", {}), "static")
    input_cfg = dict(mode="bag_and_images", bag_path=old.get("bag_path"),
                     lidar_topic=static.pop("lidar_topic", "/livox/lidar"),
                     imu_topic=static.pop("imu_topic", "/livox/imu"), images=inputs)
    return dict(schema_version=2, input=input_cfg, cameras=cameras, static=static,
                pointcloud=old.get("pointcloud", {}), solver=old.get("solver", {}),
                output=dict(directory=old.get("output_dir")))


def load_config(path, check_only=False, output_override=None):
    path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    raw = mapping(raw, "config")
    if type(raw.get("schema_version")) is not int or raw["schema_version"] not in (1, 2):
        raise ValueError("schema_version must be 1 or 2")
    if raw["schema_version"] == 1:
        raw = upgrade_legacy(raw)
    mapping(raw, "config", {"schema_version", "input", "image_selection", "static", "cameras",
                            "pointcloud", "solver", "output", "visualization"})
    inp = mapping(raw.get("input"), "input",
                  {"mode", "bag_path", "lidar_topic", "imu_topic", "lidar_frame", "images"})
    if inp.get("mode") not in ("bag_only", "bag_and_images"):
        raise ValueError("input.mode must be bag_only or bag_and_images")
    bag = resolve_path(inp.get("bag_path"), path.parent, "input.bag_path", False)
    for key, default in (("lidar_topic", "/livox/lidar"), ("imu_topic", "/livox/imu")):
        inp[key] = topic_name(inp.get(key, default), f"input.{key}")
    inp["lidar_frame"] = inp.get("lidar_frame", "livox_frame")
    if not isinstance(inp["lidar_frame"], str) or not inp["lidar_frame"]:
        raise ValueError("input.lidar_frame must be nonempty")
    static = mapping(raw.get("static", {}), "static")
    if "lidar_topic" in static or "imu_topic" in static:
        raise ValueError("version 2 topics belong in input, not static")
    static.update(lidar_topic=inp["lidar_topic"], imu_topic=inp["imu_topic"])
    static = _configuration(static)
    validate_bag(bag, static["backend"])
    selection = dict(target_offset_sec=None, max_pair_delta_sec=0.0)
    selection.update(mapping(raw.get("image_selection", {}), "image_selection", selection))
    selection["max_pair_delta_sec"] = number(selection["max_pair_delta_sec"], "max_pair_delta_sec", 0)
    if selection["target_offset_sec"] is not None:
        offset = number(selection["target_offset_sec"], "target_offset_sec", 0)
        if offset >= static["duration_sec"]:
            raise ValueError("target_offset_sec must lie within the static interval")
        selection["target_offset_sec"] = offset
    pointcloud = dict(POINT_DEFAULTS)
    pointcloud.update(mapping(raw.get("pointcloud", {}), "pointcloud", POINT_DEFAULTS))
    pointcloud["voxel_size_m"] = number(pointcloud["voxel_size_m"], "voxel_size_m", 0)
    pointcloud["split_margin_px"] = number(pointcloud["split_margin_px"], "split_margin_px", 0, integer=True)
    pointcloud["min_camera_points"] = number(pointcloud["min_camera_points"], "min_camera_points", 1, integer=True)
    solver = dict(SOLVER_DEFAULTS)
    solver.update(mapping(raw.get("solver", {}), "solver", SOLVER_DEFAULTS))
    for key in ("max_outer_iterations", "max_inner_iterations", "num_threads"):
        solver[key] = number(solver[key], key, 1, integer=True)
    if solver["calibrate_executable"] is not None:
        solver["calibrate_executable"] = str(resolve_path(solver["calibrate_executable"], path.parent, "calibrate_executable"))
    output = mapping(raw.get("output", {}), "output", {"directory"})
    # A check-only run does not write output and allows an unfinished camera template.
    output_path = output_override or output.get("directory")
    if output_path is None and check_only:
        output_path = "."
    output_path = resolve_path(output_path, path.parent, "output.directory", False)
    vis = visualization_config(raw.get("visualization", {}))
    cameras = {}
    if not check_only:
        raw_cameras = mapping(raw.get("cameras"), "cameras")
        image_inputs = mapping(inp.get("images"), "input.images")
        if set(raw_cameras) != set(CAMERAS) or set(image_inputs) != set(CAMERAS):
            raise ValueError("cameras and input.images must each contain exactly cam0 and cam1")
        topics = {inp["lidar_topic"], inp["imu_topic"]}
        for name in CAMERAS:
            cameras[name] = validate_camera(raw_cameras[name], name, path.parent)
            source = mapping(image_inputs[name], f"input.images.{name}", {"image_path", "topic"})
            if inp["mode"] == "bag_and_images":
                source["image_path"] = str(resolve_path(source.get("image_path"), path.parent, f"{name}.image_path"))
            else:
                source["topic"] = topic_name(source.get("topic"), f"{name}.topic")
                if source["topic"] in topics:
                    raise ValueError("camera and sensor topics must all differ")
                topics.add(source["topic"])
            image_inputs[name] = source
        inp["images"] = image_inputs
    return CalibrationConfig(path, bag, inp, static, cameras, selection, pointcloud, solver, output_path, vis)
