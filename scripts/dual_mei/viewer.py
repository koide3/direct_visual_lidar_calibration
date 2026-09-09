"""Standalone result review, with headless export and an OpenCV pan/zoom viewer."""
import os
from pathlib import Path

import cv2
import numpy as np
import yaml

from mei_calibration_geometry import rigid_transform
from .artifacts import read_ply, save_image
from .config import visualization_config
from .images import read_external
from .rendering import render_overlay, shared_color_range


def load_result(directory, overrides=None):
    directory = Path(directory).expanduser().resolve()
    config_path = directory / "extrinsics.yaml"
    if not config_path.is_file():
        config_path = directory / "prepared.yaml"
    result = yaml.safe_load(config_path.read_text())
    if result.get("schema_version") != 2 or result.get("direction") != "lidar_to_camera":
        raise ValueError("expected a version 2 LiDAR-to-camera result")
    if result.get("status") not in ("success", "prepared") or set(result.get("cameras", {})) != {"cam0", "cam1"}:
        raise ValueError("result must contain both cameras with success or prepared status")
    data = {}
    for name, entry in result["cameras"].items():
        def member(value):
            path = (directory / value).resolve()
            try:
                path.relative_to(directory)
            except ValueError:
                raise ValueError("review data must reside inside its result directory") from None
            return path
        image = read_external(member(entry["image"])).image
        points, intensity = read_ply(member(entry["pointcloud"]))
        camera = entry["camera"]
        if image.shape[:2] != (camera["height"], camera["width"]):
            raise ValueError(f"{name}: saved image and intrinsics have different dimensions")
        mask = None
        if entry.get("mask"):
            mask = cv2.imread(str(member(entry["mask"])), cv2.IMREAD_GRAYSCALE)
            if mask is None or mask.shape != image.shape[:2] or not np.any(mask):
                raise ValueError(f"{name}: invalid saved mask")
        transforms = {"before": rigid_transform(entry["T_cam_lidar_initial"])}
        if result["status"] == "success":
            transforms["after"] = rigid_transform(entry["T_cam_lidar"])
        options = dict(entry["visualization"])
        options.update(overrides or {})
        # Changing color semantics requires a new scale unless one was explicitly supplied.
        if overrides and "color_by" in overrides and "color_range" not in overrides:
            options["color_range"] = None
        options = visualization_config(options)
        options["color_range"] = shared_color_range(points, intensity, transforms.values(), camera, mask, options)
        data[name] = dict(image=image, points=points, intensity=intensity, camera=camera,
                          mask=mask, transforms=transforms, options=options)
    return directory, result, data


def render(data, stage, options=None):
    return render_overlay(data["image"], data["points"], data["intensity"],
                          data["transforms"][stage], data["camera"], data["mask"],
                          options or data["options"])


def export_result(directory, overrides=None, output=None):
    source, result, cameras = load_result(directory, overrides)
    target = Path(output).expanduser().resolve() if output else source
    statistics = {}
    for name, data in cameras.items():
        (target / name).mkdir(parents=True, exist_ok=True)
        statistics[name] = {}
        for stage in data["transforms"]:
            image, stats = render(data, stage)
            save_image(target / name / f"overlay_{stage}.png", image)
            statistics[name][stage] = stats
    return statistics


class Viewport:
    """Render a bounded viewport; zoom never allocates a scaled full-size image."""
    def __init__(self, image_shape, width=1280, height=900):
        self.width, self.height = width, height
        self.image_height, self.image_width = image_shape[:2]
        self.fit = min(width / self.image_width, height / self.image_height)
        self.reset()

    def reset(self):
        self.center = np.array([self.image_width / 2, self.image_height / 2], dtype=float)
        self.zoom = 1.0

    @property
    def scale(self):
        return self.fit * self.zoom

    def image_position(self, x, y):
        return self.center + (np.array([x, y]) - [self.width / 2, self.height / 2]) / self.scale

    def zoom_at(self, x, y, factor):
        position = self.image_position(x, y)
        self.zoom = float(np.clip(self.zoom * factor, 0.25, 32))
        self.center = position - (np.array([x, y]) - [self.width / 2, self.height / 2]) / self.scale

    def pan(self, dx, dy):
        self.center -= np.array([dx, dy]) / self.scale

    def display(self, image):
        scale = self.scale
        matrix = np.array([[scale, 0, self.width / 2 - self.center[0] * scale],
                           [0, scale, self.height / 2 - self.center[1] * scale]])
        return cv2.warpAffine(image, matrix, (self.width, self.height), flags=cv2.INTER_LINEAR)


def show_result(directory, overrides=None):
    if os.name != "nt" and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise ValueError("interactive review needs a desktop display; use --render-only for headless export")
    source, result, cameras = load_result(directory, overrides)
    window = "Dual MEI: zoom slider or +/- | drag pan | 0/1 camera | B/A before/after | R reset | S save | Q quit"
    first = cameras["cam0"]
    viewport = Viewport(first["image"].shape)
    state = dict(drag=None, camera="cam0", stage="after" if result["status"] == "success" else "before")
    cache = {}
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar("zoom %", window, 100, 3200, lambda value: None)
    cv2.createTrackbar("radius px", window, first["options"]["point_radius_px"], 32, lambda value: None)
    cv2.createTrackbar("opacity %", window, round(first["options"]["alpha"] * 100), 100, lambda value: None)

    def mouse(event, x, y, flags, unused):
        if event == cv2.EVENT_MOUSEWHEEL:
            viewport.zoom_at(x, y, 1.25 if flags > 0 else 0.8)
            cv2.setTrackbarPos("zoom %", window, round(viewport.zoom * 100))
        elif event == cv2.EVENT_LBUTTONDOWN:
            state["drag"] = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            state["drag"] = None
        elif event == cv2.EVENT_MOUSEMOVE and state["drag"] is not None:
            px, py = state["drag"]
            viewport.pan(x - px, y - py)
            state["drag"] = (x, y)

    cv2.setMouseCallback(window, mouse)
    try:
        # GTK reports invisible until the first imshow/event pump.
        while True:
            name, stage = state["camera"], state["stage"]
            data = cameras[name]
            if stage not in data["transforms"]:
                stage = state["stage"] = "before"
            viewport.zoom = max(25, cv2.getTrackbarPos("zoom %", window)) / 100
            options = dict(data["options"])
            options["point_radius_px"] = max(1, cv2.getTrackbarPos("radius px", window))
            options["alpha"] = cv2.getTrackbarPos("opacity %", window) / 100
            key = (name, stage, options["point_radius_px"], options["alpha"])
            if key not in cache:
                cache.clear()  # Keep one full-resolution overlay in addition to the source images.
                cache[key] = render(data, stage, options)
            overlay, stats = cache[key]
            canvas = viewport.display(overlay)
            text = f"{name} {stage} | zoom {viewport.zoom:.2f} | points {stats['drawn_points']}/{stats['input_points']} | {options['color_by']} {options['color_range'][0]:.2f}..{options['color_range'][1]:.2f}"
            cv2.rectangle(canvas, (0, 0), (viewport.width, 32), (20, 20, 20), -1)
            cv2.putText(canvas, text, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(window, canvas)
            key_code = cv2.waitKey(30) & 0xff
            if key_code in (27, ord("q")):
                break
            try:
                # GTK does not implement WND_PROP_VISIBLE; AUTOSIZE is supported.
                if cv2.getWindowProperty(window, cv2.WND_PROP_AUTOSIZE) < 0:
                    break
            except cv2.error:
                break
            if key_code in (ord("0"), ord("1")):
                state["camera"] = "cam" + chr(key_code)
                if cameras[state["camera"]]["image"].shape != data["image"].shape:
                    viewport = Viewport(cameras[state["camera"]]["image"].shape)
            elif key_code == ord("b"):
                state["stage"] = "before"
            elif key_code == ord("a"):
                state["stage"] = "after"
            elif key_code in (ord("+"), ord("="), ord("-")):
                viewport.zoom_at(viewport.width / 2, viewport.height / 2,
                                 0.8 if key_code == ord("-") else 1.25)
                cv2.setTrackbarPos("zoom %", window, round(viewport.zoom * 100))
            elif key_code == ord("r"):
                viewport.reset()
                cv2.setTrackbarPos("zoom %", window, 100)
            elif key_code == ord("s"):
                path = source / name / f"overlay_{stage}.png"
                save_image(path, overlay)
                print(f"Saved {path}", flush=True)
    finally:
        try:
            cv2.destroyWindow(window)
        except cv2.error:
            pass  # The window may already have been closed using its title bar.
