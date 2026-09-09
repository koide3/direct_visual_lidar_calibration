"""MEI review rendering in the original image coordinate system."""
import cv2
import numpy as np

from mei_calibration_geometry import project_mei


def shared_color_range(points, intensity, transforms, camera, mask, options):
    if options["color_range"] is not None:
        return list(options["color_range"])
    values = []
    for transform in transforms:
        _, valid, distance = project_mei(points, transform, camera, mask)
        values.append((distance if options["color_by"] == "range" else intensity)[valid])
    values = np.concatenate(values)
    if not len(values):
        return [0.0, 1.0]
    low, high = np.quantile(values, [0.05, 0.95])
    return [float(low), float(max(high, low + 1e-6))]


def render_overlay(image, points, intensity, transform, camera, mask, options):
    pixels, valid, distance = project_mei(points, transform, camera, mask)
    indices = np.flatnonzero(valid)
    unique_pixels = 0
    if len(indices):
        knots = np.floor(pixels[indices]).astype(np.int64)
        order = np.argsort(distance[indices], kind="stable")
        keys = knots[order, 1] * camera["width"] + knots[order, 0]
        _, first = np.unique(keys, return_index=True)
        indices = indices[order[first]]
        unique_pixels = len(indices)
        cap = options["max_points"]
        if cap and len(indices) > cap:
            indices = indices[np.linspace(0, len(indices) - 1, cap, dtype=np.int64)]
        indices = indices[np.argsort(-distance[indices], kind="stable")]
    layer = image.copy()
    if len(indices):
        near, far = options["color_range"]
        values = (distance if options["color_by"] == "range" else intensity)[indices]
        colors = cv2.applyColorMap(np.uint8(np.clip((values - near) / max(far - near, 1e-6), 0, 1) * 255).reshape(-1, 1),
                                  cv2.COLORMAP_TURBO).reshape(-1, 3)
        for i, color in zip(indices, colors):
            cv2.circle(layer, tuple(np.floor(pixels[i]).astype(int)), options["point_radius_px"],
                       tuple(int(c) for c in color), -1, cv2.LINE_AA)
    result = cv2.addWeighted(layer, options["alpha"], image, 1 - options["alpha"], 0)
    if mask is not None:
        result[mask == 0] = image[mask == 0]
    return result, dict(input_points=len(points), valid_points=int(valid.sum()),
                        unique_pixels=unique_pixels, drawn_points=len(indices))
