"""MEI geometry and point-cloud operations shared by calibration and review."""
import numpy as np


def rigid_transform(value, name="T_cam_lidar"):
    """Validate a transform mapping column LiDAR coordinates to optical coordinates."""
    try:
        matrix = np.asarray(value, dtype=np.float64).reshape(4, 4)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name}: provide a complete 4x4 matrix in metres") from exc
    if not np.isfinite(matrix).all():
        raise ValueError(f"{name}: all 16 values must be finite")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f"{name}: last row must be [0,0,0,1]")
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or abs(np.linalg.det(rotation) - 1) > 1e-5):
        raise ValueError(f"{name}: rotation must be orthonormal with determinant +1")
    return matrix


def inverse_transform(matrix):
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def project_mei(points_lidar, transform, camera, mask=None, margin_px=0):
    """Project using the physical MEI branch; retain legal >90-degree rays.

    Return pixels, validity and camera ranges. All consumers, including overlays,
    use the same k3 formula and the same 4x4 interpolation mask footprint.
    """
    xyz = np.asarray(points_lidar) @ transform[:3, :3].T + transform[:3, 3]
    distance = np.linalg.norm(xyz, axis=1)
    unit = xyz / np.maximum(distance[:, None], 1e-15)
    xi = camera["xi"]
    denominator = unit[:, 2] + xi
    valid = np.isfinite(xyz).all(axis=1) & (distance > 1e-10)
    valid &= (denominator > 1e-10) & (1 + xi * unit[:, 2] > 1e-10)
    valid &= unit[:, 2] >= np.cos(np.deg2rad(camera["max_theta_deg"]))
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        x, y = (unit[:, :2] / denominator[:, None]).T
        radius2 = x*x + y*y
        d = camera["distortion"]
        radial = 1 + radius2 * (d["k1"] + radius2 * (d["k2"] + radius2*d["k3"]))
        xd = x*radial + 2*d["p1"]*x*y + d["p2"]*(radius2 + 2*x*x)
        yd = y*radial + d["p1"]*(radius2 + 2*y*y) + 2*d["p2"]*x*y
        pixels = np.column_stack((camera["fx"]*xd + camera["cx"],
                                  camera["fy"]*yd + camera["cy"]))
    valid &= np.isfinite(pixels).all(axis=1)
    # NID uses floor(u/v) + [-1, 0, 1, 2]. Keep its footprint in the image.
    border = max(1, int(margin_px))
    valid &= ((pixels[:, 0] >= border) & (pixels[:, 0] < camera["width"]-max(2, border))
              & (pixels[:, 1] >= border) & (pixels[:, 1] < camera["height"]-max(2, border)))
    if mask is not None:
        selected = np.flatnonzero(valid)
        knots = np.floor(pixels[selected]).astype(np.int64)
        ok = np.ones(len(selected), dtype=bool)
        for dy in (-1, 0, 1, 2):
            for dx in (-1, 0, 1, 2):
                ok &= mask[knots[:, 1]+dy, knots[:, 0]+dx] != 0
        valid[selected] &= ok
    return pixels, valid, distance


def voxel_downsample(points, intensity, voxel_size):
    if voxel_size <= 0:
        return points, intensity
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    count = np.bincount(inverse)
    xyz = np.column_stack([np.bincount(inverse, weights=points[:, axis])/count
                           for axis in range(3)])
    values = np.bincount(inverse, weights=intensity)/count
    return xyz, values


def equalize_intensity(intensity):
    """Empirical histogram equalization with equal values kept equal."""
    _, inverse, counts = np.unique(intensity, return_inverse=True, return_counts=True)
    if len(counts) < 2:
        raise ValueError("LiDAR reflectivity is constant; NID calibration has no intensity information")
    cdf = np.cumsum(counts).astype(float)
    cdf = (cdf-cdf[0])/(cdf[-1]-cdf[0])
    return cdf[inverse]


# Compatibility for existing numerical callers; artifact I/O lives in its own module.
from dual_mei.artifacts import write_ply, save_image  # noqa: E402,F401
