#!/usr/bin/env python3
"""Read the initial common IMU/LiDAR interval of a ROS1 file or ROS2 bag directory.

Coordinates remain in the original LiDAR frame. IMU screening and a geometric
check between temporal halves reject motion without applying any registration
transform to the output cloud. Neither check proves complete immobility.
No ROS master is needed. ROS2 prefers native rosbag2_py; ROS1 prefers rosbags.
"""

from pathlib import Path
from numbers import Real

import numpy as np


DEFAULT_CONFIG = {
    "duration_sec": 1.0,
    "lidar_topic": "/livox/lidar",
    "imu_topic": "/livox/imu",
    "backend": "auto",
    "acceleration_scale": 9.80665,  # Recorded MID360 acceleration is in g.
    "window_sec": 0.25,
    "window_step_sec": 0.125,
    "gyro_p95_max_rad_s": 0.03,
    "accel_std_max_m_s2": 0.196133,
    "imu_max_gap_sec": 0.03,
    "lidar_max_gap_sec": 0.25,
    "min_imu_samples_per_window": 10,
    "min_points": 100,
    "min_range_m": 0.1,
    "max_range_m": 100.0,
    "min_reflectivity": 0.0,
    "max_read_record_sec": 10.0,
    "pointcloud_time_field": "auto",
    "pointcloud_time_unit": "auto",
    "pointcloud_time_reference": "auto",
    "pointcloud_max_frame_span_sec": 1.0,
    "geometry_check_enabled": True,
    "geometry_voxel_size_m": 0.08,
    "geometry_max_points": 12000,
    "geometry_max_correspondence_m": 0.30,
    "geometry_min_correspondence_ratio": 0.30,
    "geometry_max_translation_m": 0.03,
    "geometry_max_rotation_deg": 0.5,
    "geometry_max_rmse_m": 0.03,
    "geometry_max_condition_number": 10000.0,
}

NS_PER_SEC = 1_000_000_000
SUPPORTED_LIDAR_TYPES = {
    "livox_ros_driver/CustomMsg", "livox_ros_driver2/CustomMsg",
    "livox_ros_driver/msg/CustomMsg", "livox_ros_driver2/msg/CustomMsg",
}
POINTCLOUD2_TYPES = {"sensor_msgs/PointCloud2", "sensor_msgs/msg/PointCloud2"}
POINTFIELD_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2",
                    5: "i4", 6: "u4", 7: "f4", 8: "f8"}
TIME_UNIT_NS = {"s": 1_000_000_000, "ms": 1_000_000, "us": 1_000, "ns": 1}


class StaticDataError(RuntimeError):
    """An input/coverage/stillness failure with a JSON-serializable report."""

    def __init__(self, message, report):
        super().__init__(message)
        self.report = report


def _fail(report, code, message):
    report.update(status="failed", reason_code=code, message=message)
    if report.get("geometry", {}).get("status") == "checking":
        report["geometry"]["status"] = "failed"
    raise StaticDataError(message, report)


def _configuration(config):
    values = dict(DEFAULT_CONFIG)
    unknown = set(config) - set(values)
    if unknown:
        raise ValueError("Unknown static input configuration keys: " + ", ".join(sorted(unknown)))
    values.update(config)
    for name in ("duration_sec", "acceleration_scale", "window_sec", "window_step_sec",
                 "gyro_p95_max_rad_s", "accel_std_max_m_s2", "imu_max_gap_sec",
                 "lidar_max_gap_sec", "max_range_m", "max_read_record_sec",
                 "geometry_voxel_size_m", "geometry_max_correspondence_m",
                 "geometry_max_translation_m", "geometry_max_rotation_deg",
                 "geometry_max_rmse_m", "geometry_max_condition_number",
                 "pointcloud_max_frame_span_sec"):
        value = values[name]
        if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value <= 0:
            raise ValueError(name + " must be a positive finite number")
    for name in ("min_imu_samples_per_window", "min_points", "geometry_max_points"):
        value = values[name]
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(name + " must be a positive integer")
    for name in ("min_range_m", "min_reflectivity"):
        value = values[name]
        if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value < 0:
            raise ValueError(name + " must be a nonnegative finite number")
    if values["min_range_m"] >= values["max_range_m"]:
        raise ValueError("min_range_m must be less than max_range_m")
    if values["window_step_sec"] > values["window_sec"]:
        raise ValueError("window_step_sec must not exceed window_sec")
    if values["max_read_record_sec"] < values["duration_sec"]:
        raise ValueError("max_read_record_sec must be >= duration_sec")
    if values["backend"] not in ("auto", "rosbag2", "rosbags", "rosbag"):
        raise ValueError("backend must be auto, rosbag2, rosbags, or rosbag")
    if values["pointcloud_time_unit"] not in ("auto", *TIME_UNIT_NS):
        raise ValueError("pointcloud_time_unit must be auto, s, ms, us, or ns")
    if values["pointcloud_time_reference"] not in ("auto", "absolute", "relative"):
        raise ValueError("pointcloud_time_reference must be auto, absolute, or relative (to header)")
    if not isinstance(values["pointcloud_time_field"], str) or not values["pointcloud_time_field"]:
        raise ValueError("pointcloud_time_field must be auto or a nonempty field name")
    for name in ("lidar_topic", "imu_topic"):
        if not isinstance(values[name], str) or not values[name]:
            raise ValueError(name + " must be a nonempty topic name")
    if values["lidar_topic"] == values["imu_topic"]:
        raise ValueError("lidar_topic and imu_topic must differ")
    if not isinstance(values["geometry_check_enabled"], bool):
        raise ValueError("geometry_check_enabled must be true or false")
    if (isinstance(values["geometry_min_correspondence_ratio"], bool) or
            not isinstance(values["geometry_min_correspondence_ratio"], Real) or
            not 0 < values["geometry_min_correspondence_ratio"] <= 1):
        raise ValueError("geometry_min_correspondence_ratio must be in (0, 1]")
    if values["geometry_max_points"] < 100:
        raise ValueError("geometry_max_points must be at least 100")
    if round(values["duration_sec"] * NS_PER_SEC) < 1:
        raise ValueError("duration_sec must be at least one nanosecond")
    return values


def _stamp_ns(stamp):
    # Preserve integer nanoseconds: epoch-second floats lose boundary precision.
    if hasattr(stamp, "to_nsec"):
        result = int(stamp.to_nsec())
    elif hasattr(stamp, "sec"):
        result = int(stamp.sec) * NS_PER_SEC + int(stamp.nanosec)
    else:
        result = int(stamp.secs) * NS_PER_SEC + int(stamp.nsecs)
    if result <= 0:
        raise ValueError("Missing or nonpositive sensor header timestamp")
    return result


def _vector3(value):
    return np.array((value.x, value.y, value.z), dtype=np.float64)


def _decode_livox(message):
    """Decode both Livox CustomMsg variants using timebase + offset_time (ns)."""
    timebase = int(message.timebase)
    if timebase <= 0:
        raise ValueError("Livox timebase must be a positive sensor timestamp in ns")
    count = len(message.points)
    if count == 0:
        raise ValueError("Livox packet has no point timestamps")
    if hasattr(message, "point_num") and int(message.point_num) != count:
        raise ValueError("Livox point_num does not match the actual points length")
    coordinates = np.empty((count, 3), dtype=np.float64)
    intensity = np.empty(count, dtype=np.float64)
    offsets = np.empty(count, dtype=np.int64)
    for i, point in enumerate(message.points):
        coordinates[i] = (point.x, point.y, point.z)
        intensity[i] = point.reflectivity
        offsets[i] = int(point.offset_time)
    if np.any(offsets < 0) or np.any(offsets > np.iinfo(np.uint32).max):
        raise ValueError("Livox offset_time is outside its uint32 nanosecond range")
    if timebase > np.iinfo(np.int64).max - int(offsets.max()):
        raise ValueError("Livox point timestamps overflow signed 64-bit nanoseconds")
    return coordinates, intensity, timebase + offsets


def _pointcloud_records(message):
    """Create a structured view honoring ROS PointCloud2 row/point padding."""
    width, height = int(message.width), int(message.height)
    point_step, row_step = int(message.point_step), int(message.row_step)
    if width <= 0 or height <= 0 or point_step <= 0 or row_step < width * point_step:
        raise ValueError("PointCloud2 has empty or invalid dimensions/strides")
    names, formats, offsets = [], [], []
    endian = ">" if message.is_bigendian else "<"
    for field in message.fields:
        datatype, count, offset = int(field.datatype), int(field.count), int(field.offset)
        if not field.name or field.name in names or datatype not in POINTFIELD_DTYPES or count < 1:
            raise ValueError("PointCloud2 has duplicate names or invalid field types/counts")
        dtype = np.dtype(endian + POINTFIELD_DTYPES[datatype])
        if offset < 0 or offset + count * dtype.itemsize > point_step:
            raise ValueError("PointCloud2 field exceeds point_step")
        names.append(field.name)
        formats.append(dtype if count == 1 else (dtype, (count,)))
        offsets.append(offset)
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": point_step})
    try:
        buffer = memoryview(message.data)
    except TypeError:
        buffer = memoryview(bytes(message.data))
    if buffer.nbytes != row_step * height:
        raise ValueError("PointCloud2 data length does not equal row_step * height")
    return np.ndarray((height, width), dtype=dtype, buffer=buffer,
                      strides=(row_step, point_step)).reshape(-1)


def _decode_pointcloud2(message, config):
    records = _pointcloud_records(message)
    names = records.dtype.names
    intensity_name = next((name for name in ("intensity", "reflectivity") if name in names), None)
    if not {"x", "y", "z"}.issubset(names) or intensity_name is None:
        raise ValueError("PointCloud2 requires x/y/z and intensity or reflectivity fields")
    time_field = config["pointcloud_time_field"]
    if time_field == "auto":
        available = [name for name in ("timestamp", "time", "t", "offset_time") if name in names]
        if len(available) != 1:
            raise ValueError("PointCloud2 requires one unambiguous per-point time field; set pointcloud_time_field. "
                             "Header-only clouds cannot guarantee a strict initial interval and are rejected.")
        time_field = available[0]
    if time_field not in names:
        raise ValueError("PointCloud2 is missing per-point time field '" + time_field +
                         "'; header-only clouds are rejected rather than inventing point offsets")
    for name in ("x", "y", "z", intensity_name, time_field):
        if records[name].ndim != 1:
            raise ValueError("PointCloud2 coordinate, intensity, and time fields must have count=1")
    points = np.column_stack([records[name] for name in ("x", "y", "z")]).astype(np.float64)
    intensity = records[intensity_name].astype(np.float64)
    raw_times = records[time_field]
    if not np.isfinite(raw_times).all():
        raise ValueError("PointCloud2 per-point timestamps contain NaN or infinity")
    header_ns = _stamp_ns(message.header.stamp)
    units = list(TIME_UNIT_NS) if config["pointcloud_time_unit"] == "auto" else [config["pointcloud_time_unit"]]
    references = (["absolute", "relative"] if config["pointcloud_time_reference"] == "auto"
                  else [config["pointcloud_time_reference"]])
    candidates = []
    max_span_ns = config["pointcloud_max_frame_span_sec"] * NS_PER_SEC
    # Long double is used only for plausibility bounds. Convert whole/fractional
    # parts separately below, preserving integer ns even on platforms where
    # np.longdouble has the same precision as float64.
    for unit in units:
        for reference in references:
            scaled = raw_times.astype(np.longdouble) * TIME_UNIT_NS[unit]
            if reference == "relative":
                scaled += header_ns
            offsets = scaled - header_ns
            if (np.isfinite(scaled).all() and np.all(scaled > 0) and
                    np.all(scaled < np.iinfo(np.int64).max) and
                    np.max(np.abs(offsets)) <= max_span_ns and
                    scaled.max() - scaled.min() <= max_span_ns):
                if raw_times.dtype.kind in "iu":
                    converted = raw_times.astype(np.int64) * TIME_UNIT_NS[unit]
                else:
                    whole = np.floor(raw_times)
                    converted = (whole.astype(np.int64) * TIME_UNIT_NS[unit] +
                                 np.rint((raw_times - whole) * TIME_UNIT_NS[unit]).astype(np.int64))
                if reference == "relative":
                    converted += header_ns
                candidates.append((unit, reference, converted))
    if len(candidates) != 1:
        raise ValueError("PointCloud2 time unit/reference is ambiguous or inconsistent with its header; "
                         "set pointcloud_time_unit and pointcloud_time_reference explicitly")
    unit, reference, point_times = candidates[0]
    is_float = raw_times.dtype.kind == "f"
    precision_ns = (float(np.max(np.abs(np.spacing(raw_times)))) * TIME_UNIT_NS[unit]
                    if is_float else float(TIME_UNIT_NS[unit]))
    # A FLOAT64 absolute-nanosecond stamp near the dataset epoch resolves 256 ns.
    # Discard points within its rounding uncertainty of either integration boundary.
    guard_ns = int(np.ceil(precision_ns / 2)) if is_float else 0
    metadata = {"format": "PointCloud2", "field": time_field, "unit": unit,
                "reference": reference, "dtype": str(raw_times.dtype),
                "granularity": "per_point", "source_resolution_ns": precision_ns,
                "boundary_guard_ns": guard_ns,
                "note": "Floating point timestamps retain their source quantization; lost precision is not recoverable."
                        if is_float else "Integer source timestamps are retained without floating point epoch conversion."}
    return points, intensity, point_times, metadata


def _screen_imu(timestamps, gyro, acceleration, start_ns, end_ns, config, report):
    selected = (timestamps >= start_ns) & (timestamps <= end_ns)
    times = (timestamps[selected] - start_ns) / NS_PER_SEC
    gyro = gyro[selected]
    with np.errstate(over="ignore", invalid="ignore"):
        acceleration = acceleration[selected] * config["acceleration_scale"]
    if not np.isfinite(acceleration).all():
        _fail(report, "invalid_imu", "IMU acceleration conversion produced non-finite SI values")
    report["imu_samples_in_interval"] = len(times)
    duration = (end_ns - start_ns) / NS_PER_SEC
    width = min(config["window_sec"], duration)
    last_start = duration - width
    starts = list(np.arange(0.0, last_start, config["window_step_sec"]))
    if not starts or abs(starts[-1] - last_start) > 1e-9:
        starts.append(last_start)
    report["imu_windows"] = []
    for window_start in starts:
        window_end = window_start + width
        in_window = (times >= window_start - 1e-9) & (times <= window_end + 1e-9)
        count = int(in_window.sum())
        window = {"start_relative_sec": float(window_start),
                  "end_relative_sec": float(window_end), "samples": count}
        report["imu_windows"].append(window)
        if count < config["min_imu_samples_per_window"]:
            _fail(report, "insufficient_imu_samples",
                  "Too few IMU samples in an initial-interval screening window")
        with np.errstate(over="ignore", invalid="ignore"):
            gyro_p95 = float(np.percentile(np.linalg.norm(gyro[in_window], axis=1), 95))
            accel_std = float(np.linalg.norm(np.std(acceleration[in_window], axis=0)))
        if not np.isfinite(gyro_p95) or not np.isfinite(accel_std):
            _fail(report, "invalid_imu", "IMU screening statistics overflowed; input values are invalid")
        window.update(gyro_p95_rad_s=gyro_p95, accel_std_m_s2=accel_std,
                      passed=bool(gyro_p95 <= config["gyro_p95_max_rad_s"] and
                                  accel_std <= config["accel_std_max_m_s2"]))
    failures = [window for window in report["imu_windows"] if not window["passed"]]
    if failures:
        _fail(report, "motion_detected",
              "The fixed initial interval failed IMU motion screening; later intervals were not searched")


def _voxel_sample(points, voxel_size, max_points):
    cells = np.floor(points / voxel_size).astype(np.int64)
    _, indices = np.unique(cells, axis=0, return_index=True)
    if len(indices) > max_points:
        indices = indices[np.linspace(0, len(indices) - 1, max_points, dtype=np.int64)]
    return points[indices]


def _screen_geometry(early, late, config, report):
    """Check late-to-early rigid motion using trimmed point-to-plane ICP.

    This is a local consistency check, not motion compensation or a proof of
    complete immobility. A single plane cannot establish all six motion axes and
    is explicitly rejected by the normalized information-matrix test.
    """
    geometry = {"method": "first_half_vs_second_half_trimmed_point_to_plane_icp",
                "enabled": config["geometry_check_enabled"], "status": "checking",
                "transform_direction": "late LiDAR coordinates to early LiDAR coordinates",
                "limitation": "Local ICP may miss repeated-scene ambiguities or motion within each half."}
    report["geometry"] = geometry
    if not config["geometry_check_enabled"]:
        geometry.update(status="disabled")
        return
    if not np.isfinite(early).all() or not np.isfinite(late).all():
        _fail(report, "invalid_geometry", "LiDAR geometry contains non-finite coordinates")
    try:
        from scipy.spatial import cKDTree
        from scipy.spatial.transform import Rotation
    except ImportError:
        _fail(report, "geometry_dependency_missing", "LiDAR geometry checking requires scipy")
    reference = _voxel_sample(early, config["geometry_voxel_size_m"], config["geometry_max_points"])
    source = _voxel_sample(late, config["geometry_voxel_size_m"], config["geometry_max_points"])
    geometry.update(early_sampled_points=len(reference), late_sampled_points=len(source))
    if min(len(reference), len(source)) < 100:
        _fail(report, "insufficient_geometry", "Each temporal half needs at least 100 downsampled LiDAR points")
    tree = cKDTree(reference)
    neighbor_distances, neighbors = tree.query(reference, k=15)
    neighborhoods = reference[neighbors]
    centered = neighborhoods - neighborhoods.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered) / neighborhoods.shape[1]
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    normals = eigenvectors[:, :, 0]
    planar = ((eigenvalues[:, 0] < 0.05 * np.maximum(eigenvalues.sum(axis=1), 1e-12)) &
              (eigenvalues[:, 1] > 1e-6) &
              (neighbor_distances[:, -1] < 5 * config["geometry_voxel_size_m"]))
    geometry["reference_planar_points"] = int(planar.sum())
    if int(planar.sum()) < 100:
        _fail(report, "insufficient_geometry", "Too few locally planar LiDAR neighborhoods for geometric checking")
    rotation, translation = np.eye(3), np.zeros(3)
    converged = False
    for iteration in range(30):
        transformed = source @ rotation.T + translation
        distances, indices = tree.query(transformed, k=1)
        close = distances <= config["geometry_max_correspondence_m"]
        matched = close & planar[indices]
        # Spatial overlap and availability of reliable plane normals are distinct:
        # corners/edges are valid overlap, but cannot provide a plane residual.
        ratio = float(close.mean())
        geometry.update(iterations=iteration + 1, correspondence_ratio=ratio,
                        planar_correspondence_ratio=float(matched.mean()))
        if int(matched.sum()) < 100 or ratio < config["geometry_min_correspondence_ratio"]:
            _fail(report, "geometry_low_overlap", "The two LiDAR halves do not have enough valid geometric correspondences")
        current = transformed[matched]
        target = reference[indices[matched]]
        normal = normals[indices[matched]]
        residual = np.einsum("ij,ij->i", current - target, normal)
        # Keep 80% of the smallest plane residuals, retaining scene structure.
        keep = np.abs(residual) <= np.percentile(np.abs(residual), 80)
        current, normal, residual = current[keep], normal[keep], residual[keep]
        jacobian = np.concatenate((np.cross(current, normal), normal), axis=1)
        column_scale = np.linalg.norm(jacobian, axis=0)
        normalized = jacobian / np.maximum(column_scale, 1e-12)
        information_eigenvalues = np.linalg.eigvalsh(normalized.T @ normalized)
        condition = (float(information_eigenvalues[-1] / information_eigenvalues[0])
                     if information_eigenvalues[0] > 1e-12 else None)
        geometry.update(normalized_information_eigenvalues=information_eigenvalues.tolist(),
                        normalized_condition_number=condition)
        if condition is None or condition > config["geometry_max_condition_number"]:
            _fail(report, "geometry_degenerate", "LiDAR geometry cannot constrain all six motion axes reliably")
        increment = np.linalg.lstsq(jacobian, -residual, rcond=None)[0]
        if not np.isfinite(increment).all():
            _fail(report, "geometry_numerical_failure", "LiDAR ICP produced a non-finite motion increment")
        delta_rotation = Rotation.from_rotvec(increment[:3]).as_matrix()
        if not np.isfinite(delta_rotation).all():
            _fail(report, "geometry_numerical_failure", "LiDAR ICP produced a non-finite rotation")
        rotation = delta_rotation @ rotation
        translation = delta_rotation @ translation + increment[3:]
        if np.linalg.norm(increment[:3]) < 1e-5 and np.linalg.norm(increment[3:]) < 1e-4:
            converged = True
            break
    geometry["converged"] = converged
    if not converged:
        _fail(report, "geometry_not_converged", "LiDAR ICP did not converge within 30 iterations")
    transformed = source @ rotation.T + translation
    distances, indices = tree.query(transformed, k=1)
    close = distances <= config["geometry_max_correspondence_m"]
    matched = close & planar[indices]
    geometry.update(correspondence_ratio=float(close.mean()),
                    planar_correspondence_ratio=float(matched.mean()))
    if int(matched.sum()) < 100 or float(close.mean()) < config["geometry_min_correspondence_ratio"]:
        _fail(report, "geometry_low_overlap", "LiDAR ICP final transform has insufficient geometric correspondences")
    residual = np.einsum("ij,ij->i", transformed[matched] - reference[indices[matched]],
                         normals[indices[matched]])
    # Report untrimmed RMSE so outliers/dynamic geometry remain visible.
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    translation_norm = float(np.linalg.norm(translation))
    rotation_deg = float(np.linalg.norm(Rotation.from_matrix(rotation).as_rotvec()) * 180 / np.pi)
    if not np.isfinite([rmse, translation_norm, rotation_deg]).all():
        _fail(report, "geometry_numerical_failure", "LiDAR ICP final residual or motion is non-finite")
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, translation
    geometry.update(translation_m=translation_norm, rotation_deg=rotation_deg,
                    point_to_plane_rmse_m=rmse, T_early_late=transform.tolist())
    if rmse > config["geometry_max_rmse_m"]:
        _fail(report, "geometry_inconsistent", "LiDAR halves have excessive point-to-plane residuals")
    if (translation_norm > config["geometry_max_translation_m"] or
            rotation_deg > config["geometry_max_rotation_deg"]):
        _fail(report, "lidar_motion_detected", "LiDAR halves show motion above the configured static thresholds")
    geometry["status"] = "passed"


def _extract_messages(messages, config):
    """Implementation shared by both readers; accepts already-decoded records."""
    config = _configuration(config)
    report = {
        "status": "checking", "method": "initial_common_interval_imu_screening",
        "limitation": "IMU screening cannot detect constant-velocity translation or prove a static scene.",
        "config": config, "requested_duration_sec": config["duration_sec"],
        "timestamp_basis": "IMU header and LiDAR per-point sensor timestamps; bag record time only bounds I/O",
        "interval_policy": "[max(first IMU stamp, first LiDAR point stamp), start + duration_sec)",
        "imu_messages_read": 0, "lidar_messages_read": 0,
    }
    imu_times, gyros, accelerations = [], [], []
    cloud_parts, intensity_parts, time_parts = [], [], []
    lidar_intervals = []
    frames = set()
    first_record_ns = None
    last_record_ns = None
    first_lidar_ns = None
    start_ns = end_ns = None
    reached_read_limit = False
    boundary_guard_ns = 0
    for topic, record_ns, message, msgtype in messages:
        if topic not in (config["imu_topic"], config["lidar_topic"]):
            continue
        if first_record_ns is None:
            first_record_ns = record_ns
        if last_record_ns is not None and record_ns < last_record_ns:
            _fail(report, "record_time_reversal", "Bag record timestamps are not nondecreasing")
        last_record_ns = record_ns
        if record_ns - first_record_ns > round(config["max_read_record_sec"] * NS_PER_SEC):
            reached_read_limit = True
            break
        try:
            if topic == config["imu_topic"]:
                if msgtype not in ("sensor_msgs/Imu", "sensor_msgs/msg/Imu"):
                    _fail(report, "unsupported_imu_type", "IMU topic must contain sensor_msgs/Imu")
                sensor_ns = _stamp_ns(message.header.stamp)
                gyro, acceleration = _vector3(message.angular_velocity), _vector3(message.linear_acceleration)
                if not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(acceleration)):
                    _fail(report, "invalid_imu", "IMU contains non-finite acceleration or angular velocity")
                if np.max(np.abs(acceleration)) < 1e-9:
                    _fail(report, "invalid_imu", "Raw MID360 acceleration is zero; static input must include gravity")
                for field in ("angular_velocity_covariance", "linear_acceleration_covariance"):
                    covariance = getattr(message, field, None)
                    if covariance is not None and len(covariance) and covariance[0] < 0:
                        _fail(report, "invalid_imu", "IMU marks " + field + " as unavailable")
                if imu_times and sensor_ns <= imu_times[-1]:
                    _fail(report, "imu_time_reversal", "IMU header timestamps must strictly increase")
                if not imu_times:
                    report["first_imu_sensor_ns"] = sensor_ns
                    report["first_imu_record_ns"] = record_ns
                    report["imu_record_minus_sensor_sec"] = (record_ns - sensor_ns) / NS_PER_SEC
                imu_times.append(sensor_ns)
                gyros.append(gyro)
                accelerations.append(acceleration)
                report["imu_messages_read"] += 1
            else:
                if msgtype in SUPPORTED_LIDAR_TYPES:
                    points, intensity, point_times = _decode_livox(message)
                    time_metadata = {"format": "Livox CustomMsg", "field": "timebase + offset_time",
                                     "unit": "ns", "reference": "absolute", "dtype": "uint64 + uint32",
                                     "granularity": "per_point", "source_resolution_ns": 1.0,
                                     "boundary_guard_ns": 0}
                elif msgtype in POINTCLOUD2_TYPES:
                    points, intensity, point_times, time_metadata = _decode_pointcloud2(message, config)
                else:
                    _fail(report, "unsupported_lidar_type",
                          "LiDAR requires Livox CustomMsg or PointCloud2 with explicit per-point timestamps")
                previous_time = report.get("lidar_timestamp")
                if previous_time and any(previous_time[key] != time_metadata[key]
                                         for key in ("format", "field", "unit", "reference", "dtype")):
                    _fail(report, "lidar_time_layout_changed", "LiDAR point timestamp layout changes within the initial interval")
                boundary_guard_ns = max(boundary_guard_ns, time_metadata["boundary_guard_ns"])
                if previous_time:
                    time_metadata["source_resolution_ns"] = max(previous_time["source_resolution_ns"],
                                                                time_metadata["source_resolution_ns"])
                time_metadata["boundary_guard_ns"] = boundary_guard_ns
                report["lidar_timestamp"] = time_metadata
                packet_start, packet_end = int(point_times.min()), int(point_times.max())
                frame = str(message.header.frame_id)
                if not frame:
                    _fail(report, "missing_lidar_frame", "LiDAR frame_id is empty")
                frames.add(frame)
                if len(frames) != 1:
                    _fail(report, "lidar_frame_changed", "LiDAR frame_id changes within the initial interval")
                if lidar_intervals and packet_start < lidar_intervals[-1][0]:
                    _fail(report, "lidar_time_reversal", "LiDAR packet start timestamps move backwards")
                if first_lidar_ns is None:
                    first_lidar_ns = packet_start
                    report["first_lidar_sensor_ns"] = packet_start
                    report["first_lidar_record_ns"] = record_ns
                    report["first_lidar_header_ns"] = _stamp_ns(message.header.stamp)
                    report["lidar_record_minus_header_sec"] = (record_ns - report["first_lidar_header_ns"]) / NS_PER_SEC
                    report["lidar_record_minus_sensor_sec"] = (record_ns - packet_start) / NS_PER_SEC
                lidar_intervals.append((packet_start, packet_end))
                cloud_parts.append(points)
                intensity_parts.append(intensity)
                time_parts.append(point_times)
                report["lidar_messages_read"] += 1
        except StaticDataError:
            raise
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            _fail(report, "malformed_sensor_data", str(error))
        if start_ns is None and imu_times and first_lidar_ns is not None:
            start_ns = max(imu_times[0], first_lidar_ns)
            end_ns = start_ns + round(config["duration_sec"] * NS_PER_SEC)
            report.update(sensor_start_ns=start_ns, sensor_end_ns_exclusive=end_ns,
                          used_duration_sec=(end_ns - start_ns) / NS_PER_SEC)
        # The sample/packet at the end is a coverage fence, not extra integration.
        if end_ns is not None and imu_times[-1] >= end_ns and lidar_intervals[-1][1] >= end_ns:
            break
    report["record_span_read_sec"] = (0.0 if first_record_ns is None else
                                       (last_record_ns - first_record_ns) / NS_PER_SEC)
    report["read_limit_reached"] = reached_read_limit
    if not imu_times:
        _fail(report, "missing_imu", "No IMU messages were found in the bounded initial read")
    if not lidar_intervals:
        _fail(report, "missing_lidar", "No LiDAR messages were found in the bounded initial read")
    report["lidar_frame_id"] = next(iter(frames))
    report["imu_sensor_coverage_sec"] = (imu_times[-1] - start_ns) / NS_PER_SEC
    report["lidar_sensor_coverage_sec"] = (max(end for _, end in lidar_intervals) - start_ns) / NS_PER_SEC
    if imu_times[-1] < end_ns or max(end for _, end in lidar_intervals) < end_ns:
        _fail(report, "insufficient_coverage",
              "IMU and LiDAR do not both cover the entire requested initial interval")
    times = np.asarray(imu_times, dtype=np.int64)
    # Include samples bracketing the requested boundaries to detect missing data.
    relevant_gaps = ((times[:-1] < end_ns) & (times[1:] > start_ns))
    imu_gaps = np.diff(times)[relevant_gaps] / NS_PER_SEC
    report["imu_max_observed_gap_sec"] = float(imu_gaps.max()) if len(imu_gaps) else 0.0
    if report["imu_max_observed_gap_sec"] > config["imu_max_gap_sec"]:
        _fail(report, "imu_gap", "An IMU timestamp gap crosses the requested initial interval")
    covered_until = lidar_intervals[0][1]
    max_lidar_gap = 0
    for packet_start, packet_end in lidar_intervals[1:]:
        if covered_until < end_ns and packet_start > start_ns:
            max_lidar_gap = max(max_lidar_gap, min(packet_start, end_ns) - max(covered_until, start_ns))
        covered_until = max(covered_until, packet_end)
    report["lidar_max_observed_gap_sec"] = max_lidar_gap / NS_PER_SEC
    if report["lidar_max_observed_gap_sec"] > config["lidar_max_gap_sec"]:
        _fail(report, "lidar_gap", "A LiDAR packet gap crosses the requested initial interval")
    _screen_imu(times, np.asarray(gyros), np.asarray(accelerations),
                start_ns, end_ns, config, report)
    points, intensity = np.concatenate(cloud_parts), np.concatenate(intensity_parts)
    point_times = np.concatenate(time_parts)
    in_interval = ((point_times >= start_ns + boundary_guard_ns) &
                   (point_times < end_ns - boundary_guard_ns))
    report["point_boundary_guard_ns"] = boundary_guard_ns
    report["points_in_interval_before_filtering"] = int(in_interval.sum())
    finite = np.isfinite(points).all(axis=1) & np.isfinite(intensity)
    # Norm evaluated only for finite points, avoiding NaN/overflow warnings.
    ranges = np.full(len(points), np.inf)
    ranges[finite] = np.hypot(np.hypot(points[finite, 0], points[finite, 1]), points[finite, 2])
    valid = (in_interval & finite & (ranges > 0) &
             (ranges >= config["min_range_m"]) & (ranges <= config["max_range_m"]) &
             (intensity >= config["min_reflectivity"]))
    report["points_retained"] = int(valid.sum())
    report["points_filtered"] = int(in_interval.sum() - valid.sum())
    if int(valid.sum()) < config["min_points"]:
        _fail(report, "insufficient_points", "Too few valid points remain in the initial static interval")
    midpoint_ns = start_ns + (end_ns - start_ns) // 2
    _screen_geometry(points[valid & (point_times < midpoint_ns)],
                     points[valid & (point_times >= midpoint_ns)], config, report)
    if config["geometry_check_enabled"]:
        report.update(method="initial_common_interval_imu_and_lidar_screening",
                      limitation="Threshold-based IMU and local LiDAR checks cannot prove complete immobility or a static scene.")
    report.update(status="passed", reason_code="static_screening_passed",
                  message="The requested fixed initial interval passed all enabled static-data checks")
    return points[valid], intensity[valid], report


def extract_static_cloud(bag_path, config=None):
    """Return ``(points[N,3], reflectivity[N], report)`` or raise StaticDataError.

    Merge overrides from ``config`` with DEFAULT_CONFIG. Sensor times are used
    exclusively for coverage and integration; bag record time bounds I/O only.
    The half-open point interval prevents points on the moving side of the final
    boundary from entering the cloud. Input images are never deserialized.
    """
    config = _configuration(config or {})
    path = Path(bag_path).expanduser()
    report = {"status": "failed", "config": config, "bag_path": str(path)}
    from dual_mei.bag_readers import (BagInputError, choose_backend, detect_bag_format,
                                      iter_messages, read_ros2_metadata)
    try:
        backend = choose_backend(path, config["backend"])
        is_ros2 = detect_bag_format(path) == "ros2"
    except BagInputError as error:
        _fail(report, error.reason_code, str(error))
    iterator = iter_messages(path, (config["imu_topic"], config["lidar_topic"]), backend)
    source_info = {"bag_path": str(path), "backend": backend, "bag_format": "ros2" if is_ros2 else "ros1"}
    if is_ros2:
        try:
            bag_info = read_ros2_metadata(path)
            source_info["bag_metadata"] = {key: bag_info.get(key) for key in
                                           ("version", "storage_identifier", "ros_distro")}
            if bag_info.get("ros_distro") == "rosbags":
                source_info["conversion_note"] = ("ROS2 metadata identifies rosbags as producer. "
                                                  "Record/header timestamps and point contents may have been rewritten by conversion; "
                                                  "this reader uses the stored sensor stamps and does not reconstruct discarded precision.")
        except Exception as error:
            _fail(report, "bag_metadata_invalid", "Cannot read ROS2 bag metadata: " + str(error))
    try:
        points, intensity, report = _extract_messages(iterator, config)
        report.update(source_info)
        return points, intensity, report
    except StaticDataError as error:
        error.report.update(source_info)
        raise
    except Exception as error:
        report.update(source_info)
        _fail(report, "bag_read_error", "Cannot read bag with " + backend + ": " + str(error))
    finally:
        iterator.close()
