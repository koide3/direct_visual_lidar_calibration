"""Adapter for the existing native NID solver and its camera-to-LiDAR convention."""
from collections import deque
import json
import os
from pathlib import Path
import shutil
import subprocess

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from mei_calibration_geometry import equalize_intensity, inverse_transform, project_mei
from .artifacts import read_ply, save_image, write_json, write_ply


def executable_command(config):
    binary = config.get("calibrate_executable")
    if binary:
        if not Path(binary).is_file() or not os.access(binary, os.X_OK):
            raise ValueError(f"calibrate_executable is not executable: {binary}")
        return [binary]
    version = os.environ.get("ROS_VERSION")
    if version == "2" and shutil.which("ros2"):
        return ["ros2", "run", "direct_visual_lidar_calibration", "calibrate"]
    if version == "1" and shutil.which("rosrun"):
        return ["rosrun", "direct_visual_lidar_calibration", "calibrate"]
    raise ValueError("source ROS and the workspace environment, or set solver.calibrate_executable")


def prepare_camera(output_dir, work_dir, points, intensity, item, frame, config):
    c, transform, mask = item["camera"], item["transform"], item["mask"]
    _, keep, _ = project_mei(points, transform, c, mask, config.pointcloud["split_margin_px"])
    cloud, reflectivity = points[keep], intensity[keep]
    if len(cloud) < config.pointcloud["min_camera_points"]:
        raise ValueError(f"{output_dir.name}: only {len(cloud)} visible points; check initial transform, intrinsics and mask")
    normalized = equalize_intensity(reflectivity)
    gray = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
    if float(np.std(gray[mask != 0] if mask is not None else gray)) < 1e-6:
        raise ValueError(f"{output_dir.name}: image is constant; cannot calibrate with NID")
    output_dir.mkdir(parents=True)
    work_dir.mkdir(parents=True)
    # Keep exactly the float32 XYZI file given to C++; the native input is a hard link.
    write_ply(output_dir / "lidar.ply", cloud, normalized)
    os.link(output_dir / "lidar.ply", work_dir / "000000.ply")
    (output_dir / ("image" + frame.extension)).write_bytes(frame.encoded)
    save_image(work_dir / "000000.png", cv2.equalizeHist(gray))
    camera_json = dict(camera_model="mei", intrinsics=[c[k] for k in ("fx", "fy", "cx", "cy", "xi")],
                       distortion_coeffs=[c["distortion"][k] for k in ("k1", "k2", "p1", "p2", "k3")],
                       max_theta_deg=c["max_theta_deg"])
    if mask is not None:
        save_image(output_dir / "mask.png", mask)
        os.link(output_dir / "mask.png", work_dir / "mask.png")
        camera_json["mask_path"] = "mask.png"
    inverse = inverse_transform(transform)
    initial = list(inverse[:3, 3]) + list(Rotation.from_matrix(inverse[:3, :3]).as_quat())
    write_json(work_dir / "calib.json", dict(
        meta=dict(data_path=str(output_dir / "lidar.ply"), bag_names=["000000"],
                  points_topic=config.input["lidar_topic"],
                  image_topic=frame.source.get("topic", "external"), intensity_channel="normalized_intensity"),
        camera=camera_json, results=dict(init_T_lidar_camera=initial)))
    # Reuse the stored coordinates for all review images, including before optimization.
    stored_points, stored_intensity = read_ply(output_dir / "lidar.ply")
    return stored_points, stored_intensity, keep


def read_calibration_result(path):
    result = json.loads(Path(path).read_text())["results"]
    if result.get("calibration_status") != "success":
        reason = result.get("calibration_diagnostics", {}).get("termination_reason", "no success status")
        raise ValueError(f"calibration failed: {reason}")
    values = np.asarray(result["T_lidar_camera"], dtype=float)
    if values.shape != (7,) or not np.isfinite(values).all() or np.linalg.norm(values[3:]) < 1e-9:
        raise ValueError("native solver returned an invalid camera-to-LiDAR transform")
    inverse = np.eye(4)
    inverse[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    inverse[:3, 3] = values[:3]
    return inverse_transform(inverse), result.get("calibration_diagnostics", {})


def run_solver(command, work_dir, solver):
    argv = command + [str(work_dir), "--headless",
                      "--max_outer_iterations", str(solver["max_outer_iterations"]),
                      "--max_inner_iterations", str(solver["max_inner_iterations"])]
    env = dict(os.environ, OMP_NUM_THREADS=str(solver["num_threads"]))
    tail = deque(maxlen=30)
    process = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, errors="replace", env=env)
    try:
        for line in process.stdout:
            tail.append(line.rstrip()[:4000])
        code = process.wait()
    except BaseException:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        raise
    finally:
        process.stdout.close()
    if code:
        try:
            read_calibration_result(work_dir / "calib.json")
        except (ValueError, KeyError) as error:
            raise RuntimeError(f"{work_dir.name}: {error}; solver exit={code}") from error
        raise RuntimeError(f"solver exit={code}:\n" + "\n".join(tail))
    return read_calibration_result(work_dir / "calib.json")
