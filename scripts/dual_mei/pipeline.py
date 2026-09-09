"""One calibration pipeline for external images and paired bag images."""
from datetime import datetime, timezone
import os
from pathlib import Path
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation

from mei_calibration_geometry import inverse_transform, voxel_downsample
from static_bag_input import extract_static_cloud
from .artifacts import save_image, write_yaml
from .config import load_config
from .images import load_images
from .optimizer import executable_command, prepare_camera, run_solver
from .rendering import render_overlay, shared_color_range


def _static_summary(report):
    keys = ("status", "sensor_start_ns", "sensor_end_ns_exclusive", "used_duration_sec",
            "points_retained", "lidar_frame_id", "point_boundary_guard_ns")
    result = {k: report[k] for k in keys if k in report}
    if "geometry" in report:
        geometry = report["geometry"]
        result["geometry"] = {k: geometry[k] for k in
                              ("status", "translation_m", "rotation_deg", "point_to_plane_rmse_m") if k in geometry}
    return result


def _publish(staged, output):
    # rmdir refuses nonempty directories, including ones changed since preflight.
    if output.exists():
        output.rmdir()
    os.rename(staged, output)


def run(config_path, check_static_only=False, prepare_only=False, output_override=None):
    config = load_config(config_path, check_static_only, output_override)
    if not check_static_only and config.output.exists():
        if not config.output.is_dir() or any(config.output.iterdir()):
            raise ValueError(f"output directory must be new or empty: {config.output}")
    command = None if check_static_only or prepare_only else executable_command(config.solver)
    points, intensity, detection = extract_static_cloud(config.bag, config.static)
    print(f"Static interval accepted: {len(points)} points, {config.static['duration_sec']} s", flush=True)
    if check_static_only:
        print(f"Static check passed: [{detection['sensor_start_ns']}, {detection['sensor_end_ns_exclusive']}) ns")
        return 0
    if detection["lidar_frame_id"] != config.input["lidar_frame"]:
        raise ValueError(f"LiDAR frame {detection['lidar_frame_id']} differs from input.lidar_frame")
    frames, selection = load_images(config, detection)
    if config.input["mode"] == "bag_only":
        print("Selected image stamps: " + ", ".join(f"{name}={frame.source['timestamp_ns']}" for name, frame in frames.items()), flush=True)
    points, intensity = voxel_downsample(points, intensity, config.pointcloud["voxel_size_m"])
    config.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f".{config.output.name}.work-", dir=config.output.parent) as temporary:
        temporary = Path(temporary)
        staged = temporary / "result"
        staged.mkdir()
        result = dict(schema_version=2, status="prepared" if prepare_only else "success",
                      created_at=datetime.now(timezone.utc).isoformat(),
                      direction="lidar_to_camera", equation="p_cam = T_cam_lidar @ p_lidar",
                      lidar_frame=config.input["lidar_frame"], translation_unit="metre",
                      input_mode=config.input["mode"], bag_path=str(config.bag),
                      static=_static_summary(detection), static_parameters=config.static,
                      image_selection=selection, pointcloud=config.pointcloud,
                      solver={k: v for k, v in config.solver.items() if k != "calibrate_executable"},
                      visualization=config.visualization, cameras={})
        masks = {}
        for name, item in config.cameras.items():
            output_camera = staged / name
            native = temporary / "native" / name
            cloud, values, keep = prepare_camera(output_camera, native, points, intensity,
                                                 item, frames[name], config)
            masks[name] = keep
            print(f"{name}: {len(cloud)} / {len(points)} points", flush=True)
            initial = item["transform"]
            transform, diagnostics = (initial, {}) if prepare_only else run_solver(command, native, config.solver)
            vis = dict(config.visualization)
            vis["color_range"] = shared_color_range(cloud, values, (initial, transform),
                                                     item["camera"], item["mask"], vis)
            projections = {}
            stages = (("before", initial),) if prepare_only else (("before", initial), ("after", transform))
            for stage, pose in stages:
                overlay, stats = render_overlay(frames[name].image, cloud, values, pose,
                                                 item["camera"], item["mask"], vis)
                if stats["valid_points"] < config.pointcloud["min_camera_points"]:
                    raise ValueError(f"{name}: {stage} projection has too few valid points ({stats['valid_points']})")
                save_image(output_camera / f"overlay_{stage}.png", overlay)
                projections[stage] = stats
            delta = transform @ inverse_transform(initial)
            camera_result = dict(camera=item["camera"], source=frames[name].source,
                                 image=f"{name}/image{frames[name].extension}",
                                 pointcloud=f"{name}/lidar.ply",
                                 intensity_semantics="empirical_CDF_equalized_reflectivity_float32_0_to_1",
                                 mask=f"{name}/mask.png" if item["mask"] is not None else None,
                                 T_cam_lidar_initial=initial.tolist(), visualization=vis,
                                 projections=projections)
            if not prepare_only:
                camera_result.update(T_cam_lidar=transform.tolist(),
                    optimization={k: diagnostics[k] for k in
                                  ("success", "converged", "inner_converged", "termination_reason",
                                   "initial_cost", "final_cost", "outer_iterations", "inner_iterations",
                                   "visible_points") if k in diagnostics},
                    delta_translation_m=float(np.linalg.norm(delta[:3, 3])),
                    delta_rotation_deg=float(np.rad2deg(Rotation.from_matrix(delta[:3, :3]).magnitude())))
                print(f"{name}: converged; change {camera_result['delta_translation_m']:.6f} m, "
                      f"{camera_result['delta_rotation_deg']:.6f} deg", flush=True)
            result["cameras"][name] = camera_result
        result["pointcloud_summary"] = dict(downsampled_points=len(points),
                                            shared_visible_points=int(np.count_nonzero(masks["cam0"] & masks["cam1"])))
        # Preview explicitly has no final transform and no success extrinsics file.
        write_yaml(staged / ("prepared.yaml" if prepare_only else "extrinsics.yaml"), result)
        _publish(staged, config.output)
    print(f"Saved {'prepared data' if prepare_only else 'calibration'}: {config.output}", flush=True)
    return 0
