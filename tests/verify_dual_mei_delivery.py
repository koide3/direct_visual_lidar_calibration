#!/usr/bin/env python3
"""Verify real outputs, saved-data replay, mode equivalence and failure cleanup."""
import argparse
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import sys
import tempfile

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from dual_mei.pipeline import run
from dual_mei.viewer import export_result


def digest(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def baseline_input(workspace, mode):
    """Reconstruct the fixed 0908 fixture from saved results, not editable run YAML."""
    directory = workspace.parent.parent / "data/0908" / (
        "dual_mei_refined_20260909" if mode == "bag_only"
        else "dual_mei_refined_external_20260909")
    result = yaml.safe_load((directory / "extrinsics.yaml").read_text())
    static = deepcopy(result["static_parameters"])
    inputs = dict(mode=mode, bag_path=result["bag_path"],
                  lidar_topic=static.pop("lidar_topic"), imu_topic=static.pop("imu_topic"),
                  lidar_frame=result["lidar_frame"], images={})
    cameras = {}
    for name, item in result["cameras"].items():
        inputs["images"][name] = (
            dict(topic=item["source"]["topic"]) if mode == "bag_only"
            else dict(image_path=str(directory / item["image"])))
        cameras[name] = dict(item["camera"], T_cam_lidar=item["T_cam_lidar_initial"])
        if item.get("mask"):
            cameras[name]["mask_path"] = str(directory / item["mask"])
    return dict(schema_version=2, input=inputs, static=static, cameras=cameras,
                pointcloud=result["pointcloud"], solver=result["solver"],
                visualization=result["visualization"], output=dict(directory=str(directory)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    workspace = Path(args.workspace).resolve()
    audit = workspace.parent / ".task_artifacts/0909_extrinsics"
    config = baseline_input(workspace, "bag_only")
    external_config = baseline_input(workspace, "bag_and_images")
    directories = [Path(c["output"]["directory"]) for c in (config, external_config)]
    results = [yaml.safe_load((p / "extrinsics.yaml").read_text()) for p in directories]
    baseline = json.loads((audit / "baseline/real_frame92.json").read_text())
    report = dict(status="passed", modes={}, maximum_matrix_difference={},
                  baseline_matrix_difference={}, expected_image_stamp_ns=1590192301068946861)
    for directory, result in zip(directories, results):
        expected = {"extrinsics.yaml"} | {f"{name}/{file}" for name in ("cam0", "cam1")
                     for file in ("image.jpg", "lidar.ply", "overlay_before.png", "overlay_after.png")}
        actual = {str(p.relative_to(directory)) for p in directory.rglob("*") if p.is_file()}
        assert actual == expected, actual
        assert result["status"] == "success"
        assert result["static"]["sensor_start_ns"] == 1590192300583453417
        assert result["static"]["sensor_end_ns_exclusive"] == 1590192301583453417
        report["modes"][result["input_mode"]] = dict(directory=str(directory), files=len(actual))
    for name in ("cam0", "cam1"):
        a, b = (np.array(r["cameras"][name]["T_cam_lidar"]) for r in results)
        old = np.array(baseline["extrinsics"]["cameras"][name]["T_cam_lidar"])
        report["maximum_matrix_difference"][name] = float(np.max(np.abs(a - b)))
        report["baseline_matrix_difference"][name] = float(np.max(np.abs(a - old)))
        np.testing.assert_allclose(a, b, rtol=0, atol=1e-10)
        np.testing.assert_allclose(a, old, rtol=0, atol=1e-10)
        assert results[0]["cameras"][name]["source"]["timestamp_ns"] == report["expected_image_stamp_ns"]
        for filename, key in (("image.jpg", "image_sha256"), ("lidar.ply", "ply_sha256")):
            assert digest(directories[0] / name / filename) == digest(directories[1] / name / filename)
            assert digest(directories[0] / name / filename) == baseline["inputs"][name][key]
    with tempfile.TemporaryDirectory(prefix="dual_mei_delivery_") as temp:
        temp = Path(temp)
        # Export reads only the result artifacts and can write elsewhere.
        export_result(directories[0], output=temp / "replay")
        for name in ("cam0", "cam1"):
            for stage in ("before", "after"):
                rel = Path(name) / f"overlay_{stage}.png"
                assert digest(directories[0] / rel) == digest(temp / "replay" / rel)
        report["replay_pixel_and_file_identical"] = True
        # Legacy YAML from before the refactor still prepares the original external images.
        legacy = yaml.safe_load((audit / "baseline/dual_mei_static.yaml").read_text())
        legacy["output_dir"] = str(temp / "legacy")
        legacy["solver"]["calibrate_executable"] = str(workspace / "build/direct_visual_lidar_calibration/calibrate")
        path = temp / "legacy.yaml"
        path.write_text(yaml.safe_dump(legacy))
        run(path, prepare_only=True)
        assert (temp / "legacy/prepared.yaml").exists()
        assert not (temp / "legacy/extrinsics.yaml").exists()
        report["legacy_yaml_preparation"] = "passed"

        def rejected(label, change, expected):
            current = deepcopy(config)
            current["output"]["directory"] = str(temp / label)
            change(current)
            path = temp / (label + ".yaml")
            path.write_text(yaml.safe_dump(current))
            try:
                run(path)
            except (ValueError, RuntimeError) as error:
                assert expected in str(error), str(error)
                assert not (temp / label).exists()
                assert not list(temp.glob(f".{label}.work-*"))
                report[label] = str(error)
            else:
                raise AssertionError(f"{label}: expected failure")

        rejected("missing_image_topic", lambda c: c["input"]["images"]["cam1"].update(topic="/missing/image"),
                 "missing configured topics")
        rejected("wrong_dimensions", lambda c: c["cameras"]["cam1"].update(width=1920, height=1920), "differs")
        rejected("native_nonconvergence", lambda c: c["solver"].update(max_outer_iterations=1, max_inner_iterations=1),
                 "calibration failed")
        rejected("static_motion_rejection", lambda c: c["static"].update(gyro_p95_max_rad_s=1e-12),
                 "motion screening")
        report["temporary_directories_cleaned"] = not list(temp.glob(".*.work-*"))
    Path(args.report).write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
