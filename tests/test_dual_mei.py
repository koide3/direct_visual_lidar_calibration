"""Numerical contract tests; these do not substitute for real calibration."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from mei_calibration_geometry import (equalize_intensity, inverse_transform,
                                      project_mei, rigid_transform, voxel_downsample, write_ply)
from dual_mei.config import load_config
from dual_mei.optimizer import executable_command, read_calibration_result
from dual_mei.pipeline import run
from bag_test_fixtures import write_minimal_bag


def camera():
    return dict(width=1000, height=1000, fx=400., fy=400., cx=500., cy=500.,
                xi=2., max_theta_deg=95.,
                distortion=dict(k1=.19911757, k2=2.07707953, k3=-3.2787478,
                                p1=.00030418, p2=-.0027582))


class GeometryTests(unittest.TestCase):
    def test_transform_direction_and_native_json(self):
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", [15, -31, 122], degrees=True).as_matrix()
        transform[:3, 3] = [.12, -.25, .03]
        inverse = inverse_transform(transform)
        np.testing.assert_allclose(inverse @ transform, np.eye(4), atol=1e-12)
        values = inverse[:3, 3].tolist()+Rotation.from_matrix(inverse[:3, :3]).as_quat().tolist()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"calib.json"
            path.write_text(json.dumps({"results": {"calibration_status": "success", "T_lidar_camera": values}}))
            actual, _ = read_calibration_result(path)
            np.testing.assert_allclose(actual, transform, atol=1e-12)
            path.write_text(json.dumps({"results": {"calibration_status": "failed", "T_lidar_camera": values}}))
            with self.assertRaises(ValueError):
                read_calibration_result(path)

    def test_back_to_back_and_physical_branch(self):
        points = np.array([[0, 0, 5], [0, 0, -5], [5, 0, 0], [0, 0, 0]], dtype=float)
        rear = np.eye(4)
        rear[:3, :3] = np.diag([-1, 1, -1])
        _, front_valid, _ = project_mei(points, np.eye(4), camera())
        _, rear_valid, _ = project_mei(points, rear, camera())
        np.testing.assert_array_equal(front_valid, [True, False, True, False])
        np.testing.assert_array_equal(rear_valid, [False, True, True, False])
        angle = np.deg2rad([94, 96, 130, 180])
        rays = np.column_stack((np.sin(angle), np.zeros(4), np.cos(angle)))
        _, valid, _ = project_mei(rays, np.eye(4), camera())
        np.testing.assert_array_equal(valid, [True, False, False, False])

    def test_k3_is_not_silently_dropped(self):
        c = camera()
        ray = np.array([[1., 0., 0.]])
        with_k3 = project_mei(ray, np.eye(4), c)[0][0, 0]
        c["distortion"]["k3"] = 0.
        without_k3 = project_mei(ray, np.eye(4), c)[0][0, 0]
        self.assertAlmostEqual(with_k3-without_k3, 400*.5*(-3.2787478)*(.5**6), places=10)

    def test_mask_covers_cubic_footprint(self):
        c = camera()
        mask = np.ones((1000, 1000), dtype=np.uint8)*255
        mask[502, 502] = 0  # Outside a nearest or 2x2 bilinear sample; inside NID's 4x4.
        _, valid, _ = project_mei(np.array([[0., 0., 5.]]), np.eye(4), c, mask)
        self.assertFalse(valid[0])

    def test_validation_and_tied_intensities(self):
        reflection = np.diag([-1., 1., 1., 1.])
        with self.assertRaises(ValueError):
            rigid_transform(reflection)
        with self.assertRaises(ValueError):
            rigid_transform(None)
        values = equalize_intensity(np.array([5, 5, 10, 20]))
        self.assertEqual(values[0], values[1])
        self.assertTrue(np.all(np.diff(values) >= 0))
        with self.assertRaises(ValueError):
            equalize_intensity(np.ones(10))

    def test_ply_and_voxel(self):
        points = np.array([[1., 2, 3], [1.01, 2.01, 3.01], [2, 4, 6]])
        cloud, values = voxel_downsample(points, np.array([10., 20., 30.]), .1)
        self.assertEqual(len(cloud), 2)
        np.testing.assert_allclose(cloud[0], [1.005, 2.005, 3.005])
        self.assertEqual(values[0], 15)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"cloud.ply"
            write_ply(path, cloud, values)
            raw = path.read_bytes().split(b"end_header\n", 1)[1]
            np.testing.assert_allclose(np.frombuffer(raw, dtype="<f4").reshape(-1, 4),
                                       np.column_stack((cloud, values)), rtol=1e-6)


class WorkflowTests(unittest.TestCase):
    def test_ros2_directory_input_and_runtime_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            write_minimal_bag(directory/"ros2_bag", ros2=True)
            config_path = directory/"config.yaml"
            config_path.write_text(yaml.safe_dump({"schema_version": 1, "bag_path": "ros2_bag"}))
            loaded = load_config(config_path, check_only=True)
            self.assertEqual(loaded.bag, directory/"ros2_bag")
            with patch.dict("os.environ", {"ROS_VERSION": "2"}), patch("shutil.which", return_value="/opt/ros/humble/bin/ros2"):
                self.assertEqual(executable_command({}), ["ros2", "run", "direct_visual_lidar_calibration", "calibrate"])
            with patch.dict("os.environ", {"ROS_VERSION": "1"}), patch("shutil.which", return_value="/opt/ros/noetic/bin/rosrun"):
                self.assertEqual(executable_command({}), ["rosrun", "direct_visual_lidar_calibration", "calibrate"])
            with patch.dict("os.environ", {}, clear=True):
                with self.assertRaises(ValueError):
                    executable_command({})

    def pipeline(self, directory, fail_second=False):
        directory = Path(directory)
        image = np.uint8(np.indices((1000, 1000)).sum(axis=0) % 256)
        cv2.imwrite(str(directory/"image.png"), image)
        mask = np.full_like(image, 255)
        cv2.imwrite(str(directory/"mask.png"), mask)
        write_minimal_bag(directory/"input.bag")
        grid = np.linspace(-2, 2, 25)
        xx, yy = np.meshgrid(grid, grid)
        front = np.column_stack((xx.ravel(), yy.ravel(), np.full(xx.size, 5.)))
        cloud = np.concatenate((front, front*np.array([1, 1, -1])))
        intensity = np.arange(len(cloud)) % 255
        cameras = {}
        for index in range(2):
            transform = np.eye(4)
            if index:
                transform[:3, :3] = np.diag([-1., 1., -1.])
            cameras[f"cam{index}"] = dict(camera(), model="mei", image_path="image.png",
                                          mask_path="mask.png", T_cam_lidar=transform.tolist())
        config = {"schema_version": 1, "bag_path": "input.bag", "output_dir": "out",
                  "cameras": cameras, "pointcloud": {"min_camera_points": 100},
                  "solver": {"calibrate_executable": sys.executable}}
        path = directory/"input.yaml"
        path.write_text(yaml.safe_dump(config))

        def fake_optimizer(command, data, solver):
            calib_path = data/"calib.json"
            parsed = json.loads(calib_path.read_text())
            # Native convention remains cam->LiDAR. Test actual exporter inversion.
            self.assertEqual(parsed["camera"]["mask_path"], "mask.png")
            self.assertEqual(parsed["camera"]["camera_model"], "mei")
            results = parsed["results"]
            results["T_lidar_camera"] = results["init_T_lidar_camera"]
            results["calibration_status"] = "failed" if fail_second and data.name == "cam1" else "success"
            calib_path.write_text(json.dumps(parsed))
            return read_calibration_result(calib_path)

        args = SimpleNamespace(config=str(path), output=None, prepare_only=False, check_static_only=False)
        with patch("dual_mei.pipeline.extract_static_cloud", return_value=(cloud, intensity, {"status": "passed", "lidar_frame_id": "livox_frame"})), \
                patch("dual_mei.pipeline.run_solver", side_effect=fake_optimizer):
            if fail_second:
                with self.assertRaises(ValueError):
                    run(args.config)
            else:
                self.assertEqual(run(args.config), 0)
        return directory/"out", cameras

    def test_two_camera_pipeline_exports_direction_and_split(self):
        with tempfile.TemporaryDirectory() as directory:
            output, cameras = self.pipeline(directory)
            results = yaml.safe_load((output/"extrinsics.yaml").read_text())
            report = results
            self.assertEqual(report["pointcloud_summary"]["shared_visible_points"], 0)
            self.assertEqual(report["status"], "success")
            self.assertEqual(len([p for p in output.rglob("*") if p.is_file()]), 11)
            self.assertFalse(list(output.parent.glob(".out.work-*")))
            from dual_mei.viewer import export_result
            export_result(output, output=output.parent/"review")
            for name in ("cam0", "cam1"):
                for stage in ("before", "after"):
                    relative = Path(name)/f"overlay_{stage}.png"
                    self.assertEqual((output/relative).read_bytes(), (output.parent/"review"/relative).read_bytes())
            for name in cameras:
                np.testing.assert_allclose(results["cameras"][name]["T_cam_lidar"], cameras[name]["T_cam_lidar"], atol=1e-12)
                self.assertEqual(report["cameras"][name]["projections"]["before"]["input_points"], 625)

    def test_second_camera_failure_does_not_export_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            output, _ = self.pipeline(directory, fail_second=True)
            self.assertFalse((output/"extrinsics.yaml").exists())
            self.assertFalse((output/"extrin_calib.yaml").exists())
            self.assertFalse(output.exists())
            self.assertFalse(list(Path(directory).glob(".out.work-*")))


if __name__ == "__main__":
    unittest.main()
