"""Input contracts: decoding, timestamp pairing, configuration and portable review."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from dual_mei.config import load_config, visualization_config
from dual_mei.images import decode_message, select_pair
from dual_mei.artifacts import read_ply, write_ply
from dual_mei.rendering import render_overlay, shared_color_range
from dual_mei.viewer import Viewport
from bag_test_fixtures import write_minimal_bag

EPOCH = 1590192300000000000
TOPICS = ("/cam0/image", "/cam1/image")


def image_message(t, image=None, compressed=True):
    image = np.arange(12 * 16 * 3, dtype=np.uint8).reshape(12, 16, 3) if image is None else image
    header = NS(stamp=NS(sec=t // 10**9, nanosec=t % 10**9), frame_id="optical")
    if compressed:
        ok, data = cv2.imencode(".png", image)
        assert ok
        return NS(header=header, data=data.tobytes(), format="png")
    return NS(header=header, data=image.tobytes(), height=image.shape[0], width=image.shape[1],
              step=image.shape[1] * 3, encoding="bgr8", is_bigendian=False)


def records(times0, times1):
    rows = []
    for topic, times in zip(TOPICS, (times0, times1)):
        for t in times:
            stamp = EPOCH + t
            rows.append((topic, stamp, image_message(stamp), "sensor_msgs/msg/CompressedImage"))
    return sorted(rows, key=lambda row: row[1])


class ImageTests(unittest.TestCase):
    def select(self, rows, **options):
        return select_pair(iter(rows), TOPICS, EPOCH, EPOCH + 10**9,
                           dict(target_offset_sec=None, max_pair_delta_sec=0.0, **options))

    def test_exact_pair_midpoint_and_half_open_boundaries(self):
        frames, info = self.select(records([-1, 0, 490_000_000, 600_000_000, 10**9],
                                          [-1, 0, 490_000_000, 600_000_000, 10**9]))
        self.assertEqual(info["frames_in_interval"], {"cam0": 3, "cam1": 3})
        self.assertEqual(frames["cam0"].source["timestamp_ns"], EPOCH + 490_000_000)
        self.assertEqual(info["pair_delta_ns"], 0)
        with self.assertRaisesRegex(ValueError, "no image pair"):
            self.select(records([-1, 10**9], [-1, 10**9]))

    def test_tolerance_and_target_override(self):
        rows = records([100_000_000, 500_000_000], [105_000_000, 502_000_000])
        with self.assertRaisesRegex(ValueError, "no image pair"):
            self.select(rows)
        frames, info = select_pair(iter(rows), TOPICS, EPOCH, EPOCH + 10**9,
                                  dict(target_offset_sec=0.1, max_pair_delta_sec=.006))
        self.assertEqual(frames["cam0"].source["timestamp_ns"], EPOCH + 100_000_000)
        self.assertEqual(info["pair_delta_ns"], 5_000_000)

    def test_pairing_considers_alternatives_and_tie_breaks_earlier(self):
        rows = records([400_000_000, 600_000_000], [400_000_000, 600_000_000])
        frames, _ = self.select(rows)
        self.assertEqual(frames["cam0"].source["timestamp_ns"], EPOCH + 400_000_000)
        rows = records([480_000_000, 490_000_000], [505_000_000])
        frames, _ = select_pair(iter(rows), TOPICS, EPOCH, EPOCH + 10**9,
                                dict(target_offset_sec=None, max_pair_delta_sec=.03))
        self.assertEqual(frames["cam0"].source["timestamp_ns"], EPOCH + 490_000_000)

    def test_bad_type_duplicate_stamps_and_missing_pair(self):
        rows = records([0, 0], [0])
        with self.assertRaisesRegex(ValueError, "strictly increase"):
            self.select(rows)
        rows = records([0], [])
        with self.assertRaisesRegex(ValueError, "no image pair"):
            self.select(rows)
        topic, t, msg, _ = rows[0]
        with self.assertRaisesRegex(ValueError, "expected Image"):
            self.select([(topic, t, msg, "sensor_msgs/msg/Imu")])

    def test_compressed_payload_preserved_and_corrupt_rejected(self):
        msg = image_message(EPOCH)
        image, raw, suffix = decode_message(msg, "sensor_msgs/msg/CompressedImage")
        self.assertEqual(raw, msg.data)
        self.assertEqual(suffix, ".png")
        np.testing.assert_array_equal(image, np.arange(12 * 16 * 3, dtype=np.uint8).reshape(12, 16, 3))
        msg.data = b"corrupt"
        with self.assertRaises(ValueError):
            decode_message(msg, "sensor_msgs/msg/CompressedImage")

    def test_raw_rgb_with_row_padding(self):
        msg = image_message(EPOCH, compressed=False)
        rgb = np.array([[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]], np.uint8)
        msg.width, msg.height, msg.step, msg.encoding = 2, 2, 8, "rgb8"
        msg.data = b"".join(row.tobytes() + b"xx" for row in rgb)
        image, encoded, suffix = decode_message(msg, "sensor_msgs/msg/Image")
        np.testing.assert_array_equal(image, rgb[:, :, ::-1])
        np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR), image)
        self.assertEqual(suffix, ".png")
        msg.data = msg.data[:-1]
        with self.assertRaisesRegex(ValueError, "payload"):
            decode_message(msg, "sensor_msgs/msg/Image")

    def test_raw_gray_alpha_and_unsupported_encoding(self):
        for encoding, source, expected in (
            ("mono8", [42], [42, 42, 42]),
            ("rgba8", [1, 2, 3, 80], [3, 2, 1]),
            ("bgra8", [1, 2, 3, 80], [1, 2, 3]),
        ):
            msg = NS(width=1, height=1, step=len(source), data=bytes(source), encoding=encoding)
            image, _, _ = decode_message(msg, "sensor_msgs/msg/Image")
            np.testing.assert_array_equal(image[0, 0], expected)
        msg.encoding = "16UC1"
        with self.assertRaisesRegex(ValueError, "unsupported Image encoding"):
            decode_message(msg, "sensor_msgs/msg/Image")

    def test_stalled_stream_memory_is_bounded(self):
        with self.assertRaisesRegex(ValueError, "too many unpaired"):
            self.select(records(list(range(300)), []))


class ConfigTests(unittest.TestCase):
    def config(self, directory, mode="bag_only", width=16):
        directory = Path(directory)
        write_minimal_bag(directory / "bag", ros2=True)
        camera = dict(model="mei", width=width, height=12, fx=10, fy=10, cx=8, cy=6,
                      xi=2, distortion=dict(k1=0,k2=0,k3=0,p1=0,p2=0), T_cam_lidar=np.eye(4).tolist())
        cfg = dict(schema_version=2, input=dict(mode=mode, bag_path="bag", images={
            "cam0":dict(topic=TOPICS[0], image_path="missing.jpg"),
            "cam1":dict(topic=TOPICS[1], image_path="missing.jpg")}),
            cameras=dict(cam0=deepcopy(camera),cam1=deepcopy(camera)), output=dict(directory="out"))
        return cfg

    def test_mode_specific_paths_and_invalid_keys(self):
        with tempfile.TemporaryDirectory() as t:
            cfg = self.config(t)
            path = Path(t) / "config.yaml"
            path.write_text(yaml.safe_dump(cfg))
            parsed = load_config(path)
            self.assertEqual(parsed.output, Path(t) / "out")
            cfg["input"]["mode"] = "bag_and_images"
            path.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError, "file does not exist"):
                load_config(path)
            cfg["input"]["mode"] = "bag_only"
            cfg["solver"] = {"max_outer_iteration": 10}
            path.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError, "unknown"):
                load_config(path)

    def test_duplicate_topics_and_outside_target(self):
        with tempfile.TemporaryDirectory() as t:
            cfg = self.config(t)
            path = Path(t) / "config.yaml"
            cfg["input"]["images"]["cam1"]["topic"] = TOPICS[0]
            path.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError, "differ"):
                load_config(path)
            cfg["input"]["images"]["cam1"]["topic"] = TOPICS[1]
            cfg["image_selection"] = {"target_offset_sec": 1.0}
            path.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError, "within"):
                load_config(path)

    def test_legacy_and_scaled_intrinsics(self):
        with tempfile.TemporaryDirectory() as t:
            cfg = self.config(t)
            for name in ("cam0", "cam1"):
                cv2.imwrite(str(Path(t) / f"{name}.png"), np.zeros((12, 16, 3), np.uint8))
                cfg["cameras"][name]["image_path"] = f"{name}.png"
            legacy = dict(schema_version=1, bag_path="bag", output_dir="out", cameras=cfg["cameras"])
            path = Path(t) / "config.yaml"
            path.write_text(yaml.safe_dump(legacy))
            parsed = load_config(path)
            self.assertEqual(parsed.input["mode"], "bag_and_images")
            from dual_mei.images import load_images
            load_images(parsed, {})
            # Same images must fail if the camera dimensions claim another scale.
            parsed.cameras["cam0"]["camera"]["width"] = 32
            with self.assertRaisesRegex(ValueError, "differs"):
                load_images(parsed, {})


class ReviewTests(unittest.TestCase):
    def test_viewport_zoom_keeps_cursor_position_and_pan_changes_center(self):
        view = Viewport((3840, 3840, 3))
        point = view.image_position(800, 200)
        view.zoom_at(800, 200, 2)
        np.testing.assert_allclose(view.image_position(800, 200), point)
        center = view.center.copy()
        view.pan(100, -50)
        np.testing.assert_allclose(view.center, center - np.array([100, -50]) / view.scale)
        self.assertEqual(view.display(np.zeros((3840, 3840, 3),np.uint8)).shape, (900,1280,3))
        view.reset()
        self.assertEqual(view.zoom, 1)

    def test_ply_round_trip_and_truncation(self):
        with tempfile.TemporaryDirectory() as t:
            path = Path(t) / "cloud.ply"
            xyz = np.array([[1.1,2.2,3.3],[4,5,6]])
            write_ply(path, xyz, [.2,.8])
            points, values = read_ply(path)
            np.testing.assert_array_equal(points, xyz.astype(np.float32).astype(float))
            path.write_bytes(path.read_bytes()[:-1])
            with self.assertRaisesRegex(ValueError, "payload"):
                read_ply(path)

    def test_render_no_hidden_60000_limit_and_radius_changes_no_geometry(self):
        xx, yy = np.meshgrid(np.linspace(-.8,.8,300), np.linspace(-.8,.8,300))
        points = np.column_stack((xx.ravel(),yy.ravel(),np.ones(xx.size)))
        c = dict(width=1000,height=1000,fx=400.,fy=400.,cx=500.,cy=500.,xi=0.,
                 max_theta_deg=95,distortion=dict(k1=0,k2=0,k3=0,p1=0,p2=0))
        options = visualization_config({})
        values = np.linspace(0,1,len(points))
        options["color_range"] = shared_color_range(points,values,[np.eye(4)],c,None,options)
        _, a = render_overlay(np.zeros((1000,1000,3),np.uint8),points,values,np.eye(4),c,None,options)
        self.assertGreater(a["drawn_points"],60000)
        options["point_radius_px"] = 6
        options["max_points"] = 500
        _, b = render_overlay(np.zeros((1000,1000,3),np.uint8),points,values,np.eye(4),c,None,options)
        self.assertEqual(a["valid_points"],b["valid_points"])
        self.assertEqual(b["drawn_points"],500)


if __name__ == "__main__":
    unittest.main()
