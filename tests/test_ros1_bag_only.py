"""Real serialized ROS1 inputs through the shared selection/static/prepare pipeline.

Every bag is written and read from a TemporaryDirectory. These tests exercise
the offline input contract, not convergence of the native NID optimizer.
"""
from copy import deepcopy
from importlib import import_module
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from dual_mei.bag_readers import choose_backend, iter_messages
from dual_mei.config import load_config
from dual_mei.images import decode_message, load_images, select_pair
from dual_mei.pipeline import run
from static_bag_input import (DEFAULT_CONFIG, StaticDataError,
                              _decode_livox, _decode_pointcloud2, extract_static_cloud)

try:
    from rosbags.rosbag1 import Writer as Ros1Writer
    from rosbags.rosbag2 import Writer as Ros2Writer
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError:
    Ros1Writer = None

SENSOR = 1_590_192_300_000_000_000  # 2020 sensor clock.
RECORD = 1_778_843_736_000_000_000  # 2026 recorder clock.
TOPICS = ("/cam0/image", "/cam1/image")
CUSTOM = "livox_ros_driver2/msg/CustomMsg"
POINT = "livox_ros_driver2/msg/CustomPoint"
CUSTOM_DEF = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_ros_driver2/CustomPoint[] points
"""
POINT_DEF = """uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""


class BagFixture:
    """Use generated ROS classes and real ROS1/CDR serializers, never mock bags."""

    def __init__(self, ros2=False):
        self.ros2 = ros2
        self.store = get_typestore(Stores.ROS2_HUMBLE if ros2 else Stores.ROS1_NOETIC)
        self.store.register({**get_types_from_msg(POINT_DEF, POINT),
                             **get_types_from_msg(CUSTOM_DEF, CUSTOM)})
        self.types = self.store.types
        y, x = np.indices((64, 64))
        self.pixels = np.stack((x * 3, y * 3, (x + y) * 2), axis=-1).astype(np.uint8)

    def header(self, stamp, frame="livox_frame"):
        time = self.types["builtin_interfaces/msg/Time"](stamp // 10**9, stamp % 10**9)
        args = (time, frame) if self.ros2 else (0, time, frame)
        return self.types["std_msgs/msg/Header"](*args)

    def image(self, offset, suffix=".png", raw=False):
        header = self.header(SENSOR + offset, "optical")
        if raw:
            # RGB, with a non-image sentinel at the end of every row.
            payload = b"".join(row[:, ::-1].tobytes() + b"PAD!" for row in self.pixels)
            return self.types["sensor_msgs/msg/Image"](
                header, 64, 64, "rgb8", 0, 64 * 3 + 4, np.frombuffer(payload, np.uint8).copy())
        ok, payload = cv2.imencode(suffix, self.pixels)
        assert ok
        return self.types["sensor_msgs/msg/CompressedImage"](header, suffix[1:], payload.ravel())

    def image_rows(self, times0=(0, 490_000_000, 600_000_000, 10**9), times1=None):
        times1 = times0 if times1 is None else times1
        rows = []
        for index, times in enumerate((times0, times1)):
            for ordinal, offset in enumerate(times):
                msg = self.image(offset, ".jpg" if index == 0 else ".png")
                # Record order remains monotonic even in deliberately bad header cases.
                rows.append((TOPICS[index], RECORD + ordinal * 100_000_000 + index,
                             msg, msg.__msgtype__))
        return rows

    @staticmethod
    def scene():
        u, v = np.meshgrid(np.linspace(-1.2, 1.2, 18), np.linspace(-1.0, 1.0, 18))
        return np.concatenate((np.column_stack((u.ravel(), v.ravel(), np.full(u.size, 4.))),
                               np.column_stack((np.full(u.size, -2.), u.ravel(), v.ravel() + 4.)),
                               np.column_stack((u.ravel(), np.full(u.size, 2.), v.ravel() + 4.))))

    def imu(self, offset, gyro=0.0):
        vector = self.types["geometry_msgs/msg/Vector3"]
        return self.types["sensor_msgs/msg/Imu"](
            self.header(SENSOR + offset), self.types["geometry_msgs/msg/Quaternion"](0., 0., 0., 1.),
            np.zeros(9), vector(gyro, 0., 0.), np.zeros(9), vector(0., 0., 1.), np.zeros(9))

    def lidar(self, offset, pointcloud=False, shift=None):
        xyz = self.scene().astype(np.float32)
        if shift is not None:
            xyz += np.asarray(shift, np.float32)
        offsets = np.linspace(0, 100_000_000, len(xyz)).astype(np.uint32)
        reflectivity = (np.arange(len(xyz)) % 250 + 1).astype(np.uint8)
        if not pointcloud:
            points = [self.types[POINT](int(t), float(p[0]), float(p[1]), float(p[2]), int(r), 0, 0)
                      for p, t, r in zip(xyz, offsets, reflectivity)]
            return self.types[CUSTOM](self.header(SENSOR + offset), SENSOR + offset,
                                      len(points), 0, np.zeros(3, np.uint8), points)
        # Organized cloud, explicit uint32 relative ns and padded rows.
        width, height, point_step = 18, len(xyz) // 18, 20
        row_step = width * point_step + 8
        raw = bytearray([254] * row_step * height)
        dtype = np.dtype(dict(names=["x", "y", "z", "intensity", "offset_time"],
                              formats=["<f4", "<f4", "<f4", "<f4", "<u4"],
                              offsets=[0, 4, 8, 12, 16], itemsize=point_step))
        values = np.ndarray((height, width), dtype=dtype, buffer=raw,
                            strides=(row_step, point_step))
        for axis, name in enumerate(("x", "y", "z")):
            values[name] = xyz[:, axis].reshape(height, width)
        values["intensity"] = reflectivity.reshape(height, width)
        values["offset_time"] = offsets.reshape(height, width)
        field = self.types["sensor_msgs/msg/PointField"]
        fields = [field(name, index * 4, 6 if name == "offset_time" else 7, 1)
                  for index, name in enumerate(dtype.names)]
        return self.types["sensor_msgs/msg/PointCloud2"](
            self.header(SENSOR + offset), height, width, fields, False, point_step, row_step,
            np.frombuffer(raw, np.uint8).copy(), True)

    def sensor_rows(self, pointcloud=False, motion=False, translated=False, duration=1.1):
        rows = []
        for i in range(round(duration * 100) + 1):
            offset = i * 10_000_000
            msg = self.imu(offset, .2 if motion and 200_000_000 <= offset <= 400_000_000 else 0.)
            rows.append(("/livox/imu", RECORD + offset, msg, msg.__msgtype__))
        for i in range(round(duration * 10) + 1):
            offset = i * 100_000_000
            shift = (.08, -.03, .04) if translated and offset >= 500_000_000 else None
            msg = self.lidar(offset, pointcloud, shift)
            rows.append(("/livox/lidar", RECORD + offset + 80_000_000, msg, msg.__msgtype__))
        return rows

    def write(self, path, rows):
        writer = Ros2Writer(path, version=9) if self.ros2 else Ros1Writer(path)
        with writer:
            connections = {}
            serialize = self.store.serialize_cdr if self.ros2 else self.store.serialize_ros1
            for topic, record, message, kind in sorted(rows, key=lambda row: row[1]):
                key = topic, kind
                if key not in connections:
                    connections[key] = writer.add_connection(topic, kind, typestore=self.store)
                writer.write(connections[key], record, serialize(message, kind))
        return path


@unittest.skipIf(Ros1Writer is None, "real ROS1 input tests require rosbags")
class Ros1BagOnlyTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="dual-mei-ros1-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.fixture = BagFixture()

    def write(self, rows, name="input.bag"):
        return self.fixture.write(self.root / name, rows)

    def config(self, bag, **static):
        camera = dict(model="mei", width=64, height=64, fx=35., fy=35., cx=32., cy=32., xi=2.,
                      distortion=dict(k1=0., k2=0., k3=0., p1=0., p2=0.), T_cam_lidar=np.eye(4).tolist())
        raw = dict(schema_version=2,
                   input=dict(mode="bag_only", bag_path=str(bag),
                              images={name: dict(topic=topic) for name, topic in zip(("cam0", "cam1"), TOPICS)}),
                   static=dict(backend="rosbags", min_points=100, **static),
                   cameras={name: deepcopy(camera) for name in ("cam0", "cam1")},
                   pointcloud=dict(voxel_size_m=0., min_camera_points=100, split_margin_px=4),
                   solver=dict(calibrate_executable=sys.executable), output=dict(directory="out"))
        path = self.root / "config.yaml"
        path.write_text(yaml.safe_dump(raw))
        return path

    def select(self, bag, **selection):
        reader = iter_messages(bag, TOPICS, "rosbags")
        options = dict(target_offset_sec=None, max_pair_delta_sec=0.)
        options.update(selection)
        try:
            return select_pair(reader, TOPICS, SENSOR, SENSOR + 10**9, options)
        finally:
            reader.close()

    def assert_no_result(self):
        self.assertFalse((self.root / "out").exists())
        self.assertFalse(list(self.root.glob(".out.work-*")))
        # Linux Reader resources must be released even when selection raises.
        for fd in Path("/proc/self/fd").glob("*"):
            try:
                target = str(fd.resolve(strict=True))
            except FileNotFoundError:
                continue
            self.assertFalse(target.startswith(str(self.root)) and target.endswith(".bag"), target)

    def test_serialized_jpeg_png_padding_and_nested_livox(self):
        messages = [("/jpeg", self.fixture.image(0, ".jpg")),
                    ("/png", self.fixture.image(0)),
                    ("/raw", self.fixture.image(0, raw=True)),
                    ("/livox/lidar", self.fixture.lidar(0))]
        bag = self.write([(topic, RECORD + i, msg, msg.__msgtype__)
                          for i, (topic, msg) in enumerate(messages)])
        rows = list(iter_messages(bag, [p[0] for p in messages], "auto"))
        self.assertEqual(choose_backend(bag, "auto"), "rosbags")
        for row, (_, original) in zip(rows[:3], messages[:3]):
            decoded, payload, suffix = decode_message(row[2], row[3])
            self.assertIn("/msg/", row[3])  # rosbags canonical ROS1 type names.
            if row[0] != "/raw":
                self.assertEqual(payload, bytes(original.data))
            if row[0] != "/jpeg":
                np.testing.assert_array_equal(decoded, self.fixture.pixels)
                self.assertEqual(suffix, ".png")
            else:
                self.assertEqual(suffix, ".jpg")
        xyz, intensity, stamps = _decode_livox(rows[-1][2])
        np.testing.assert_array_equal(xyz, self.fixture.scene().astype(np.float32))
        self.assertEqual((stamps[0], stamps[-1]), (SENSOR, SENSOR + 100_000_000))
        self.assertEqual(intensity[0], 1)

    def test_header_selection_half_open_target_tolerance_and_bounds(self):
        bag = self.write(self.fixture.image_rows((-1, 0, 490_000_000, 600_000_000, 10**9)))
        frames, info = self.select(bag)
        self.assertEqual(info["frames_in_interval"], dict(cam0=3, cam1=3))
        self.assertEqual(frames["cam0"].source["timestamp_ns"], SENSOR + 490_000_000)
        self.assertGreater(frames["cam0"].source["record_timestamp_ns"], RECORD)
        selected, _ = select_pair(iter_messages(bag, TOPICS, "rosbags"), TOPICS,
                                  SENSOR, SENSOR + 10**9,
                                  dict(target_offset_sec=0., max_pair_delta_sec=0.))
        self.assertEqual(selected["cam0"].source["timestamp_ns"], SENSOR)
        bounded = list(iter_messages(bag, TOPICS, "rosbags", RECORD + 100_000_000))
        self.assertTrue(bounded)
        self.assertTrue(all(row[1] <= RECORD + 100_000_000 for row in bounded))
        tolerant = self.write(self.fixture.image_rows((100_000_000, 500_000_000),
                                                      (105_000_000, 502_000_000)), "tolerant.bag")
        with self.assertRaisesRegex(ValueError, "no image pair"):
            self.select(tolerant)
        pair, info = select_pair(iter_messages(tolerant, TOPICS, "rosbags"), TOPICS,
                                 SENSOR, SENSOR + 10**9,
                                 dict(target_offset_sec=.1, max_pair_delta_sec=.006))
        self.assertEqual(pair["cam0"].source["timestamp_ns"], SENSOR + 100_000_000)
        self.assertEqual(info["pair_delta_ns"], 5_000_000)

    def test_serialized_missing_duplicate_backward_and_stalled_images(self):
        for label, times0, times1, error in (
            ("boundary", (-1, 10**9), (-1, 10**9), "no image pair"),
            ("duplicate", (0, 0), (0, 200_000_000), "strictly increase"),
            ("backward", (400_000_000, 200_000_000), (400_000_000, 600_000_000), "strictly increase"),
            ("unpaired", (100_000_000,), (200_000_000,), "no image pair"),
            ("stalled", tuple(range(300)), (900_000_000,), "too many unpaired"),
        ):
            with self.subTest(label=label):
                rows = self.fixture.image_rows(times0, times1)
                if label == "stalled":
                    rows = [(t, r + (100 * 10**9 if t == TOPICS[1] else 0), m, k) for t, r, m, k in rows]
                bag = self.write(rows, label + ".bag")
                with self.assertRaisesRegex(ValueError, error):
                    self.select(bag)

    def test_invalid_serialized_payload_and_missing_point_time_fail_explicitly(self):
        corrupt_image = self.fixture.image(0)
        corrupt_image.data = np.frombuffer(b"invalid PNG payload", np.uint8)
        bad_step = self.fixture.image(0, raw=True)
        bad_step.step = 1
        cloud = self.fixture.lidar(0, pointcloud=True)
        cloud.fields = [field for field in cloud.fields if field.name != "offset_time"]
        for label, message, error in (("compressed", corrupt_image, "JPEG or PNG"),
                                       ("raw", bad_step, "row step"),
                                       ("point_time", cloud, "Header-only")):
            with self.subTest(label=label):
                bag = self.write([("/input", RECORD, message, message.__msgtype__)], label + ".bag")
                decoded = list(iter_messages(bag, ("/input",), "rosbags"))[0]
                with self.assertRaisesRegex(ValueError, error):
                    if label == "point_time":
                        _decode_pointcloud2(decoded[2], DEFAULT_CONFIG)
                    else:
                        decode_message(decoded[2], decoded[3])

    def test_nested_definition_missing_conflict_and_md5_fail_before_yield(self):
        message = self.fixture.lidar(0)
        definition, digest = self.fixture.store.generate_msgdef(CUSTOM, ros_version=1)
        raw = self.fixture.store.serialize_ros1(message, CUSTOM)
        for label, msgdef, md5, reason in (
            ("missing", CUSTOM_DEF, digest, "missing ROS1 message definition"),
            ("conflict", definition.replace("float32 x\n", "float64 x\n"), digest,
             "conflicting ROS1 message definition"),
            ("digest", definition, "0" * 32, "MD5 mismatch"),
        ):
            with self.subTest(label=label):
                bag = self.root / (label + ".bag")
                with Ros1Writer(bag) as writer:
                    if label == "conflict":
                        good = writer.add_connection("/good", CUSTOM, typestore=self.fixture.store)
                        writer.write(good, RECORD, raw)
                    bad = writer.add_connection("/bad", CUSTOM, msgdef=msgdef, md5sum=md5)
                    writer.write(bad, RECORD + 1, raw)
                topics = ("/good", "/bad") if label == "conflict" else ("/bad",)
                reader = iter_messages(bag, topics, "rosbags")
                try:
                    with self.assertRaisesRegex(ValueError, reason):
                        next(reader)
                finally:
                    reader.close()
                self.assert_no_result()

    def test_ros1_static_check_prepare_and_sensor_time_clipping(self):
        bag = self.write(self.fixture.sensor_rows() + self.fixture.image_rows())
        path = self.config(bag)
        cfg = load_config(path)
        cloud, _, report = extract_static_cloud(bag, cfg.static)
        self.assertEqual(report["sensor_start_ns"], SENSOR)
        self.assertEqual(report["sensor_end_ns_exclusive"], SENSOR + 10**9)
        self.assertEqual(report["geometry"]["status"], "passed")
        self.assertGreater(report["imu_record_minus_sensor_sec"], 100_000_000)
        self.assertGreater(report["lidar_record_minus_sensor_sec"], report["imu_record_minus_sensor_sec"])
        expected_count = 10 * len(self.fixture.scene()) - 1  # Exclude final packet's exact end.
        self.assertEqual(len(cloud), expected_count)
        np.testing.assert_array_equal(cloud, np.tile(self.fixture.scene().astype(np.float32), (10, 1))[:-1])
        self.assertEqual(run(path, check_static_only=True), 0)
        self.assert_no_result()
        self.assertEqual(run(path, prepare_only=True), 0)
        out = self.root / "out"
        result = yaml.safe_load((out / "prepared.yaml").read_text())
        self.assertEqual(result["status"], "prepared")
        self.assertEqual(result["direction"], "lidar_to_camera")
        self.assertEqual(len(list(p for p in out.rglob("*") if p.is_file())), 7)
        self.assertFalse((out / "extrinsics.yaml").exists())
        self.assertFalse(list(self.root.glob(".out.work-*")))
        with self.assertRaisesRegex(ValueError, "new or empty"):
            run(path, prepare_only=True)

    def test_ros1_ros2_same_physics_equal_frames_times_and_optimizer_ply(self):
        outputs = []
        decoded = []
        for label, ros2, pointcloud in (("custom", False, False),
                                        ("ros1", False, True), ("ros2", True, True)):
            fixture = BagFixture(ros2)
            bag = fixture.write(self.root / (label if ros2 else label + ".bag"),
                                fixture.sensor_rows(pointcloud=pointcloud) + fixture.image_rows())
            path = self.config(bag, pointcloud_time_unit="ns", pointcloud_time_reference="relative")
            cfg = load_config(path)
            cloud, intensity, report = extract_static_cloud(bag, cfg.static)
            frames, selection = load_images(cfg, report)
            packets = list(iter_messages(bag, ("/livox/lidar",), "rosbags"))
            if pointcloud:
                xyz, values, stamps, metadata = _decode_pointcloud2(packets[0][2], cfg.static)
                self.assertEqual(metadata["boundary_guard_ns"], 0)
            else:
                xyz, values, stamps = _decode_livox(packets[0][2])
            self.assertEqual((stamps[0], stamps[-1]), (SENSOR, SENSOR + 100_000_000))
            np.testing.assert_array_equal(xyz, fixture.scene().astype(np.float32))
            decoded.append((cloud, intensity, stamps,
                            [(f.source["timestamp_ns"], hashlib.sha256(f.encoded).hexdigest())
                             for f in frames.values()]))
            output = self.root / ("prepared_" + label)
            self.assertEqual(run(path, prepare_only=True, output_override=str(output)), 0)
            outputs.append(output)
        for index in (1, 2):
            for a, b in zip(decoded[0][:3], decoded[index][:3]):
                np.testing.assert_array_equal(a, b)
            self.assertEqual(decoded[0][3], decoded[index][3])
            for camera in ("cam0", "cam1"):
                self.assertEqual((outputs[0] / camera / "lidar.ply").read_bytes(),
                                 (outputs[index] / camera / "lidar.ply").read_bytes())
                self.assertEqual((outputs[0] / camera / "overlay_before.png").read_bytes(),
                                 (outputs[index] / camera / "overlay_before.png").read_bytes())

    def test_static_motion_geometry_and_coverage_fail_without_interval_search(self):
        for label, changes, reason in (
            ("imu_motion", dict(motion=True, duration=2.), "motion_detected"),
            ("lidar_motion", dict(translated=True), "lidar_motion_detected"),
            ("short", dict(duration=.8), "insufficient_coverage"),
        ):
            with self.subTest(label=label):
                bag = self.write(self.fixture.sensor_rows(**changes) + self.fixture.image_rows(), label + ".bag")
                path = self.config(bag)
                with self.assertRaises(StaticDataError) as caught:
                    run(path, prepare_only=True)
                self.assertEqual(caught.exception.report["reason_code"], reason)
                self.assertEqual(caught.exception.report.get("sensor_start_ns"), SENSOR)
                self.assert_no_result()

    def test_bad_inputs_and_second_solver_failure_leave_no_result_or_workdir(self):
        sensors = self.fixture.sensor_rows()
        images = self.fixture.image_rows()
        wrong_type = self.fixture.imu(490_000_000)
        for label, rows, message in (
            ("missing_topic", sensors + [row for row in images if row[0] == TOPICS[0]], "missing configured topics"),
            ("wrong_type", sensors + [row for row in images if row[0] == TOPICS[0]] +
             [(TOPICS[1], RECORD, wrong_type, wrong_type.__msgtype__)], "expected Image"),
            ("unpaired", sensors + self.fixture.image_rows((0,), (100_000_000,)), "no image pair"),
        ):
            with self.subTest(label=label):
                bag = self.write(rows, label + ".bag")
                with self.assertRaisesRegex((ValueError, StaticDataError), message):
                    run(self.config(bag), prepare_only=True)
                self.assert_no_result()
        bag = self.write(sensors + images, "valid.bag")
        path = self.config(bag)
        raw = yaml.safe_load(path.read_text())
        raw["cameras"]["cam1"]["width"] = 65
        path.write_text(yaml.safe_dump(raw))
        with self.assertRaisesRegex(ValueError, "differs"):
            run(path, prepare_only=True)
        self.assert_no_result()
        path = self.config(bag)
        diagnostics = dict(success=True, converged=True, inner_converged=True,
                           termination_reason="test_stub", initial_cost=1., final_cost=.5,
                           outer_iterations=1, inner_iterations=1, visible_points=100)
        with patch("dual_mei.pipeline.run_solver", side_effect=[(np.eye(4), diagnostics),
                                                                ValueError("test second solver failed")]) as solver:
            with self.assertRaisesRegex(ValueError, "second solver failed"):
                run(path)
        self.assertEqual(solver.call_count, 2)
        self.assert_no_result()

    def test_format_conflicts_invalid_files_corruption_and_dependencies(self):
        bag = self.write(self.fixture.sensor_rows())
        path = self.config(bag)
        self.assertEqual(load_config(path, check_only=True).bag, bag)
        with self.assertRaisesRegex(ValueError, "rosbag2|ROS2"):
            choose_backend(bag, "rosbag2")
        ros2 = BagFixture(True)
        directory = ros2.write(self.root / "ros2", ros2.sensor_rows(pointcloud=True))
        with self.assertRaisesRegex(ValueError, "rosbag|ROS1"):
            choose_backend(directory, "rosbag")
        with self.assertRaisesRegex(ValueError, "single ROS2"):
            load_config(self.config(next(directory.glob("*.db3"))), check_only=True)
        for name, content in (("text.bag", b"not a bag"),
                              ("single.db3", b"SQLite format 3\x00" + bytes(100)),
                              ("damaged.bag", b"#ROSBAG V2.0\n" + bytes(100))):
            with self.subTest(name=name):
                invalid = self.root / name
                invalid.write_bytes(content)
                with self.assertRaises((ValueError, RuntimeError)):
                    load_config(self.config(invalid), check_only=True)
        imported = import_module

        def no_rosbags(name, *args, **kwargs):
            if name == "rosbags" or name.startswith("rosbags."):
                raise ImportError("test dependency unavailable: rosbags")
            return imported(name, *args, **kwargs)

        with patch("dual_mei.bag_readers.import_module", side_effect=no_rosbags):
            with self.assertRaisesRegex((ImportError, ValueError, RuntimeError), "rosbags"):
                choose_backend(bag, "rosbags")

        def no_ros1_backend(name, *args, **kwargs):
            if name == "rosbag" or name.startswith("rosbags."):
                raise ImportError("test offline reader unavailable")
            return imported(name, *args, **kwargs)

        with patch("dual_mei.bag_readers.import_module", side_effect=no_ros1_backend):
            with self.assertRaisesRegex(ValueError, "no available backend for ROS1"):
                choose_backend(bag, "auto")


if __name__ == "__main__":
    unittest.main()
