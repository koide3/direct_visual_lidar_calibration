"""Synthetic temporal boundaries, sensor failures and geometric observability."""

import sys
import unittest
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from static_bag_input import (DEFAULT_CONFIG, StaticDataError, _configuration,
                              _decode_livox, _decode_pointcloud2, _pointcloud_records,
                              _extract_messages, _screen_geometry)


EPOCH = 1_590_192_300_000_000_000
RECORD_OFFSET = 198_651_436_000_000_000


def stamp(ns):
    return NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def imu(relative_ns, gyro=0.0, acceleration=(0.0, 0.0, 1.0)):
    time = EPOCH + relative_ns
    msg = NS(header=NS(stamp=stamp(time), frame_id="livox_frame"),
             angular_velocity=NS(x=gyro, y=0.0, z=0.0),
             linear_acceleration=NS(x=acceleration[0], y=acceleration[1], z=acceleration[2]))
    return "/livox/imu", time + RECORD_OFFSET, msg, "sensor_msgs/msg/Imu"


def lidar(start_ns, offsets=None, coords=None):
    if offsets is None:
        offsets = np.arange(0, 100_000_001, 10_000_000)
    if coords is None:
        coords = [(1 + (start_ns + int(x)) / 1e9, 1.0, 1.0) for x in offsets]
    pts = [NS(x=p[0], y=p[1], z=p[2], reflectivity=20, offset_time=int(t))
           for p, t in zip(coords, offsets)]
    time = EPOCH + start_ns
    msg = NS(timebase=time, point_num=len(pts), points=pts,
             header=NS(stamp=stamp(time), frame_id="livox_frame"))
    return "/livox/lidar", time + RECORD_OFFSET + 100_000_000, msg, "livox_ros_driver2/msg/CustomMsg"


def records(duration=1.1):
    result = [imu(int(i * 10_000_000)) for i in range(round(duration * 100) + 1)]
    result += [lidar(i * 100_000_000) for i in range(round(duration * 10) + 1)]
    return sorted(result, key=lambda row: row[1])


def config(**kwargs):
    return {"min_points": 1, "geometry_check_enabled": False, **kwargs}


def pointcloud(points=None, point_times=None, bigendian=False, organized=True,
               relative=False, reflectivity=False):
    points = np.asarray(points if points is not None else
                        [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]])
    width = 2 if organized else len(points)
    height = len(points) // width
    intensity_field = "reflectivity" if reflectivity else "intensity"
    time_field = "offset_time" if relative else "timestamp"
    fields = [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1),
              (intensity_field, 12, 4, 1), ("tag", 14, 1, 1), ("line", 15, 2, 1),
              ("unused_uint32", 16, 6, 1), ("unused_int16_pair", 20, 3, 2),
              (time_field, 24, 6 if relative else 8, 1), ("unused_int32", 32, 5, 1)]
    step, row_step = 40, width * 40 + 8
    msg = NS(header=NS(stamp=stamp(EPOCH), frame_id="livox_frame"),
             width=width, height=height, point_step=step, row_step=row_step,
             is_bigendian=bigendian, fields=[NS(name=n, offset=o, datatype=d, count=c)
                                           for n, o, d, c in fields],
             data=bytearray([255] * row_step * height))
    type_map = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    endian = ">" if bigendian else "<"
    dtype = np.dtype({"names": [f[0] for f in fields],
                      "formats": [endian + type_map[d] if c == 1 else (endian + type_map[d], (c,))
                                  for _, _, d, c in fields],
                      "offsets": [f[1] for f in fields], "itemsize": step})
    values = np.ndarray((height, width), dtype=dtype, buffer=msg.data,
                        strides=(row_step, step))
    for axis, name in enumerate(("x", "y", "z")):
        values[name] = points[:, axis].reshape(height, width)
    values[intensity_field] = np.arange(len(points)).reshape(height, width) + 20
    if point_times is None:
        point_times = np.arange(len(points)) * 10_000_000
        if not relative:
            point_times += EPOCH
    values[time_field] = np.asarray(point_times).reshape(height, width)
    return msg


class StaticInputTests(unittest.TestCase):
    def assert_failure(self, data, reason, **kwargs):
        with self.assertRaises(StaticDataError) as caught:
            _extract_messages(data, config(**kwargs))
        self.assertEqual(caught.exception.report["reason_code"], reason)
        self.assertEqual(caught.exception.report["status"], "failed")
        return caught.exception.report

    def test_exact_duration_and_record_clock_offset(self):
        points, intensity, report = _extract_messages(records(), config())
        self.assertEqual(report["sensor_start_ns"], EPOCH)
        self.assertEqual(report["sensor_end_ns_exclusive"], EPOCH + 1_000_000_000)
        self.assertEqual(report["used_duration_sec"], 1.0)
        self.assertGreater(report["imu_record_minus_sensor_sec"], 100_000_000)
        self.assertTrue(np.all(points[:, 0] < 2.0))
        self.assertTrue(np.all(intensity == 20))

    def test_timestamp_nanosecond_precision(self):
        msg = lidar(0, offsets=[0, 1, 999_999_999, 1_000_000_000])[2]
        _, _, times = _decode_livox(msg)
        np.testing.assert_array_equal(times - EPOCH, [0, 1, 999_999_999, 1_000_000_000])

    def test_common_start_uses_later_sensor(self):
        data = [r for r in records(1.2) if r[0] != "/livox/imu" or r[2].header.stamp.nanosec >= 50_000_000]
        # Keep later whole seconds (whose nanosec wraps to zero).
        data += [r for r in records(1.2) if r[0] == "/livox/imu" and
                 r[2].header.stamp.sec > EPOCH // 1_000_000_000 and r[2].header.stamp.nanosec < 50_000_000]
        data.sort(key=lambda row: row[1])
        _, _, report = _extract_messages(data, config())
        self.assertEqual(report["sensor_start_ns"], EPOCH + 50_000_000)

    def test_coverage_is_not_rounded_up(self):
        self.assert_failure(records(0.9), "insufficient_coverage")

    def test_motion_inside_first_interval_is_rejected(self):
        data = records(2.0)
        for row in data:
            if row[0] == "/livox/imu" and 200_000_000 <= row[2].header.stamp.nanosec <= 400_000_000:
                row[2].angular_velocity.x = 0.2
        report = self.assert_failure(data, "motion_detected")
        self.assertEqual(report["sensor_start_ns"], EPOCH)

    def test_acceleration_unit_scale_is_applied(self):
        data = records()
        count = 0
        for row in data:
            if row[0] == "/livox/imu":
                row[2].linear_acceleration.x = (-1) ** count * 0.03
                count += 1
        _extract_messages(data, config(acceleration_scale=1.0))
        self.assert_failure(data, "motion_detected", acceleration_scale=9.80665)

    def test_imu_gap_rejected(self):
        data = [r for r in records() if not (r[0] == "/livox/imu" and
                                             200_000_000 < r[2].header.stamp.nanosec < 500_000_000)]
        self.assert_failure(data, "imu_gap")

    def test_lidar_gap_rejected_including_gap_across_end(self):
        data = [r for r in records(1.2) if r[0] != "/livox/lidar" or
                r[2].timebase < EPOCH + 500_000_000 or r[2].timebase >= EPOCH + 1_100_000_000]
        self.assert_failure(data, "lidar_gap")

    def test_nan_imu_rejected(self):
        data = records()
        data[0][2].linear_acceleration.x = np.nan
        self.assert_failure(data, "invalid_imu")

    def test_zero_acceleration_or_unavailable_imu_rejected(self):
        data = records()
        data[0][2].linear_acceleration.z = 0.0
        self.assert_failure(data, "invalid_imu")
        data = records()
        data[0][2].angular_velocity_covariance = [-1.0] + [0.0] * 8
        self.assert_failure(data, "invalid_imu")

    def test_imu_numeric_overflow_fails_with_serializable_report(self):
        data = records()
        for row in data:
            if row[0] == "/livox/imu":
                row[2].linear_acceleration.z = 1e308
        report = self.assert_failure(data, "invalid_imu")
        json.dumps(report, allow_nan=False)
        data = records()
        for row in data:
            if row[0] == "/livox/imu":
                row[2].angular_velocity.x = 1e308
        report = self.assert_failure(data, "invalid_imu")
        json.dumps(report, allow_nan=False)

    def test_zero_nan_and_low_reflectivity_points_filtered(self):
        data = records()
        packet = next(row[2] for row in data if row[0] == "/livox/lidar")
        packet.points[0].x = packet.points[0].y = packet.points[0].z = 0.0
        packet.points[1].x = np.nan
        packet.points[2].reflectivity = 0
        points, intensity, report = _extract_messages(data, config(min_reflectivity=1))
        self.assertEqual(report["points_filtered"], 3)
        self.assertTrue(np.isfinite(points).all())
        self.assertTrue(np.all(intensity >= 1))

    def test_missing_topics_and_unsupported_cloud_type(self):
        self.assert_failure([], "missing_imu")
        self.assert_failure([imu(0)], "missing_lidar")
        self.assert_failure([lidar(0)], "missing_imu")
        row = lidar(0)
        self.assert_failure([imu(0), (*row[:3], "geometry_msgs/msg/Pose")], "unsupported_lidar_type")

    def test_driver1_message_name_supported(self):
        data = [(*row[:3], "livox_ros_driver/CustomMsg") if row[0] == "/livox/lidar" else row
                for row in records()]
        _extract_messages(data, config())

    def test_lidar_frame_change_rejected(self):
        data = records()
        packets = [row[2] for row in data if row[0] == "/livox/lidar"]
        packets[1].header.frame_id = "map"
        self.assert_failure(data, "lidar_frame_changed")

    def test_duplicate_imu_sensor_stamp_rejected(self):
        data = records()
        imus = [row[2] for row in data if row[0] == "/livox/imu"]
        imus[1].header.stamp = imus[0].header.stamp
        self.assert_failure(data, "imu_time_reversal")

    def test_record_bound_and_early_finish(self):
        consumed = []

        def stream():
            for row in records(8):
                consumed.append(row)
                yield row

        _, _, report = _extract_messages(stream(), config())
        self.assertLess(report["record_span_read_sec"], 1.2)
        self.assertLess(len(consumed), 130)
        data = [row for row in records(3) if row[0] == "/livox/imu"]
        report = self.assert_failure(data, "missing_lidar", max_read_record_sec=1.1)
        self.assertTrue(report["read_limit_reached"])

    def test_configuration_rejects_typo_or_invalid_window(self):
        for settings in ({"duratio_sec": 1}, {"duration_sec": 0}, {"acceleration_scale": -1},
                         {"window_step_sec": 0.5}, {"geometry_check_enabled": "false"},
                         {"duration_sec": "1"}, {"geometry_min_correspondence_ratio": "0.3"},
                         {"duration_sec": 11.0}):
            with self.assertRaises(ValueError):
                _configuration(settings)


class PointCloud2Tests(unittest.TestCase):
    def test_organized_padding_and_both_endian_layouts(self):
        for bigendian in (False, True):
            msg = pointcloud(bigendian=bigendian)
            points, intensity, times, metadata = _decode_pointcloud2(msg, DEFAULT_CONFIG)
            np.testing.assert_array_equal(points, np.arange(1, 13).reshape(4, 3))
            np.testing.assert_array_equal(intensity, [20, 21, 22, 23])
            self.assertLessEqual(np.max(np.abs(times - (EPOCH + np.arange(4) * 10_000_000))), 128)
            self.assertEqual(metadata["source_resolution_ns"], 256.0)
            self.assertEqual(metadata["boundary_guard_ns"], 128)
            self.assertEqual(metadata["unit"], "ns")
            self.assertEqual(metadata["reference"], "absolute")

    def test_integer_relative_nanoseconds_and_reflectivity(self):
        msg = pointcloud(relative=True, reflectivity=True, point_times=[0, 1, 2, 3])
        settings = _configuration({"pointcloud_time_unit": "ns", "pointcloud_time_reference": "relative"})
        _, intensity, times, metadata = _decode_pointcloud2(msg, settings)
        np.testing.assert_array_equal(times - EPOCH, [0, 1, 2, 3])
        np.testing.assert_array_equal(intensity, [20, 21, 22, 23])
        self.assertEqual(metadata["boundary_guard_ns"], 0)

    def test_ambiguous_relative_units_are_not_guessed(self):
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            _decode_pointcloud2(pointcloud(relative=True, point_times=[0, 10, 20, 30]), DEFAULT_CONFIG)

    def test_explicit_absolute_seconds(self):
        raw = EPOCH / 1e9 + np.arange(4) * 0.01
        settings = _configuration({"pointcloud_time_unit": "s", "pointcloud_time_reference": "absolute"})
        _, _, times, metadata = _decode_pointcloud2(pointcloud(point_times=raw), settings)
        self.assertLessEqual(np.max(np.abs(times - (EPOCH + np.arange(4) * 10_000_000))), 200)
        self.assertEqual(metadata["unit"], "s")

    def test_missing_timestamp_fails_without_header_fabrication(self):
        msg = pointcloud()
        msg.fields = [f for f in msg.fields if f.name != "timestamp"]
        with self.assertRaisesRegex(ValueError, "Header-only"):
            _decode_pointcloud2(msg, DEFAULT_CONFIG)

    def test_bad_layout_and_nonfinite_time_are_rejected(self):
        msg = pointcloud()
        msg.row_step = msg.width * msg.point_step - 1
        with self.assertRaisesRegex(ValueError, "strides"):
            _decode_pointcloud2(msg, DEFAULT_CONFIG)
        msg = pointcloud()
        msg.data = msg.data[:-1]
        with self.assertRaisesRegex(ValueError, "data length"):
            _decode_pointcloud2(msg, DEFAULT_CONFIG)
        msg = pointcloud(point_times=[EPOCH, EPOCH, np.nan, EPOCH])
        with self.assertRaisesRegex(ValueError, "NaN"):
            _decode_pointcloud2(msg, DEFAULT_CONFIG)

    def test_extractor_supports_pointcloud2_and_quantized_boundary_guard(self):
        data = [row for row in records(1.2) if row[0] == "/livox/imu"]
        # Relative uint32 times remain exact, permitting an exact boundary test.
        offsets = [0, 100_000_000, 900_000_000, 1_000_000_000]
        msg = pointcloud(relative=True, point_times=offsets)
        data.append(("/livox/lidar", EPOCH + RECORD_OFFSET, msg, "sensor_msgs/msg/PointCloud2"))
        data.sort(key=lambda row: row[1])
        points, _, report = _extract_messages(data, config(pointcloud_time_unit="ns",
                                                         pointcloud_time_reference="relative"))
        self.assertEqual(len(points), 3)
        self.assertEqual(report["lidar_timestamp"]["granularity"], "per_point")
        msg = pointcloud(point_times=[EPOCH, EPOCH + 100_000_000,
                                     EPOCH + 900_000_000, EPOCH + 1_000_000_000])
        data = [row for row in data if row[0] == "/livox/imu"]
        data.append(("/livox/lidar", EPOCH + RECORD_OFFSET, msg, "sensor_msgs/msg/PointCloud2"))
        data.sort(key=lambda row: row[1])
        points, _, report = _extract_messages(data, config())
        self.assertEqual(len(points), 2)  # Quantization-uncertain boundary points excluded.
        self.assertEqual(report["point_boundary_guard_ns"], 128)


class GeometryTests(unittest.TestCase):
    @staticmethod
    def scene():
        rng = np.random.default_rng(7)
        uv = rng.uniform(-1.5, 1.5, (3500, 2))
        return np.concatenate((np.column_stack((uv, np.full(len(uv), 2.0))),
                               np.column_stack((np.full(len(uv), 2.0), uv)),
                               np.column_stack((uv[:, 0], np.full(len(uv), -2.0), uv[:, 1]))))

    def test_static_scene_passes(self):
        scene = self.scene()
        report = {}
        _screen_geometry(scene, scene + np.random.default_rng(8).normal(0, 0.001, scene.shape),
                         DEFAULT_CONFIG, report)
        self.assertEqual(report["geometry"]["status"], "passed")
        self.assertLess(report["geometry"]["translation_m"], 0.003)

    def test_translation_without_imu_acceleration_is_detected(self):
        scene = self.scene()
        report = {}
        with self.assertRaises(StaticDataError) as caught:
            _screen_geometry(scene, scene + (0.08, -0.03, 0.04), DEFAULT_CONFIG, report)
        self.assertEqual(caught.exception.report["reason_code"], "lidar_motion_detected")
        self.assertGreater(report["geometry"]["translation_m"], 0.07)

    def test_planar_scene_rejected_as_degenerate(self):
        plane = self.scene()[:3500]
        with self.assertRaises(StaticDataError) as caught:
            _screen_geometry(plane, plane, DEFAULT_CONFIG, {})
        self.assertEqual(caught.exception.report["reason_code"], "geometry_degenerate")

    def test_no_overlap_is_not_reported_as_static(self):
        scene = self.scene()
        with self.assertRaises(StaticDataError) as caught:
            _screen_geometry(scene, scene + (20.0, 0.0, 0.0), DEFAULT_CONFIG, {})
        self.assertEqual(caught.exception.report["reason_code"], "geometry_low_overlap")

    def test_nonfinite_geometry_fails_with_serializable_report(self):
        scene = self.scene()
        bad = scene.copy()
        bad[0, 0] = np.nan
        with self.assertRaises(StaticDataError) as caught:
            _screen_geometry(scene, bad, DEFAULT_CONFIG, {})
        self.assertEqual(caught.exception.report["reason_code"], "invalid_geometry")
        json.dumps(caught.exception.report, allow_nan=False)


if __name__ == "__main__":
    unittest.main()
