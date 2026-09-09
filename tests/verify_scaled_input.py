#!/usr/bin/env python3
"""Positive 1920x1920 bag-input check using a disposable opening-interval fixture."""
from hashlib import sha256
import json
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from dual_mei.bag_readers import iter_messages
from dual_mei.pipeline import run
from verify_dual_mei_delivery import baseline_input

workspace = Path(__file__).resolve().parents[3]
source_config = baseline_input(workspace, "bag_only")
source_result = Path(source_config["output"]["directory"])
result = yaml.safe_load((source_result / "extrinsics.yaml").read_text())
scale_config = Path("/home/chenyu/ws_chenyu/recon/data/0908/insta_livox_bag_output/short_scale05_20260909_v2/mei_calibration.yaml")
intrinsics = yaml.safe_load(scale_config.read_text())["streams"]
with tempfile.TemporaryDirectory(prefix="dual_mei_scale_") as temp:
    temp = Path(temp)
    original_bag = Path(source_config["input"]["bag_path"])
    info = yaml.safe_load((original_bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    end = info["starting_time"]["nanoseconds_since_epoch"] + 1_300_000_000
    events = []
    types = {}
    for topic, stamp, message, msgtype in iter_messages(original_bag, ("/livox/lidar", "/livox/imu"), "rosbag2", end):
        types[topic] = msgtype
        events.append((stamp, topic, serialize_message(message)))
    for i, name in enumerate(("cam0", "cam1")):
        topic = source_config["input"]["images"][name]["topic"]
        image = cv2.imread(str(source_result / name / "image.jpg"))
        image = cv2.resize(image, (1920, 1920), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        assert ok
        stamp = result["cameras"][name]["source"]["timestamp_ns"]
        message = CompressedImage()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
        message.header.frame_id = name + "_optical_frame"
        message.format = "jpeg"
        message.data = encoded.tobytes()
        types[topic] = "sensor_msgs/msg/CompressedImage"
        events.append((stamp, topic, serialize_message(message)))
        scaled = intrinsics[i]["output_intrinsics"]
        camera = source_config["cameras"][name]
        for key in ("width", "height", "fx", "fy", "cx", "cy", "xi"):
            camera[key] = scaled[key]
        camera["distortion"] = {key: scaled[key] for key in ("k1", "k2", "k3", "p1", "p2")}
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(temp / "bag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    for topic, msgtype in types.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=msgtype, serialization_format="cdr"))
    for stamp, topic, raw in sorted(events, key=lambda row: (row[0], row[1])):
        writer.write(topic, raw, stamp)
    del writer
    source_config["input"]["bag_path"] = str(temp / "bag")
    source_config["output"]["directory"] = str(temp / "prepared")
    path = temp / "input.yaml"
    path.write_text(yaml.safe_dump(source_config))
    run(path, prepare_only=True)
    prepared = yaml.safe_load((temp / "prepared/prepared.yaml").read_text())
    assert prepared["static"]["sensor_start_ns"] == result["static"]["sensor_start_ns"]
    for name in ("cam0", "cam1"):
        item = prepared["cameras"][name]
        assert item["camera"]["width"] == item["camera"]["height"] == 1920
        assert cv2.imread(str(temp / "prepared" / item["image"])).shape == (1920,1920,3)
        assert item["source"]["timestamp_ns"] == result["cameras"][name]["source"]["timestamp_ns"]
        assert item["projections"]["before"]["input_points"] > 40000
    report = dict(status="passed", fixture="original opening LiDAR/IMU plus resized selected JPEG pair",
                  image_size=[1920,1920], intrinsics_source=str(scale_config),
                  temporary_fixture_removed_on_exit=True)
    (workspace.parent / ".task_artifacts/0909_extrinsics/scaled_validation.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
