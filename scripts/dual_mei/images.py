"""External images and bounded, header-time paired ROS1/ROS2 image selection."""
from collections import deque
from dataclasses import dataclass
import hashlib
from pathlib import Path

import cv2
import numpy as np

from .bag_readers import iter_messages
from static_bag_input import _stamp_ns

IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage",
               "sensor_msgs/Image", "sensor_msgs/CompressedImage"}
MAX_PENDING_BYTES = 128 * 1024 * 1024
MAX_PENDING_FRAMES = 256


@dataclass
class ImageFrame:
    image: np.ndarray
    encoded: bytes
    extension: str
    source: dict


def _compressed(payload):
    raw = bytes(payload)
    if raw.startswith(b"\xff\xd8\xff"):
        extension = ".jpg"
    elif raw.startswith(b"\x89PNG\r\n\x1a\n"):
        extension = ".png"
    else:
        raise ValueError("compressed image must contain a JPEG or PNG payload")
    image = cv2.imdecode(np.frombuffer(raw, np.uint8),
                         cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if image is None:
        raise ValueError("cannot decode JPEG/PNG image")
    return image, raw, extension


def read_external(path):
    path = Path(path)
    payload = path.read_bytes()
    image = cv2.imdecode(np.frombuffer(payload, np.uint8),
                         cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if image is None:
        raise ValueError(f"cannot decode external image: {path}")
    if payload.startswith(b"\xff\xd8\xff"):
        extension = ".jpg"
    elif payload.startswith(b"\x89PNG\r\n\x1a\n"):
        extension = ".png"
    else:
        ok, encoded = cv2.imencode(".png", image)
        if not ok:
            raise ValueError(f"cannot encode external image: {path}")
        payload, extension = encoded.tobytes(), ".png"
    return ImageFrame(image, payload, extension,
                      dict(kind="external", path=str(path), timestamp_ns=None))


def decode_message(message, msgtype):
    if msgtype not in IMAGE_TYPES:
        raise ValueError(f"unsupported image message type: {msgtype}")
    if msgtype.endswith("CompressedImage"):
        if "compresseddepth" in message.format.lower():
            raise ValueError("compressedDepth is not a camera color image")
        return _compressed(message.data)
    encodings = {"mono8": (1, None), "8UC1": (1, None),
                 "bgr8": (3, None), "rgb8": (3, cv2.COLOR_RGB2BGR),
                 "bgra8": (4, cv2.COLOR_BGRA2BGR), "rgba8": (4, cv2.COLOR_RGBA2BGR)}
    if message.encoding not in encodings:
        raise ValueError(f"unsupported Image encoding: {message.encoding}; use mono8/bgr8/rgb8/bgra8/rgba8")
    channels, conversion = encodings[message.encoding]
    height, width, step = int(message.height), int(message.width), int(message.step)
    payload = bytes(message.data)
    if min(height, width) <= 0 or step < width * channels or len(payload) != height * step:
        raise ValueError("Image dimensions, row step or payload length are invalid")
    # 8-bit channels are endian-independent. Honor row padding explicitly.
    rows = np.frombuffer(payload, np.uint8).reshape(height, step)
    image = rows[:, :width * channels].reshape(height, width, channels).copy()
    if channels == 1:
        image = cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
    elif conversion is not None:
        image = cv2.cvtColor(image, conversion)
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise ValueError("cannot encode Image as PNG")
    return image, encoded.tobytes(), ".png"


@dataclass
class Candidate:
    stamp: int
    record_stamp: int
    topic: str
    message: object
    msgtype: str

    @property
    def size(self):
        return len(self.message.data)


def select_pair(messages, topics, start_ns, end_ns, selection):
    """Choose minimum midpoint distance, then pair skew, then earlier stamps.

    A frame is retained only while it could match a future frame from the other
    stream. Explicit memory limits reject a stalled or severely skewed source.
    Only the chosen pair is decoded to pixels.
    """
    tolerance = round(selection["max_pair_delta_sec"] * 1_000_000_000)
    offset = selection.get("target_offset_sec")
    target = start_ns + ((end_ns - start_ns) // 2 if offset is None else round(offset * 1e9))
    if not start_ns <= target < end_ns:
        raise ValueError("image target is outside the static interval")
    topic_index = {topic: i for i, topic in enumerate(topics)}
    pending = [deque(), deque()]
    sizes = [0, 0]
    last = [None, None]
    ended = [False, False]
    counts = [0, 0]
    best, best_score = None, None
    last_record = None
    for topic, record, message, msgtype in messages:
        if topic not in topic_index:
            continue
        if last_record is not None and record < last_record:
            raise ValueError("image bag record timestamps move backwards")
        last_record = record
        i = topic_index[topic]
        if msgtype not in IMAGE_TYPES:
            raise ValueError(f"{topic}: expected Image or CompressedImage, got {msgtype}")
        stamp = _stamp_ns(message.header.stamp)
        if last[i] is not None and stamp <= last[i]:
            raise ValueError(f"{topic}: image header timestamps must strictly increase")
        last[i] = stamp
        for j in (0, 1):
            other_last = last[1 - j]
            while pending[j] and other_last is not None and pending[j][0].stamp < other_last - tolerance:
                sizes[j] -= pending[j].popleft().size
        if stamp >= end_ns:
            ended[i] = True
            if all(ended):
                break
            continue
        if stamp < start_ns:
            continue
        counts[i] += 1
        current = Candidate(stamp, record, topic, message, msgtype)
        for other in pending[1 - i]:
            if abs(other.stamp - stamp) <= tolerance:
                pair = (current, other) if i == 0 else (other, current)
                score = (abs(pair[0].stamp + pair[1].stamp - 2 * target),
                         abs(pair[0].stamp - pair[1].stamp), pair[0].stamp, pair[1].stamp)
                if best_score is None or score < best_score:
                    best, best_score = pair, score
        pending[i].append(current)
        sizes[i] += current.size
        if sum(sizes) > MAX_PENDING_BYTES or sum(map(len, pending)) > MAX_PENDING_FRAMES:
            raise ValueError("too many unpaired image bytes/frames; check topics, timestamp skew and pair tolerance")
    if best is None:
        raise ValueError(f"no image pair inside static interval within {tolerance} ns; per-camera frames={counts}")
    frames = {}
    for name, candidate in zip(("cam0", "cam1"), best):
        image, encoded, extension = decode_message(candidate.message, candidate.msgtype)
        frames[name] = ImageFrame(image, encoded, extension, dict(
            kind="bag", topic=candidate.topic, message_type=candidate.msgtype,
            timestamp_ns=candidate.stamp, record_timestamp_ns=candidate.record_stamp,
            frame_id=str(candidate.message.header.frame_id)))
    return frames, dict(target_timestamp_ns=target, pair_delta_ns=best_score[1],
                        frames_in_interval=dict(zip(("cam0", "cam1"), counts)))


def load_images(config, detection):
    if config.input["mode"] == "bag_and_images":
        frames = {name: read_external(config.input["images"][name]["image_path"])
                  for name in config.cameras}
        selection = dict(policy="external", timestamp_validation="not_available")
    else:
        topics = [config.input["images"][name]["topic"] for name in ("cam0", "cam1")]
        anchor = min(detection["first_imu_record_ns"], detection["first_lidar_record_ns"])
        record_end = anchor + round(config.static["max_read_record_sec"] * 1e9)
        iterator = iter_messages(config.bag, topics, config.static["backend"], record_end)
        try:
            frames, selection = select_pair(iterator, topics, detection["sensor_start_ns"],
                                             detection["sensor_end_ns_exclusive"], config.image_selection)
        finally:
            iterator.close()
        selection["policy"] = "nearest_pair_midpoint_to_target"
    for name, frame in frames.items():
        c = config.cameras[name]["camera"]
        if frame.image.shape[:2] != (c["height"], c["width"]):
            raise ValueError(f"{name}: decoded image size {frame.image.shape[:2]} differs from YAML {(c['height'], c['width'])}; no implicit resizing")
        frame.source["saved_image_sha256"] = hashlib.sha256(frame.encoded).hexdigest()
    return frames, selection
