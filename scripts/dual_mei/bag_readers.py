"""Validated ROS1/ROS2 readers with topic filters and bounded record-time I/O."""
from importlib import import_module
from pathlib import Path


class BagInputError(ValueError):
    """A format, dependency or bag-reading error suitable for a static report."""

    def __init__(self, message, reason_code="bag_read_error"):
        super().__init__(message)
        self.reason_code = reason_code


def read_ros2_metadata(path):
    import yaml

    path = Path(path)
    metadata = path / "metadata.yaml"
    if not metadata.is_file():
        raise BagInputError(f"ROS2 bag directory must contain metadata.yaml: {path}",
                            "bag_metadata_missing")
    try:
        document = yaml.safe_load(metadata.read_text(encoding="utf-8"))
        info = document["rosbag2_bagfile_information"]
        if not isinstance(info, dict):
            raise ValueError("rosbag2_bagfile_information must be a mapping")
        storage = info.get("storage_identifier")
        files = info.get("relative_file_paths")
        if not isinstance(storage, str) or not storage.strip():
            raise ValueError("storage_identifier must be nonempty")
        if not isinstance(files, list) or not files:
            raise ValueError("relative_file_paths must list bag storage files")
        for filename in files:
            if not isinstance(filename, str) or not filename or Path(filename).is_absolute():
                raise ValueError("relative_file_paths must contain relative file names")
            if not (path / filename).is_file():
                raise ValueError(f"bag storage file is missing: {filename}")
        return info
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
        raise BagInputError(f"invalid ROS2 metadata at {metadata}: {error}",
                            "bag_metadata_invalid") from error


def detect_bag_format(path):
    """Inspect actual file magic or ROS2 metadata, without scanning messages."""
    path = Path(path)
    if path.is_dir():
        read_ros2_metadata(path)
        return "ros2"
    if not path.is_file():
        raise BagInputError(f"ROS1 bag file or ROS2 bag directory not found: {path}",
                            "bag_not_found")
    try:
        with path.open("rb") as stream:
            magic = stream.read(13)
    except OSError as error:
        raise BagInputError(f"cannot read bag file {path}: {error}") from error
    if magic != b"#ROSBAG V2.0\n":
        raise BagInputError(f"invalid ROS1 bag file {path}: expected #ROSBAG V2.0 header; "
                            "a single ROS2 .db3/.mcap file is not a bag directory")
    return "ros1"


def _require_backend(backend, bag_format):
    modules = {
        "rosbag2": ("rosbag2_py", "rclpy.serialization", "rosidl_runtime_py.utilities"),
        "rosbags": (f"rosbags.rosbag{1 if bag_format == 'ros1' else 2}", "rosbags.typesys"),
        "rosbag": ("rosbag",),
    }
    try:
        for name in modules[backend]:
            import_module(name)
    except ImportError as error:
        hint = "install rosbags for offline ROS1/ROS2 input" if backend != "rosbag2" else "source the ROS2 environment or install rosbags"
        raise BagInputError(f"backend {backend} is unavailable (missing dependency: {error}); {hint}") from error


def choose_backend(path, backend="auto"):
    """Resolve compatible installed readers; automatic fallback is dependency-only."""
    if backend not in ("auto", "rosbag2", "rosbags", "rosbag"):
        raise BagInputError("backend must be auto, rosbag2, rosbags, or rosbag")
    bag_format = detect_bag_format(path)
    if (backend == "rosbag2" and bag_format == "ros1") or (backend == "rosbag" and bag_format == "ros2"):
        raise BagInputError(f"backend {backend} cannot read {bag_format.upper()} input: {path}; "
                            "rosbag2 reads ROS2 directories; rosbag reads ROS1 files",
                            "backend_format_mismatch")
    if backend != "auto":
        _require_backend(backend, bag_format)
        return backend
    candidates = ("rosbags", "rosbag") if bag_format == "ros1" else ("rosbag2", "rosbags")
    errors = []
    for candidate in candidates:
        try:
            _require_backend(candidate, bag_format)
            return candidate
        except BagInputError as error:
            errors.append(str(error))
    raise BagInputError(f"no available backend for {bag_format.upper()} input: " + "; ".join(errors))


def validate_bag(path, backend="auto"):
    """Validate configuration input, including a ROS1 index (never message payloads).

    Reader opens inspect the bag's recorded index, not a full message scan.
    Runtime readers perform their own open so changed inputs still fail normally.
    ROS2 configuration validation uses its metadata and listed storage files.
    """
    backend = choose_backend(path, backend)
    if Path(path).is_file():
        try:
            if backend == "rosbags":
                from rosbags.rosbag1 import Reader
                with Reader(Path(path)):
                    pass
            else:
                import rosbag
                with rosbag.Bag(str(path), "r"):
                    pass
        except Exception as error:
            raise BagInputError(f"invalid ROS1 bag {path}: {error}") from error
    return backend


def _check_topics(available, topics):
    missing = set(topics) - set(available)
    if missing:
        raise BagInputError(f"bag is missing configured topics: {sorted(missing)}")


def _rosbag2(path, topics, end_ns):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    info = read_ros2_metadata(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=info["storage_identifier"]),
                rosbag2_py.ConverterOptions("", ""))
    try:
        available = {t.name: t.type for t in reader.get_all_topics_and_types()}
        _check_topics(available, topics)
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
        classes = {t: get_message(available[t]) for t in topics}
        while reader.has_next():
            topic, raw, stamp = reader.read_next()
            if end_ns is not None and stamp > end_ns:
                break
            yield topic, int(stamp), deserialize_message(raw, classes[topic]), available[topic]
    finally:
        # Humble's reader has no close(); destruction releases storage handles.
        del reader


def _register_ros1_types(store, connections):
    """Validate all selected definitions before decoding, including nested types."""
    from rosbags.interfaces import Nodetype
    from rosbags.typesys import get_types_from_msg

    definitions = {}
    for connection in connections:
        definition = getattr(connection, "msgdef", None)
        definition = getattr(definition, "data", definition)
        if not definition:
            continue
        try:
            parsed = get_types_from_msg(definition, connection.msgtype)
        except Exception as error:
            raise BagInputError(f"invalid ROS1 message definition for {connection.msgtype} "
                                f"on {connection.topic}: {error}") from error
        for name, fields in parsed.items():
            existing = definitions.get(name, store.fielddefs.get(name))
            if existing is not None and existing != fields:
                raise BagInputError(f"conflicting ROS1 message definition for {name} on {connection.topic}")
            definitions[name] = fields
    try:
        store.register(definitions)
    except Exception as error:
        raise BagInputError(f"cannot register ROS1 message definitions: {error}") from error
    for connection in connections:
        # Diagnose missing dependencies directly: rosbags versions may raise
        # different exception classes when generating an unknown nested type.
        pending, checked = [connection.msgtype], set()
        while pending:
            name = pending.pop()
            if name in checked:
                continue
            if name not in store.fielddefs:
                raise BagInputError(f"missing ROS1 message definition {name} required by "
                                    f"{connection.msgtype} on {connection.topic}")
            checked.add(name)
            for _, descriptor in store.fielddefs[name][1]:
                while descriptor[0] in (Nodetype.ARRAY, Nodetype.SEQUENCE):
                    descriptor = descriptor[1][0]
                if descriptor[0] == Nodetype.NAME:
                    pending.append(descriptor[1])
        try:
            # Both generation steps traverse nested types before any data is yielded.
            _, digest = store.generate_msgdef(connection.msgtype, ros_version=1)
            store.get_msgdef(connection.msgtype)
        except KeyError as error:
            raise BagInputError(f"missing ROS1 message definition {error} required by "
                                f"{connection.msgtype} on {connection.topic}") from error
        except Exception as error:
            raise BagInputError(f"invalid ROS1 message definition for {connection.msgtype} "
                                f"on {connection.topic}: {error}") from error
        if connection.digest and connection.digest != digest:
            raise BagInputError(f"ROS1 message definition MD5 mismatch for {connection.msgtype} "
                                f"on {connection.topic}: bag={connection.digest}, definition={digest}")


def _rosbags(path, topics, end_ns):
    from rosbags.typesys import Stores, get_typestore
    is_ros2 = detect_bag_format(path) == "ros2"
    if is_ros2:
        from rosbags.rosbag2 import Reader
    else:
        from rosbags.rosbag1 import Reader
    with Reader(Path(path)) as reader:
        _check_topics({c.topic for c in reader.connections}, topics)
        connections = [c for c in reader.connections if c.topic in topics]
        store = get_typestore(Stores.ROS2_HUMBLE if is_ros2 else Stores.ROS1_NOETIC)
        if not is_ros2:
            _register_ros1_types(store, connections)
        deserialize = store.deserialize_cdr if is_ros2 else store.deserialize_ros1
        # rosbags stop is exclusive; this interface has always included end_ns.
        stop = None if end_ns is None else int(end_ns) + 1
        for c, stamp, raw in reader.messages(connections=connections, stop=stop):
            yield c.topic, int(stamp), deserialize(raw, c.msgtype), c.msgtype


def _rosbag(path, topics, end_ns):
    import rosbag
    with rosbag.Bag(str(path), "r") as bag:
        _check_topics(bag.get_type_and_topic_info().topics, topics)
        for topic, message, stamp in bag.read_messages(topics=list(topics)):
            if end_ns is not None and stamp.to_nsec() > end_ns:
                break
            yield topic, int(stamp.to_nsec()), message, message._type


def iter_messages(path, topics, backend="auto", record_end_ns=None):
    backend = choose_backend(path, backend)
    readers = {"rosbag2": _rosbag2, "rosbags": _rosbags, "rosbag": _rosbag}
    try:
        yield from readers[backend](path, tuple(topics), record_end_ns)
    except BagInputError:
        raise
    except Exception as error:
        raise BagInputError(f"cannot read bag {path} with backend {backend}: {error}") from error
