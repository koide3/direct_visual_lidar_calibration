"""Small indexed files for configuration tests that do not inspect bag contents."""
from pathlib import Path


def write_minimal_bag(path, ros2=False):
    from rosbags.typesys import Stores, get_typestore
    from rosbags.rosbag1 import Writer as Ros1Writer
    from rosbags.rosbag2 import Writer as Ros2Writer
    path = Path(path)
    store = get_typestore(Stores.ROS2_HUMBLE if ros2 else Stores.ROS1_NOETIC)
    message_type = "std_msgs/msg/String"
    message = store.types[message_type]("configuration fixture")
    writer = Ros2Writer(path, version=9) if ros2 else Ros1Writer(path)
    with writer:
        connection = writer.add_connection("/fixture", message_type, typestore=store)
        serialize = store.serialize_cdr if ros2 else store.serialize_ros1
        writer.write(connection, 1_590_192_300_000_000_000, serialize(message, message_type))
    return path
