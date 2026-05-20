#!/usr/bin/env python3
import argparse
from pathlib import Path

import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from sensor_msgs_py import point_cloud2


def _detect_storage_id(bag_path):
    if bag_path.is_file():
        if bag_path.suffix == ".mcap":
            return "mcap"
        if bag_path.suffix == ".db3":
            return "sqlite3"

    metadata_path = bag_path / "metadata.yaml"
    if metadata_path.is_file():
        with metadata_path.open("r", encoding="utf-8") as metadata_file:
            metadata = yaml.safe_load(metadata_file) or {}

        bag_info = metadata.get("rosbag2_bagfile_information", {})
        storage_id = bag_info.get("storage_identifier")
        if storage_id:
            return storage_id

    if bag_path.is_dir():
        if any(bag_path.glob("*.mcap")):
            return "mcap"
        if any(bag_path.glob("*.db3")):
            return "sqlite3"

    return "sqlite3"


def _reader_uri(bag_path):
    metadata_path = bag_path / "metadata.yaml" if bag_path.is_dir() else None
    if bag_path.is_dir() and metadata_path and metadata_path.is_file():
        return bag_path

    mcap_files = sorted(bag_path.glob("*.mcap")) if bag_path.is_dir() else []
    if len(mcap_files) == 1:
        return mcap_files[0]

    db3_files = sorted(bag_path.glob("*.db3")) if bag_path.is_dir() else []
    if len(db3_files) == 1:
        return db3_files[0]

    return bag_path


def _open_reader(bag_path, storage_id):
    storage_options = rosbag2_py.StorageOptions(uri=str(_reader_uri(bag_path)), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def _topic_types(reader):
    return {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }


def _header_stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _sample_indices(width, count):
    if width <= 0:
        return []
    if count <= 1:
        return [0]

    return sorted({round(i * (width - 1) / (count - 1)) for i in range(count)})


def _timestamp_field_names(msg):
    return {field.name for field in msg.fields}


def print_lidar_timestamp_samples(bag_path, topic, frame_count, point_count, storage_id):
    storage_id = storage_id or _detect_storage_id(bag_path)
    print(f"opening bag with storage_id={storage_id!r}: {bag_path}")

    reader = _open_reader(bag_path, storage_id)
    topics = _topic_types(reader)

    if topic not in topics:
        available_topics = ", ".join(sorted(topics))
        raise RuntimeError(f"Topic {topic!r} not found. Available topics: {available_topics}")

    msg_type = get_message(topics[topic])
    frames_seen = 0

    while reader.has_next() and frames_seen < frame_count:
        topic_name, data, bag_time_ns = reader.read_next()
        if topic_name != topic:
            continue

        msg = deserialize_message(data, msg_type)
        field_names = _timestamp_field_names(msg)
        if "timestamp" not in field_names:
            raise RuntimeError(f"Topic {topic!r} has no PointCloud2 field named 'timestamp'")

        indices = _sample_indices(msg.width * msg.height, point_count)
        points = point_cloud2.read_points(
            msg,
            field_names=("timestamp",),
            skip_nans=False,
            uvs=indices,
        )
        timestamps = [float(point["timestamp"]) for point in points]

        print(f"frame {frames_seen}")
        print(f"  bag_record_time: {bag_time_ns * 1e-9:.9f}")
        print(f"  header_stamp:    {_header_stamp_sec(msg):.9f}")
        print(f"  points:          {msg.width * msg.height}")
        for index, timestamp in zip(indices, timestamps):
            delta = timestamp - _header_stamp_sec(msg)
            print(f"  point[{index:>6}] timestamp: {timestamp:.9f}  delta_from_header: {delta:+.9f}")

        frames_seen += 1

    if frames_seen == 0:
        raise RuntimeError(f"No messages read from topic {topic!r}")


def main():
    parser = argparse.ArgumentParser(
        description="Print sample per-point timestamp fields from the first lidar frames in a ROS 2 bag."
    )
    parser.add_argument("bag_path", type=Path, help="Path to the ROS 2 bag directory")
    parser.add_argument("--topic", default="/lidar_points", help="PointCloud2 topic to inspect")
    parser.add_argument("--frames", type=int, default=5, help="Number of frames/messages to inspect")
    parser.add_argument("--points", type=int, default=5, help="Number of points to sample per frame")
    parser.add_argument(
        "--storage-id",
        choices=("mcap", "sqlite3"),
        default=None,
        help="Force a ROS 2 bag storage plugin. By default this is read from metadata.yaml.",
    )
    args = parser.parse_args()

    print_lidar_timestamp_samples(
        args.bag_path,
        args.topic,
        args.frames,
        args.points,
        args.storage_id,
    )


if __name__ == "__main__":
    main()
