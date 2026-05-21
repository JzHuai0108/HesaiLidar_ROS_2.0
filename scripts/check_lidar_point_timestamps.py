#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2


POINT_FIELD_DATATYPES = {
    PointField.INT8: "INT8",
    PointField.UINT8: "UINT8",
    PointField.INT16: "INT16",
    PointField.UINT16: "UINT16",
    PointField.INT32: "INT32",
    PointField.UINT32: "UINT32",
    PointField.FLOAT32: "FLOAT32",
    PointField.FLOAT64: "FLOAT64",
}
DUAL_RETURN_PAIR_OFFSETS = (1, 32)
DUAL_RETURN_DIRECTION_DELTA_MAX = 1e-4
DUAL_RETURN_TIMESTAMP_DELTA_MAX = 1e-6
DUAL_RETURN_CHECK_PAIR_LIMIT = 5000


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


def _format_point_field(field):
    datatype = POINT_FIELD_DATATYPES.get(field.datatype, f"UNKNOWN({field.datatype})")
    return f"{field.name}(offset={field.offset}, datatype={datatype}, count={field.count})"


def _point_float(point, field_name):
    return float(point[field_name])


def _point_norm(point):
    x = _point_float(point, "x")
    y = _point_float(point, "y")
    z = _point_float(point, "z")
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None

    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 1e-6:
        return None

    return x, y, z, norm


def _direction_delta(left, right):
    left_norm = _point_norm(left)
    right_norm = _point_norm(right)
    if left_norm is None or right_norm is None:
        return None

    left_x, left_y, left_z, left_distance = left_norm
    right_x, right_y, right_z, right_distance = right_norm

    dx = left_x / left_distance - right_x / right_distance
    dy = left_y / left_distance - right_y / right_distance
    dz = left_z / left_distance - right_z / right_distance
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _same_ring(left, right, has_ring):
    if not has_ring:
        return True

    return int(left["ring"]) == int(right["ring"])


def _same_timestamp(left, right):
    return abs(_point_float(left, "timestamp") - _point_float(right, "timestamp")) <= DUAL_RETURN_TIMESTAMP_DELTA_MAX


def _candidate_pair_indices(point_count, offset):
    max_start = point_count - offset
    if max_start <= 0:
        return []

    indices = [index for index in range(max_start) if index % (offset * 2) < offset]
    if len(indices) <= DUAL_RETURN_CHECK_PAIR_LIMIT:
        return indices

    stride = math.ceil(len(indices) / DUAL_RETURN_CHECK_PAIR_LIMIT)
    return indices[::stride]


def _score_dual_return_offset(points, offset, has_ring):
    candidate_indices = _candidate_pair_indices(len(points), offset)
    stats = {
        "offset": offset,
        "checked": len(candidate_indices),
        "valid": 0,
        "same_ring": 0,
        "same_timestamp": 0,
        "same_ray": 0,
        "examples": [],
    }

    for index in candidate_indices:
        left = points[index]
        right = points[index + offset]
        direction_delta = _direction_delta(left, right)
        if direction_delta is None:
            continue

        stats["valid"] += 1
        same_ring = _same_ring(left, right, has_ring)
        same_timestamp = _same_timestamp(left, right)
        if same_ring:
            stats["same_ring"] += 1
        if same_timestamp:
            stats["same_timestamp"] += 1
        if same_ring and same_timestamp and direction_delta <= DUAL_RETURN_DIRECTION_DELTA_MAX:
            stats["same_ray"] += 1
            if len(stats["examples"]) < 3:
                stats["examples"].append((index, index + offset, direction_delta))

    stats["score"] = stats["same_ray"] / stats["valid"] if stats["valid"] else 0.0
    return stats


def _dual_return_layout(points, has_ring):
    stats = [
        _score_dual_return_offset(points, offset, has_ring)
        for offset in DUAL_RETURN_PAIR_OFFSETS
    ]
    best = max(stats, key=lambda item: item["score"])
    if best["same_ray"] < 8 or best["score"] < 0.60:
        layout = "not detected or inconclusive"
    elif best["offset"] == 1:
        layout = "adjacent"
    else:
        layout = f"separated_by_{best['offset']}"

    return layout, stats


def _print_dual_return_layout(points, has_ring):
    layout, stats = _dual_return_layout(points, has_ring)
    print(f"  dual_return_layout_check: {layout}")
    print(
        "    heuristic: paired returns should share ring, timestamp, and direction; "
        f"tested offsets {', '.join(str(offset) for offset in DUAL_RETURN_PAIR_OFFSETS)}"
    )
    for item in stats:
        print(
            f"    offset {item['offset']:>2}: "
            f"score={item['score']:.3f}  "
            f"same_ray={item['same_ray']}/{item['valid']} valid  "
            f"same_ring={item['same_ring']}  "
            f"same_timestamp={item['same_timestamp']}"
        )
        for left_index, right_index, direction_delta in item["examples"]:
            print(
                f"      example pair point[{left_index}] <-> point[{right_index}] "
                f"direction_delta={direction_delta:.3e}"
            )


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
        sample_field_names = ("x", "y", "z", "intensity", "timestamp")
        missing_field_names = [name for name in sample_field_names if name not in field_names]
        if missing_field_names:
            available_field_names = ", ".join(sorted(field_names))
            missing = ", ".join(missing_field_names)
            raise RuntimeError(
                f"Topic {topic!r} is missing PointCloud2 fields: {missing}. "
                f"Available fields: {available_field_names}"
            )

        point_total = msg.width * msg.height
        indices = _sample_indices(point_total, point_count)
        has_ring = "ring" in field_names
        read_field_names = sample_field_names + (("ring",) if has_ring else ())
        all_points = list(point_cloud2.read_points(
            msg,
            field_names=read_field_names,
            skip_nans=False,
        ))
        sampled_points = [all_points[index] for index in indices]

        print(f"frame {frames_seen}")
        print(f"  bag_record_time: {bag_time_ns * 1e-9:.9f}")
        print(f"  header_stamp:    {_header_stamp_sec(msg):.9f}")
        print(f"  points:          {point_total}")
        print("  fields:")
        for field in msg.fields:
            print(f"    {_format_point_field(field)}")
        _print_dual_return_layout(all_points, has_ring)

        for index, point in zip(indices, sampled_points):
            timestamp = float(point["timestamp"])
            delta = timestamp - _header_stamp_sec(msg)
            print(
                f"  point[{index:>6}] "
                f"x: {float(point['x']): .6f}  "
                f"y: {float(point['y']): .6f}  "
                f"z: {float(point['z']): .6f}  "
                f"intensity: {float(point['intensity']): .6f}  "
                f"timestamp: {timestamp:.9f}  "
                f"delta_from_header: {delta:+.9f}"
            )

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
