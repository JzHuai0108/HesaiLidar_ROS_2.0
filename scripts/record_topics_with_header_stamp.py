#!/usr/bin/env python3
import argparse
import signal
import threading
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def _header_stamp_ns(msg):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None

    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _parse_topic_spec(topic_spec):
    if "=" not in topic_spec:
        raise ValueError(
            f"Invalid topic spec {topic_spec!r}; expected /topic=package/msg/Type"
        )

    topic_name, type_name = topic_spec.split("=", 1)
    topic_name = topic_name.strip()
    type_name = type_name.strip()
    if not topic_name or not type_name:
        raise ValueError(
            f"Invalid topic spec {topic_spec!r}; expected /topic=package/msg/Type"
        )

    return topic_name, type_name


class HeaderStampBagRecorder(Node):
    def __init__(self, output_path, topic_specs, storage_id):
        super().__init__("header_stamp_bag_recorder")
        self._writer = rosbag2_py.SequentialWriter()
        self._writer_closed = False
        self._topic_subscriptions = []
        self._message_counts = {}

        storage_options = rosbag2_py.StorageOptions(str(output_path), storage_id)
        converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")
        self._writer.open(storage_options, converter_options)

        for topic_id, topic_spec in enumerate(topic_specs):
            topic_name, type_name = _parse_topic_spec(topic_spec)
            msg_type = get_message(type_name)

            topic_metadata = rosbag2_py.TopicMetadata(
                id=topic_id,
                name=topic_name,
                type=type_name,
                serialization_format="cdr",
            )
            self._writer.create_topic(topic_metadata)
            self._message_counts[topic_name] = 0

            callback = self._make_callback(topic_name)
            subscription = self.create_subscription(msg_type, topic_name, callback, 100)
            self._topic_subscriptions.append(subscription)
            self.get_logger().info(
                f"recording {topic_name} [{type_name}] with header stamp bag time"
            )

    def _make_callback(self, topic_name):
        def callback(msg):
            timestamp_ns = _header_stamp_ns(msg)
            if timestamp_ns is None:
                timestamp_ns = self.get_clock().now().nanoseconds
                self.get_logger().warn(
                    f"{topic_name} has no header.stamp; using recorder clock",
                    throttle_duration_sec=5.0,
                )

            self._writer.write(topic_name, serialize_message(msg), timestamp_ns)
            self._message_counts[topic_name] += 1

        return callback

    def print_summary(self):
        for topic_name, count in self._message_counts.items():
            self.get_logger().info(f"wrote {count} messages on {topic_name}")

    def close_writer(self):
        if self._writer_closed:
            return

        self._writer.close()
        self._writer_closed = True


def main():
    parser = argparse.ArgumentParser(
        description="Record ROS 2 topics to a bag using each message header.stamp as bag time."
    )
    parser.add_argument("--output", required=True, type=Path, help="Output bag directory")
    parser.add_argument(
        "--storage-id",
        default="mcap",
        choices=("mcap", "sqlite3"),
        help="ROS 2 bag storage plugin",
    )
    parser.add_argument(
        "--topic",
        action="append",
        required=True,
        help="Topic spec, e.g. /lidar_points=sensor_msgs/msg/PointCloud2",
    )
    args = parser.parse_args()

    stop_requested = threading.Event()

    def request_stop(_signum, _frame):
        stop_requested.set()

    rclpy.init()
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    node = HeaderStampBagRecorder(args.output, args.topic, args.storage_id)

    try:
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.print_summary()
        node.close_writer()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
