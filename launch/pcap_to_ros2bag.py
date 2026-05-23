import os
import shlex
import tempfile
from datetime import datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _expand_path(path):
    return os.path.abspath(os.path.expandvars(os.path.expanduser(path)))


def _require_existing_file(path, label):
    if not path:
        raise RuntimeError(f"{label} is required")

    expanded_path = _expand_path(path)
    if not os.path.isfile(expanded_path):
        raise RuntimeError(f"{label} does not exist: {expanded_path}")

    return expanded_path


def _update_lidar_config(
    config,
    pcap_path,
    correction_file_path,
    firetimes_path,
    default_frame_frequency,
):
    lidars = config.get("lidar")
    if not isinstance(lidars, list):
        raise RuntimeError("Expected 'lidar' to be a list in the Hesai config")

    for lidar in lidars:
        driver = lidar.setdefault("driver", {})
        pcap_type = driver.setdefault("pcap_type", {})

        driver["source_type"] = 2
        pcap_type["pcap_path"] = pcap_path
        pcap_type["correction_file_path"] = correction_file_path
        pcap_type["firetimes_path"] = firetimes_path
        pcap_type["pcap_play_in_loop"] = False
        if default_frame_frequency is not None:
            driver["default_frame_frequency"] = default_frame_frequency


def _effective_config_value(config, key):
    lidars = config.get("lidar", [])
    if not lidars:
        return ""

    driver = lidars[0].get("driver", {})
    pcap_type = driver.get("pcap_type", {})
    return pcap_type.get(key, "")


def _ros_config(config):
    lidars = config.get("lidar", [])
    if not lidars:
        return {}

    return lidars[0].get("ros", {})


def _topic_type_map(config):
    ros_config = _ros_config(config)
    mapping = {
        ros_config.get("ros_send_point_cloud_topic", "/lidar_points"): "sensor_msgs/msg/PointCloud2",
        ros_config.get("ros_send_imu_topic", "/lidar_imu"): "sensor_msgs/msg/Imu",
        ros_config.get("ros_send_packet_topic", "/lidar_packets"): "hesai_ros_driver/msg/UdpFrame",
        ros_config.get("ros_send_every_packet_topic", ""): "hesai_ros_driver/msg/UdpPacket",
        ros_config.get("ros_send_packet_loss_topic", "/lidar_packets_loss"): "hesai_ros_driver/msg/LossPacket",
        ros_config.get("ros_send_ptp_topic", ""): "hesai_ros_driver/msg/Ptp",
        ros_config.get("ros_send_correction_topic", ""): "std_msgs/msg/UInt8MultiArray",
    }
    return {topic: topic_type for topic, topic_type in mapping.items() if topic}


def _topic_specs(config, record_topics):
    topic_type_map = _topic_type_map(config)
    topic_specs = []

    for topic in record_topics:
        topic_type = topic_type_map.get(topic)
        if topic_type is None:
            known_topics = ", ".join(sorted(topic_type_map))
            raise RuntimeError(
                f"Do not know message type for record topic {topic!r}. "
                f"Known topics from config: {known_topics}"
            )
        topic_specs.append(f"{topic}={topic_type}")

    return topic_specs


def _recorder_script_path(share_dir):
    script_path = os.path.join(
        share_dir, "scripts", "record_topics_with_header_stamp.py"
    )
    if os.path.isfile(script_path):
        return script_path

    source_script_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "scripts",
            "record_topics_with_header_stamp.py",
        )
    )
    if os.path.isfile(source_script_path):
        return source_script_path

    raise RuntimeError("Cannot find scripts/record_topics_with_header_stamp.py")


def _make_runtime_actions(context):
    share_dir = get_package_share_directory("hesai_ros_driver")

    base_config_path = _expand_path(LaunchConfiguration("base_config_path").perform(context))
    pcap_path_arg = LaunchConfiguration("pcap_path").perform(context).strip()
    correction_file_path_arg = LaunchConfiguration("correction_file_path").perform(context).strip()
    firetimes_path_arg = LaunchConfiguration("firetimes_path").perform(context).strip()
    output_bag_path_arg = LaunchConfiguration("output_bag_path").perform(context).strip()
    record_topics_arg = LaunchConfiguration("record_topics").perform(context).strip()
    record_delay_arg = LaunchConfiguration("record_delay").perform(context).strip()
    default_frame_frequency_arg = (
        LaunchConfiguration("default_frame_frequency").perform(context).strip()
    )

    if not os.path.isfile(base_config_path):
        raise RuntimeError(f"base_config_path does not exist: {base_config_path}")

    with open(base_config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    if config is None:
        raise RuntimeError(f"base_config_path is empty: {base_config_path}")

    pcap_path = pcap_path_arg or _effective_config_value(config, "pcap_path")
    correction_file_path = correction_file_path_arg or _effective_config_value(
        config, "correction_file_path"
    )
    firetimes_path = firetimes_path_arg or _effective_config_value(config, "firetimes_path")

    pcap_path = _require_existing_file(pcap_path, "pcap_path")
    correction_file_path = _require_existing_file(
        correction_file_path, "correction_file_path"
    )
    if firetimes_path:
        firetimes_path = _require_existing_file(firetimes_path, "firetimes_path")

    if default_frame_frequency_arg:
        try:
            default_frame_frequency = float(default_frame_frequency_arg)
        except ValueError as exc:
            raise RuntimeError(
                f"default_frame_frequency must be a number: {default_frame_frequency_arg}"
            ) from exc
    else:
        default_frame_frequency = None

    _update_lidar_config(
        config,
        pcap_path,
        correction_file_path,
        firetimes_path,
        default_frame_frequency,
    )

    with tempfile.NamedTemporaryFile(
        mode="w",
        prefix="hesai_pcap_to_bag_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    ) as runtime_config_file:
        yaml.safe_dump(config, runtime_config_file, sort_keys=False)
        runtime_config_path = runtime_config_file.name

    if output_bag_path_arg:
        output_bag_path = _expand_path(output_bag_path_arg)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_bag_path = os.path.join(
            tempfile.gettempdir(), f"hesai_lidar_{timestamp}"
        )

    output_bag_parent = os.path.dirname(output_bag_path)
    if output_bag_parent:
        os.makedirs(output_bag_parent, exist_ok=True)
    if os.path.exists(output_bag_path):
        raise RuntimeError(f"output_bag_path already exists: {output_bag_path}")

    record_topics = shlex.split(record_topics_arg.replace(",", " "))
    if not record_topics:
        raise RuntimeError("record_topics must contain at least one ROS topic")
    topic_specs = _topic_specs(config, record_topics)

    try:
        record_delay = float(record_delay_arg)
    except ValueError as exc:
        raise RuntimeError(f"record_delay must be a number: {record_delay_arg}") from exc

    rviz_config = os.path.join(share_dir, "rviz", "rviz2.rviz")
    recorder_script = _recorder_script_path(share_dir)
    rosbag_record = ExecuteProcess(
        cmd=[
            "python3",
            recorder_script,
            "--output",
            output_bag_path,
            "--storage-id",
            "mcap",
        ] + [arg for topic_spec in topic_specs for arg in ("--topic", topic_spec)],
        output="screen",
        sigterm_timeout="5",
        sigkill_timeout="5",
    )

    driver_node = Node(
        namespace="hesai_ros_driver",
        package="hesai_ros_driver",
        executable="hesai_ros_driver_node",
        name="hesai_ros_driver_node",
        output="screen",
        parameters=[{"config_path": runtime_config_path}],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        namespace="rviz2",
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
    )

    shutdown_when_driver_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=driver_node,
            on_exit=[EmitEvent(event=Shutdown(reason="Hesai PCAP playback finished"))],
        )
    )

    return [
        LogInfo(msg=f"Recording ROS 2 bag to: {output_bag_path}"),
        LogInfo(msg=f"Using runtime Hesai config: {runtime_config_path}"),
        rosbag_record,
        TimerAction(period=record_delay, actions=[driver_node]),
        shutdown_when_driver_exits,
        rviz_node,
    ]


def generate_launch_description():
    share_dir = get_package_share_directory("hesai_ros_driver")
    default_config_path = str(Path(share_dir) / "config" / "config.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pcap_path",
                default_value="",
                description="Input PCAP file. Empty means use base_config_path.",
            ),
            DeclareLaunchArgument(
                "correction_file_path",
                default_value="",
                description="Hesai correction/calibration file. Empty means use base_config_path.",
            ),
            DeclareLaunchArgument(
                "firetimes_path",
                default_value="",
                description="Hesai firetimes file. Empty means use base_config_path.",
            ),
            DeclareLaunchArgument(
                "output_bag_path",
                default_value="",
                description="Output ROS 2 bag directory. Empty creates a timestamped bag in /tmp.",
            ),
            DeclareLaunchArgument(
                "record_topics",
                default_value="/lidar_points",
                description="Whitespace- or comma-separated topics for ros2 bag record.",
            ),
            DeclareLaunchArgument(
                "record_delay",
                default_value="2.0",
                description="Seconds to wait after starting ros2 bag record before launching the driver.",
            ),
            DeclareLaunchArgument(
                "default_frame_frequency",
                default_value="",
                description="Override driver.default_frame_frequency in the runtime config. Empty means use base_config_path.",
            ),
            DeclareLaunchArgument(
                "base_config_path",
                default_value=default_config_path,
                description="Base Hesai YAML config used to build a runtime PCAP config.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Whether to launch RViz while converting the PCAP.",
            ),
            OpaqueFunction(function=_make_runtime_actions),
        ]
    )
