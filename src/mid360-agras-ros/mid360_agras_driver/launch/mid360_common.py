from pathlib import Path

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DRIVER = "mid360_agras_driver"
BRIDGE = "mid360_agras_livox_compat"


def parse_bool(value):
    if value.lower() not in ("true", "false"):
        raise ValueError(f"需要 true 或 false，收到 {value!r}")
    return value.lower() == "true"


def read_params(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    try:
        params = data["/**"]["ros__parameters"]
    except (KeyError, TypeError) as error:
        raise ValueError(f"{path} 需要 /** → ros__parameters") from error
    if not isinstance(params, dict):
        raise ValueError(f"{path} 的 ros__parameters 必须是参数映射")
    return dict(params)


def resolve_topic(prefix, name):
    if name.startswith("/"):
        return name
    if prefix:
        return f"/{prefix}/{name}"
    return f"/{name}"


def build(default_xfer, default_rviz):
    share = Path(get_package_share_directory(DRIVER))
    native_config = share / "config" / "MID360_agras.yaml"
    overrides = {
        "lidar_ip": str,
        "host_ip": str,
        "cmd_port": int,
        "point_port": int,
        "imu_port": int,
        "fov_start": int,
        "fov_stop": int,
        "xfer_format": int,
        "publish_freq": float,
        "frame_id": str,
        "auto_sampling": parse_bool,
        "configure_dest": parse_bool,
        "publish_fill": parse_bool,
        "standby_on_exit": parse_bool,
        "max_range": float,
        "line_num": int,
    }

    def setup(context):
        def value(name):
            return LaunchConfiguration(name).perform(context)

        params = read_params(value("params_file"))
        for name, parse in overrides.items():
            argument = value(name)
            if argument != "":
                params[name] = parse(argument)

        xfer_format = params.setdefault("xfer_format", default_xfer)
        if type(xfer_format) is not int or xfer_format not in (0, 1, 4):
            raise ValueError("xfer_format 必须是 0（PointCloud2）、1（CustomMsg）或 4（双格式）")
        show_rviz = parse_bool(value("rviz"))
        if show_rviz and xfer_format == 1:
            raise ValueError("RViz 需要 PointCloud2，使用 xfer_format:=0 或 4")

        custom_type = params.pop("custom_msg_type", "agras")
        if custom_type not in ("agras", "livox"):
            raise ValueError("custom_msg_type 必须是 agras 或 livox")
        prefix = params.pop("topic_prefix", "").strip("/")
        raw_topic = params.get("lidar_topic", "livox/lidar")
        pointcloud_default = raw_topic + "/pointcloud" if xfer_format == 4 else raw_topic
        pointcloud_topic = resolve_topic(prefix, params.get("pointcloud_topic") or pointcloud_default)
        lidar_topic = resolve_topic(prefix, raw_topic)
        imu_topic = resolve_topic(prefix, params.get("imu_topic", "livox/imu"))
        compat_topic = resolve_topic(prefix, params.pop("livox_compat_topic", "/livox/lidar"))
        use_bridge = custom_type == "livox" and xfer_format in (1, 4)

        active_topics = [imu_topic]
        if xfer_format in (1, 4):
            active_topics.append(lidar_topic)
        if xfer_format in (0, 4):
            active_topics.append(pointcloud_topic)
        if use_bridge:
            active_topics.append(compat_topic)
            try:
                get_package_share_directory(BRIDGE)
                get_package_share_directory("livox_ros_driver2")
            except PackageNotFoundError as error:
                raise RuntimeError(
                    f"Livox 兼容模式需要 {BRIDGE} 和 livox_ros_driver2 的 ROS2 消息接口。"
                    f"独立运行可改用 params_file:={native_config}"
                ) from error
        if len(set(active_topics)) != len(active_topics):
            raise ValueError("点云、IMU 和兼容模式的输入与输出需要不同的话题名")

        params.update(
            lidar_topic=lidar_topic, pointcloud_topic=pointcloud_topic, imu_topic=imu_topic
        )
        actions = [
            Node(
                package=DRIVER,
                executable="mid360_agras_driver_node",
                name=DRIVER,
                output="screen",
                parameters=[params],
                on_exit=[Shutdown(reason="Agras 驱动已退出")],
            )
        ]
        if use_bridge:
            actions.append(
                Node(
                    package=BRIDGE,
                    executable="custom_msg_bridge",
                    name=BRIDGE,
                    output="screen",
                    parameters=[{"input_topic": lidar_topic, "output_topic": compat_topic}],
                    on_exit=[Shutdown(reason="Livox 消息转换节点已退出")],
                )
            )
        if show_rviz:
            actions.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    arguments=[
                        "-d", str(share / "rviz" / "mid360.rviz"),
                        "-f", params.get("frame_id", "livox_frame"),
                    ],
                    remappings=[("/livox/lidar", pointcloud_topic)],
                )
            )
        return actions

    arguments = [
        DeclareLaunchArgument(
            "params_file", default_value=str(share / "config" / "point_lio_compat.yaml"),
            description="Agras ROS2 参数配置文件",
        ),
        DeclareLaunchArgument("rviz", default_value="true" if default_rviz else "false"),
    ]
    arguments.extend(
        DeclareLaunchArgument(name, default_value="", description="留空时使用所选配置")
        for name in overrides
    )
    return LaunchDescription(arguments + [OpaqueFunction(function=setup)])
