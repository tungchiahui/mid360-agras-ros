import importlib.util
from ipaddress import IPv4Address
from math import isfinite
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE = "mid360_agras_livox_compat"
DRIVER_SHARE = Path(get_package_share_directory("mid360_agras_driver"))
COMPAT_SHARE = Path(get_package_share_directory(PACKAGE))
_spec = importlib.util.spec_from_file_location(
    "agras_compat_launch_common", DRIVER_SHARE / "launch" / "mid360_common.py"
)
common = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(common)

DEVICE_PARAMETERS = {
    "lidar_ip": str, "host_ip": str,
    "cmd_port": int, "point_port": int, "imu_port": int, "discover_port": int,
    "discover_sec": float, "fov_start": int, "fov_stop": int, "line_num": int, "max_range": float,
    "auto_sampling": bool, "configure_dest": bool, "publish_fill": bool, "standby_on_exit": bool,
}
ROS_PARAMETERS = {
    "xfer_format": int, "publish_freq": float, "multi_topic": int, "data_src": int,
    "output_data_type": int, "frame_id": str, "imu_frame_id": str, "timestamp_source": str,
}


def bridge_node(input_topic, output_topic, depth, namespace=None):
    try:
        get_package_share_directory("livox_ros_driver2")
    except PackageNotFoundError as error:
        raise RuntimeError("CustomMsg 需要安装并加载官方 livox_ros_driver2 的 ROS2 消息接口") from error
    return Node(
        package=PACKAGE,
        executable="custom_msg_bridge",
        name=PACKAGE,
        namespace=namespace,
        output="screen",
        parameters=[{"input_topic": input_topic, "output_topic": output_topic, "qos_depth": depth}],
        on_exit=[Shutdown(reason="Livox 消息转换节点已退出")],
    )


def read_device_params(path, example):
    try:
        config = common.read_params(path)
    except ValueError as error:
        raise ValueError(
            f"user_config_path 需要 Agras 设备 YAML（参照 {example}）。如使用 Livox SDK2 JSON，"
            "请将设备和主机地址迁移到 lidar_ip、host_ip，并使用 Agras UDP 端口。"
        ) from error
    native_ros_keys = {
        "custom_msg_type", "topic_prefix", "lidar_topic", "pointcloud_topic", "imu_topic",
        "publish_freq", "frame_id", "xfer_format",
    }
    unsupported = config.keys() - DEVICE_PARAMETERS.keys() - native_ros_keys
    if unsupported:
        raise ValueError(f"user_config_path 含有不支持的设备参数 {sorted(unsupported)}")
    return {name: config[name] for name in DEVICE_PARAMETERS if name in config}


def build(default_xfer, default_rviz):
    native_config = DRIVER_SHARE / "config" / "MID360_agras.yaml"
    profile = COMPAT_SHARE / "config" / "MID360_livox_compat.yaml"
    rviz_config = COMPAT_SHARE / "config" / "MID360_livox_compat.rviz"
    parameters = {**DEVICE_PARAMETERS, **ROS_PARAMETERS}

    def setup(context):
        def value(name):
            return LaunchConfiguration(name).perform(context)

        params = read_device_params(native_config, native_config)
        device_path = Path(value("user_config_path"))
        if device_path != native_config:
            params.update(read_device_params(device_path, native_config))
        ros_config = common.read_params(value("params_file"))
        unsupported = ros_config.keys() - ROS_PARAMETERS.keys()
        if unsupported:
            raise ValueError(f"params_file 含有不支持的 ROS 参数 {sorted(unsupported)}")
        params.update(ros_config)
        for name, kind in parameters.items():
            argument = value(name)
            if argument:
                parse = common.parse_bool if kind is bool else kind
                params[name] = parse(argument)
            if name not in params:
                continue
            if kind is float and type(params[name]) in (int, float):
                params[name] = float(params[name])
                if not isfinite(params[name]):
                    raise ValueError(f"{name} 必须是有限数值")
            elif type(params[name]) is not kind:
                raise ValueError(f"{name} 的参数类型无效")

        lidar_ip = IPv4Address(params["lidar_ip"])
        IPv4Address(params["host_ip"])
        for port in ("cmd_port", "point_port", "imu_port", "discover_port"):
            if not 1 <= params[port] <= 65535:
                raise ValueError(f"{port} 必须在 1 到 65535 之间")
        xfer = params.setdefault("xfer_format", default_xfer)
        multi_topic = params.get("multi_topic", 0)
        if xfer not in (0, 1) or multi_topic not in (0, 1):
            raise ValueError("Livox 兼容模式要求 xfer_format=0/1 和 multi_topic=0/1")
        if params.get("data_src", 0) != 0 or params.get("output_data_type", 0) != 0:
            raise ValueError("Livox 兼容模式仅支持实时 UDP 输入 data_src=0 和 ROS 发布 output_data_type=0")
        if params.get("timestamp_source", "host") not in ("host", "device"):
            raise ValueError("timestamp_source 必须是 host 或 device")
        if params["line_num"] != 4:
            raise ValueError("Livox 兼容模式的 line_num 必须为 4")
        show_rviz = common.parse_bool(value("rviz"))
        if show_rviz and xfer != 0:
            raise ValueError("RViz 需要 PointCloud2，使用 xfer_format:=0")

        suffix = "_" + str(lidar_ip).replace(".", "_") if multi_topic else ""
        point_topic = "livox/lidar" + suffix
        bridge_input = "mid360_agras/custom" + suffix
        params.update(
            livox_compatibility=True, lidar_topic=bridge_input if xfer == 1 else point_topic,
            pointcloud_topic=point_topic, imu_topic="livox/imu" + suffix,
        )
        namespace = value("namespace") or None
        nodes = [common.driver_node(params, namespace)]
        if xfer == 1:
            nodes.append(bridge_node(bridge_input, point_topic, 64 if multi_topic else 256, namespace))
        if show_rviz:
            nodes.append(common.rviz_node(rviz_config, params.get("frame_id", "livox_frame"), point_topic, namespace))
        return nodes

    arguments = [
        DeclareLaunchArgument("user_config_path", default_value=str(native_config),
                              description="Agras 设备及 UDP 设置 YAML"),
        DeclareLaunchArgument("params_file", default_value=str(profile), description="Livox 兼容 ROS 参数 YAML"),
        DeclareLaunchArgument("rviz", default_value="true" if default_rviz else "false"),
        DeclareLaunchArgument("namespace", default_value=""),
    ]
    arguments.extend(DeclareLaunchArgument(name, default_value="") for name in parameters)
    return LaunchDescription(arguments + [OpaqueFunction(function=setup)])
