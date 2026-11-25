import importlib.util
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    try:
        get_package_share_directory("livox_ros_driver2")
    except PackageNotFoundError as error:
        raise RuntimeError(
            "Point-LIO 兼容启动需要 livox_ros_driver2 的 ROS2 消息接口，"
            "请先安装并加载该包；无需加载整个北极熊工作区。"
        ) from error

    driver_share = Path(get_package_share_directory("mid360_agras_driver"))
    compat_share = Path(get_package_share_directory("mid360_agras_livox_compat"))
    spec = importlib.util.spec_from_file_location(
        "agras_compat_launch_common", driver_share / "launch" / "mid360_common.py"
    )
    common = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(common)

    def bridge(input_topic, output_topic):
        return Node(
            package="mid360_agras_livox_compat",
            executable="custom_msg_bridge",
            name="mid360_agras_livox_compat",
            output="screen",
            parameters=[{"input_topic": input_topic, "output_topic": output_topic}],
            on_exit=[Shutdown(reason="Livox 消息转换节点已退出")],
        )

    return common.build(
        4,
        False,
        default_params_file=compat_share / "config" / "point_lio_compat.yaml",
        bridge_factory=bridge,
    )
