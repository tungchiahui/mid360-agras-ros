# Agras MID360 ROS 驱动

这个项目提供 DJI Agras 版 MID360 的 ROS2 驱动。主驱动直接接收并解析 Agras UDP 数据，独立于 Livox SDK2。ROS1 驱动和两个版本共用的协议源码仍在项目中。

- `mid360_agras_driver/` 是 ROS2 功能包，包含驱动、配置、launch 和 RViz。
- `mid360_agras_livox_compat/` 是可选的 ROS2 消息转换包。
- `common/` 是 ROS1 和 ROS2 共用的 UDP 和协议实现。
- `ros1/` 是保留的 ROS1 功能包。内有 `COLCON_IGNORE`，避免 ROS2 构建时出现同名包。

原作者的 [中文 README](mid360_agras_driver/README.md) 和 [英文 README](mid360_agras_driver/README.en.md) 按原文保存在驱动包内。其中的目录和启动默认值对应原版本。

## 构建

使用 ROS2 Humble。在本仓库根目录（包含 `src/`）执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mid360_agras_driver
source install/setup.bash
```

也可以把这个项目文件夹放进现有工作区，保持 `src/mid360-agras-ros/mid360_agras_driver/` 及同级目录的结构。在已有工作区根目录执行同样的构建命令。

要用默认的 Livox 兼容模式发布 CustomMsg，先加载能提供 `livox_ros_driver2` ROS2 消息接口的工作区，或让 colcon 在当前工作区找到该包的源码，然后构建：

```bash
colcon build --packages-up-to mid360_agras_livox_compat
source install/setup.bash
```

兼容节点仅链接 ROS2 消息类型支持库。使用兼容输出时，系统需要 Livox 的 ROS2 消息接口，启动时只需运行 Agras 驱动和转换节点。主驱动可单独编译，PointCloud2 和 Agras 原生 CustomMsg 均可独立使用。

## 网口和端口

Agras 所在的主机有线网口设为 `192.168.1.20/24`，直连时网关留空。雷达默认是 `192.168.1.10`。点云 UDP `60001`、IMU UDP `60003`，控制口 UDP `60000`，设备发现 UDP `50000`。默认沿用雷达已有的数据目的地址。

## 启动

三个入口都默认加载 `point_lio_compat.yaml`。ROS2 点云默认 **20 Hz**（约每 50 ms 发布一个非空帧），IMU 按实际收包转发。

| 启动文件 | 默认行为 |
| --- | --- |
| `driver_MID360_launch.py` | 无界面发布 PointCloud2 和 IMU |
| `rviz_MID360_launch.py` | 发布 PointCloud2 和 IMU，并打开 RViz；点云话题和固定坐标系跟随所选配置 |
| `msg_MID360_launch.py` | 从同一批点发布 CustomMsg 和 PointCloud2，另发 IMU；默认启用 Livox 消息转换 |

```bash
ros2 launch mid360_agras_driver driver_MID360_launch.py
ros2 launch mid360_agras_driver rviz_MID360_launch.py
ros2 launch mid360_agras_driver msg_MID360_launch.py
```

`xfer_format:=0` 发布 PointCloud2，`xfer_format:=1` 只发布 CustomMsg 点云，`xfer_format:=4` 发布两种格式。三种模式都发布 IMU。`publish_freq:=10.0` 可临时覆盖 YAML 中的频率。`rviz:=false` 可以关闭图形界面。

| `msg` 入口 | 兼容配置 `point_lio_compat.yaml` | 原生配置 `MID360_agras.yaml` |
| --- | --- | --- |
| CustomMsg | `/livox/lidar` · `livox_ros_driver2/msg/CustomMsg` | `/agras/livox/lidar` · `mid360_agras_driver/msg/CustomMsg` |
| PointCloud2 | `/livox/lidar/pointcloud` | `/agras/livox/lidar/pointcloud` |
| IMU | `/livox/imu` | `/agras/livox/imu` |
| 坐标系 | `front_mid360` | `agras_mid360` |

兼容模式的源消息仍在 `/agras/livox/lidar`。转换节点复制完整帧头、`timebase` 和每点的 `offset_time`。IMU 的角速度单位为 rad/s，加速度单位为 g。直接运行已有 Point-LIO 时，其 `/livox/lidar` 和 `/livox/imu` 应只接收一台雷达的数据。

## 双雷达或独立运行

普通 MID360 继续使用它自己的驱动和 `/livox/...` 时，对 Agras 显式选原生配置：

```bash
agras_share="$(ros2 pkg prefix --share mid360_agras_driver)"
ros2 launch mid360_agras_driver msg_MID360_launch.py \
  "params_file:=${agras_share}/config/MID360_agras.yaml"
```

使用自己的 YAML 时，保留 `/**` 下的 `ros__parameters`。其中 `custom_msg_type` 选 `agras` 或 `livox`，`topic_prefix` 给相对话题加前缀，`lidar_topic`、`pointcloud_topic`、`imu_topic` 和 `livox_compat_topic` 支持绝对话题名。两个驱动的接收地址、UDP 绑定、话题和坐标系要分别配置。把两台雷达的点云合用于定位还需要外参和时间同步。

## ROS1

使用 ROS Noetic。将 `src/mid360-agras-ros/ros1/mid360_agras_driver` 符号链接到独立 catkin 工作区的 `src/`，再运行 `catkin_make`。ROS1 的 launch 名和点云默认频率沿用原版本。
