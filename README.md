# Agras MID360 ROS 驱动

这个项目提供 DJI Agras 版 MID360 的 ROS2 驱动。主驱动直接接收并解析 Agras UDP 数据，独立于 Livox SDK2。ROS1 驱动和两个版本共用的协议源码仍在项目中。

- `mid360_agras_driver/` 是 ROS2 功能包，包含驱动、配置、launch 和 RViz。
- `mid360_agras_livox_compat/` 是可选的 ROS2 消息转换包。
- `common/` 是 ROS1 和 ROS2 共用的 UDP 和协议实现。
- `ros1/` 是保留的 ROS1 功能包。内有 `COLCON_IGNORE`，避免 ROS2 构建时出现同名包。

上游项目为 [xiaobai0818/mid360-agras-ros（Gitee）](https://gitee.com/xiaobai0818/mid360-agras-ros.git)。

原作者的 [中文 README](src/mid360-agras-ros/mid360_agras_driver/README.md) 和 [英文 README](src/mid360-agras-ros/mid360_agras_driver/README.en.md) 按原文保存在驱动包内。其中的目录和启动默认值对应原版本。

## 构建

### 只测试主驱动（不需要北极熊或 Livox 包）

使用 ROS2 Humble。在本仓库根目录（包含 `src/`）执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mid360_agras_driver
source install/setup.bash
```

这条命令只构建主驱动，不构建可选的 `mid360_agras_livox_compat`。主驱动可独立发布 PointCloud2、Agras 原生 CustomMsg 和 IMU，不需要加载北极熊工作区，也不依赖 `livox_ros_driver2` 或 Livox SDK2。

只测试主驱动时，不要直接执行不带包选择的 `colcon build`：它会同时构建转换包，未安装 `livox_ros_driver2` 时会报找不到 `livox_ros_driver2Config.cmake`。这表示缺少可选兼容依赖，不表示主驱动依赖北极熊。

主驱动的三个启动入口均默认使用原生配置 `MID360_agras.yaml`。只编译主驱动后，可以直接启动：

```bash
ros2 launch mid360_agras_driver msg_MID360_launch.py
```

该命令发布 Agras 原生 CustomMsg、PointCloud2 和 IMU。只需要标准点云和 IMU 时，将启动文件换成 `driver_MID360_launch.py`；需要 RViz 时换成 `rviz_MID360_launch.py`。

也可以把这个项目文件夹放进现有工作区，保持 `src/mid360-agras-ros/mid360_agras_driver/` 及同级目录的结构。在已有工作区根目录执行同样的构建命令。

### 按需安装 Livox 兼容支持（用于现有 Point-LIO 等程序）

只有下游程序要求 `livox_ros_driver2/msg/CustomMsg` 时，才需要构建转换包。字段布局相同的 Agras 原生 CustomMsg 仍是不同的 ROS2 消息类型，不能直接替代 Livox CustomMsg。

先单独安装并加载能提供 `livox_ros_driver2` ROS2 消息接口的工作区，或让 colcon 在当前工作区找到该包的可构建源码，然后执行：

```bash
colcon build --symlink-install --packages-up-to mid360_agras_livox_compat
source install/setup.bash
```

`livox_ros_driver2` 可以独立提供，不要求安装整个北极熊工程。兼容节点仅链接 ROS2 消息类型支持库；完整 Livox 包自身的构建仍须满足其依赖，不能据此认为它也无需 SDK。使用兼容输出时，启动 Agras 驱动和转换节点即可，不需要启动 Livox 雷达采集节点。转换包提供独立启动入口，一次启动主驱动和转换节点：

```bash
ros2 launch mid360_agras_livox_compat point_lio_compat_launch.py
```

兼容配置位于转换包的 `config/point_lio_compat.yaml`。原来使用 `ros2 launch mid360_agras_driver msg_MID360_launch.py` 接入 Point-LIO 的用户，请改用上述命令。基础模式的 `msg` 入口现在只发布 Agras 原生 CustomMsg；主驱动入口不会自动启动转换节点。

## 网口和端口

Agras 所在的主机有线网口设为 `192.168.1.20/24`，直连时网关留空。雷达默认是 `192.168.1.10`。点云 UDP `60001`、IMU UDP `60003`，控制口 UDP `60000`，设备发现 UDP `50000`。默认沿用雷达已有的数据目的地址。

## 启动

主驱动的三个入口都默认加载 `MID360_agras.yaml`，不需要 Livox 消息接口。ROS2 点云默认 **20 Hz**（约每 50 ms 发布一个非空帧），IMU 按实际收包转发。

| 启动文件 | 默认行为 |
| --- | --- |
| `driver_MID360_launch.py` | 无界面发布 PointCloud2 和 IMU |
| `rviz_MID360_launch.py` | 发布 PointCloud2 和 IMU，并打开 RViz；点云话题和固定坐标系跟随所选配置 |
| `msg_MID360_launch.py` | 从同一批点发布 Agras 原生 CustomMsg 和 PointCloud2，另发 IMU |

```bash
ros2 launch mid360_agras_driver driver_MID360_launch.py
ros2 launch mid360_agras_driver rviz_MID360_launch.py
ros2 launch mid360_agras_driver msg_MID360_launch.py
```

`xfer_format:=0` 发布 PointCloud2，`xfer_format:=1` 只发布 CustomMsg 点云，`xfer_format:=4` 发布两种格式。三种模式都发布 IMU。兼容入口仅支持 `xfer_format:=1` 或 `4`，并默认使用 `4`。`publish_freq:=10.0` 可临时覆盖 YAML 中的频率。`rviz:=false` 可以关闭图形界面。

| 数据 | 转换包 `point_lio_compat_launch.py` | 主驱动 `msg_MID360_launch.py` |
| --- | --- | --- |
| CustomMsg | `/livox/lidar` · `livox_ros_driver2/msg/CustomMsg` | `/agras/livox/lidar` · `mid360_agras_driver/msg/CustomMsg` |
| PointCloud2 | `/livox/lidar/pointcloud` | `/agras/livox/lidar/pointcloud` |
| IMU | `/livox/imu` | `/agras/livox/imu` |
| 坐标系 | `front_mid360` | `agras_mid360` |

兼容模式的源消息仍在 `/agras/livox/lidar`。转换节点复制完整帧头、`timebase` 和每点的 `offset_time`。IMU 的角速度单位为 rad/s，加速度单位为 g。直接运行已有 Point-LIO 时，其 `/livox/lidar` 和 `/livox/imu` 应只接收一台雷达的数据。

## 双雷达或自定义配置

普通 MID360 继续使用它自己的驱动和 `/livox/...` 时，Agras 的原生默认配置使用 `/agras/...` 话题。也可显式指定配置：

```bash
agras_share="$(ros2 pkg prefix --share mid360_agras_driver)"
ros2 launch mid360_agras_driver msg_MID360_launch.py \
  "params_file:=${agras_share}/config/MID360_agras.yaml"
```

使用自己的 YAML 时，保留 `/**` 下的 `ros__parameters`。主驱动使用 `custom_msg_type: agras`，转换包的兼容入口使用 `custom_msg_type: livox`。`topic_prefix` 给相对话题加前缀，`lidar_topic`、`pointcloud_topic`、`imu_topic` 和 `livox_compat_topic` 支持绝对话题名。两个驱动的接收地址、UDP 绑定、话题和坐标系要分别配置。把两台雷达的点云合用于定位还需要外参和时间同步。

## 无雷达自检

编译并加载主驱动后，可以在本仓库根目录执行回环 UDP 测试：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187 python3 \
  src/mid360-agras-ros/common/test/ros2_smoke_test.py
```

安装并加载兼容包及 Livox 消息依赖后，增加 `--compat` 可同时测试消息转换：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187 python3 \
  src/mid360-agras-ros/common/test/ros2_smoke_test.py --compat
```

测试使用 `127.0.0.1` / `127.0.0.2` 和临时 UDP 端口发送模拟点云、IMU，检查原生启动、消息类型、字段值及兼容转换。RViz 启动入口以 `rviz:=false` 测试；这些检查不包含真实雷达连接、控制应答、实测频率或 Point-LIO 定位效果。

## ROS1

使用 ROS Noetic。将 `src/mid360-agras-ros/ros1/mid360_agras_driver` 符号链接到独立 catkin 工作区的 `src/`，再运行 `catkin_make`。ROS1 的 launch 名和点云默认频率沿用原版本。
