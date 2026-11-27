# Agras MID360 ROS 驱动

这个项目提供 DJI Agras 版 MID360 的 ROS2 驱动。主驱动直接接收并解析 Agras UDP 数据，独立于 Livox SDK2。ROS1 驱动和两个版本共用的协议源码仍在项目中。

- `mid360_agras_driver/` 是 ROS2 功能包，包含驱动、配置、launch 和 RViz。
- `mid360_agras_livox_compat/` 提供可选的 Livox ROS 2 兼容模式。
- `common/` 是 ROS1 和 ROS2 共用的 UDP 和协议实现。
- `ros1/` 是保留的 ROS1 功能包。内有 `COLCON_IGNORE`，避免 ROS2 构建时出现同名包。

上游项目为 [xiaobai0818/mid360-agras-ros（Gitee）](https://gitee.com/xiaobai0818/mid360-agras-ros.git)。

原作者的 [中文 README](mid360_agras_driver/README.md) 和 [英文 README](mid360_agras_driver/README.en.md) 按原文保存在驱动包内。其中的目录和启动默认值对应原版本。

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


## 网口和端口

Agras 所在的主机有线网口设为 `192.168.1.20/24`，直连时网关留空。雷达默认是 `192.168.1.10`。点云 UDP `60001`、IMU UDP `60003`，控制口 UDP `60000`，设备发现 UDP `50000`。默认沿用雷达已有的数据目的地址。

## Agras 原生模式

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

`xfer_format:=0` 发布 PointCloud2，`xfer_format:=1` 只发布 Agras 原生 CustomMsg 点云，`xfer_format:=4` 发布两种格式。三种模式都发布 IMU。`publish_freq:=10.0` 可覆盖 YAML 中的频率，`rviz:=false` 可关闭图形界面。自定义原生配置可用 `params_file:=路径`。

## Livox ROS 2 Humble 兼容模式

这个模式以官方普通 MID360 的 ROS 2 接口为基准。格式、发布时间和 QoS 对照官方 [`livox_ros_driver2` 源码](https://github.com/Livox-SDK/livox_ros_driver2/tree/21445540f0d100dc86a7e6df312dd70bbdb4afdf)。

安装并加载提供 `livox_ros_driver2/msg/CustomMsg` 的工作区，或将官方包放进当前工作区后构建。Agras 兼容包链接官方消息类型支持，启动时只运行 Agras 驱动和所需的消息桥。

```bash
colcon build --symlink-install --packages-up-to mid360_agras_livox_compat
source install/setup.bash
ros2 launch mid360_agras_livox_compat msg_MID360_launch.py
ros2 launch mid360_agras_livox_compat rviz_MID360_launch.py
```

| 兼容入口 | 默认格式 | 点云话题 | IMU |
| --- | --- | --- | --- |
| `msg_MID360_launch.py` | `xfer_format=1`，官方 `livox_ros_driver2/msg/CustomMsg` | `/livox/lidar` | `/livox/imu` |
| `rviz_MID360_launch.py` | `xfer_format=0`，`sensor_msgs/msg/PointCloud2`，附 RViz | `/livox/lidar` | `/livox/imu` |

- `publish_freq` 默认 `20.0 Hz`，有限值限幅到 `0.5–100 Hz`。可传 `publish_freq:=10.0` 使用普通 MID360 的官方默认频率。有数据时，点云按来包组帧，IMU 独立转发。两种点云格式互斥。使用 `rviz:=false` 可关闭图形界面。
- 点云和 IMU 默认 Frame 均为 `livox_frame`。`frame_id` 只设置点云 Frame，`imu_frame_id` 独立设置 IMU Frame。调整 Frame 名称只修改消息头。
- 默认 QoS 为 Reliable、Volatile、Keep Last，深度 `256`。`multi_topic:=1` 使用设备 IP 后缀，例如 `livox/lidar_192_168_1_10`，深度变为 `64`。`namespace:=agras` 可给该实例的话题加 ROS 命名空间。
- `user_config_path` 读取 Agras 设备 YAML，默认 `MID360_agras.yaml`。用于设置 IP、UDP 端口、FOV、过滤和采样。`params_file` 读取 ROS 参数，默认 `MID360_livox_compat.yaml`。可显式传入 Launch 参数覆盖相应 YAML。设备 YAML 中旧的 ROS 输出字段由通用配置接管。
- `data_src=0` 代表实时 Agras UDP，`output_data_type=0` 代表 ROS 发布。普通 MID360 的 SDK2 JSON、控制端口、扫描模式和设备外参指令属于其硬件配置。Agras 使用自己的设备 YAML、设备协议和端口。非零点云外参需通过外部 TF 或点云变换处理。

### 时间戳和 IMU

`timestamp_source:=host` 为默认值。点云与 IMU 使用主机收包时的系统时间。包内点时间按官方驱动的 `time_interval × 100 / dot_num` 计算，单位纳秒。点云帧头使用首个发布点的时间。CustomMsg 的 `timebase` 为同一整数时间，`offset_time` 是相对首点的纳秒差。PointCloud2 的 `timestamp` 为绝对纳秒，以 `FLOAT64` 存储。主机收包时间含网络和调度延迟。

`timestamp_source:=device` 需要先在实体设备上验证。此模式等待点云和 IMU 都有连续、同类型的同步时间，检查可表示范围、相对主机的粗略时间域以及两路时间一致性。小范围倒退包会被丢弃并计数。时间失效或大幅跳变时暂停两路输出，恢复设备时钟后重启驱动。时间戳和组帧使用物理时钟，`use_sim_time` 无法替换设备或主机收包时间。

`diagnostics` 报告两路原始 `time_type`、时间戳、异常和队列丢弃、估算丢包及过滤统计。兼容 IMU 透传原始六轴值并拒发 NaN 和 Inf。官方普通 MID360 实际使用 `rad/s` 和 `g`，Agras 原始单位和轴向须由实体设备验证。IMU 的 `orientation_covariance[0]=-1` 表示无姿态估计。诊断会显示六轴重复样本计数及幅值峰值，硬件量程和饱和阈值有待实测。

Tag 原始字节透传，Agras Tag 位含义及双回波线号待真机验证。单回波 `line` 按包内顺序对 4 取模。兼容模式保留 Agras 的全零填充点过滤与 `80 m` 量程裁剪。可用 `publish_fill:=true max_range:=0.0` 查看未经这两种过滤的数据。

## 无雷达自检

编译并加载主驱动后，可以在本仓库根目录执行回环 UDP 测试：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187 python3 \
  src/mid360-agras-ros/common/test/ros2_smoke_test.py
```

安装并加载兼容包及 Livox 消息依赖后，增加 `--compat` 可同时验证通用 Livox 入口。

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187 python3 \
  src/mid360-agras-ros/common/test/ros2_smoke_test.py --compat
```

测试使用 `127.0.0.1` / `127.0.0.2` 和临时 UDP 端口发送模拟数据，检查话题、消息字段、QoS、发布时间、参数校验、设备时钟失效和异常 IMU。回环测试关闭 RViz。

## Humble 实机测试

2026-11-27 在一台 Agras MID360 上，以默认 `timestamp_source:=host` 和 `publish_freq=20.0` 分别测试两个通用 Livox 入口。

| 项目 | 实测结果 |
| --- | --- |
| PointCloud2 | 9 秒收到 180 帧，约 20 Hz。校验了字段偏移、26 字节点布局和逐点绝对时间 |
| Livox CustomMsg | 9 秒收到 180 帧，约 20 Hz。校验了官方消息类型、点数、`timebase` 和 `offset_time` |
| IMU | 原始 UDP 和 ROS 各测 25 秒，平均分别为 190.96 Hz 和 191.15 Hz，每秒 186–197 个样本。ROS 测量窗口内驱动收到 4,779 包，订阅端收到 4,779 条消息 |
| 设备时间 | 两路原始 `time_type=0`。8 秒采样内原始时间戳连续递增。选用 `timestamp_source:=device` 时，驱动报告未同步并暂停发布点云与 IMU |

这台设备实际从 `192.168.1.10` 发往 `192.168.1.20:60001/60003`。控制口在同次测试中读回 `192.168.1.5:56301/56401`，与抓到的 UDP 目的端口不同。实测使用默认设置 `configure_dest:=false`。

IMU 实测频率和波动来自收到的设备数据。`publish_freq` 只控制点云组帧。启动通用模式后，可分别运行以下命令观察发布频率与时间源状态。

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /diagnostics --once
```

## ROS1

使用 ROS Noetic。将 `src/mid360-agras-ros/ros1/mid360_agras_driver` 符号链接到独立 catkin 工作区的 `src/`，再运行 `catkin_make`。ROS1 的 launch 名和点云默认频率沿用原版本。
