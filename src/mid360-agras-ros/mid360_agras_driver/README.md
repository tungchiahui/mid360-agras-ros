# MID360 Agras ROS 驱动

DJI Agras 版 MID360 的 ROS1 / ROS2 驱动。协议按 [mid360-agras-viewer](https://gitee.com/xiaobai0818/mid360-agras-viewer) 在固件 `26.1.1.46` 上用过的 Agras 控制帧实现，不链接 Livox SDK2。零售 MID360 的 `0xAA` 协议和 `56100` 端口这台雷达不应答。

```
common/                         协议、UDP、视场和采样，ROS 无关
ros1/mid360_agras_driver/       catkin 包
ros2/mid360_agras_driver/       ament 包
```

两个包名字相同。catkin 工作空间只放 ROS1 包，colcon 工作空间只放 ROS2 包，不要把整个仓库直接丢进 `src`。

## 网络

本机有线网卡设为 `192.168.1.20/24`。雷达默认 `192.168.1.10`。

| UDP 端口 | 用途 |
| --- | --- |
| `50000` | 设备发现 |
| `60000` | 控制 |
| `60001` | 点云 |
| `60003` | IMU |

不要同时开着上位机或另一个驱动，这些端口只能有一个接收程序。驱动不会发送提高转速的 `32/0x1B`。

## 话题

和 `livox_ros_driver2` 的 MID360 使用同一组话题名。

| 话题 | 类型 |
| --- | --- |
| `/livox/lidar` | `xfer_format=0` 时为 `sensor_msgs/PointCloud2`；`xfer_format=1` 时为 `mid360_agras_driver/CustomMsg` |
| `/livox/imu` | `sensor_msgs/Imu` |

`PointCloud2` 的字段和官方驱动一致：`x y z intensity tag line timestamp`，每个点 26 字节。坐标单位是米。`line` 为点序号对 4 取余，对应官方驱动里的 `kLineNumberMid360`。

`CustomMsg` / `CustomPoint` 的字段布局和 `livox_ros_driver2/CustomMsg` 相同。FAST-LIO 如果写死了 `livox_ros_driver2/CustomMsg`，把依赖和头文件换成 `mid360_agras_driver/CustomMsg` 即可，点字段不用改。

IMU 角速度单位是 rad/s，加速度单位是 g，与 `livox_ros_driver2` 相同。FAST-LIO 会再乘重力加速度。

默认每 0.1 秒发布一帧点云，IMU 按到达转发，大约 200 Hz。

## 视场

`fov_start` 和 `fov_stop` 是航向角，单位度。默认 `0` 和 `360`，表示全周：只把 ROI 使能 `0x00010023` 写成 0。

其它窗口会先关掉使能，写入窗口 0 的起止，再把使能写成 1。结束角小于起始角、且结束角不是 0 时，窗口跨过 0°，使能写成 3。俯仰沿用雷达里原来的窗口，不在参数里改。

发布前还会按同一角度再裁一次。设备写入失败时，话题里仍然只有设定角度内的点。

```bash
roslaunch mid360_agras_driver rviz_MID360.launch fov_start:=30 fov_stop:=150
roslaunch mid360_agras_driver rviz_MID360.launch fov_start:=300 fov_stop:=40
```

## ROS1

当前机器可直接编 Noetic。把本仓库放进 catkin 工作空间的 `src`，包在子目录里，catkin 会找到它。

```bash
mkdir -p ~/agras_ws/src
ln -s /home/xiaobai/mid360-agras-ros/ros1/mid360_agras_driver ~/agras_ws/src/
cd ~/agras_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch mid360_agras_driver rviz_MID360.launch
```

`rviz_MID360.launch` 用 `PointCloud2` 并打开 RViz，固定坐标系是 `livox_frame`。`msg_MID360.launch` 用 `CustomMsg`，不打开 RViz。

## ROS2

```bash
mkdir -p ~/agras_ws2/src
ln -s /home/xiaobai/mid360-agras-ros/ros2/mid360_agras_driver ~/agras_ws2/src/
cd ~/agras_ws2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch mid360_agras_driver rviz_MID360_launch.py
ros2 launch mid360_agras_driver msg_MID360_launch.py
```

## 参数

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `lidar_ip` | `192.168.1.10` | 雷达地址 |
| `host_ip` | `192.168.1.20` | 绑定的本机地址。本机没有这个地址时改为 `0.0.0.0` |
| `fov_start` / `fov_stop` | `0` / `360` | 航向视场，度。`0` 到 `360` 为全周 |
| `xfer_format` | `0` | `0` PointCloud2，`1` CustomMsg |
| `publish_freq` | `10.0` | 点云发布频率 |
| `frame_id` | `livox_frame` | 点云和 IMU 的坐标系 |
| `lidar_topic` / `imu_topic` | `livox/lidar` / `livox/imu` | 话题名 |
| `auto_sampling` | `true` | 连接后写采样状态 |
| `standby_on_exit` | `true` | 退出时切到待机。Agras 固件上待机只关激光 |
| `configure_dest` | `false` | 是否重写点云和 IMU 的目的地址。默认不写，避免字节序不对时雷达不再把数据发到本机 |
| `publish_fill` | `false` | 是否发布坐标全 0 的填充点 |
| `max_range` | `80` | 米。超过这个距离的点丢掉，用来去掉大约 200 m 的无效回波。`0` 表示不裁剪 |
| `line_num` | `4` | `line` 字段的模 |
| `discover_sec` | `1.0` | 启动时监听发现端口的秒数，`0` 为跳过 |

无雷达时的协议自检：

```bash
g++ -std=c++17 -Wall -Wextra -I common/include \
  common/src/protocol.cpp common/src/device.cpp common/test/protocol_test.cpp \
  -pthread -o /tmp/agras_protocol_test && /tmp/agras_protocol_test
```
