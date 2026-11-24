# MID360 Agras ROS driver

ROS1 and ROS2 driver for the DJI Agras MID360. The wire format comes from [mid360-agras-viewer](https://gitee.com/xiaobai0818/mid360-agras-viewer) and does not link Livox SDK2. The retail `0xAA` protocol on port `56100` is not used.

`common/` is shared. `ros1/mid360_agras_driver` is the catkin package and `ros2/mid360_agras_driver` is the ament package. Both packages use the same name, so keep them in separate workspaces.

Set the host NIC to `192.168.1.20/24`. The lidar defaults to `192.168.1.10`. Point clouds arrive on UDP `60001`, IMU on `60003`, and control uses `60000`.

Topics match a normal MID360 driver:

| Topic | Type |
| --- | --- |
| `/livox/lidar` | `sensor_msgs/PointCloud2` when `xfer_format` is 0, otherwise `mid360_agras_driver/CustomMsg` |
| `/livox/imu` | `sensor_msgs/Imu` |

The PointCloud2 layout is `x y z intensity tag line timestamp`, 26 bytes per point, meters. `CustomMsg` has the same fields as `livox_ros_driver2/CustomMsg`. Gyro is rad/s and acceleration is in g.

`fov_start` and `fov_stop` are heading degrees. The default `0` and `360` is a full turn and only clears ROI enable. A stop angle below the start angle wraps across 0°.

```bash
# ROS1
catkin_make
source devel/setup.bash
roslaunch mid360_agras_driver rviz_MID360.launch
roslaunch mid360_agras_driver msg_MID360.launch

# ROS2
colcon build --packages-select mid360_agras_driver
ros2 launch mid360_agras_driver rviz_MID360_launch.py
```

`rviz_MID360` publishes PointCloud2 and opens RViz with fixed frame `livox_frame`. `msg_MID360` publishes CustomMsg. See `README.md` for the full parameter list.
