"""ROS2 launch/data smoke checks using loopback UDP; no physical lidar required."""

import argparse
import os
import signal
import socket
import struct
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from mid360_agras_driver.msg import CustomMsg, CustomPoint
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu, PointCloud2


STAMP = 1800000000123456789


def udp_packet(imu=False):
    body = (struct.pack('<6f', 1, 2, 3, 4, 5, 6) if imu else
            struct.pack('<iiiBBiiiBB', 1000, 2000, 3000, 40, 7,
                        4000, 5000, 6000, 60, 9))
    packet = bytearray(36) + body
    struct.pack_into('<HHHH', packet, 1, len(packet), 1000, 0 if imu else 2, 1)
    packet[10:12] = bytes([0 if imu else 1, 1])
    struct.pack_into('<Q', packet, 28, STAMP)
    return packet


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait_for(node, predicate, process, seconds=12):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        assert process.poll() is None, 'launch exited unexpectedly'
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('timed out waiting for ROS messages or discovery')


def check_case(launch_file, *, compat=False, xfer=None):
    package = 'mid360_agras_livox_compat' if compat else 'mid360_agras_driver'
    fmt = xfer if xfer is not None else (0 if launch_file != 'msg_MID360_launch.py' and not compat else 4)
    ports = [free_port() for _ in range(3)]
    while len(set(ports)) < 3:
        ports = [free_port() for _ in range(3)]
    command = ['ros2', 'launch', package, launch_file, 'rviz:=false',
               'lidar_ip:=127.0.0.2', 'host_ip:=127.0.0.1',
               f'cmd_port:={ports[0]}', f'point_port:={ports[1]}', f'imu_port:={ports[2]}',
               'discover_sec:=0', 'auto_sampling:=false', 'configure_dest:=false',
               'standby_on_exit:=false']
    if xfer is not None:
        command.append(f'xfer_format:={xfer}')
    node = rclpy.create_node('agras_smoke_observer')
    received = {}
    subscriptions = []
    expected = {}
    custom_topic = '/agras/livox/lidar'
    frame = 'front_mid360' if compat else 'agras_mid360'
    if fmt in (1, 4):
        expected[custom_topic] = CustomMsg
    cloud_topic = '/livox/lidar/pointcloud' if compat else '/agras/livox/lidar' + ('/pointcloud' if fmt == 4 else '')
    if fmt in (0, 4):
        expected[cloud_topic] = PointCloud2
    imu_topic = '/livox/imu' if compat else '/agras/livox/imu'
    expected[imu_topic] = Imu
    if compat:
        from livox_ros_driver2.msg import CustomMsg as LivoxMsg
        expected['/livox/lidar'] = LivoxMsg
    for topic, msg_type in expected.items():
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, topic=topic: received.__setitem__(topic, msg),
            qos_profile_sensor_data))
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_for(node, lambda: all(node.count_publishers(t) > 0 for t in expected), process)
            wait_for(node, lambda: node.get_node_names().count('mid360_agras_driver') == 1
                     and node.get_node_names().count('mid360_agras_livox_compat') == int(compat), process)
            graph = dict(node.get_topic_names_and_types())
            for topic, msg_type in expected.items():
                assert graph[topic] == [msg_type.__module__.split('.')[0] + '/msg/' + msg_type.__name__], graph
            names = node.get_node_names()
            assert names.count('mid360_agras_driver') == 1, names
            assert names.count('mid360_agras_livox_compat') == int(compat), names
            assert not any('livox_ros_driver2' in name for name in names), names
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.bind(('127.0.0.2', 0))
                deadline = time.monotonic() + 12
                while set(received) != set(expected) and time.monotonic() < deadline:
                    sender.sendto(udp_packet(), ('127.0.0.1', ports[1]))
                    sender.sendto(udp_packet(True), ('127.0.0.1', ports[2]))
                    rclpy.spin_once(node, timeout_sec=0.1)
                assert set(received) == set(expected), received.keys()
            for message in received.values():
                assert message.header.frame_id == frame
                assert message.header.stamp.sec * 10**9 + message.header.stamp.nanosec == STAMP
            imu = received[imu_topic]
            assert (imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z) == (1, 2, 3)
            assert (imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z) == (4, 5, 6)
            if fmt in (1, 4):
                cloud = received[custom_topic]
                assert cloud.timebase == STAMP and cloud.point_num == len(cloud.points) >= 2
                first, second = cloud.points[:2]
                assert (first.x, first.y, first.z, first.reflectivity, first.tag, first.line, first.offset_time) == (1, 2, 3, 40, 7, 0, 0)
                assert (second.x, second.y, second.z, second.reflectivity, second.tag, second.line, second.offset_time) == (4, 5, 6, 60, 9, 1, 50000)
                if compat:
                    assert message_to_ordereddict(cloud) == message_to_ordereddict(received['/livox/lidar'])
            if fmt in (0, 4):
                cloud = received[cloud_topic]
                assert cloud.point_step == 26 and cloud.width >= 2
                first = struct.unpack_from('<ffffBBd', bytes(cloud.data))
                assert first[:6] == (1, 2, 3, 40, 7, 0)
                assert first[6] == float(STAMP)
            if compat:
                # A synthetic message exercises fields which UDP output normally leaves zero.
                publisher = node.create_publisher(CustomMsg, custom_topic, 10)
                wait_for(node, lambda: publisher.get_subscription_count() >= 2, process)
                source = CustomMsg()
                source.header.frame_id = 'synthetic_bridge_frame'
                source.header.stamp.sec, source.header.stamp.nanosec = 123, 456
                source.timebase, source.point_num, source.lidar_id = 987654321, 1, 9
                source.rsvd = [3, 2, 1]
                source.points = [CustomPoint(offset_time=76543, x=1.25, y=-2.5, z=3.75,
                                             reflectivity=89, tag=12, line=3)]
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    publisher.publish(source)
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if message_to_ordereddict(received['/livox/lidar']) == message_to_ordereddict(source):
                        break
                else:
                    raise AssertionError('bridge did not preserve all message fields')
            print(f'PASS {package}/{launch_file} xfer_format={fmt}', flush=True)
        except BaseException:
            log.seek(0)
            print(log.read(), flush=True)
            raise
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()


def check_rejected(package, launch_file, arguments, diagnostic):
    result = subprocess.run(['ros2', 'launch', package, launch_file, *arguments],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode != 0 and diagnostic in result.stdout + result.stderr, result
    print(f'PASS rejected {arguments}: {diagnostic}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compat', action='store_true', help='also test the optional Livox bridge')
    args = parser.parse_args()
    rclpy.init()
    try:
        for entry in ('driver_MID360_launch.py', 'rviz_MID360_launch.py', 'msg_MID360_launch.py'):
            check_case(entry)
        check_case('msg_MID360_launch.py', xfer=1)
        check_rejected('mid360_agras_driver', 'msg_MID360_launch.py', ['xfer_format:=2'], 'xfer_format 必须是')
        check_rejected('mid360_agras_driver', 'msg_MID360_launch.py', ['rviz:=true', 'xfer_format:=1'], 'RViz 需要 PointCloud2')
        if args.compat:
            check_case('point_lio_compat_launch.py', compat=True)
            check_case('point_lio_compat_launch.py', compat=True, xfer=1)
            check_rejected('mid360_agras_livox_compat', 'point_lio_compat_launch.py', ['xfer_format:=0'], '兼容启动需要')
            share = get_package_share_directory('mid360_agras_livox_compat')
            check_rejected('mid360_agras_driver', 'msg_MID360_launch.py',
                           [f'params_file:={share}/config/point_lio_compat.yaml'], '兼容模式请使用')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
