"""ROS2 launch/data smoke checks using loopback UDP; no physical lidar required."""

import argparse
import os
import signal
import socket
import struct
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from mid360_agras_driver.msg import CustomMsg, CustomPoint
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu, PointCloud2, PointField


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


def check_native_case(launch_file, *, xfer=None):
    package = 'mid360_agras_driver'
    fmt = xfer if xfer is not None else (4 if launch_file == 'msg_MID360_launch.py' else 0)
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
    if fmt in (1, 4):
        expected[custom_topic] = CustomMsg
    cloud_topic = custom_topic + ('/pointcloud' if fmt == 4 else '')
    if fmt in (0, 4):
        expected[cloud_topic] = PointCloud2
    imu_topic = '/agras/livox/imu'
    expected[imu_topic] = Imu
    for topic, msg_type in expected.items():
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, topic=topic: received.__setitem__(topic, msg),
            qos_profile_sensor_data))
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_for(node, lambda: all(node.count_publishers(t) > 0 for t in expected), process)
            wait_for(node, lambda: node.get_node_names().count('mid360_agras_driver') == 1, process)
            graph = dict(node.get_topic_names_and_types())
            for topic, msg_type in expected.items():
                assert graph[topic] == [msg_type.__module__.split('.')[0] + '/msg/' + msg_type.__name__], graph
            names = node.get_node_names()
            assert names.count('mid360_agras_driver') == 1, names
            assert 'mid360_agras_livox_compat' not in names, names
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
                assert message.header.frame_id == 'agras_mid360'
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
            if fmt in (0, 4):
                cloud = received[cloud_topic]
                assert cloud.point_step == 26 and cloud.width >= 2
                first = struct.unpack_from('<ffffBBd', bytes(cloud.data))
                assert first[:6] == (1, 2, 3, 40, 7, 0)
                assert first[6] == float(STAMP)
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


def stamp_ns(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


def check_bridge_fields(node, process, received):
    # Exercise fields with values that the hardware path normally leaves at their default.
    publisher = node.create_publisher(CustomMsg, '/mid360_agras/custom', 10)
    try:
        wait_for(node, lambda: publisher.get_subscription_count() == 1, process)
        source = CustomMsg()
        source.header.frame_id = 'synthetic_bridge_frame'
        source.header.stamp.sec, source.header.stamp.nanosec = 123, 456
        source.timebase, source.point_num, source.lidar_id = 987654321, 1, 9
        source.rsvd = [3, 2, 1]
        source.points = [CustomPoint(offset_time=76543, x=1.25, y=-2.5, z=3.75,
                                     reflectivity=89, tag=12, line=3)]
        expected = message_to_ordereddict(source)
        publisher.publish(source)
        wait_for(node, lambda: message_to_ordereddict(received[-1]) == expected, process, seconds=5)
    finally:
        node.destroy_publisher(publisher)


def check_livox_profile(*, pointcloud=False, device_time=False, frequency=None):
    from livox_ros_driver2.msg import CustomMsg as LivoxMsg

    launch_file = 'rviz_MID360_launch.py' if pointcloud else 'msg_MID360_launch.py'
    ports = set()
    while len(ports) < 3:
        ports.add(free_port())
    cmd_port, point_port, imu_port = sorted(ports)
    namespace = '/agras_test' if pointcloud else ''
    suffix = '_127_0_0_2' if pointcloud else ''
    point_topic = namespace + '/livox/lidar' + suffix
    imu_topic = namespace + '/livox/imu' + suffix
    diagnostics_topic = namespace + '/diagnostics'
    received = {point_topic: [], imu_topic: [], diagnostics_topic: []}
    lidar_type = PointCloud2 if pointcloud else LivoxMsg
    expected = {point_topic: lidar_type, imu_topic: Imu, diagnostics_topic: DiagnosticArray}
    node = rclpy.create_node('agras_livox_profile_observer')
    subscriptions = [node.create_subscription(kind, topic, received[topic].append, 256)
                     for topic, kind in expected.items()]

    def diagnostic(name):
        return next(item for item in received[diagnostics_topic][-1].status if item.name.endswith(': ' + name))

    def values(name):
        return {item.key: item.value for item in diagnostic(name).values}

    def send(sender, stamp, sequence, *, imu=False, bad_value=None, timestamp_type=None):
        packet = udp_packet(imu)
        packet[11] = int(device_time) if timestamp_type is None else timestamp_type
        struct.pack_into('<Q', packet, 28, stamp)
        struct.pack_into('<H', packet, 7, sequence)
        if bad_value is not None:
            struct.pack_into('<f', packet, 36, bad_value)
        sender.sendto(packet, ('127.0.0.1', imu_port if imu else point_port))

    with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile(mode='w+') as log:
        config = Path(directory) / 'device.yaml'
        config.write_text(f'''/**:
  ros__parameters:
    lidar_ip: "127.0.0.3"
    host_ip: "127.0.0.1"
    cmd_port: {cmd_port}
    point_port: {point_port}
    imu_port: {imu_port}
    discover_sec: 0.0
    auto_sampling: false
    configure_dest: false
    standby_on_exit: false
    frame_id: "native_frame_from_device_yaml"
''', encoding='utf-8')
        command = ['ros2', 'launch', 'mid360_agras_livox_compat', launch_file, 'rviz:=false',
                   f'user_config_path:={config}', 'lidar_ip:=127.0.0.2']
        if pointcloud:
            command += ['namespace:=agras_test', 'multi_topic:=1', 'frame_id:=cloud_frame']
        if device_time:
            command += ['timestamp_source:=device']
        if frequency is not None:
            command += [f'publish_freq:={frequency}']
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_for(node, lambda: received[diagnostics_topic], process)
            graph = dict(node.get_topic_names_and_types())
            for topic, msg_type in expected.items():
                pkg = msg_type.__module__.split('.')[0]
                assert graph[topic] == [f'{pkg}/msg/{msg_type.__name__}'], graph
            assert '/livox/lidar/pointcloud' not in graph
            assert (namespace + '/mid360_agras/custom' + suffix in graph) == (not pointcloud)
            expected_depth = 64 if pointcloud else 256
            for topic in (point_topic, imu_topic):
                qos = node.get_publishers_info_by_topic(topic)[0].qos_profile
                assert qos.reliability.name == 'RELIABLE' and qos.durability.name == 'VOLATILE', qos
                assert qos.depth in (0, expected_depth), qos  # Fast DDS may omit remote history depth.

            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.bind(('127.0.0.2', 0))
                wall = time.time_ns()
                base = wall // 10**9 * 10**9
                for sequence in range(1, 91):
                    stamp = base + sequence * 10_000_000
                    send(sender, stamp, sequence)
                    send(sender, stamp, sequence, imu=True)
                    time.sleep(0.009)
                    rclpy.spin_once(node, timeout_sec=0)
                wait_for(node, lambda: received[point_topic] and len(received[imu_topic]) >= 80, process)
                point, imu = received[point_topic][0], received[imu_topic][0]
                assert point.header.frame_id == ('cloud_frame' if pointcloud else 'livox_frame')
                assert imu.header.frame_id == 'livox_frame' and imu.orientation_covariance[0] == -1.0
                assert (imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z) == (1, 2, 3)
                assert (imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z) == (4, 5, 6)
                if device_time:
                    assert stamp_ns(imu) == base + 20_000_000
                    assert stamp_ns(point) == base + 30_000_000
                else:
                    assert wall <= stamp_ns(imu) <= time.time_ns()
                    assert wall <= stamp_ns(point) <= time.time_ns()
                if pointcloud:
                    names = ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp']
                    datatypes = [PointField.FLOAT32] * 4 + [PointField.UINT8] * 2 + [PointField.FLOAT64]
                    expected_fields = list(zip(names, [0, 4, 8, 12, 16, 17, 18], datatypes, [1] * 7))
                    assert [(f.name, f.offset, f.datatype, f.count) for f in point.fields] == expected_fields
                    assert point.height == 1 and point.point_step == 26 and point.row_step == point.width * 26
                    assert point.is_dense and not point.is_bigendian and len(point.data) == point.row_step
                    first, second = list(struct.iter_unpack('<ffffBBd', point.data))[:2]
                    assert first == (1, 2, 3, 40, 7, 0, float(stamp_ns(point)))
                    assert second == (4, 5, 6, 60, 9, 1, float(stamp_ns(point) + 50_000))
                else:
                    assert point.timebase == stamp_ns(point) and point.point_num == len(point.points) >= 2
                    assert point.lidar_id == 127 and list(point.rsvd) == [0, 0, 0]
                    assert [p.offset_time for p in point.points[:2]] == [0, 50_000]
                    assert [(p.reflectivity, p.tag, p.line) for p in point.points[:2]] == [(40, 7, 0), (60, 9, 1)]
                    assert list(point.points) == sorted(point.points, key=lambda p: p.offset_time)

                if frequency is None:
                    assert 13 <= len(received[point_topic]) <= 24, len(received[point_topic])
                else:
                    assert len(received[point_topic]) >= 30
                send(sender, base + 920_000_000, 92, imu=True, bad_value=float('nan'))
                send(sender, base + 930_000_000, 93, imu=True, bad_value=float('inf'))
                wait_for(node, lambda: values('imu')['invalid_imu_samples'] == '2', process)
                assert all(msg.angular_velocity.x == 1 for msg in received[imu_topic])
                if device_time:
                    send(sender, base + 940_000_000, 94, imu=True, timestamp_type=0)
                    wait_for(node, lambda: diagnostic('timestamps').level == DiagnosticStatus.ERROR, process)
                    counts = (len(received[point_topic]), len(received[imu_topic]))
                    for sequence in range(95, 125):
                        stamp = base + sequence * 10_000_000
                        send(sender, stamp, sequence)
                        send(sender, stamp, sequence, imu=True)
                        rclpy.spin_once(node, timeout_sec=0.01)
                    assert (len(received[point_topic]), len(received[imu_topic])) == counts
                else:
                    clock = values('timestamps')
                    assert clock['timestamp_source'] == 'host' and int(clock['pointcloud.invalid_or_unsynchronized']) > 0

            if not pointcloud and not device_time:
                check_bridge_fields(node, process, received[point_topic])
            log.seek(0)
            output = log.read()
            effective = 100.0 if frequency is not None else 20.0
            assert f'点云 {effective:.3f} Hz，QoS 深度 {expected_depth}' in output, output
            if not pointcloud:
                assert f'Livox CustomMsg QoS: input={expected_depth} output={expected_depth}' in output
            print(f'PASS Livox profile {launch_file}, source={"device" if device_time else "host"}', flush=True)
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
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compat', action='store_true', help='also test the optional Livox bridge')
    args = parser.parse_args()
    rclpy.init()
    try:
        for entry in ('driver_MID360_launch.py', 'rviz_MID360_launch.py', 'msg_MID360_launch.py'):
            check_native_case(entry)
        check_native_case('msg_MID360_launch.py', xfer=1)
        check_rejected('mid360_agras_driver', 'msg_MID360_launch.py', ['xfer_format:=2'], 'xfer_format 必须是')
        check_rejected('mid360_agras_driver', 'msg_MID360_launch.py', ['rviz:=true', 'xfer_format:=1'], 'RViz 需要 PointCloud2')
        with tempfile.TemporaryDirectory() as directory:
            incompatible = Path(directory) / 'custom_msg.yaml'
            incompatible.write_text('/**:\n  ros__parameters:\n    custom_msg_type: livox\n', encoding='utf-8')
            check_rejected('mid360_agras_driver', 'msg_MID360_launch.py',
                           [f'params_file:={incompatible}'], '兼容模式请使用')
        if args.compat:
            check_livox_profile()
            check_livox_profile(pointcloud=True)
            check_livox_profile(pointcloud=True, frequency=250.0)
            check_livox_profile(device_time=True)
            for argument, diagnostic in (
                ('xfer_format:=4', 'xfer_format=0/1'),
                ('publish_freq:=nan', 'publish_freq 必须是有限数值'),
                ('multi_topic:=2', 'multi_topic=0/1'),
                ('data_src:=1', 'data_src=0'),
                ('output_data_type:=1', 'output_data_type=0'),
            ):
                check_rejected('mid360_agras_livox_compat', 'msg_MID360_launch.py', [argument], diagnostic)
            with tempfile.TemporaryDirectory() as directory:
                retail_config = Path(directory) / 'MID360_config.json'
                retail_config.write_text('{"lidar_summary_info":{"lidar_type":8},"MID360":{}}', encoding='utf-8')
                check_rejected('mid360_agras_livox_compat', 'msg_MID360_launch.py',
                               [f'user_config_path:={retail_config}'], 'user_config_path 需要 Agras')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
