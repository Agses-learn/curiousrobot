"""ROS2/PTY integration test. Uses a private ROS domain and NEVER opens hardware."""
import os
os.environ['ROS_DOMAIN_ID'] = '73'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import math
import pty
import select
import signal
import struct
import subprocess
import tempfile
import time
import tty
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data
from stm32_base_bridge.protocol import crc16
from stm32_base_bridge.mock_mcu import telemetry

work = Path(tempfile.mkdtemp(prefix='curiousrobot-integration-'))
link = work/'serial'
rclpy.init()
node = rclpy.create_node('curiousrobot_integration')
odom, imu, filtered, transforms, diagnostics, commands = [], [], [], [], [], []
subs = [
    node.create_subscription(Odometry, '/odom/unfiltered', odom.append, 100),
    node.create_subscription(Imu, '/imu/data', imu.append, qos_profile_sensor_data),
    node.create_subscription(Odometry, '/odom', filtered.append, 100),
    node.create_subscription(TFMessage, '/tf', lambda m: transforms.extend(m.transforms), 100),
    node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics.append, 10),
]
pub = node.create_publisher(Twist, '/cmd_vel', 1)
master = slave = None
wire = bytearray()
seq = 0
ms_base = 10000
epoch = time.monotonic()
flags = 0x12
emit = True
corrupt = False
last_send = 0
serial_log = open(work/'bringup.log','w')

def new_port():
    global master, slave
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    link.symlink_to(os.ttyname(slave))

def pump(duration, command=None):
    global seq, last_send
    end = time.monotonic() + duration
    last_command = 0
    while time.monotonic() < end:
        now = time.monotonic()
        if emit and now - last_send >= 0.02:
            frame = telemetry(seq, ms_base + int((now-epoch)*1000),
                              x=0.7,y=-0.2,yaw=0.3,vx=0.123,wz=0.0,flags=flags)
            if corrupt:
                frame = bytearray(frame)
                frame[-1] ^= 0x80
            try:
                os.write(master, frame)
            except BlockingIOError:
                pass
            seq = (seq+1)&0xFFFF
            last_send = now
        try:
            wire.extend(os.read(master,4096))
        except BlockingIOError:
            pass
        while len(wire) >= 13:
            if wire[:4] != bytes.fromhex('a55a0102') or wire[6] != 4:
                del wire[0]
                continue
            packet=wire[:13]
            assert crc16(packet[2:11]) == struct.unpack_from('<H',packet,11)[0]
            commands.append((now,*struct.unpack_from('<hh',packet,7)))
            del wire[:13]
        if command is not None and now-last_command>=0.05:
            msg=Twist()
            msg.linear.x, msg.angular.z = command
            pub.publish(msg)
            last_command=now
        rclpy.spin_once(node,timeout_sec=0.003)

new_port()
process = subprocess.Popen(
    ['ros2','launch','stm32_robot_bringup','bringup.launch.py','port:='+str(link)],
    stdout=serial_log,stderr=serial_log,start_new_session=True)
try:
    pump(5)
    assert process.poll() is None, 'Bringup exited'
    assert odom and imu and filtered and commands, 'Missing bridge/EKF topics'
    assert commands[0][1:] == (0,0), 'Startup command not zero'
    assert odom[-1].header.frame_id == 'odom'
    assert odom[-1].child_frame_id == 'base_footprint'
    assert abs(odom[-1].pose.pose.position.x-0.7)<1e-6
    assert abs(odom[-1].twist.twist.linear.x-0.123)<1e-6
    assert imu[-1].header.frame_id == 'imu_link'
    assert imu[-1].orientation_covariance[0] == -1
    assert abs(imu[-1].linear_acceleration.z-9.80665)<1e-6
    assert any(t.header.frame_id=='odom' and t.child_frame_id=='base_footprint' for t in transforms)
    print('PASS valid telemetry -> raw odom / IMU -> EKF /odom and TF',flush=True)

    start=len(commands)
    pump(0.6, (0.1, -0.4))
    assert any(c[1:] == (100,-400) for c in commands[start:])
    pump(0.4)
    assert all(c[1:]==(0,0) for c in commands[-3:])
    print('PASS signed command encoding and upstream watchdog',flush=True)

    flags=0
    pump(0.15)
    counts=(len(odom),len(imu))
    pump(0.25)
    assert counts==(len(odom),len(imu)), 'Invalid sensor sample published'
    flags=0x12
    corrupt=True
    pump(0.15)
    counts=(len(odom),len(imu))
    pump(0.25)
    assert counts==(len(odom),len(imu)), 'CRC-invalid sample published'
    corrupt=False
    pump(0.4)
    print('PASS invalid flags and corrupt frames do not publish measurements',flush=True)

    pump(0.3,(0.1,0.0))
    emit=False
    pump(0.8,(0.1,0.0))
    assert all(c[1:]==(0,0) for c in commands[-3:])
    emit=True
    pump(0.2)
    assert all(c[1:]==(0,0) for c in commands[-3:])
    print('PASS telemetry loss stops despite incoming cmd_vel; resumption keeps zero',flush=True)

    pump(0.25,(0.1,0.0))
    ms_base=0
    epoch=time.monotonic()
    pump(0.12)
    assert commands[-1][1:]==(0,0)
    print('PASS MCU clock reset clears command',flush=True)

    pump(0.25,(0.1,0.0))
    os.close(master)
    os.close(slave)
    link.unlink()
    wire.clear()
    new_port()
    pump(1.5)
    assert all(c[1:]==(0,0) for c in commands[-3:])
    start=len(commands)
    pump(0.3,(0.1,0.0))
    assert any(c[1:]==(100,0) for c in commands[start:])
    print('PASS serial reconnect requires a fresh command',flush=True)
    pump(0.4)
    flags=0x1a
    pump(0.3,(0.1,0.0))
    assert all(c[1:]==(0,0) for c in commands[-3:])
    print('PASS motor fault inhibits motion',flush=True)
    print('ALL INTEGRATION CHECKS PASSED; logs:',work,flush=True)
finally:
    os.killpg(process.pid,signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGKILL)
        process.wait(timeout=5)
    serial_log.close()
    node.destroy_node()
    rclpy.shutdown()
    os.close(master)
    os.close(slave)
    link.unlink(missing_ok=True)
