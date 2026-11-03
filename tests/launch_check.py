#!/usr/bin/env python3
"""Check public startup entrypoints; only PTY and a nonexistent lidar port are used."""
import os
os.environ['ROS_DOMAIN_ID'] = '75'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import signal
import subprocess
import tempfile
import time
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

rclpy.init()
node = rclpy.create_node('curiousrobot_launch_check')
odom = []
diag = {}
node.create_subscription(Odometry, '/odom', lambda m: odom.append(m), 10)
def diagnostic(msg):
    for status in msg.status:
        diag[status.name] = (status.level, status.message)
node.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, 10)
root = Path(tempfile.mkdtemp(prefix='curiousrobot-launch-check-'))
def close(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
try:
    with (root/'demo.log').open('w') as log:
        proc = subprocess.Popen(['ros2', 'launch', 'stm32_robot_bringup', 'demo.launch.py'],
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+12
            while time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if odom and diag.get('curiousrobot/stm32_serial', (None,))[0] == DiagnosticStatus.OK:
                    break
            print('Observed odometry messages:', len(odom), 'diagnostics:', diag, flush=True)
            assert odom and diag.get('curiousrobot/stm32_serial', (None,))[0] == DiagnosticStatus.OK, str(root/'demo.log')
            print('PASS offline demo starts mock MCU, bridge and EKF with healthy serial diagnostics', flush=True)
        finally:
            close(proc)
finally:
    node.destroy_node()
    rclpy.shutdown()

with (root/'lidar-no-device.log').open('w') as log:
    proc = subprocess.Popen(['ros2', 'launch', 'stm32_robot_bringup', 'lidar.launch.py',
                             'port:='+str(root/'missing-port')],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    finally:
        close(proc)
s = (root/'lidar-no-device.log').read_text()
assert 'Caught exception in launch' not in s, s
assert 'parameter' not in '\n'.join(l for l in s.splitlines() if '[ERROR]' in l).lower(), s
assert str(root/'missing-port') in s, s
print('LIDAR STARTUP LOG (expected missing test port):\n'+s, flush=True)
print('Launch check logs: '+str(root), flush=True)

