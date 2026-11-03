#!/usr/bin/env python3
"""ROS runtime smoke checks with synthetic sensors, isolated domain, no serial devices."""
import math
import os
os.environ['ROS_DOMAIN_ID'] = '74'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import signal
import subprocess
import tempfile
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

root = Path(tempfile.mkdtemp(prefix='curiousrobot-nav-check-'))
size = 120
pixels = bytearray(0 if x in (0,size-1) or y in (0,size-1) else 254 for y in range(size) for x in range(size))
(root/'room.pgm').write_bytes(f'P5\n{size} {size}\n255\n'.encode()+pixels)
(root/'room.yaml').write_text('image: room.pgm\nmode: trinary\nresolution: 0.05\norigin: [-3.0, -3.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
rclpy.init()
node = rclpy.create_node('curiousrobot_nav_test')
tf = TransformBroadcaster(node)
static = StaticTransformBroadcaster(node)
mount = TransformStamped()
mount.header.frame_id = 'base_footprint'
mount.child_frame_id = 'laser'
mount.transform.translation.z = 0.1
mount.transform.rotation.w = 1.
mount.header.stamp = node.get_clock().now().to_msg()
static.sendTransform(mount)
scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
odom_pub = node.create_publisher(Odometry, '/odom', 10)
pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
cmd_pub = node.create_publisher(Twist, '/cmd_vel_nav', 10)
received = []
maps = []
node.create_subscription(Twist, '/cmd_vel', lambda m: received.append((time.monotonic(), m.linear.x, m.angular.z)), 10)
node.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
send_scans = True
send_velocity = False
slam_moving = False
trajectory_start = time.monotonic()
def tick():
    x = 0.3 * math.sin((time.monotonic()-trajectory_start)*0.8) if slam_moving else 0.0
    stamp = node.get_clock().now().to_msg()
    t = TransformStamped()
    t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'odom', 'base_footprint'
    t.transform.translation.x = x
    t.transform.rotation.w = 1.
    tf.sendTransform(t)
    odom = Odometry()
    odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
    odom.pose.pose.position.x = x
    odom.pose.pose.orientation.w = 1.
    odom_pub.publish(odom)
    if send_scans:
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'laser'
        scan.angle_min, scan.angle_max = -math.pi, math.pi - math.pi/180
        scan.angle_increment = math.pi/180
        scan.range_min, scan.range_max, scan.scan_time = 0.05, 12., 0.1
        scan.ranges = []
        for i in range(360):
            c, s = math.cos(-math.pi+i*math.pi/180), math.sin(-math.pi+i*math.pi/180)
            scan.ranges.append(min((2.95-math.copysign(1., c)*x)/max(abs(c), 1e-9),
                                   2.95/max(abs(s), 1e-9)))
        scan_pub.publish(scan)
    if send_velocity:
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd_pub.publish(cmd)
timer = node.create_timer(0.05, tick)
def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
def until(predicate, timeout, message):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=0.05)
    raise AssertionError(message)
def future_result(f, timeout=15):
    until(f.done, timeout, 'ROS request timed out')
    return f.result()
def active(name):
    client = node.create_client(GetState, name+'/get_state')
    until(client.service_is_ready, 30, name+' state service missing')
    state = future_result(client.call_async(GetState.Request())).current_state
    node.destroy_client(client)
    return state.id == 3
def start(launch, **args):
    log = open(root/(launch+'.log'), 'w')
    proc = subprocess.Popen(['ros2','launch','stm32_robot_navigation',launch+'.launch.py']+
        [f'{k}:={v}' for k,v in args.items()], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return proc, log
def stop(proc, log):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    log.close()
proc = log = None
try:
    proc, log = start('navigation', map=root/'room.yaml')
    spin(12)
    assert proc.poll() is None, 'Navigation launch exited; inspect '+str(root/'navigation.log')
    # AMCL requires an explicit initial pose before it can provide map -> odom.
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.orientation.w = 1.
    pose.pose.covariance[0] = pose.pose.covariance[7] = 0.01
    pose.pose.covariance[35] = 0.01
    for _ in range(3):
        pose.header.stamp = node.get_clock().now().to_msg()
        pose_pub.publish(pose)
        spin(1)
    names = ['map_server','amcl','controller_server','planner_server','smoother_server',
             'behavior_server','bt_navigator','waypoint_follower','velocity_smoother','collision_monitor']
    for name in names:
        deadline = time.monotonic()+35
        while not active('/'+name):
            assert proc.poll() is None, 'Navigation launch exited'
            assert time.monotonic()<deadline, name+' failed to activate'
            spin(1)
    print('PASS all 10 Nav2/localization lifecycle nodes active', flush=True)
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    until(lambda: client.server_is_ready(), 10, 'Planner action missing')
    goal = ComputePathToPose.Goal()
    goal.goal.header.frame_id = 'map'
    goal.goal.header.stamp = node.get_clock().now().to_msg()
    goal.goal.pose.position.x = 1.
    goal.goal.pose.orientation.w = 1.
    goal.planner_id = 'GridBased'
    handle = future_result(client.send_goal_async(goal))
    assert handle.accepted, 'Planner rejected goal'
    result = future_result(handle.get_result_async())
    assert result.status == 4 and len(result.result.path.poses)>2, str(result)
    print('PASS planner produces a path through the temporary map', flush=True)
    send_velocity = True
    spin(2)
    assert any(v > 0.05 for _,v,_ in received[-20:]), 'No nonzero final Twist from smoother/collision monitor'
    print('PASS cmd_vel_nav -> smoother -> collision monitor -> cmd_vel (Twist)', flush=True)
    send_scans = False
    cutoff = time.monotonic()+0.8
    spin(1.5)
    recent = [v for stamp,v,_ in received if stamp>cutoff]
    assert recent and all(abs(v)<1e-6 for v in recent), 'Scan loss did not stop final velocity'
    print('PASS scan loss stops final velocity while input commands continue', flush=True)
    send_velocity = False
    stop(proc, log)
    proc = log = None
    send_scans = True
    spin(0.5)
    maps.clear()
    slam_moving = True
    trajectory_start = time.monotonic()
    proc, log = start('slam')
    until(lambda: bool(maps) and any(v==100 for v in maps[-1].data), 30,
          'SLAM did not produce occupied cells from the simulated trajectory')
    assert maps[-1].info.width>0 and any(v==100 for v in maps[-1].data), 'SLAM map contains no occupied cells'
    print('PASS SLAM Toolbox produces an occupancy map from synthetic scans', flush=True)
    print('ALL NAVIGATION RUNTIME CHECKS PASSED; logs: '+str(root), flush=True)
finally:
    if proc is not None:
        stop(proc, log)
    node.destroy_node()
    rclpy.shutdown()

