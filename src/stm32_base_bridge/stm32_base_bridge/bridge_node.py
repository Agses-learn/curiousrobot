"""Single-owner nonblocking serial bridge; no hardware movement without fresh cmd_vel."""
import math
import time

import serial
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Temperature
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .control import CommandGate
from .protocol import (TelemetryParser, encode_command, IMU_VALID, ODOM_VALID,
                       MOTOR_FAULT, COMMAND_ACTIVE)


class BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_base_bridge')
        defaults = {
            'port': '/dev/stm32_motor', 'baudrate': 115200,
            'cmd_vel_topic': '/cmd_vel', 'odom_topic': '/odom/unfiltered',
            'imu_topic': '/imu/data', 'cmd_timeout': 0.2,
            'telemetry_timeout': 0.5, 'reconnect_interval': 1.0, 'command_rate': 20.0,
            'max_linear': 0.5, 'max_angular': 3.0,
            'odom_frame': 'odom', 'base_frame': 'base_footprint', 'imu_frame': 'imu_link',
            'odom_vx_variance': 0.01, 'odom_vy_variance': 0.05, 'odom_wz_variance': 0.04,
            'gyro_variance': 0.0004, 'accel_variance': 0.04,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        for key in ('cmd_timeout', 'telemetry_timeout', 'reconnect_interval',
                    'command_rate', 'max_linear', 'max_angular',
                    'odom_vx_variance', 'odom_vy_variance', 'odom_wz_variance',
                    'gyro_variance', 'accel_variance'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(key + ' must be finite and positive')
        if not 10 <= self.cfg['command_rate'] <= 50:
            raise ValueError('command_rate must be 10..50 Hz')
        if self.cfg['cmd_timeout'] > 0.3:
            raise ValueError('cmd_timeout must not exceed 0.3 s')
        if self.cfg['max_linear'] > 0.5 or self.cfg['max_angular'] > 3.0:
            raise ValueError('Host limits must not exceed STM32 limits')
        if self.cfg['baudrate'] != 115200:
            raise ValueError('Current STM32 firmware requires 115200 baud')
        self.gate = CommandGate(self.cfg['cmd_timeout'], self.cfg['max_linear'],
                                self.cfg['max_angular'])
        self.ser = None
        self.parser = TelemetryParser()
        self.last_rx = None
        self.last_stamp = None
        self.last_seq = None
        self.flags = 0
        self.next_connect = 0.0
        self.last_tx = 0.0
        self.seq = 0
        self.received_frames = 0
        self.sequence_gaps = 0
        self.restarts = 0
        self.serial_errors = 0
        self.last_error = ''
        self.odom_pub = self.create_publisher(Odometry, self.cfg['odom_topic'], 10)
        self.imu_pub = self.create_publisher(Imu, self.cfg['imu_topic'], qos_profile_sensor_data)
        self.temp_pub = self.create_publisher(Temperature, '/imu/temperature', qos_profile_sensor_data)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(Twist, self.cfg['cmd_vel_topic'], self.on_command, 1)
        # Watchdogs and heartbeats must work even if /clock pauses.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.005, self.poll, clock=steady)
        self.create_timer(1.0, self.diagnostics, clock=steady)
        self.get_logger().info('STM32 v3 telemetry / v1 command bridge; waiting for fresh telemetry')

    def ready(self, now):
        return (self.ser is not None and self.last_rx is not None
                and now - self.last_rx < self.cfg['telemetry_timeout']
                and not self.flags & MOTOR_FAULT)

    def on_command(self, msg):
        now = time.monotonic()
        self.gate.accept(msg.linear.x, msg.angular.z, now, self.ready(now),
                         (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y))

    def disconnect(self, error):
        self.last_error = str(error)
        self.serial_errors += 1
        self.gate.clear()
        self.last_rx = None
        if self.ser is not None:
            self.ser.close()
        self.ser = None
        self.next_connect = time.monotonic() + self.cfg['reconnect_interval']
        self.get_logger().warning('Serial unavailable: ' + self.last_error)

    def send(self, vx, wz):
        frame = encode_command(self.seq, vx, wz)
        if self.ser.write(frame) != len(frame):
            raise serial.SerialTimeoutException('Short command write')
        self.seq = (self.seq + 1) & 0xFFFF

    def poll(self):
        now = time.monotonic()
        if self.ser is None:
            if now < self.next_connect:
                return
            try:
                self.ser = serial.Serial(self.cfg['port'], self.cfg['baudrate'],
                                         timeout=0, write_timeout=0.02, exclusive=True)
                self.ser.reset_input_buffer()
                self.parser.buffer.clear()
                self.gate.clear()
                self.last_rx = self.last_stamp = self.last_seq = None
                self.flags = 0
                self.send(0.0, 0.0)
                self.last_tx = now
                self.get_logger().info('Serial connected; command cleared')
            except (OSError, serial.SerialException) as exc:
                self.disconnect(exc)
            return
        try:
            # Invalidate old commands BEFORE processing telemetry that resumes after an outage.
            if not self.ready(now):
                self.gate.clear()
            chunk = self.ser.read(min(self.ser.in_waiting, 4096))
            for sample in self.parser.feed(chunk):
                if self.last_stamp is not None:
                    dt = (sample.timestamp_ms - self.last_stamp) & 0xFFFFFFFF
                    if dt == 0:
                        continue
                    if dt > 0x7FFFFFFF:
                        # MCU clock moved backwards: reboot, not ordinary uint32 rollover.
                        self.gate.clear()
                        self.restarts += 1
                        self.last_seq = None
                if self.last_seq is not None:
                    gap = (sample.sequence - self.last_seq) & 0xFFFF
                    if 1 < gap < 0x8000:
                        self.sequence_gaps += gap - 1
                self.last_stamp, self.last_seq = sample.timestamp_ms, sample.sequence
                self.last_rx = now
                self.flags = sample.flags
                self.received_frames += 1
                self.last_error = ''
                if self.flags & MOTOR_FAULT:
                    self.gate.clear()
                self.publish_sample(sample)
            if now - self.last_tx >= 1.0 / self.cfg['command_rate']:
                self.send(*self.gate.output(now, self.ready(now)))
                self.last_tx = now
        except (OSError, serial.SerialException) as exc:
            self.disconnect(exc)

    def publish_sample(self, sample):
        stamp = self.get_clock().now().to_msg()
        # Timestamp is host reception time. MCU millis is used only for restart detection.
        if sample.flags & ODOM_VALID:
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = self.cfg['odom_frame']
            msg.child_frame_id = self.cfg['base_frame']
            msg.pose.pose.position.x, msg.pose.pose.position.y = sample.x, sample.y
            msg.pose.pose.orientation.z = math.sin(sample.yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(sample.yaw / 2.0)
            msg.twist.twist.linear.x, msg.twist.twist.linear.y = sample.vx, sample.vy
            msg.twist.twist.angular.z = sample.wz
            for index in (0, 7, 14, 21, 28, 35):
                msg.pose.covariance[index] = 1e6
                msg.twist.covariance[index] = 1e6
            # Fixed starting variances; EKF intentionally fuses twist only.
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.1
            msg.pose.covariance[35] = 0.2
            msg.twist.covariance[0] = self.cfg['odom_vx_variance']
            msg.twist.covariance[7] = self.cfg['odom_vy_variance']
            msg.twist.covariance[35] = self.cfg['odom_wz_variance']
            self.odom_pub.publish(msg)
        if sample.flags & IMU_VALID:
            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = self.cfg['imu_frame']
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0  # MPU6050 has no orientation estimate.
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample.gyro
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z) = sample.accel
            for i in (0, 4, 8):
                msg.angular_velocity_covariance[i] = self.cfg['gyro_variance']
                msg.linear_acceleration_covariance[i] = self.cfg['accel_variance']
            self.imu_pub.publish(msg)
            temp = Temperature()
            temp.header = msg.header
            temp.temperature = sample.temperature
            self.temp_pub.publish(temp)

    def diagnostics(self):
        now = time.monotonic()
        status = DiagnosticStatus()
        status.name = 'curiousrobot/stm32_serial'
        status.hardware_id = self.cfg['port']
        if not self.ready(now):
            status.level = DiagnosticStatus.ERROR
            status.message = 'Disconnected, stale telemetry, or motor write fault'
        elif not (self.flags & ODOM_VALID and self.flags & IMU_VALID):
            status.level = DiagnosticStatus.WARN
            status.message = 'Telemetry received; sensor validity incomplete'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Telemetry healthy'
        values = {
            'connected': self.ser is not None,
            'telemetry_age_s': -1 if self.last_rx is None else round(now - self.last_rx, 3),
            'frames': self.received_frames, 'crc_errors': self.parser.crc_errors,
            'format_errors': self.parser.format_errors, 'sequence_gaps': self.sequence_gaps,
            'mcu_restarts': self.restarts, 'serial_errors': self.serial_errors,
            'command_active': bool(self.flags & COMMAND_ACTIVE),
            'motor_fault': bool(self.flags & MOTOR_FAULT),
            'odom_valid': bool(self.flags & ODOM_VALID), 'imu_valid': bool(self.flags & IMU_VALID),
            'last_error': self.last_error,
        }
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diag_pub.publish(msg)

    def close(self):
        self.gate.clear()
        if self.ser is not None:
            try:
                self.send(0.0, 0.0)
            except (OSError, serial.SerialException):
                pass
            finally:
                self.ser.close()
                self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
