# CuriousRobot bringup

All commands run in Ubuntu after sourcing /opt/ros/jazzy/setup.bash and
~/work/curiousrobot/install/setup.bash.

Offline demo (no physical serial port opened):
```bash
ros2 launch stm32_robot_bringup demo.launch.py rviz:=true
```

Real STM32:
```bash
ros2 launch stm32_robot_bringup bringup.launch.py port:=/dev/serial/by-id/YOUR_STM32_ADAPTER rviz:=true
```
Set lidar:=true lidar_port:=/dev/serial/by-id/YOUR_LIDAR_ADAPTER to start STL-06N.
Set measured sensor mounting with lidar_x/y/z/roll/pitch/yaw launch arguments.
config/lidar.yaml contains driver settings, not mounting geometry.
Only one node should open each serial port. No physical serial device has yet
been verified in VMware. Reconnect SSH/login after dialout group changes.

TF ownership:
- EKF: odom -> base_footprint.
- robot_state_publisher: base_footprint -> base_link -> imu_link and wheels.
- optional lidar launch: base_link -> laser.
- SLAM or AMCL (not both): map -> odom.

preview_joints:=true supplies zero wheel positions for visualization only.
Disable when real joint states become available. Firmware v3 does not report
individual encoder values, so wheel angles cannot be recovered exactly.
IMU orientation is unknown; EKF fuses wheel vx/vy/wz and IMU wz, not IMU yaw.
Do not run a second raw odometry TF publisher.

The model body dimensions and sensor mounting poses still need measurement.
Linorobot2 source is used as reference and remains unmodified.
