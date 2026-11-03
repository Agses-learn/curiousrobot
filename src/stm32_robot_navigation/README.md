# CuriousRobot mapping and navigation

Based on Linorobot2 Jazzy, using standard SLAM Toolbox, AMCL and Nav2.
Base bringup, EKF, and the lidar driver must already be running.
Nav2 final /cmd_vel is geometry_msgs/Twist (stamped commands disabled explicitly).

Mapping:
```bash
ros2 launch stm32_robot_navigation slam.launch.py rviz:=true
```
This starts SLAM only; it does not start Nav2 motion controllers.
Drive with teleop at low speed, then save a map:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.4
ros2 run nav2_map_server map_saver_cli -f /home/zhangkaiqi/work/curiousrobot/maps/room
```
The maps directory is provided. The installed Jazzy keyboard node publishes on
keypress only; it does not support repeat_rate or key_timeout. Holding a key
uses OS key repeat. Updates must be less than 0.2 s apart; the initial keyboard
repeat delay may cause a brief stop. Release the key to let the watchdog stop,
or press k to send zero. See the workspace Chinese README for full instructions.

Stop SLAM before localization/navigation:
```bash
ros2 launch stm32_robot_navigation navigation.launch.py map:=/absolute/path/room.yaml rviz:=true
```
Set the real initial pose with RViz 2D Pose Estimate before sending a Nav2 goal.
There is no default map or assumed starting pose.

Command chain: controller/behaviors -> cmd_vel_nav -> velocity_smoother ->
cmd_vel_smoothed -> collision_monitor -> cmd_vel -> STM32 bridge.
Only the final output controls hardware. Avoid simultaneous teleop and Nav2
publishers; use one command source, or add an explicit command multiplexer later.
Missing /scan prevents normal navigation and causes the collision monitor to stop.

Current limits are conservative starting values: 0.2 m/s and 0.8 rad/s.
Footprint is derived from the provisional model bounding box, NOT verified body
measurements. Check config/navigation.yaml footprint, inflation, acceleration
and stopping distances against the real chassis before autonomous floor tests.
Wheel track and axle distance are both 0.185 m. A skid-steer base may need effective
track calibration on the actual floor. The bridge publishes measured STM32 odom;
it does not integrate target velocity.

Fake scans/maps used during automated smoke tests live only in /tmp and must
never be used to judge real collision avoidance. Navigation packages can start
without hardware, but real SLAM quality and obstacle avoidance require the lidar.

The launch explicitly manages eight navigation nodes plus AMCL/map_server.
It excludes route_server and docking_server, which are not needed by this robot.
