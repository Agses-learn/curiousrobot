# curiousrobot_description

ROS 2 Jazzy description based on Linorobot2's four-wheel skid-steer layout.
This package is standalone: no Linorobot2 overlay or micro-ROS is needed.

## Build and preview (Ubuntu desktop terminal)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/work/curiousrobot
colcon build --symlink-install --packages-select curiousrobot_description
source install/setup.bash
ros2 launch curiousrobot_description display.launch.py
```

Description only:
```bash
ros2 launch curiousrobot_description description.launch.py
```

Headless preview with placeholder wheel states:
```bash
ros2 launch curiousrobot_description description.launch.py publish_joints:=true
```

The display launch publishes zero wheel joint positions for visualization.
These are NOT encoder measurements. Disable placeholder joint states when a
real joint-state publisher is introduced. This package does not publish odom
or the odom -> base_footprint transform; the future EKF owns that transform.

## Geometry

Edit urdf/curiousrobot_properties.urdf.xacro. Units: metres, kilograms, radians.
Values copied from current STM32 chassis_config.h: wheel diameter 0.065,
left-right wheel track 0.185. They are configured values, not measurements
verified by this package.

User confirmed front/rear axle distance and left/right wheel track are both
0.185 m. wheel_pos_x = wheelbase / 2 = 0.0925 m; wheel_pos_y = 0.0925 m.

Body dimensions/masses, wheel width, wheel centre height and IMU pose remain provisional:
- Body length/width/height: 0.300 / 0.140 / 0.100.
- Wheel width: 0.030.
- Body/wheel mass: 5.0 / 0.25.
- Wheel centre z relative to body: -0.0325.
- IMU pose relative to base_link: xyz 0 0 0.050, rpy 0 0 0.

Measure these before using the model for collision geometry or simulation.
The body origin is its centre and the midpoint between the axles.
x forward, y left, z up. base_footprint is on the ground.
A/B/C/D map to front_left/front_right/rear_left/rear_right.
No lidar or camera frame is included until mounting details are available.

## Files

- urdf/robots/curiousrobot_4wd.urdf.xacro: model entry point.
- urdf/curiousrobot_properties.urdf.xacro: geometry and IMU parameters.
- urdf/mech/: body and wheel macros.
- launch/description.launch.py: robot_state_publisher and optional preview.
- launch/display.launch.py: RViz preview.
- rviz/description.rviz: RViz configuration.

No Gazebo controller, ros2_control hardware interface, or motor commands are
included. Real control will use the STM32 serial bridge.
See NOTICE and LICENSE for upstream attribution. The package maintainer email
is a placeholder to replace before publication.
