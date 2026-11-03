# CuriousRobot 上位机（Ubuntu 24.04 / ROS 2 Jazzy）

工作空间：`/home/zhangkaiqi/work/curiousrobot`。
参考项目：`/home/zhangkaiqi/work/linorobot2`，不需要同时启动或 source 参考工作空间。

## 包与职责

| 包 | 功能 |
| --- | --- |
| curiousrobot_description | 四轮底盘 xacro、robot_state_publisher、RViz 模型预览 |
| stm32_base_bridge | STM32 二进制串口协议、速度下发、里程计/IMU 发布、超时停车、模拟下位机 |
| stm32_robot_bringup | 模型、串口、EKF、可选 STL-06N 雷达统一启动 |
| stm32_robot_navigation | SLAM Toolbox 建图、AMCL 定位、Nav2 规划/控制/速度平滑/碰撞监测 |
| ldlidar（src/stl06n_vendor 内） | LDROBOT STL-06N 厂商驱动 |

STM32 保留底盘运动学、轮速控制和原始里程计计算。上位机不重复计算电机目标。
STM32 USART2 使用 115200 8N1；STL-06N 使用独立串口、230400 波特率。

## 编译与环境

在只加载 ROS Jazzy 的新终端编译，避免反复叠加同一个工作空间：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zhangkaiqi/work/curiousrobot
colcon build --symlink-install
source install/setup.bash
```

以后每个运行终端先执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/zhangkaiqi/work/curiousrobot/install/setup.bash
```

当前虚拟机已安装本项目实际使用的依赖，但 rosdep 数据库尚未初始化。
在新的 Jazzy 环境，先完成一次 `sudo rosdep init` 与 `rosdep update`，
再在工作空间运行 `rosdep install --from-paths src --ignore-src -r -y`。
已初始化的机器无需再次运行 init。
配置文件修改后重启对应 launch 生效。串口桥参数在启动时读取，不支持热更新。

## 1. 不接硬件演示

在 Ubuntu 桌面终端启动（RViz 需要图形会话）：

```bash
ros2 launch stm32_robot_bringup demo.launch.py rviz:=true
```

该入口创建独立 PTY 模拟 STM32，同时运行实际串口桥和 EKF。
默认不启动雷达、建图或导航。纯 SSH 可省略 `rviz:=true`。

另一个终端可发送下面的短时命令，观察模型位姿变化；这条命令只用于上述离线演示：

```bash
timeout --signal=INT 3s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.2}}"
ros2 topic echo /odom --once
ros2 topic echo /diagnostics --once
```

停止发布后，上位机命令超时会置零。演示中车轮关节固定在零位；
当前 STM32 遥测没有四个轮子的独立累计编码器值，不能把可视化轮角当作真实反馈。
模拟器仅用于验证软件链路，不模拟轮胎打滑或真实避障。

## 2. 接入 STM32 和 STL-06N

先在 VMware 的“可移动设备”中将两个 USB 串口连接到 Ubuntu。
在 Ubuntu 检查：

```bash
ls -l /dev/serial/by-id/
groups
```

用户应属于 `dialout`；更改用户组后需要重新登录。端口通过设备名称识别，
不要根据插拔顺序假定哪一个是 `ttyUSB0`。当前并未创建
`/dev/stm32_motor`、`/dev/stl06n` 的 udev 别名，下面必须填实际路径。

先只启动底盘：

```bash
ros2 launch stm32_robot_bringup bringup.launch.py \
  port:=/dev/serial/by-id/实际的STM32串口设备 \
  rviz:=true
```

同时启动雷达：

```bash
ros2 launch stm32_robot_bringup bringup.launch.py \
  port:=/dev/serial/by-id/实际的STM32串口设备 \
  lidar:=true \
  lidar_port:=/dev/serial/by-id/实际的STL06N串口设备 \
  rviz:=true
```

也可单独检查雷达：

```bash
ros2 launch stm32_robot_bringup lidar.launch.py port:=/dev/serial/by-id/实际的STL06N串口设备
ros2 topic hz /scan
ros2 topic echo /scan --once --qos-reliability best_effort
```

不要同时使用独立雷达 launch 和 bringup 的 `lidar:=true` 打开同一端口。
STL-06N 话题为 `/scan`，坐标系为 `laser`。
雷达安装位置通过 bringup 的 `lidar_x/y/z/roll/pitch/yaw` 参数设置，
独立雷达 launch 对应 `x/y/z/roll/pitch/yaw`。单位为米、弧度，相对 `base_link`。
默认位置 (0, 0, 0.10)、姿态 (0, 0, 0) 只是占位，必须按实际安装修改。

## 3. 建图

保持实车 bringup 和雷达运行，在另一个终端启动：

```bash
ros2 launch stm32_robot_navigation slam.launch.py rviz:=true
```

该入口只运行建图，不运行 Nav2 控制器。低速键盘控制：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.4
```

当前安装的 Jazzy 键盘节点只在收到按键时发布，未实现 ROS1 的
`repeat_rate/key_timeout` 参数。按住方向键依赖系统键盘重复输入；
首次长按的重复延迟可能导致短暂停顿，这是 0.2 秒命令超时停车的结果。
键盘重复间隔需小于 0.2 秒，松开后停止更新即可触发停车，`k` 可显式发送停止。
运行时保持终端焦点。不要为了让单次按键一直运动而关闭命令超时。

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/zhangkaiqi/work/curiousrobot/maps/room
```

生成 `room.yaml` 和地图图像。先退出键盘节点并停止 SLAM，再启动下面的定位导航。

## 4. 已有地图导航

保持底盘与雷达 bringup 运行：

```bash
ros2 launch stm32_robot_navigation navigation.launch.py \
  map:=/home/zhangkaiqi/work/curiousrobot/maps/room.yaml \
  rviz:=true
```

在 RViz 用 **2D Pose Estimate** 指定真实初始位姿，确认激光与地图对齐，
再通过 **Nav2 Goal** 设置目标。没有默认地图，也不会假定机器人在地图原点。

速度链路：
`controller/behaviors → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → collision_monitor → /cmd_vel → STM32`。
最终接口是 `geometry_msgs/msg/Twist`。
本项目显式启动所需的八个导航服务和两个定位节点，不启动充电桩对接、路线图服务。
不要同时让键盘和 Nav2 向底盘发布速度；尚未加入多控制源仲裁器。
手动控制直达串口桥，不经过 Nav2 碰撞监测。
自动导航配置速度上限为 0.2 m/s、0.8 rad/s；真实停止距离仍需实测。

## 接口与坐标系

| 接口 | 类型/用途 |
| --- | --- |
| /cmd_vel | geometry_msgs/Twist，线速度 x (m/s)、角速度 z (rad/s) |
| /odom/unfiltered | nav_msgs/Odometry，STM32 回传的原始位姿与速度 |
| /imu/data | sensor_msgs/Imu，SI 单位的加速度/角速度；无姿态估计 |
| /imu/temperature | sensor_msgs/Temperature，摄氏度 |
| /odom | nav_msgs/Odometry，EKF 融合结果 |
| /scan | sensor_msgs/LaserScan，STL-06N 扫描 |
| /diagnostics | 串口连接、遥测时效、CRC、序号缺失、传感器有效位及故障状态 |

TF：`map → odom → base_footprint → base_link → imu_link / laser / wheels`。
`map → odom` 由 SLAM 或 AMCL 二选一发布；
`odom → base_footprint` 由 EKF 发布；串口桥不发布 TF。
EKF 使用底盘 vx/vy/wz 和 IMU wz，不重复融合由相同轮速积分的原始位置。
原始 IMU 不提供朝向，orientation_covariance[0] 为 -1；
消息时间采用主机接收时刻，尚无 MCU 与主机时钟同步。

## 故障处理与待标定项目

- 串口桥以 20 Hz 下发命令；0.2 秒收不到新速度后发送零速度。
- 遥测超过 0.5 秒未更新、串口断开、MCU 重启或电机故障会清除旧命令。
  恢复通信后需重新收到有效速度。下位机另有 300 ms 命令接收超时。
- Nav2 碰撞监测在扫描超时 0.3 秒后停止输出运动命令。
- 已确认轴距和左右轮距均为 0.185 m。轮径 0.065 m 来自当前固件配置。
- 车体长宽高、轮宽、质量、IMU 安装姿态、雷达外参仍为初始值。
  导航 footprint 由临时模型外轮廓计算，真实上车前需复核。
- 四轮差速底盘的有效轮距、轮径、IMU 零偏、协方差、加减速和停止距离需要实测标定。
- 当前 USB 设备尚未接入虚拟机；软件验证不代表已验证真实电机、IMU、雷达和自主避障。

## 验证与源码来源

```bash
cd /home/zhangkaiqi/work/curiousrobot
colcon test --packages-select stm32_base_bridge
colcon test-result --verbose
python3 tests/integration_check.py
python3 tests/navigation_check.py
python3 tests/launch_check.py
```

单元测试检查协议、拆包、CRC、单位换算和速度超时。
集成脚本使用独立 ROS 域 73/74/75、PTY、临时地图和合成激光，不打开实体串口。
测试覆盖串口桥/EKF、通信恢复、故障停车及导航启动/规划/扫描失效。
临时测试日志保留在 `/tmp/curiousrobot-*`；不要在域 73/74/75 上连接实车运行测试。

描述模型、导航配置与 RViz 配置参考 Linorobot2，保留 LICENSE/NOTICE。
雷达驱动来自 https://github.com/ldrobotsensor/stl-06n ，固定提交
`b7a7d8c862355771d1a233f37e64b9afd01fd831`。
`src/stl06n_vendor/LOCAL_CHANGES.md` 记录 Ubuntu 24.04 所需的头文件兼容修复。


本次验证结果及未覆盖范围见 [docs/validation.md](docs/validation.md)。
