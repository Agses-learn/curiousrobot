# 上位机软件验证记录

日期：2026-11-03。环境：Ubuntu 24.04.4、ROS 2 Jazzy、Python 3.12.3。

## 已通过

- 工作空间 5 个 ROS 包全部编译完成（description、bridge、bringup、navigation、ldlidar）。
- colcon test：12 tests、0 errors、0 failures、0 skipped。
- 串口集成：真实桥接节点通过 PTY 接收 v3 遥测、发布传感器数据并驱动 EKF；断流、串口重连、MCU 时间复位、电机故障均清除旧速度。
- 导航：AMCL、地图服务器和八个导航服务进入 active；实际规划器产生有效路径；Twist 速度链路贯通；持续输入速度时扫描失效仍触发停车。
- SLAM：在具有一致里程计、TF 和合成扫描的短距离移动测试中生成包含障碍单元的地图。
- 离线 demo：启动模拟下位机、串口桥和 EKF，持续输出 /odom，串口诊断及 EKF 诊断正常。
- STL-06N：厂商驱动已编译；启动确认产品名 LDLiDAR_STL06N、230400 波特率、/scan 和 laser 坐标系。使用不存在的临时测试端口时正确报告无法打开设备；此项不算硬件数据验证。

## 验证范围

测试使用隔离 ROS 域 73、74、75；没有向实车电机发送指令，没有使用真实雷达数据。
测试结束后停止测试 launch 及子进程。原始 Linorobot2 已跟踪源码无修改，原有 build/install 为构建产物。
rosdep check 因当前机器尚未初始化 rosdep 数据库而未完成；实际使用的依赖已通过编译与运行测试确认。
待接硬件后验证：USB 透传与稳定设备名、轮子方向、真实遥测单位与 IMU 安装轴向、雷达扫描、里程计尺度、定位建图质量、导航停止距离。
车体尺寸、传感器安装位置及导航 footprint 仍需实测，轴距/轮距已采用 0.185 m。

## 复现

先 source /opt/ros/jazzy/setup.bash 和工作空间 install/setup.bash，再按 README 执行测试。
测试脚本位于 tests/，不要在相同 ROS 域连接实车。

### 串口集成输出

```
PASS valid telemetry -> raw odom / IMU -> EKF /odom and TF
PASS signed command encoding and upstream watchdog
PASS invalid flags and corrupt frames do not publish measurements
PASS telemetry loss stops despite incoming cmd_vel; resumption keeps zero
PASS MCU clock reset clears command
PASS serial reconnect requires a fresh command
PASS motor fault inhibits motion
ALL INTEGRATION CHECKS PASSED; logs: /tmp/curiousrobot-integration-2ykwwav0
```

### 导航与建图输出

```
PASS all 10 Nav2/localization lifecycle nodes active
PASS planner produces a path through the temporary map
PASS cmd_vel_nav -> smoother -> collision monitor -> cmd_vel (Twist)
PASS scan loss stops final velocity while input commands continue
PASS SLAM Toolbox produces an occupancy map from synthetic scans
ALL NAVIGATION RUNTIME CHECKS PASSED; logs: /tmp/curiousrobot-nav-check-hkr1kuqy
```
