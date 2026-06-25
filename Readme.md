# EGO Planner（D1 实机）

ROS 2 下的 EGO 避障规划，经 `d1_planner_bridge` 转成 D1 的 `cmd_vel`。感知建图使用 OpenVINS + RealSense 深度。

**文档**（建议按顺序阅读）：

- [系统总览](docs/00_overview.md) — 感知、规划 FSM、控制数据流
- [规划数学原理](docs/01_planning_math.md) — B 样条优化、2D/odom 改动
- [控制数学原理](docs/02_control_math.md) — 轨迹采样与差速跟踪
- [AprilTag 感知接入](docs/04_apriltag_integration.md) — 检测迁入 ego_control、与规划对接
- [AprilTag 跟随 FSM](APRILTAG_TRACKING_INTEGRATION.md) — 规划侧追踪状态机与参数

## 依赖

ROS 2 Humble、PCL（VTK 编译时勾选 Qt）、OpenVINS（`ov_msckf`）、RealSense D435i。

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## DDS（建议）

FastDDS 易卡顿，改用 CycloneDDS：

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

## 运行

```bash
# 终端 1：相机 + OpenVINS
ros2 launch ov_msckf d435i_openvins.launch.py

# 终端 2：规划（地图系 global，与 OpenVINS 一致）
ros2 launch ego_planner single_run.launch.py

# 终端 3：底盘桥接
ros2 launch d1_planner_bridge d1_planner_bridge.launch.py

# 可选 RViz：Fixed Frame 选 global
ros2 launch ego_planner rviz.launch.py
```

深度内参请用 `ros2 topic echo /camera/camera/depth/camera_info --once` 核对后覆盖 launch 的 `cx/cy/fx/fy`。

**D1 话题与速度上限**（规划、traj_server、bridge 共用）统一在：

`src/planner/plan_manage/config/d1_robot.yaml`

改 `max_vel` / `max_wz` / `max_acc` 或话题名时只改此文件；launch 仍可用 `max_vel:=0.5` 等临时覆盖。

桥接控制调参见 `src/d1_planner_bridge/config/d1_bridge.yaml`。

若使用自定义 VIO，通过 launch 参数覆盖：

```bash
ros2 launch ego_planner single_run.launch.py odom_topic:=/your_vio/odom pose_topic:=/your_vio/pose_stamped
ros2 launch d1_planner_bridge d1_planner_bridge.launch.py odom_topic:=/your_vio/odom
```

一键启动时日志写在 `ego_log/stack_YYYYMMDD_HHMMSS/`（见 `start_ego_stack.sh`）。
