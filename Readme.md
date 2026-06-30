# EGO Planner（D1 实机）

ROS 2 下的 EGO 避障规划，经 `d1_planner_bridge` 转成 D1 的 `cmd_vel`。感知建图使用 OpenVINS + RealSense 深度。

**文档**（建议按顺序阅读）：

- [Demo 指南](docs/05_demo.md) — 架构、流程、关键技术点与实机演示（图文并茂）
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

## 运行（推荐）

实机默认使用仓库根目录一键脚本（RealSense + OpenVINS + 规划 + D1 桥接 + 可选 RViz / AprilTag）：

```bash
cd ego_control
./start_ego_stack.sh                        # 默认：RViz 手动 2D Goal
./start_ego_stack.sh enable_tag_tracking=true  # AprilTag 追踪模式
./start_ego_stack.sh --no-rviz              # 不启动 RViz
./start_ego_stack.sh --skip-wait              # 跳过话题就绪检测（调试用）
```

脚本内部依次拉起：

1. `realsense2_camera`（`rs_launch.py`）
2. OpenVINS（`ov_msckf subscribe.launch.py config:=rs_d435i use_stereo:=true max_cameras:=2`）
3. [可选] `apriltag_detect`（`enable_tag_tracking=true` 时）
4. `ego_planner single_run.launch.py`
5. `d1_planner_bridge`
6. [可选] RViz（Fixed Frame 选 `global`）

日志目录：`ego_log/stack_YYYYMMDD_HHMMSS/`（各节点独立 `.log`）。

## 手动分终端（调试）

需要单独重启某一环时，与脚本等价的手动顺序：

```bash
# 终端 1：RealSense
ros2 launch realsense2_camera rs_launch.py

# 终端 2：OpenVINS（需已 source ../openvins/install/setup.bash）
ros2 launch ov_msckf subscribe.launch.py config:=rs_d435i use_stereo:=true max_cameras:=2

# 终端 3：规划
ros2 launch ego_planner single_run.launch.py

# 终端 4：底盘桥接
ros2 launch d1_planner_bridge d1_planner_bridge.launch.py

# 可选 RViz
ros2 launch ego_planner rviz.launch.py
```

AprilTag 追踪时，在规划前增加：`ros2 launch apriltag_detect apriltag.launch.py`，并给规划传 `enable_tag_tracking:=true`。

深度内参请用 `ros2 topic echo /camera/camera/depth/camera_info --once` 核对后覆盖 launch 的 `cx/cy/fx/fy`。

**D1 话题与速度上限**（规划、traj_server、bridge 共用）统一在：

`src/planner/plan_manage/config/d1_robot.yaml`

改 `max_vel` / `max_wz` / `max_acc` 或话题名时只改此文件；launch 仍可用 `max_vel:=0.5` 等临时覆盖。

**GridMap、FSM 细项、优化器权重**（如 `grid_map/resolution`、`optimization/lambda_*`）仍在 `single_run.launch.py` 硬编码，调参需改 launch 或后续迁入 yaml。

桥接控制调参见 `src/d1_planner_bridge/config/d1_bridge.yaml`（含 `hard_stop_plan_speed`，默认 0.005 m/s）。

后续 PR 与文档进度见 [REMAINING_PRS.md](REMAINING_PRS.md)。

若使用自定义 VIO，通过 launch 参数覆盖：

```bash
ros2 launch ego_planner single_run.launch.py odom_topic:=/your_vio/odom pose_topic:=/your_vio/pose_stamped
ros2 launch d1_planner_bridge d1_planner_bridge.launch.py odom_topic:=/your_vio/odom
```
