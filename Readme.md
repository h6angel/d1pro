# D1 实机规划与控制（ego_control）

ROS 2 下的在线避障规划（动态 A\* + B 样条 L-BFGS）与差速执行：经 `d1_planner_bridge` 转成 D1 的 `cmd_vel`。感知建图使用 OpenVINS + RealSense 深度。

**文档**（建议按顺序阅读）：

- [Demo 指南](docs/05_demo.md) — 架构、流程、实机演示
- [系统总览](docs/00_overview.md) — 感知、规划 FSM、控制数据流
- [规划数学原理](docs/01_planning_math.md) — B 样条优化、2.5D / odom 改动
- [控制数学原理](docs/02_control_math.md) — 轨迹采样与差速跟踪
- [地面障碍建模](docs/06_ground_obstacle_modeling.md) — 地面滤波与高度柱
- [改进路线图](docs/todo.md) — 算法学习与落地顺序
- [AprilTag 感知接入](docs/04_apriltag_integration.md)
- [AprilTag 跟随 FSM](APRILTAG_TRACKING_INTEGRATION.md)

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

```bash
cd ego_control
./start_ego_stack.sh                        # 默认：RViz 手动 2D Goal
./start_ego_stack.sh enable_tag_tracking=true  # AprilTag 追踪模式
./start_ego_stack.sh --no-rviz
./start_ego_stack.sh --skip-wait              # 跳过话题就绪检测（调试用）
```

脚本内部依次拉起：

1. `realsense2_camera`（`rs_launch.py`）
2. OpenVINS（`ov_msckf subscribe.launch.py config:=rs_d435i use_stereo:=true max_cameras:=2`）
3. [可选] `apriltag_detect`（`enable_tag_tracking=true` 时）
4. 规划栈（`src/planner/plan_manage/launch/single_run.launch.py`）
5. `d1_planner_bridge`
6. [可选] RViz（Fixed Frame 选 `global`）

日志目录：`ego_log/stack_YYYYMMDD_HHMMSS/`。

## 手动分终端（调试）

需要单独重启某一环时，按 `start_ego_stack.sh` 内顺序分别启动 RealSense、OpenVINS、规划 launch、桥接、可选 RViz。AprilTag 追踪时额外：`ros2 launch apriltag_detect apriltag.launch.py`，并给规划传 `enable_tag_tracking:=true`。

深度内参请用 `ros2 topic echo /camera/camera/depth/camera_info --once` 核对后写入 `d1_robot.yaml` 的 `camera` 段。

**统一配置源：**

`src/planner/plan_manage/config/d1_robot.yaml`

含话题、限速、FSM、GridMap、优化权重、traj_server 等；由 `d1_robot_config.py` 注入节点。launch CLI 可临时覆盖部分项（如 `max_vel:=0.5`）。

桥接调参见 `src/d1_planner_bridge/config/d1_bridge.yaml`（航向增益、看门狗超时等）。

后续工程项见 [REMAINING_PRS.md](REMAINING_PRS.md)。

若使用自定义 VIO，通过 launch 参数覆盖 `odom_topic` / `pose_topic`（写法见 `start_ego_stack.sh` 与各 launch 的 DeclareLaunchArgument）。
