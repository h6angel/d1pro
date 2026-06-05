# EGO Planner（D1 + Gazebo）

ROS 2 下的 EGO 避障规划，经 `d1_planner_bridge` 转成 D1 的 `cmd_vel`。

**文档**（建议按顺序阅读）：

- [系统总览](docs/00_overview.md) — 感知、规划 FSM、控制数据流
- [规划数学原理](docs/01_planning_math.md) — B 样条优化、2D/odom 改动
- [控制数学原理](docs/02_control_math.md) — 轨迹采样与差速跟踪

## 依赖

ROS 2 Humble、PCL（VTK 编译时勾选 Qt）。仿真需发布 `/odom`、`/gazebo_obstacles`。

## 仿真环境依赖

- [https://github.com/h6angel/d1sim](https://github.com/h6angel/d1sim)

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

### Gazebo 仿真（障碍来自 `/gazebo_obstacles` 点云）

先起 Gazebo + D1，再各开终端 `source install/setup.bash`：

```bash
ros2 launch ego_planner rviz.launch.py              # 可选，RViz 里 2D Goal 设目标
ros2 launch ego_planner single_run_d1.launch.py     # 规划 + traj_server
ros2 launch d1_planner_bridge d1_planner_bridge.launch.py
```

### 实机 OpenVINS + RealSense（深度 + `/ov_msckf/pose_stamped` 建图）

```bash
# 终端 1：相机 + OpenVINS
ros2 launch ov_msckf d435i_openvins.launch.py

# 终端 2：规划（地图系 global，与 OpenVINS 一致）
ros2 launch ego_planner single_run_openvins.launch.py

# 终端 3：桥接（里程计也用 OpenVINS）
ros2 launch d1_planner_bridge d1_planner_bridge.launch.py odom_topic:=/ov_msckf/odomimu

# 可选 RViz：Fixed Frame 选 global
ros2 launch ego_planner rviz.launch.py
```

深度内参请用 `ros2 topic echo /camera/camera/depth/camera_info --once` 核对后覆盖 launch 的 `cx/cy/fx/fy`。

默认：`flight_type:=1` 用 RViz 设目标；`flight_type:=2` 用 launch 预设航点。桥接参数见 `src/d1_planner_bridge/config/d1_bridge.yaml`。

终端日志默认写在工程根目录 `ego_log/`，记录egoplanner的状态、d１机器人的位置以及接收到的速度命令、ego规划的速度。关闭：`save_log:=false`。