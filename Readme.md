# Usage
## 1. Required Libraries 
* vtk (A dependency library for PCL installation, need to check Qt during compilation)
* PCL

## 2. Prerequisites
It might be due to some incorrect settings in my publish/subscribe configurations. Using ROS2's default FastDDS causes significant lag during program execution. The reason hasn't been identified yet. Please follow the steps below to change the DDS to cyclonedds.

### 2.1 Install cyclonedds
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 2.2 Change default DDS
```
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 Verify the change
```
ros2 doctor --report | grep "RMW middleware"
```
If the output shows rmw_cyclonedds_cpp, the modification is successful.

## 3. Running the Code (D1 + Gazebo)
### 3.1 Launch Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 3.2 Run the planner
```
ros2 launch ego_planner single_run_d1.launch.py
```
Launch logs are saved by default under `<workspace>/ego_log/` (one file per run; disable with `save_log:=false`).

### 3.3 Run the D1 bridge
```
ros2 launch d1_planner_bridge d1_planner_bridge.launch.py
```

**Launch logs (default on):** `<workspace>/ego_log/` — e.g. `ego_planner/ego_log/ego_planner_single_run_d1_*.log`, `d1_planner_bridge_d1_bridge_*.log`. Not colcon `log/` nor `~/.ros/log`. Override: `log_dir:=/path` or `save_log:=false`.

Defaults: subscribe `/odom` (robot pose) and `/gazebo_obstacles` (world-frame obstacle cloud); publish trajectory commands on `/drone_0_planning/pos_cmd`; bridge outputs `/command/cmd_twist`.

Optional parameters:
```
ros2 launch ego_planner single_run_d1.launch.py \
  odom_topic:=/odom \
  cloud_topic:=/gazebo_obstacles \
  pos_cmd_topic:=/your_d1_cmd_topic \
  flight_type:=1
```
* `flight_type:=1` — set goal in RViz (2D Goal Pose)
* `flight_type:=2` — use preset waypoints from launch file

**Note:** `traj_server` uses odom-synced playback and the FSM uses odom-based goal reach (see `single_run_d1.launch.py`).

# 使用方法
## 1. 需要的库 
* vtk(是安装PCL的依赖库，编译时需要勾选Qt)
* PCL

## 2. 前置条件
可能是我一些发布订阅的设置写的不太对，使用ROS2默认的FastDDS会导致程序运行很卡，目前还没找到原因，所以请按照下述方法将DDS修改为cyclonedds

### 2.1 安装cyclonedds
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 2.2 修改默认的DDS
```
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 检查是否修改成功
```
ros2 doctor --report | grep "RMW middleware"
```
输出显示rmw_cyclonedds_cpp则说明修改成功

## 3. 代码运行（D1 + Gazebo）
### 3.1 运行Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 3.2 运行规划程序
```
ros2 launch ego_planner single_run_d1.launch.py
```
默认会把终端日志写入工程根目录下的 `ego_log/`（每次运行一个文件；关闭：`save_log:=false`）。

### 3.3 运行 D1 桥接
```
ros2 launch d1_planner_bridge d1_planner_bridge.launch.py
```

**日志（默认开启）：** `<工作空间>/ego_log/`，例如 `ego_planner/ego_log/ego_planner_single_run_d1_*.log`。与 colcon 的 `log/`、`~/.ros/log` 无关。可改 `log_dir:=/路径` 或 `save_log:=false`。

默认订阅 `/odom`（机体位姿）与 `/gazebo_obstacles`（世界系障碍点云），输出轨迹指令 `/drone_0_planning/pos_cmd`，桥接节点发布 `/command/cmd_twist`。

可选参数示例：
```
ros2 launch ego_planner single_run_d1.launch.py \
  odom_topic:=/odom \
  cloud_topic:=/gazebo_obstacles \
  pos_cmd_topic:=/your_d1_cmd_topic \
  flight_type:=1
```
* `flight_type:=1` — 在 RViz 用 2D Goal 设目标点
* `flight_type:=2` — 使用 launch 里预设航点

**说明：** 使用 `single_run_d1.launch.py` 时，`traj_server` 按 `/odom` 在轨迹上推进，FSM 按 odom 判到达。
