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

## 3. Running the Code
### 3.1 Launch Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 3.2 Run the planning program (single robot / D1 + Gazebo)
Open a new terminal and execute:
```
ros2 launch ego_planner single_run_in_sim.launch.py
```
For a **real D1** (slow ground robot), prefer:
```
ros2 launch ego_planner single_run_d1.launch.py
```
(`max_vel:=0.6`, position-based goal finish; also run `d1_planner_bridge`)

Defaults: subscribe `/odom` (robot pose) and `/gazebo_obstacles` (world-frame obstacle cloud); publish trajectory commands on `/drone_0_planning/pos_cmd`.

Optional parameters:
```
ros2 launch ego_planner single_run_in_sim.launch.py \
  odom_topic:=/odom \
  cloud_topic:=/gazebo_obstacles \
  pos_cmd_topic:=/your_d1_cmd_topic \
  flight_type:=1
```
* `flight_type:=1` — set goal in RViz (2D Goal Pose)
* `flight_type:=2` — use preset waypoints from launch file
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

## 3. 代码运行
### 3.1 运行Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 3.2 运行规划程序（单机 D1 + Gazebo）
新开一个终端：
```
ros2 launch ego_planner single_run_in_sim.launch.py
```
**真机 D1** 建议使用（规划速度与机体一致、到点才结束）：
```
ros2 launch ego_planner single_run_d1.launch.py
```
并同时运行 `d1_planner_bridge`。

**问题与修复（D1）**：规划线按“时钟”走完但车未到时，使用 `single_run_d1.launch.py`（`traj_server` 按 `/odom` 在轨迹上推进，FSM 按 odom 判到达）。

默认订阅 `/odom`（机体位姿）与 `/gazebo_obstacles`（世界系障碍点云），输出轨迹指令 `/drone_0_planning/pos_cmd`。

可选参数示例：
```
ros2 launch ego_planner single_run_in_sim.launch.py \
  odom_topic:=/odom \
  cloud_topic:=/gazebo_obstacles \
  pos_cmd_topic:=/your_d1_cmd_topic \
  flight_type:=1
```
* `flight_type:=1` — 在 RViz 用 2D Goal 设目标点
* `flight_type:=2` — 使用 launch 里预设航点