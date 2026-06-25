# AprilTag 目标追踪接入方案

在现有 **EGOReplanFSM** 上扩展 AprilTag 跟随，复用 `planNextWaypoint()` 及后续避障 / 控制链路，**不改动** `d1_planner_bridge`、`traj_server`。

| 模式 | 开关 | 目标来源 |
|------|------|----------|
| 手动（默认） | `enable_tag_tracking=false` | RViz 2D Goal → `/move_base_simple/goal` |
| 追踪 | `enable_tag_tracking=true` | AprilTag 话题（忽略 RViz） |

开关在 **launch 时固定**，不支持运行时切换。

---

## 1. 输入话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/apriltag/target_pose_global` | `PoseStamped` | Tag 在 `global` 系下位姿，~30 Hz；`frame_id` 须为 `global` |
| `/apriltag/target_detected` | `Bool` | `true` 检测到 / `false` 未检测到 |

- 以 **`target_detected` 为准** 判断有无目标；pose 回调只缓存，检测到后再用最新 pose。
- 首版规划 **仅用 position**，`orientation` 预留。

---

## 2. 规划目标

```
T = Tag 中心 (global)
O = (tag_follow_offset_x, tag_follow_offset_y, tag_follow_offset_z)  # launch 配置，global 系固定向量
G = T + O                                                              # 写入 end_pt_
G.z = odom_pos_.z()                                                    # D1 平面规划，与 RViz goal 一致
```

默认偏移 `O = (0, -0.3, 0)`：Tag 后方 0.3 m 为跟随点，不直接朝 Tag 中心走。

**丢失后收尾判定：** `|R.xy - G_last.xy| ≤ thresh_goal_reach_meter`（默认 0.3 m）。

---

## 3. 追踪状态（`enable_tag_tracking=true`）

| 状态 | 条件 | 行为 |
|------|------|------|
| `NEVER_SEEN` | 从未 `detected=true` | `WAIT_TARGET`，不规划 |
| `ACTIVE` | `detected=true` | `G=T+O`，节流后 `planNextWaypoint(G)`，**持续跟随，不因到达 G 而停车** |
| `HOLD` | 曾检测过，`detected=false` | 冻结 `T_last`，目标 `G_last=T_last+O`，继续执行 |
| `DONE` | `HOLD` 且丢失 ≥30 s **且** 已到 `G_last` | 清目标 → `WAIT_TARGET` |

补充：

- `HOLD` 中若 30 s 已到但未到达 `G_last`：继续走，到达后立即 `WAIT_TARGET`。
- `HOLD` 中 Tag 重现：回到 `ACTIVE`，重置丢失计时。
- 30 Hz 更新节流：`|G_new - end_pt_| > 0.08 m` 或距上次重规划 > 0.5 s 才 `planNextWaypoint`。

**FSM 必改点：** 追踪模式下跳过 `EXEC_TRAJ` 里「到达 goal → WAIT_TARGET」；`waypointCallback` 在追踪模式下直接 return。

---

## 4. 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `fsm/enable_tag_tracking` | `false` | 追踪开关 |
| `fsm/tag_pose_topic` | `/apriltag/target_pose_global` | |
| `fsm/tag_detected_topic` | `/apriltag/target_detected` | |
| `fsm/tag_follow_offset_x/y/z` | `0 / -0.3 / 0` | global 系偏移向量 (m) |
| `fsm/tag_update_min_dist` | `0.08` | 目标位移阈值 (m) |
| `fsm/tag_replan_min_period` | `0.5` | 最小重规划间隔 (s) |
| `fsm/tag_lost_timeout_sec` | `30.0` | 丢失后最短等待 (s) |

`fsm/flight_type` 保持 `1`（MANUAL_TARGET）。

---

## 5. 启动方式

使用仓库根目录 **`start_ego_stack.sh`** 一键启动（RealSense + OpenVINS + EGO + D1 桥接 + RViz）。

```bash
# 手动模式：RViz 里用 2D Goal 设点（默认）
./start_ego_stack.sh

# AprilTag 追踪模式
./start_ego_stack.sh enable_tag_tracking=true

# 其它已有选项可组合
./start_ego_stack.sh enable_tag_tracking=true --no-rviz
./start_ego_stack.sh --no-rviz --skip-wait
```

脚本将 `enable_tag_tracking` 传给 `ros2 launch ego_planner single_run.launch.py`。  

**AprilTag 感知节点**（`apriltag_ros` + `target_pose_node`）的迁入步骤、目录结构与 `start_ego_stack.sh` 改法见 [docs/04_apriltag_integration.md](docs/04_apriltag_integration.md)。  
当前脚本尚未自动启动 AprilTag；迁入后应在 `enable_tag_tracking=true` 时由脚本一并拉起。

| 模式 | 设目标方式 |
|------|------------|
| 默认 | RViz → 2D Goal Pose |
| `enable_tag_tracking=true` | Tag 进入视野后自动跟随；未检测时保持 `WAIT_TARGET` |

---

## 6. 代码改动（`EGOReplanFSM`）

**文件：** `ego_replan_fsm.h` / `ego_replan_fsm.cpp`，`single_run.launch.py`，`config/d1_robot.yaml`，`start_ego_stack.sh`

1. `init()`：读 §4 参数；追踪开启时订阅 pose + detected。
2. `tagPoseCallback`：缓存 `T`。
3. `tagDetectedCallback`：驱动 §3 状态，调用 `computeFollowGoal()` → `applyTagGoal()` → `planNextWaypoint()`。
4. `execFSMCallback` 开头：`HOLD` 下检查 30 s + `at_follow_goal` → `DONE`。
5. 追踪模式：禁用 RViz goal、禁用「到达即停」。

核心计算：

```cpp
Eigen::Vector3d computeFollowGoal(const Eigen::Vector3d & tag_pos) const {
  Eigen::Vector3d g = tag_pos + tag_follow_offset_;
  g.z() = odom_pos_.z();
  return g;
}
```

---

## 7. 注意

- `O` 在 **global 系**固定，不随 Tag 朝向旋转；需相对 Tag 朝向的偏移要后续扩展。
- `O=0` 会顶到 Tag 中心，勿用于实机。
- pose / detected 可能不同步：以 detected 触发更新为准。
- RViz `/drone_0_plan_vis/goal_point` 可观察当前规划点 `G`。

---

**版本：** v1.2 · **状态：** 已实现（`EGOReplanFSM` + launch）
