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
- 规划 **仅用 position**，`orientation` 预留。

---

## 2. 规划目标与停车判定

```
T = Tag 中心 (global)
G = T                          # 写入 end_pt_，规划目标即 Tag 中心
G.z = odom_pos_.z()            # D1 平面规划，与 RViz goal 一致
```

**停车判定（追踪模式专用）：** 机器人到 Tag 中心的 XY 距离

```
|R.xy - T.xy| ≤ tag_stop_dist   # 默认 0.25 m（d1_robot.yaml / launch）
```

与手动模式的 `goal_reach_thresh`（默认 0.3 m）不同：追踪模式以 **距 Tag 中心** 为准，而非「到达规划点 G」。

---

## 3. 追踪状态（`enable_tag_tracking=true`）

| 状态 | 条件 | 行为 |
|------|------|------|
| `NEVER_SEEN` | 从未 `detected=true` | `WAIT_TARGET`，不规划 |
| `ACTIVE` | `detected=true` | `G=T`，节流后 `planNextWaypoint(G)`，**持续跟随，不因到达 G 而停车** |
| `HOLD` | 曾检测过，`detected=false` | 冻结 `T_last`，目标 `G=T_last`，继续朝冻结位置执行 |
| `DONE` | 距 Tag（或冻结位置）≤ `tag_stop_dist` | 紧急停车 → 速度归零 → `WAIT_TARGET` |

补充：

- **ACTIVE**：Tag 在视野内且机器人距 Tag ≤ `tag_stop_dist` → `finishTagTracking` → `EMERGENCY_STOP` → `WAIT_TARGET`。
- **HOLD**：目标丢失后朝最后已知 Tag 位置继续走；**无超时**；距冻结 Tag ≤ `tag_stop_dist` 时同样停车收尾。
- **HOLD 中 Tag 重现**：回到 `ACTIVE`，强制重规划，继续跟随新位置。
- 重规划节流：`|G_new - end_pt_| > tag_update_min_dist`（默认 0.08 m）或距上次重规划 > `tag_replan_min_period`（默认 0.5 s）才 `planNextWaypoint`。

**FSM 改动点：** 追踪模式下 `isTagFollowing()` 为 true 时，跳过 `EXEC_TRAJ` 里「到达 goal → WAIT_TARGET」；`waypointCallback` 在追踪模式下直接 return。

---

## 4. 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `fsm/enable_tag_tracking` | `false` | 追踪开关 |
| `fsm/tag_pose_topic` | `/apriltag/target_pose_global` | Tag 位姿话题 |
| `fsm/tag_detected_topic` | `/apriltag/target_detected` | 检测标志话题 |
| `fsm/tag_stop_dist` | `0.25` | 距 Tag 中心 XY 距离 ≤ 此值则停车 (m) |
| `fsm/tag_update_min_dist` | `0.08` | 目标位移阈值，超过才重规划 (m) |
| `fsm/tag_replan_min_period` | `0.5` | 最小重规划间隔 (s) |

`tag_stop_dist` 在 `config/d1_robot.yaml` 的 `planner.tag_stop_dist` 中配置，由 `single_run.launch.py` 传入。

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

**AprilTag 感知节点**（`apriltag_ros` + `target_pose_node`）的迁入步骤见 [docs/04_apriltag_integration.md](docs/04_apriltag_integration.md)。

| 模式 | 设目标方式 |
|------|------------|
| 默认 | RViz → 2D Goal Pose |
| `enable_tag_tracking=true` | Tag 进入视野后自动跟随；未检测时保持 `WAIT_TARGET` |

---

## 6. 核心代码（`EGOReplanFSM`）

**文件：** `ego_replan_fsm.h` / `ego_replan_fsm.cpp`，`single_run.launch.py`，`config/d1_robot.yaml`

```cpp
Eigen::Vector3d computeFollowGoal(const Eigen::Vector3d &tag_pos) const {
  Eigen::Vector3d goal = tag_pos;
  if (have_odom_)
    goal.z() = odom_pos_.z();
  return goal;
}

bool isCloseToTag(const Eigen::Vector3d &tag_pos) const {
  return (odom_pos_.head<2>() - tag_pos.head<2>()).norm() <= tag_stop_dist_;
}
```

停车流程：`updateTagTrackingOnExecTick()` 检测 `isCloseToTag` → `finishTagTracking()` → `EMERGENCY_STOP` → `odom_vel < 0.1` → `WAIT_TARGET`（不走 `GEN_NEW_TRAJ` fail_safe 重规划）。

---

## 7. 注意

- 规划目标是 **Tag 中心**，机器人会走到 Tag 附近（`tag_stop_dist`）再停；实机请根据 Tag 尺寸与安全距离调整 `tag_stop_dist`。
- pose / detected 可能不同步：以 `target_detected` 触发状态更新为准。
- RViz `/drone_0_plan_vis/goal_point` 可观察当前规划点 `G`。

---

**版本：** v1.3 · **状态：** 已实现（`EGOReplanFSM` + launch）
