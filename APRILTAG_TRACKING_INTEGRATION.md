# AprilTag 目标追踪接入方案

在现有重规划 FSM（`ego_replan_fsm.cpp`）上扩展 AprilTag 跟随，复用 `planNextWaypoint()` 及后续避障 / 控制链路，**不改动** `d1_planner_bridge`、`traj_server`。

| 模式 | 开关 | 目标来源 |
|------|------|----------|
| 手动（默认） | `enable_tag_tracking=false` | RViz 2D Goal → `/move_base_simple/goal` |
| 追踪 | `enable_tag_tracking=true` | AprilTag 话题（忽略 RViz） |

开关在 **launch 时固定**，不支持运行时切换。

---

## 1. 输入话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/apriltag/target_pose_global` | `PoseStamped` | Tag 在 `global` 系，~30 Hz |
| `/apriltag/target_detected` | `Bool` | 是否检测到 |

- 以 **`target_detected` 为准**；pose 回调只缓存。
- 规划 **仅用 position**。

---

## 2. 规划目标与停车判定

```
T = Tag 中心 (global)
G = T                          # end_pt_
G.z = odom_pos_.z()            # 平面规划
```

**停车（追踪模式）：** `|R.xy - T.xy| ≤ tag_stop_dist`（默认 0.25 m）

与手动模式 `goal_reach_thresh`（0.3 m）不同：追踪以 **距 Tag 中心** 为准。

---

## 3. 追踪状态（`enable_tag_tracking=true`）

| 状态 | 条件 | 行为 |
|------|------|------|
| `NEVER_SEEN` | 从未 detected | `WAIT_TARGET` |
| `ACTIVE` | detected | `G=T`，节流后 `planNextWaypoint`，持续跟随 |
| `HOLD` | 曾检测过现丢失 | 冻结 `T_last`，继续朝冻结点 |
| `DONE` | 距 Tag ≤ `tag_stop_dist` | 紧急停车 → `WAIT_TARGET` |

- ACTIVE / HOLD 到达停车距 → `finishTagTracking` → `EMERGENCY_STOP` → `WAIT_TARGET`（不走 fail_safe 重规划）。
- HOLD 中 Tag 重现 → `ACTIVE`，强制重规划。
- 节流：`tag_update_min_dist`（0.08 m）或 `tag_replan_min_period`（0.5 s）。

追踪模式下跳过「到达 goal → WAIT_TARGET」；`waypointCallback` 直接 return。

---

## 4. 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `fsm/enable_tag_tracking` | false | 追踪开关 |
| `fsm/tag_pose_topic` | `/apriltag/target_pose_global` | |
| `fsm/tag_detected_topic` | `/apriltag/target_detected` | |
| `fsm/tag_stop_dist` | 0.25 | 停车距 (m) |
| `fsm/tag_update_min_dist` | 0.08 | 位移阈值 |
| `fsm/tag_replan_min_period` | 0.5 | 最小重规划间隔 (s) |

`tag_stop_dist` 等在 `d1_robot.yaml` 配置，由 launch 注入。

---

## 5. 启动方式

```bash
./start_ego_stack.sh
./start_ego_stack.sh enable_tag_tracking=true
./start_ego_stack.sh enable_tag_tracking=true --no-rviz
```

感知节点迁入说明见 [docs/04_apriltag_integration.md](docs/04_apriltag_integration.md)。

---

## 6. 核心逻辑（FSM）

文件：`ego_replan_fsm.h/.cpp`，`single_run.launch.py`，`d1_robot.yaml`

- `computeFollowGoal`：Tag 位置，z 用 odom
- `isCloseToTag`：XY 距 ≤ `tag_stop_dist`
- `finishTagTracking` → `enterEmergencyStop("TAG_DONE")` → 低速后 `WAIT_TARGET`

---

## 7. 注意

- 规划目标是 Tag 中心；按尺寸与安全距离调 `tag_stop_dist`
- 以 `target_detected` 驱动状态
- RViz `/drone_0_plan_vis/goal_point` 可观察当前 G

**版本：** v1.4 · **状态：** 已实现
