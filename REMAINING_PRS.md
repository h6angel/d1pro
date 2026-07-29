# 急停与安全 — 后续项清单

记录急停指令下发与安全分层落地后，仍待推进的改动。系统架构见 [docs/00_overview.md](docs/00_overview.md)。

**已合并行为（简要）：**

- `enterEmergencyStop()`：同步发布停车 B 样条（6 重合控制点）
- **`[SAFETY_TIER]`**：轨迹撞障且局部 replan 失败时，优先停车样条 + 全局重建进 `GEN_NEW_TRAJ`；仅本体膨胀占据且逼近、或连续失败达阈值才正式急停
- **非 EXEC**：`EMERGENCY_STOP` / `GEN_NEW_TRAJ` 不对旧轨迹前向扫描（仅可选 odom 本体检查）
- **急停恢复**：`maybeReplanGlobalAfterEstop()` 按漂移阈值重建全局路径
- **执行层**：`traj_server` 终点附近零速；bridge **看门狗**超时零速（无独立 `hard_stop_plan_speed`）

---

## 优先级总览

| 项 | 主题 | 优先级 | 状态 | 主要文件 |
|----|------|--------|------|----------|
| 1 | 急停后全局参考（直线 minSnap → 可执行折线） | P1 | 部分完成 | `ego_replan_fsm.cpp` |
| 2 | 占据 / 急停分层（含高度/膨胀实机参数） | P1 | **已落地；参数已调妥** | FSM + `d1_robot.yaml` |
| 3 | 非 EXEC 安全检测收敛 | P2 | **已落地** | `ego_replan_fsm.cpp` |
| 4 | 执行层急停体验（惯性 / 航向抑制） | P2 | 待做 | bridge / traj_server |
| 5 | 文档与指标对齐 | P3 | **本次已同步** | `docs/*` |

---

## 项 1：急停恢复全局路径（P1，部分完成）

### 已实现

`maybeReplanGlobalAfterEstop()`：漂移 ≤ `global_replan_drift_thresh`（0.25 m）则 skip；否则 `planGlobalTraj()` 重建。

`planGlobalTraj` 几何层已改为 **平面 A\* 折线 + min-snap**（直线通畅则跳过搜索；失败可 fallback 直线）。见 [docs/07_astar_hybrid_fusion.md](docs/07_astar_hybrid_fusion.md)。

### 仍待做

- **Hybrid 可执行化**：走廊内运动原语 / 曲率约束（去质点），见 `07` 方案 L/M
- 漂移 ≤ 阈值时仍可能因旧 global 形状导致连续失败 → 可下调阈值或强制重建

### 验收

- log 出现 `[global_replan]`
- `GEN_NEW_TRAJ` 首次成功率上升；同场景急停次数下降

---

## 项 2：误急停压降与参数标定（P1）

### 已实现

- `shouldEmergencyStopOnTrajHit` / `handleTrajHitAfterReplanFailed`
- `estop_imminent_time`、`estop_min_approach_speed`、`safety_replan_trials`、`safety_slowdown_enable`、`safety_fail_estop_count`
- 高度柱 + 地面滤波 + 膨胀 0.20 m（见 [docs/06](docs/06_ground_obstacle_modeling.md)）

### 实机参数

`camera_to_ground` / `obstacle_min_height` / 膨胀与 footprint **已确认合适**，日常不必再当待办。

### 可选后续

- 「First 3 CP in obstacle」时若 odom 自由，尝试 `flag_polyInit=true` 脱困
- 换安装高度或机身外形后再复核上述参数

---

## 项 3：非 EXEC 安全检测（P2，已完成）

`checkCollisionCallback` 在 `EMERGENCY_STOP` / `GEN_NEW_TRAJ` 跳过轨迹扫描。文档保留说明，无需再开发。

---

## 项 4：执行层急停体验（P2）

### 要做什么

1. **traj_server**：检测停车样条（6 点 XY 方差≈0）→ 强制 vel/yaw_dot=0，重置 `t_progress_`
2. **bridge**：急停窗口内抑制航向 P / `min_turn_wz`，避免急停前满角速度惯性
3. 可选：显式 FSM 状态 topic 给 bridge

### 验收

- 急停后 200 ms 内 `cmd_vel` 全零（含 `angular.z`）
- odom 漂移较现状缩短

---

## 项 5：文档（P3）

已与当前实现对齐：配置单一源、控制精简律、高度柱、`[SAFETY_TIER]`、去外部规划品牌名。后续随代码变更继续更新。

---

## 与 `docs/todo.md` 的关系

| 本文项 | `todo.md` 长期项 |
|--------|------------------|
| 项 1 进阶 Hybrid A\* | Hybrid A\* / State Lattice |
| 项 2 ESDF / 障碍 | ESDF |
| 项 4 bridge MPC | 最优控制 / 执行层 MPC |

本文聚焦 **近期可合并工程**；算法级替换见 `docs/todo.md`。

---

## 建议提交信息（供 Git 使用）

1. `fix(planner): regenerate global reference on emergency recovery`
2. `fix(planner): calibrate safety tier / inflation for fewer false stops`
3. `fix(bridge): suppress yaw tracking during hard-stop window`
4. `docs: keep FSM/safety docs aligned with implementation`
