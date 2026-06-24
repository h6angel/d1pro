# 急停与恢复 — 后续 PR 清单

本文档记录 **P0（急停指令下发）已完成** 之后，仍待落地的改动项、原因与验收标准。  
背景讨论与 log 分析见会话记录；系统架构见 [docs/00_overview.md](docs/00_overview.md)。

**P0 已合并行为（简要）：**

- `enterEmergencyStop()`：进入 `EMERGENCY_STOP` 时同步发布停车 B-spline
- `d1_planner_bridge`：`plan_vel < 0.02` 时 `hard_stop`，`cmd_vel = (0, 0)`

**P0 实机 log 验证（`ego_log/stack_20260617_114536/`）：** 4 次急停均在 ~1ms 内发出 `n=6` 重合控制点样条，`traj_server` 速度归零，bridge 出现 `hard_stop twist=(0,0)`。指令链路已通。

---

## PR 优先级总览

| PR | 主题 | 优先级 | 主要文件 |
|----|------|--------|----------|
| PR-1 | 急停后从当前位姿重算全局参考 | **P1** | `ego_replan_fsm.cpp`, `planner_manager.cpp` |
| PR-2 | 占据判定分层，减少误急停 | **P1** | `ego_replan_fsm.cpp`, `grid_map` 参数 |
| PR-3 | 非 EXEC 状态下安全检测收敛 | **P2** | `ego_replan_fsm.cpp` |
| PR-4 | 执行层急停体验与惯性 | **P2** | `d1_planner_bridge`, `traj_server.cpp` |
| PR-5 | 文档与指标对齐 | **P3** | `docs/00_overview.md`, `docs/03_planning_metrics.md` |

建议合并顺序：**PR-1 → PR-2 → PR-3 → PR-4 → PR-5**。

---

## PR-1：急停恢复时重算全局路径（P1）

### 要做什么

在 `GEN_NEW_TRAJ`（或专用 `RECOVER_TRAJ`）中，局部 `planFromGlobalTraj` **之前**：

1. 以当前 `odom_pos_` / `odom_vel_` 为起点，对 `end_pt_` 调用 `planGlobalTraj()`，重建 `global_data_`
2. 重置 `global_data_.last_progress_time_ = 0`（或按 odom 在 new global 上的最近点初始化）
3. 再执行现有 `planFromGlobalTraj` → `reboundReplan`

**进阶（可拆子 PR）：** 全局层不用直线 minSnap，改为 **当前 odom → goal 的 A\* / Hybrid A\*** 折线再多项式平滑（`dyn_a_star` 或新模块）。

### 原因

- 文档写「从全局路径重新生成」，但当前 `GEN_NEW_TRAJ` 只在**已有** `global_data_` 上 `getLocalTarget()` 切片，**不会**重算 start→goal
- `planGlobalTraj()` 仅在 `planNextWaypoint()`（新目标）时调用一次
- log 中急停 #2：`GEN_NEW_TRAJ` 连续 replan 5~9 失败 ~2.5s 才出 `traj_id=6`；恢复后 0.9s 内再次急停 — 典型「车已偏离原 global 多项式，局部 rebound 无解」

### 验收

- 急停恢复后 log 出现 global 重建（可打 `[global_replan]`）
- `GEN_NEW_TRAJ` 首次 `planFromGlobalTraj` 成功率上升；连续 replan 空转次数下降
- 同场景急停次数减少（配合 PR-2 更佳）

---

## PR-2：占据判定分层，减少误急停（P1）

### 要做什么

1. **区分两类占据**
   - **本体占据**：`getInflateOccupancy(odom_pos_)`（含 footprint 豁免）为真
   - **轨迹占据**：仅 forward 轨迹采样点占据，odom 自由

2. **建议策略**
   - 仅轨迹占据 + `planFromCurrentTraj` 失败 → 优先 `REPLAN_TRAJ`，**不**直接 `EMERGENCY_STOP`（或缩短急停条件）
   - 仅当 **odom 占据** 或 **碰撞时间 < emergency_time 且 odom 也近障碍** 时才 `enterEmergencyStop()`
   - 可调参：`fsm/emergency_time`、`grid_map/obstacles_inflation`（当前 D1 约 0.09m）

3. **优化器侧（可选）**
   - 「First 3 control points in obstacles」失败时，若 odom 点自由，尝试 `flag_polyInit=true` 脱困初值，而非直接失败链到急停

### 原因

- `checkCollisionCallback` 对轨迹每 0.01s 查 `getInflateOccupancy`，膨胀区边缘易误判
- footprint 豁免只对**距 robot 中心足够近**的查询点生效，**轨迹前方点不受影响**
- log `stack_20260617_114536`：**16s 内 4 次** `EMERGENCY_STOP`，多数 `time=0.0s` 或 `planFromCurrentTraj` 失败后立即急停
- 急停过于频繁 → 任务中断、恢复循环、实机体验差

### 验收

- 同 bag / 同场景：`EMERGENCY_STOP` 次数显著下降
- 真实碰撞风险场景仍能急停（人工挡路或仿真占据 odom）
- 指标见 [docs/03_planning_metrics.md](docs/03_planning_metrics.md) §6.4

---

## PR-3：非 EXEC 状态下安全检测收敛（P2）

### 要做什么

`checkCollisionCallback` 中：

- 在 `EMERGENCY_STOP` / `GEN_NEW_TRAJ` 下**不对旧 `local_data` 轨迹**做 forward 碰撞扫描；或
- 改为**仅检查 odom 点**是否占据（带 footprint）

避免：

- 急停期间旧轨迹反复触发状态抖动
- `GEN_NEW_TRAJ` 空转时安全回调与 FSM 互相干扰

### 原因

- 当前仅排除 `WAIT_TARGET`，**不排除** `EMERGENCY_STOP` / `GEN_NEW_TRAJ`
- 急停后 `local_data_` 已是停车样条，但恢复失败时 planner 仍持有旧 EXEC 轨迹副本，检测逻辑易混乱

### 验收

- 急停 → 恢复循环中 FSM 转移次数减少，无异常 `EXEC_TRAJ` ↔ `EMERGENCY_STOP` 振荡
- log 中急停期间不再出现对旧 `traj_id` 的 `planFromCurrentTraj` 成功跳回 `EXEC_TRAJ`（除非有意设计）

---

## PR-4：执行层急停体验与惯性（P2）

### 要做什么

P0 已做：`hard_stop` 零速 + EMA 重置。可选增强：

1. **traj_server**
   - 检测停车样条（6 控制点 XY 方差 ≈ 0）→ 强制 `vel=0`、`yaw_dot=0`，重置 `t_progress_`

2. **d1_planner_bridge**
   - 收到 `hard_stop` 或 `trajectory_id` 跳变且零速时，**跳过后续若干帧**航向 P 控制（避免急停前 `wz=±1.0` 的角动量）
   - 或：急停后短窗口内禁止 `min_turn_wz` 抬升

3. **可选 topic**
   - `/planning/fsm_state` 或 `PositionCommand.trajectory_flag` 扩展，bridge 显式跟 FSM（最后防线，非必须）

### 原因

- log 显示急停**前**常见 `twist=(0, ±1.0)`（航向未对齐，原地满角速度转）
- 急停**后** `pos_cmd vel=0`，但 odom 仍漂移（如 #1 滑 ~0.35m）：指令已零，物理惯性 + 转圈惯性仍在
- P0 解决「指令链不断」；PR-4 解决「停得不够硬、体感差」

### 验收

- 急停后 200ms 内 `cmd_vel` 全零（含 `angular.z`）
- 同场景 odom 漂移距离较 P0 基线缩短（定性对比即可）

---

## PR-5：文档与指标对齐（P3）

### 要做什么

- 更新 [docs/00_overview.md](docs/00_overview.md) §3.2 状态图：补充 `enterEmergencyStop()`、bridge `hard_stop`
- 更新 [docs/03_planning_metrics.md](docs/03_planning_metrics.md)：急停验收 checklist（`bspline_rx` 停车样条、`hard_stop` log）
- 在 [Readme.md](Readme.md) 或本文档链接 PR 进度

### 原因

- 原文档写 `EMERGENCY_STOP → GEN_NEW_TRAJ` 会「从全局路径重新生成」，与实现不符，易误导后续开发

---

## 与 `docs/todo.md` 的关系

| 本文 PR | `docs/todo.md` 中长期项 |
|---------|-------------------------|
| PR-1 进阶（Hybrid A* 全局） | §3 Hybrid A* / State Lattice |
| PR-2 ESDF / 更平滑障碍 | §3 ESDF 距离场 |
| PR-4 bridge MPC | §3 最优控制 / 执行层 MPC |

本文档聚焦 **当前 FSM + D1 栈的近期可合并 PR**；算法级替换见 `docs/todo.md`。

---

## 参考 log 片段（P0 生效 + 待改进现象）

**急停指令生效（#1）：**

```
[SAFETY]: from EXEC_TRAJ to EMERGENCY_STOP
[bspline_rx] traj_id=2 ... n=6 [p0=(0.024,-0.018) ... 全同]
[pos_cmd_pub] traj_id=2 vel=(0.000,0.000,0.000)
[cmd_vel_pub] hard_stop traj_id=2 twist=(0,0)
```

**待 PR-1/2 改进：**

```
[FSM]: from EMERGENCY_STOP to GEN_NEW_TRAJ
[drone -1 replan 5] ... refine_success=0   # 连续失败
[drone -1 replan 6] ...                   # 多次空转后才成功
[SAFETY]: from EXEC_TRAJ to EMERGENCY_STOP  # 恢复后再次急停
```

**待 PR-4 改进：**

```
# 急停前
[cmd_vel_pub] cmd_vel=(0.695,0.840) twist=(0.000, 1.000)   # 只转不走
# 急停后指令已零，odom 仍从 (0.025,-0.018) 漂到 (0.370,-0.095)
```

---

## 建议 PR 标题（供 Git 使用）

1. `fix(planner): regenerate global reference on emergency recovery`
2. `fix(planner): tier occupancy checks to reduce false emergency stops`
3. `fix(planner): skip stale traj safety check in EMERGENCY_STOP/GEN_NEW_TRAJ`
4. `fix(bridge): suppress yaw tracking during hard stop window`
5. `docs: align FSM emergency-stop behavior with implementation`
