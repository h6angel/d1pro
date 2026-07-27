# 规划评估指标：来源与计算方法

本文说明本仓库规划—控制栈中各类评估指标的**定义、数据来源、计算公式与统计方式**，用于对比算法改进（见 [todo.md](todo.md)）。系统背景见 [00_overview.md](00_overview.md)，代价函数见 [01_planning_math.md](01_planning_math.md)，跟踪见 [02_control_math.md](02_control_math.md)。

---

## 1. 指标分层

| 层级 | 回答的问题 | 典型读者 |
|------|------------|----------|
| **L0 优化器** | 单次 L-BFGS 收敛快不快、代价降多少 | 改 `bspline_optimizer` |
| **L1 规划器** | 单次 `reboundReplan` 成功与否、耗时多少 | 改 A\* / 初值 / ESDF |
| **L2 轨迹** | 规划出的 B 样条质量如何 | 对比轨迹表示与障碍项 |
| **L3 任务** | 机器人是否到点、是否碰撞 | 对外汇报、A/B 总评 |
| **执行** | 规划与底盘跟踪是否一致 | 改 bridge / MPC |

**原则：** L3 是最终裁判；L3 异常时用 L1/L2/执行层定位瓶颈。

---

## 2. 数据来源总览

### 2.1 ROS 话题（`ros2 bag record`）

| 话题 | 类型 | 用途 |
|------|------|------|
| `/ov_msckf/odomimu` | `nav_msgs/Odometry` | 位姿、速度；到点误差 |
| `drone_0_planning/bspline` | `traj_utils/Bspline` | 重建 B 样条，算 L2 |
| `/drone_0_planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | 前瞻点、规划速度、yaw |
| `/command/cmd_twist` | `geometry_msgs/Twist` | 实际下发速度 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 任务起点时间戳 |

### 2.2 终端 / 日志（`ego_log/`）

| 日志关键字 | 来源文件 | 用途 |
|------------|----------|------|
| `total time:` / `optimize:` / `refine:` / `avg_time=` | `planner_manager.cpp` | L1 耗时 |
| `plan_success=` / `refine_success=` | `planner_manager.cpp`, `ego_replan_fsm.cpp` | 单步成败 |
| `iter(+1)=` / `cost=` / `rebound.` | `bspline_optimizer.cpp` | L0 迭代与 rebound |
| `[goal_reached]` | `ego_replan_fsm.cpp` | L3 任务成功 |
| `[goal_timeout]` | `ego_replan_fsm.cpp` | L3 近目标失败 |
| `EMERGENCY_STOP` / `[SAFETY_TIER]` | `ego_replan_fsm.cpp` | L3 安全事件 |
| `[global_replan]` | `ego_replan_fsm.cpp` | 急停恢复全局重建 |
| `[cmd_vel_pub]` / `[watchdog]` | `d1_planner_bridge_node.cpp` | 执行层指令 / 超时停速 |
| `h_err=` | bridge 日志 | 航向误差 |

### 2.3 代码内可直接读取的量

| 量 | 位置 |
|----|------|
| `continous_failures_count_` | `planner_manager.h` |
| `GridMap::getInflateOccupancy(pos)` | `plan_env/grid_map.h` |
| `UniformBspline::evaluateDeBoorT(t)` | `uniform_bspline.cpp` |
| `calcSmoothnessCost` 等 | `bspline_optimizer.cpp` |

---

## 3. L0 — 优化器内部指标

### 3.1 迭代次数 `iter_num`

来源：`BsplineOptimizer::iter_num_`，日志 `iter(+1)=N,...`。

### 3.2 最终总代价 `final_cost`

$$
J = \lambda_1 J_{\text{smooth}} + \lambda_2' J_{\text{dist}} + \lambda_3 J_{\text{feas}} + \lambda_2' J_{\text{swarm}} + \lambda_2 J_{\text{term}}
$$

仅在同一 $\lambda$ 与 `clearance` 下可比。子项定义见 [01_planning_math.md](01_planning_math.md) §4。

### 3.3 Rebound 重启次数

解析单次 `reboundReplan` 日志中 `rebound.` 出现次数。

### 3.4 单次优化耗时

与 `planner_manager` 打印的 `optimize:`（STEP 2 墙钟）区分：内层 `time(ms)` 为单轮代价+步。

---

## 4. L1 — 规划器层指标

### 4.1 重规划成功率

$$
R_{\text{replan}} = \frac{\#\{\text{reboundReplan 返回 true}\}}{\#\{\text{reboundReplan 调用}\}} \times 100\%
$$

区分「单次 FSM 调度最终成功」与「每次 `reboundReplan` 调用」。

### 4.2 连续失败次数

`continous_failures_count_`（`planner_manager.cpp`）；成功时归零。

### 4.3 规划耗时

日志：`total time:XXX,optimize:YYY,refine:ZZZ,avg_time=AAA`。

实时性参考：`fsm/thresh_replan_time`（默认 2.5 s）是重规划**周期**，不是耗时上限。

### 4.4 Refine 成功率

`refine_success=`；失败时可增大 `lambda_fitness`。

---

## 5. L2 — 轨迹质量指标

由 `drone_0_planning/bspline` 重建 `UniformBspline`，采样 $\Delta t = 0.05\,\text{s}$。

| 指标 | 要点 |
|------|------|
| $d_{\min}$ | 相对膨胀占据的最小间隙；穿障记 0 |
| $L$ | XY 弧长 |
| $J_{\text{smooth}}$ / RMS jerk | 与优化器一致 |
| $v_{\max}^{\text{traj}}, a_{\max}^{\text{traj}}$ | 控制点差分；限速 0.6 / 1.0 |
| $\kappa_{\max}$ | 平面曲率；低速跳过 |
| $e_{\text{term}}$ | 端点相对局部目标 XY |

---

## 6. L3 — 任务层指标

### 6.1 任务成功率

推荐：出现 `[goal_reached]`，且 XY 距目标 ≤ `goal_reach_thresh`（0.3 m），且急停次数可接受。

### 6.2–6.6

到点误差、到达时间、碰撞/急停次数、`[goal_timeout]` 率、`REPLAN_TRAJ` 频率——解析日志与 bag 即可。

### 6.7 急停链路验收 checklist

| 步骤 | 期望 log / 现象 |
|------|-----------------|
| 1 | FSM → `EMERGENCY_STOP`，或 `[SAFETY_TIER] ... -> EMERGENCY_STOP` |
| 2 | `[bspline_rx] ... n=6`，控制点 XY 重合 |
| 3 | `[pos_cmd_pub]` 速度近零 / 终点 `endpoint_stop_dist` 内零速 |
| 4 | bridge `[cmd_vel_pub]` 的 `twist` 近零（停车样条或看门狗） |
| 5 | 可选：`[global_replan] skip/replan` 后 `GEN_NEW_TRAJ` |

安全分层相关：`estop_imminent_time`、`safety_fail_estop_count`、`safety_slowdown_enable`（`d1_robot.yaml` → `fsm`）。

---

## 7. 执行层指标

### 7.1 航向误差

$$
e_\psi = \mathrm{wrapToPi}(\psi_p - \psi_r)
$$

来源：bridge 日志 `h_err=`（$\psi_r$ 为 body +Z 水平投影）。

### 7.2 进度滞后（需自加 debug）

$\Delta t_{\text{lag}} = t_{\text{closest}} - t_{\text{progress}}$。

### 7.3 速度跟踪误差

$e_v = |v_{\text{cmd}} - \|\mathbf{v}^{xy}\||$。

> 注：当前 bridge **不再**输出有符号横向误差 CTE；若需 CTE，请离线用 odom + B 样条最近点复算。

---

## 8. 统计与对比协议

- 相同地图 / bag、相同起终点与限速、每场景 $M\ge 5$、单变量改动
- 同时记录 L1 + L2 + L3
- 成功率用百分比 + 置信区间；耗时/误差用 mean、P95、max

---

## 9. 指标 ↔ 改进方向速查

| 改进方向 | 应变好 | 不能变差 |
|----------|--------|----------|
| Hybrid A\* 初值 | $R_{\text{replan}}$, 初值碰撞率, $\kappa_{\max}$ | $t_{\text{init}}$ |
| ESDF 障碍 | $d_{\min}$ 稳定性, rebound 次数 | 建图耗时 |
| iLQR / MPC | $t_{\text{opt}}$, 违反积分, $\|e_\psi\|$ | — |
| bridge 跟踪 | $\|e_\psi\|$, $R_{\text{task}}$ | — |

---

## 10. 当前缺口（尚未自动落盘）

| 指标 | 建议实现 |
|------|----------|
| $d_{\min}$, $L$, $\kappa_{\max}$ | bag 解析脚本 |
| 代价子项分解 | 成功出口打印子项 |
| $\Delta t_{\text{lag}}$ | traj_server debug topic |
| 结构化 CSV | 每次 `reboundReplan` 一行 |

---

## 11. 相关文档

- [规划改进路线图](todo.md)
- [系统总览](00_overview.md)
- [规划数学原理](01_planning_math.md)
- [控制数学原理](02_control_math.md)
