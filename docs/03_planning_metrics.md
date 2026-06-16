# 规划评估指标：来源与计算方法

本文说明 **EGO Planner × D1** 栈中各类评估指标的**定义、数据来源、计算公式与统计方式**，用于对比算法改进（见 [todo.md](todo.md)）是否真正有效。系统背景见 [00_overview.md](00_overview.md)，代价函数细节见 [01_planning_math.md](01_planning_math.md)，跟踪误差定义见 [02_control_math.md](02_control_math.md)。

---

## 1. 指标分层

| 层级 | 回答的问题 | 典型读者 |
|------|------------|----------|
| **L0 优化器** | 单次 L-BFGS 收敛快不快、代价降多少 | 改 `bspline_optimizer` |
| **L1 规划器** | 单次 `reboundReplan` 成功与否、耗时多少 | 改 A* / 初值 / ESDF |
| **L2 轨迹** | 规划出的 B 样条质量如何 | 对比轨迹表示与障碍项 |
| **L3 任务** | 机器人是否到点、是否碰撞 | 对外汇报、A/B 总评 |
| **执行** | 规划与底盘跟踪是否一致 | 改 bridge / MPC |

**原则：** L3 是最终裁判；L3 异常时用 L1/L2/执行层定位瓶颈。

---

## 2. 数据来源总览

### 2.1 ROS 话题（`ros2 bag record`）

| 话题 | 类型 | 用途 |
|------|------|------|
| `/ov_msckf/odomimu` | `nav_msgs/Odometry` | 位姿、速度；到点误差、CTE |
| `drone_0_planning/bspline` | `traj_utils/Bspline` | 重建 B 样条，算 L2 轨迹指标 |
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
| `EMERGENCY_STOP` / `Suddenly discovered obstacles` | `ego_replan_fsm.cpp` | L3 安全事件 |
| `current traj in collision, replan` | `ego_replan_fsm.cpp` | 执行中碰撞预警 |
| bridge `lateral_error` / `heading_error` | `d1_planner_bridge_node.cpp` | 执行层误差 |

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

**含义：** Rebound 优化主循环执行轮数（含 rebound 重启后的累计）。

**来源：** `BsplineOptimizer::iter_num_`，终端打印：

```
iter(+1)=N,time(ms)=...,total_t(ms)=...,cost=...
```

**计算：**

$$
N_{\text{iter}} = \text{iter\_num\_} \quad \text{（单次 reboundReplan 的 STEP 2 内）}
$$

**统计：** 对同一场景重复 $M$ 次，报告 $\mathrm{mean}(N_{\text{iter}})$、P95、max。ESDF / 更好初值通常使 $N_{\text{iter}}$ 下降。

---

### 3.2 最终总代价 `final_cost`

**含义：** Rebound 阶段 `combineCostRebound` 在收敛或退出时的标量代价（已加权）。

**来源：** `bspline_optimizer.cpp` 优化结束时的 `printf(..., final_cost)`。

**计算公式（与代码一致）：**

$$
J = \lambda_1 J_{\text{smooth}} + \lambda_2' J_{\text{dist}} + \lambda_3 J_{\text{feas}} + \lambda_2' J_{\text{swarm}} + \lambda_2 J_{\text{term}}
$$

各子项定义见 [01_planning_math.md](01_planning_math.md) §4。

**注意：** $J$ 仅在**同一 $\lambda$ 与同一 `clearance`** 下可比；改权重后应单独记录子项 $J_{\text{smooth}}, J_{\text{dist}}, \ldots$

---

### 3.3 代价子项（建议离线复算）

若需分解对比，对**最终控制点** $\mathbf{Q}$ 调用与优化器相同的函数（或按下列公式复算）：

#### 平滑项 $J_{\text{smooth}}$（Jerk）

```1089:1093:src/planner/bspline_opt/src/bspline_optimizer.cpp
      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
```

$$
\mathbf{j}_i = \mathbf{q}_{i+3} - 3\mathbf{q}_{i+2} + 3\mathbf{q}_{i+1} - \mathbf{q}_i, \quad
J_{\text{smooth}} = \sum_i \|\mathbf{j}_i\|^2
$$

#### 避障项 $J_{\text{dist}}$（Rebound）

Signed distance $d_i = (\mathbf{q}_i - \mathbf{b}) \cdot \mathbf{n}$，$d_{\text{err}} = \text{clearance} - d_i$。分段三次惩罚，见 `calcDistanceCostRebound`。

#### 可行性项 $J_{\text{feas}}$（默认分支）

离散速度 $\mathbf{v}_i = (\mathbf{q}_{i+1}-\mathbf{q}_i)/t_s$，加速度 $\mathbf{a}_i = (\mathbf{q}_{i+2} - 2\mathbf{q}_{i+1} + \mathbf{q}_i)/t_s^2$：

$$
J_{\text{feas}} \mathrel{+}= \sum_{i,j} \bigl[\max(0, |v_{i,j}|-v_{\max})\bigr]^2 \cdot t_s^{-2}
+ \sum_{i,j} \bigl[\max(0, |a_{i,j}|-a_{\max})\bigr]^2
$$

（$j$ 为 x/y/z 轴索引；D1 平面规划时 z 分量通常被 `enforcePlanningZ` 锚定。）

#### 终端项 $J_{\text{term}}$

$$
\mathbf{p}_{\text{end}} = \tfrac{1}{6}(\mathbf{q}_{N-3} + 4\mathbf{q}_{N-2} + \mathbf{q}_{N-1}), \quad
J_{\text{term}} = \|\mathbf{p}_{\text{end}} - \mathbf{p}_g\|^2
$$

---

### 3.4 Rebound 重启次数

**含义：** 优化过程中 `check_collision_and_rebound()` 触发次数（日志含 `rebound.` 或 `collided, keep optimizing`）。

**计算：** 解析单次 `reboundReplan` 日志中 `rebound.` 出现次数。

**解读：** ESDF / 更平滑障碍梯度应使重启次数下降。

---

### 3.5 单次优化耗时 $t_{\text{opt,inner}}$

**来源：** 每轮 `iter(+1)=...,time(ms)=...` 中的 `time_ms`（该轮 `combineCostRebound` + 求解器步）。

**统计：**

$$
t_{\text{opt}} = \sum_{\text{rounds}} time\_ms_{\text{round}} / 1000 \quad \text{(s)}
$$

与 `planner_manager` 打印的 `optimize:` 时间（含 STEP 1 部分时需看分解）区分：后者为 **STEP 1+2 墙钟时间**。

---

## 4. L1 — 规划器层指标

### 4.1 重规划成功率

**含义：** `reboundReplan()` 返回 `true` 的比例。

**来源：**

- 成功：`plan_success=1` 且函数 `return true`；`continous_failures_count_` 清零
- 失败：`plan_success=0`、`refine_success=0`，或 `continous_failures_count_++`

**计算：**

$$
R_{\text{replan}} = \frac{\#\{\text{reboundReplan 返回 true}\}}{\#\{\text{reboundReplan 调用}\}} \times 100\%
$$

**注意：** `planFromGlobalTraj` 内部可能 **多次 trial**（默认最多 10 次），应区分：

- **单次 call 成功率**：一次 FSM 调度内是否最终成功  
- **单次 rebound 成功率**：每次 `reboundReplan` 调用  

---

### 4.2 连续失败次数

**含义：** 连续 `reboundReplan` 失败计数，成功时归零；用于随机多项式脱困缩放。

**来源：** `EGOPlannerManager::continous_failures_count_`（`planner_manager.cpp`）。

**统计：** 单次任务中 $\max(\text{continous\_failures\_count\_})$；或失败事件分布。

---

### 4.3 规划耗时 $t_{\text{init}}, t_{\text{opt}}, t_{\text{refine}}, t_{\text{total}}$

**含义：** `reboundReplan` 三步墙钟时间（秒）。

**来源：** `planner_manager.cpp` 内 `rclcpp::Clock().now()` 差分，终端行：

```
total time:XXX,optimize:YYY,refine:ZZZ,avg_time=AAA
```

**计算：**

| 符号 | 代码区间 |
|------|----------|
| $t_{\text{init}}$ | STEP 1 结束 − 函数入口 |
| $t_{\text{opt}}$ | STEP 2 结束 − STEP 1 结束（仅 BsplineOptimizeTrajRebound） |
| $t_{\text{refine}}$ | STEP 3 结束 − STEP 3 开始 |
| $t_{\text{total}}$ | $t_{\text{init}} + t_{\text{opt}} + t_{\text{refine}}$ |

**统计建议：**

$$
\text{报告} \quad \mathrm{mean},\ \mathrm{P95},\ \max \quad \text{of } t_{\text{total}}
$$

**实时性参考：** `fsm/thresh_replan_time`（D1 默认 2.5 s）是重规划**周期**，不是规划耗时上限；通常要求 $t_{\text{total}}$ 的 P95 远小于该值（例如 < 100–300 ms，需按 Jetson 实测校准）。

---

### 4.4 平均规划耗时 `avg_time`

**含义：** 自进程启动以来，所有**成功** `reboundReplan` 的 $t_{\text{total}}$ 运行平均。

**来源：** `planner_manager.cpp` 中 `static sum_time / count_success`。

**注意：** 非滑动窗口；长跑后对新改动的敏感度低。对比实验宜解析每次 `total time` 自行统计。

---

### 4.5 Refine 成功率

**含义：** STEP 3 `refineTrajAlgo` 是否成功（时间重分配后不穿障）。

**来源：** `refine_success=`（`ego_replan_fsm.cpp` → `callReboundReplan`）；失败时提示增大 `lambda_fitness`。

---

### 4.6 初值碰撞率（需自行加计数或离线算）

**含义：** STEP 2 优化**前**，初值 B 样条在占据图上是否穿障。

**离线计算：**

1. 从日志或调试发布拿到 `point_set` / `ctrl_pts`（或 INIT 阶段的可视化路径）  
2. 参数化为 B 样条 $\mathbf{p}(t)$  
3. 对 $t \in [0, T]$，步长 $\Delta t = 0.05\,\text{s}$：

$$
\text{collision\_init} = \exists t:\ \texttt{getInflateOccupancy}(\mathbf{p}(t)) = 1
$$

（D1：`p_chk(2) = z_{\text{odom}}`，与 `checkCollisionCallback` 一致。）

---

## 5. L2 — 轨迹质量指标

以下均在**规划成功**后，由 `drone_0_planning/bspline` 消息重建 `UniformBspline`，再密集采样得到。`traj_server` 使用相同步长 $\Delta t = 0.05\,\text{s}$（`closestTimeOnTrajXY`）。

### 5.1 轨迹重建

从 `Bspline` 消息：

```python
# 伪代码
pos_pts = [(pt.x, pt.y, pt.z) for pt in msg.pos_pts]
knots = msg.knots
order = msg.order
ts = msg.knots[order+1] - msg.knots[order]  # 或与 traj_server 一致取 0.1 后 setKnot
traj = UniformBspline(pos_pts, order, ts)
traj.setKnot(knots)
T = traj.getTimeSum()
```

### 5.2 最小障碍间隙 $d_{\min}$

**含义：** 轨迹采样点到**膨胀占据**的最小安全距离（栅格分辨率量级下限）。

**障碍查询（与执行安全检查一致）：**

```378:394:src/planner/plan_env/include/plan_env/grid_map.h
inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
{
  ...
  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}
```

返回值：`1` = 占据，`0` = 自由，`-1` = 地图外。

**计算（工程近似）：**

对每个采样点 $\mathbf{p}_k = \mathbf{p}(t_k)$：

- 若 `getInflateOccupancy(p_k) == 1`，则 $d_k = 0$（穿障）  
- 否则在 XY 平面做 **局部栅格搜索**（或未来 ESDF 查表）：

$$
d_k \approx \min_{\text{free cells within radius } r} \|\mathbf{p}_k^{xy} - \mathbf{c}_{\text{cell}}^{xy}\| - r_{\text{robot}}
$$

其中 $r_{\text{robot}} =$ `robot_footprint_radius`（launch 配置）。

$$
d_{\min} = \min_k d_k
$$

**对比：** 与优化参数 `optimization/dist0`（默认 clearance）对照；$d_{\min} < \text{dist0}$ 表示轨迹比设计 clearance 更贴障或穿障。

---

### 5.3 路径长度 $L$

**含义：** B 样条弧长（XY 平面，D1 主指标）。

$$
L = \sum_{k} \|\mathbf{p}_{k+1}^{xy} - \mathbf{p}_k^{xy}\|, \quad t_k = k \cdot \Delta t,\ \Delta t = 0.05
$$

---

### 5.4 平滑度 $J_{\text{smooth}}$ / RMS Jerk

**与优化器一致：**

$$
J_{\text{smooth}} = \sum_i \|\mathbf{j}_i\|^2, \quad
\text{RMS\_jerk} = \sqrt{J_{\text{smooth}} / N_{\text{jerk}}}
$$

其中 $N_{\text{jerk}} = N_{\text{ctrl}} - 3$。

---

### 5.5 速度 / 加速度峰值与违反度

由控制点差分（与 `calcFeasibilityCost` 一致，$t_s =$ `bspline_interval_`）：

$$
\mathbf{v}_i = \frac{\mathbf{q}_{i+1} - \mathbf{q}_i}{t_s}, \quad
\mathbf{a}_i = \frac{\mathbf{q}_{i+2} - 2\mathbf{q}_{i+1} + \mathbf{q}_i}{t_s^2}
$$

| 指标 | 公式 |
|------|------|
| 速度峰值 | $v_{\max}^{\text{traj}} = \max_{i,j} |v_{i,j}|$ |
| 加速度峰值 | $a_{\max}^{\text{traj}} = \max_{i,j} |a_{i,j}|$ |
| 速度违反积分 | $\sum_{i,j} [\max(0, |v_{i,j}|-v_{\lim})]^2$ |
| 加速度违反积分 | $\sum_{i,j} [\max(0, |a_{i,j}|-a_{\lim})]^2$ |

$v_{\lim}, a_{\lim}$ 取 `manager/max_vel`、`manager/max_acc`（D1 默认 1.6 / 2.0）。

**Refine 触发：** `UniformBspline::checkFeasibility(ratio)` 若超限则计算时间缩放比 `ratio` 并进入 STEP 3。

---

### 5.6 曲率 $\kappa$ 与峰值 $\kappa_{\max}$（平面）

对弧长参数曲线 $\mathbf{p}^{xy}(s)$：

$$
\kappa = \frac{|v_x a_y - v_y a_x|}{(v_x^2 + v_y^2)^{3/2}}, \quad
\mathbf{v} = \dot{\mathbf{p}}^{xy},\ \mathbf{a} = \ddot{\mathbf{p}}^{xy}
$$

在采样点 $t_k$ 上用 B 样条一阶、二阶导数代入。低速时 $\|\mathbf{v}\| \to 0$ 处需跳过或设阈值（如 $\|\mathbf{v}^{xy}\| < 0.05$）。

**差速可跟踪性（经验）：** $\kappa_{\max} \lesssim \omega_{\max} / v_{\min}$，其中 $\omega_{\max}, v_{\min}$ 来自 bridge（如 `max_wz`, `min_vx`）。

---

### 5.7 终端位置误差（轨迹末端 vs 局部目标）

$$
e_{\text{term}} = \left\|\frac{1}{6}(\mathbf{q}_{N-3} + 4\mathbf{q}_{N-2} + \mathbf{q}_{N-1}) - \mathbf{p}_g\right\|_{xy}
$$

$\mathbf{p}_g$ 为当次 `local_target_pt`（可从 FSM 日志或内部状态获取）。

---

## 6. L3 — 任务层指标

### 6.1 任务成功率

**含义：** 固定场景集中，完成导航目标且无不可接受安全事件的比例。

**成功条件（推荐操作定义）：**

1. 日志出现 `[goal_reached]`  
2. 且 $\|\mathbf{p}_{\text{odom}}^{xy} - \mathbf{p}_{\text{goal}}^{xy}\| \le \texttt{fsm/thresh\_goal\_reach\_meter}$（默认 0.3 m，`single_run.launch.py`）  
3. 且任务过程中无 `EMERGENCY_STOP`（或碰撞次数 = 0，见下）

**计算：**

$$
R_{\text{task}} = \frac{\#\text{成功任务}}{\#\text{总任务}} \times 100\%
$$

每个场景重复 $M \ge 5$ 次，报告 mean 与 95% Wilson 区间（或简单 $\pm$ 标准误）。

---

### 6.2 到点误差 $e_{\text{goal}}$

**含义：** 判定到达时刻（或轨迹结束时刻）机器人与目标的 XY 距离。

**来源：**

- 在线：`[goal_reached]` 日志中的 `dist_xy=`  
- 离线：bag 中最后一帧 odom 与 goal：

$$
e_{\text{goal}} = \sqrt{(x_{\text{odom}} - x_{\text{goal}})^2 + (y_{\text{odom}} - y_{\text{goal}})^2}
$$

**统计：** mean、P95、max。

---

### 6.3 到达时间 $T_{\text{goal}}$

**含义：** 从**首次收到目标**（或 `have_target_=true`）到 `[goal_reached]` 的墙钟时间。

**计算：**

$$
T_{\text{goal}} = t_{\text{goal\_reached}} - t_{\text{goal\_received}}
$$

从 bag 中 `/move_base_simple/goal` 与日志时间戳对齐。

---

### 6.4 碰撞 / 占据事件次数

**含义：** 执行中轨迹进入膨胀占据的次数或持续时间。

**来源（在线逻辑）：** `checkCollisionCallback`（`ego_replan_fsm.cpp`）：

- 对 $t \in [t_{\text{cur}}, \min(T,\ \frac{2}{3}T)]$，步长 0.01 s  
- `p_chk(2) = odom_pos_(2)`，查询 `getInflateOccupancy(p_chk)`

**指标定义：**

| 指标 | 计算 |
|------|------|
| 碰撞检测触发次数 | 日志 `current traj in collision` + `Suddenly discovered obstacles` 计数 |
| 急停次数 | 进入 `EMERGENCY_STOP` 次数 |
| 离线占据率 | bag 回放：odom 位置每帧 `getInflateOccupancy`（需同步地图或录占据可视化） |

---

### 6.5 近目标超时率

**含义：** 轨迹时间走完但未进入 goal 阈值的次数。

**来源：** `[goal_timeout] traj ended dist_goal_xy=... > thresh=...`

$$
R_{\text{timeout}} = \frac{\#\text{goal\_timeout}}{\#\text{任务}} \times 100\%
$$

---

### 6.6 重规划频率

**含义：** 单位时间内 FSM 进入 `REPLAN_TRAJ` 的次数。

**计算：** 统计日志中 `changeFSMExecState(REPLAN_TRAJ` 或 `replan` 相关行，除以任务总时长。

**解读：** L2 轨迹差或跟踪差会导致重规划频繁上升。

---

## 7. 执行层指标（规划—底盘一致性）

### 7.1 横向误差 CTE（Signed Lateral Error）

**含义：** 机器人相对**规划前瞻点**的带符号横向偏差（与 bridge 控制律一致）。

**来源：** `trajectory_tracker.cpp` → `signedLateralError`：

```47:54:src/d1_planner_bridge/src/trajectory_tracker.cpp
double TrajectoryTracker::signedLateralError(
  double robot_x, double robot_y, double ref_x, double ref_y, double path_yaw)
{
  const double ex = robot_x - ref_x;
  const double ey = robot_y - ref_y;
  const double ux = std::cos(path_yaw);
  const double uy = std::sin(path_yaw);
  return uy * ex - ux * ey;
}
```

其中 $(x_r,y_r)=$ `cmd.position`（**非** odom），$\psi_p = \mathrm{atan2}(v_y, v_x)$（$\|v^{xy}\|>0.05$ 时）。

**离线复算（更标准的几何 CTE）：** 对 odom $(x,y)$，在 B 样条上求 $t^* = \arg\min_t \|\mathbf{p}^{xy}(t) - (x,y)\|^2$（与 `closestTimeOnTrajXY` 相同，$\Delta t=0.05$）：

$$
e_{\text{lat}} = -\sin\psi_p (x - x_{\text{ref}}) + \cos\psi_p (y - y_{\text{ref}})
$$

$(x_{\text{ref}}, y_{\text{ref}}) = \mathbf{p}^{xy}(t^*)$，$\psi_p$ 为轨迹在 $t^*$ 的切向。

**统计：** 全程 $|e_{\text{lat}}|$ 的 mean、P95、max；bridge 日志中的 `lateral_error` 可直接解析。

---

### 7.2 航向误差

$$
e_\psi = \mathrm{wrapToPi}(\psi_p - \psi_{\text{odom}})
$$

**来源：** `ground.heading_error`（bridge 日志）。

---

### 7.3 进度滞后

**含义：** 轨迹时间进度落后于几何最近点的程度。

**来源：** `traj_server` 的 $t_{\text{progress}}$ 与 $t_{\text{closest}} = \texttt{closestTimeOnTrajXY}(\mathbf{p}_{\text{odom}}^{xy})$。

$$
\Delta t_{\text{lag}} = t_{\text{closest}} - t_{\text{progress}}
$$

（需临时发布或从 debug 日志获取；正值表示机器人「落在」轨迹后面。）

---

### 7.4 速度跟踪误差

$$
e_v = |v_{\text{cmd}} - v_{\text{plan}}|
$$

其中 $v_{\text{plan}} = \|\mathbf{v}^{xy}(t_{\text{cur}})\|$，$v_{\text{cmd}}$ 为 `/command/cmd_twist`.linear.x（车体前向，已投影）。

---

## 8. 统计与对比协议

### 8.1 单次实验记录模板

| 字段 | 示例 |
|------|------|
| `scenario_id` | narrow_corridor_01 |
| `algo_tag` | ego_baseline / hybrid_astar_v1 |
| `replan_success` | true |
| `t_init, t_opt, t_refine` | 0.012, 0.085, 0.003 |
| `iter_num` | 24 |
| `final_cost` | 12.3 |
| `d_min` | 0.58 |
| `L` | 4.2 |
| `kappa_max` | 0.9 |
| `e_goal` | 0.18 |
| `T_goal` | 23.5 |
| `emergency_stop` | 0 |

### 8.2 聚合方式

| 指标类型 | 推荐统计量 |
|----------|------------|
| 成功率 | 百分比 + $M$ 次二项置信区间 |
| 耗时、误差 | mean、P95、max |
| 计数（急停、rebound） | sum / 任务时长 |

### 8.3 公平对比 checklist

- [ ] 相同地图快照或相同 bag 回放感知  
- [ ] 相同起终点、相同 `max_vel` / `max_acc` / `dist0`  
- [ ] 每场景 $M \ge 5$ 次  
- [ ] 只改一个模块（单变量）  
- [ ] 同时记录 L1 + L2 + L3，避免只看任务成功率  

---

## 9. 指标 ↔ 改进方向速查

| 改进方向 | 应变好 | 不能变差 |
|----------|--------|----------|
| Hybrid A* 初值 | $R_{\text{replan}}$, 初值碰撞率, $\kappa_{\max}$, $N_{\text{iter}}$ | $t_{\text{init}}$ |
| ESDF 障碍 | $d_{\min}$ 稳定性, rebound 次数, $N_{\text{iter}}$ | 建图耗时 |
| iLQR / MPC | $t_{\text{opt}}$, 违反积分, $|e_{\text{lat}}|$ | — |
| bridge / MPC 跟踪 | $|e_{\text{lat}}|$, $|e_\psi|$, $R_{\text{task}}$ | — |

---

## 10. 当前缺口（尚未自动落盘）

以下指标**公式已明确**，但需自行加发布或写 bag 后处理脚本：

| 指标 | 建议实现 |
|------|----------|
| $d_{\min}$, $L$, $\kappa_{\max}$ | bag 解析 `bspline` + 占据地图离线脚本 |
| 代价子项分解 | `reboundReplan` 成功出口打印 $J_{\text{smooth}}, \ldots$ |
| $\Delta t_{\text{lag}}$ | `traj_server` 发布 debug topic |
| 结构化 CSV | `reboundReplan` 每次一行：`scenario_id, success, t_*, iter, cost, ...` |

---

## 11. 相关文档

- [规划改进路线图](todo.md)
- [系统总览](00_overview.md)
- [规划数学原理](01_planning_math.md)
- [控制数学原理](02_control_math.md)
