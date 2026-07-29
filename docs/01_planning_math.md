# 规划数学原理

本文说明本仓库中的 **轨迹表示、初值生成、弹性带（rebound）优化与 D1 平面 / 高度柱改动**。系统级流程见 [00_overview.md](00_overview.md)。

---

## 1. 问题表述

给定：

- 起点状态 $(\mathbf{p}_s, \mathbf{v}_s, \mathbf{a}_s)$（来自 odom）
- 局部目标 $(\mathbf{p}_g, \mathbf{v}_g)$（由 FSM `getLocalTarget()` 在全局路径上选取）
- 膨胀占据地图 $\mathcal{O}$

求一条 **均匀 B 样条轨迹** $\mathbf{p}(t)$，满足：

1. 平滑（低 jerk）
2. 与障碍保持安全距离
3. 速度、加速度不超过 `max_vel`、`max_acc`
4. 终端接近局部目标

**决策变量**：均匀 B 样条的控制点矩阵 $\mathbf{Q} \in \mathbb{R}^{3 \times N}$，每列 $\mathbf{q}_i$ 为一个控制点。

范式：**引导搜索（动态 A\*）+ 轨迹参数化（B 样条）+ 惩罚项无约束优化（L-BFGS）**。

---

## 2. 均匀 B 样条

### 2.1 轨迹阶次与时间

- 阶次 $p = 3$（三次 B 样条），`order_ = 3`
- 结点间隔 $t_s =$ `bspline_interval_`（由起终点距离与控制点间距估算）

位置轨迹由控制点经 De Boor 算法求值：

$$
\mathbf{p}(t) = \text{DeBoor}(\mathbf{Q}, t)
$$

速度、加速度样条为位置样条的导数（`getDerivative()`），用于可行性代价与 `traj_server` 采样。

### 2.2 路径点 → 初始控制点

`UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts)` 将 $K$ 个几何路径点及边界速度、加速度约束，组装为线性系统 $\mathbf{A}\mathbf{p}=\mathbf{b}$（每轴独立求解）：

- **过点约束**：三次 B 样条局部凸组合 $(\mathbf{q}_{i-1} + 4\mathbf{q}_i + \mathbf{q}_{i+1})/6 = \mathbf{p}_k$
- **速度边界**：$(\mathbf{q}_{i+1}-\mathbf{q}_{i-1})/(2t_s) = \mathbf{v}$
- **加速度边界**：$(\mathbf{q}_{i+1} - 2\mathbf{q}_i + \mathbf{q}_{i-1})/t_s^2 = \mathbf{a}$

输出 `ctrl_pts` 尺寸为 $3 \times (K+2)$。

---

## 3. 规划流水线：`reboundReplan`

入口：`planner_manager.cpp` → `reboundReplan`，分三步：

```
STEP 1 INIT   → 初值控制点 + initControlPoints（A* 弹性方向）
STEP 2 OPT    → BsplineOptimizeTrajRebound（L-BFGS, combineCostRebound）
STEP 3 REFINE → checkFeasibility + refineTrajAlgo（combineCostRefine，单机）
```

### 3.1 STEP 1：初值两条路径

| 模式 | 触发 | 初值来源 |
|------|------|----------|
| 多项式 | `flag_polyInit=true` | `PolynomialTraj::one_segment_traj_gen` 或 `minSnapTraj` |
| Warm-start | `flag_polyInit=false` | 上一段 B 样条从 $t_{cur}$ 起弧长重采样 + 混合钉点 |

#### 3.1.1 多项式初值

**单段**：边界匹配 $\mathbf{p}_s,\mathbf{v}_s,\mathbf{a}_s \to \mathbf{p}_g,\mathbf{v}_g$，加速度终端为零。

**时间** $T$：由距离 $d=\|\mathbf{p}_g-\mathbf{p}_s\|$ 与梯形速度轮廓估算：

$$
T = \begin{cases}
\sqrt{d / a_{\max}} & \frac{v_{\max}^2}{a_{\max}} > d \\
\frac{d - v_{\max}^2/a_{\max}}{v_{\max}} + \frac{2v_{\max}}{a_{\max}} & \text{otherwise}
\end{cases}
$$

按步长 $t_s$ 重采样得 `point_set`（至少 7 点），再 `parameterizeToBspline`。

**随机脱困**（`flag_randomPolyTraj`）：在 XY 中点沿垂直于起终点方向插入扰动点，再 `minSnapTraj`。开启 `use_planning_z` 时扰动仅在 **水平面**，$z = z_{\text{ref}}$。

#### 3.1.2 Warm-start 初值（局部重规划）

当 `flag_polyInit=false`：

1. 取墙钟时间 $t_{cur} = \min(t_{\text{wall}}, T_{\text{dur}})$
2. 从 $t_{cur}$ 到轨迹末端按 $t_s$ 采样 B 样条，建 **伪弧长** 表
3. 必要时在末端接一段多项式延伸到 `local_target_pt`
4. 按弧长等距重采样得 `point_set`
5. 边界导数使用 **`start_vel`、`start_acc`（odom）**，而非轨迹上的值

**混合钉点**：参数化得到 `ctrl_pts` 后，强制前 3 个控制点：

$$
\mathbf{q}_i = \mathbf{p}_s + \mathbf{v}_s \cdot (i \cdot t_s), \quad i = 0,1,2
$$

L-BFGS 从第 `order_` 个控制点开始优化；钉点使轨迹起点与实测 odom 一致。

### 3.2 STEP 1 续：`initControlPoints` 与 A\*

沿初值 B 样条检测碰撞段，对每段在自由空间运行 **动态 A\***，得到绕行折线 `a_star_pathes`。

对每个可能碰撞的控制点 $i$，构造：

- **基点** $\mathbf{b}_{i,j}$：障碍边界上的参考点
- **法向** $\mathbf{n}_{i,j}$：由 A\* 路径方向确定的单位向量（弹性方向）

Signed distance：

$$
d_i = (\mathbf{q}_i - \mathbf{b}_{i,j}) \cdot \mathbf{n}_{i,j}
$$

若 $d_i < \text{clearance}$，产生 rebound 代价梯度。

---

## 4. 优化：代价函数与梯度

Rebound 阶段总代价（`combineCostRebound`）：

$$
J = \lambda_1 J_{\text{smooth}} + \lambda_2' J_{\text{dist}} + \lambda_3 J_{\text{feas}} + \lambda_2' J_{\text{swarm}} + \lambda_2 J_{\text{term}}
$$

Refine 阶段（`combineCostRefine`）：

$$
J = \lambda_1 J_{\text{smooth}} + \lambda_4 J_{\text{fitness}} + \lambda_3 J_{\text{feas}}
$$

默认 D1 参数见 `config/d1_robot.yaml`：`max_vel=0.6`，`max_acc=1.0`，`optimization_dist0=0.55`，`lambda_fitness=1.5`，以及 `lambda_smooth / collision / feasibility`。

### 4.1 平滑项 $J_{\text{smooth}}$（Jerk）

$$
\mathbf{j}_i = \mathbf{q}_{i+3} - 3\mathbf{q}_{i+2} + 3\mathbf{q}_{i+1} - \mathbf{q}_i
$$

$$
J_{\text{smooth}} = \sum_i \|\mathbf{j}_i\|^2
$$

### 4.2 避障反弹项 $J_{\text{dist}}$

$$
d = (\mathbf{q}_i - \mathbf{b}) \cdot \mathbf{n}, \quad d_{\text{err}} = \text{clearance} - d
$$

- $d_{\text{err}} < 0$：无惩罚
- $0 \le d_{\text{err}} < d_0$：$J \mathrel{+}= d_{\text{err}}^3$，$\nabla_{\mathbf{q}_i} \mathrel{+}= -3 d_{\text{err}}^2 \mathbf{n}$
- $d_{\text{err}} \ge d_0$：三次多项式延拓保证 $C^1$ 连续（$a=3d_0,\, b=-3d_0^2,\, c=d_0^3$）

迭代中若轨迹足够平滑，会 `check_collision_and_rebound()` 更新弹性方向并可能 **earlyExit** 重启 L-BFGS。

### 4.3 可行性项 $J_{\text{feas}}$

$$
\mathbf{v}_i = \frac{\mathbf{q}_{i+1} - \mathbf{q}_i}{t_s}, \quad
\mathbf{a}_i = \frac{\mathbf{q}_{i+2} - 2\mathbf{q}_{i+1} + \mathbf{q}_i}{t_s^2}
$$

默认分支：超限时加平方惩罚（含 $t_s^{-2}$ 权重项）。

### 4.4 终端项 $J_{\text{term}}$

$$
\mathbf{p}_{\text{end}} = \frac{1}{6}(\mathbf{q}_{N-3} + 4\mathbf{q}_{N-2} + \mathbf{q}_{N-1})
$$

$$
J_{\text{term}} = \|\mathbf{p}_{\text{end}} - \mathbf{p}_g\|^2
$$

### 4.5 拟合项 $J_{\text{fitness}}$（Refine）

$$
\mathbf{x} = \frac{\mathbf{q}_{i-1} + 4\mathbf{q}_i + \mathbf{q}_{i+1}}{6} - \mathbf{r}_i, \quad
f = \frac{(\mathbf{x}\cdot\mathbf{v})^2}{a^2} + \frac{\|\mathbf{x}\times\mathbf{v}\|^2}{b^2}
$$

$a^2=25,\, b^2=1$。

### 4.6 求解器

- **L-BFGS**（`lbfgs.hpp`），回调 `costFunctionRebound` / `costFunctionRefine`
- 优化变量从索引 `order_` 起
- 若启用 `use_planning_z`：迭代中强制 $q_z = z_{\text{ref}}$，并清零 $z$ 向梯度

---

## 5. 动态 A\*

- 在 `100×100×100` 局部池内搜索
- 启发式：对角线距离 `getDiagHeu` 或曼哈顿 `getManhHeu`
- 邻居：26 连通（3D）

**平面模式**（`dyn_a_star.cpp`）：当 $\|z_{\text{start}} - z_{\text{end}}\| < 10^{-4}$ 时 `search_planar=true`：

- 扩展邻居时 **禁止 $\Delta z \neq 0$**
- 邻居 $z$ 索引固定为 `start_idx(2)`

等价于在固定高度层做 **8 邻域 2D A\***。

---

## 6. D1 改动：固定 $z$ 与高度柱

相对通用 3D 空中规划，本仓库将问题退化为 **地面机 2.5D**：

### 6.1 `setRobotPlanningZ` / `use_planning_z`

每次 `callReboundReplan`：

- `setRobotPlanningZ(odom_pos_(2))`
- `start_pt` / `local_target` 的 $z$ 与竖直速度、加速度清零

`BsplineOptimizer`：

| 操作 | 作用 |
|------|------|
| `checkOccupancy(pos)` | 查询时使用 `planning_z_`（柱检查时作为柱上沿） |
| `enforcePlanningZOnControlPoints` | 所有 $\mathbf{q}_i(2) = z_{\text{ref}}$ |
| `enforcePlanningZOnGradient` | $\partial J / \partial z = 0$ |
| `enforcePlanningZOnSolverVars` | L-BFGS 迭代中强制 $q_z$ |

参数：`manager/use_robot_z_planning`（`d1_robot.yaml`，默认 `true`）。

### 6.2 高度柱碰撞（已落地）

`grid_map/column_collision_enable: true` 时，`getInflateOccupancy` 对 $(x,y)$ 扫描

$$
[z_{\text{floor}}+\varepsilon,\ \max(z_{\text{query}}, z_{\text{floor}}+\varepsilon)]
$$

任一膨胀格占用即视为障碍（$\varepsilon=$ `column_collision_z_eps`）。详见 [06_ground_obstacle_modeling.md](06_ground_obstacle_modeling.md)。

### 6.3 地面滤波（已落地）

`ground_filter_enable` + `camera_to_ground` + `obstacle_min_height`：估计地面 $z_{\text{g}} = z_{\text{cam}} - h_{\text{cam}}$，低于 $z_{\text{g}} + h_{\min}$ 的点不当障碍。

### 6.4 随机脱困仅在 XY

插入点法向 $\mathbf{h} = (-d_y, d_x, 0)$。

### 6.5 FSM：XY 到达与近目标逻辑

- `at_goal`：$\|\mathbf{p}_{\text{odom}}^{xy} - \mathbf{p}_{\text{goal}}^{xy}\| < \texttt{goal\_reach\_thresh}$
- 轨迹时间结束但 XY 未到目标 → `[goal_timeout]` → `REPLAN_TRAJ`

---

## 7. 当前行为要点（相对早期版本）

| 主题 | 当前行为 |
|------|----------|
| 全局参考 | **平面 A\* 折线**（可绕障）+ min-snap；Hybrid 约束未上 |
| 局部重规划 | 旧 B 样条几何 warm-start + odom 运动学锚定 + 固定 $z$ |
| 安全 | 轨迹撞障优先 replan / `[SAFETY_TIER]`，分层急停；非 EXEC 不扫旧轨 |
| 全局恢复 | 急停后按漂移阈值可选重建 `global_data_` |
| 建图 | 地面滤波 + 高度柱；膨胀默认 0.20 m |

---

## 8. 关键源码

| 内容 | 文件 |
|------|------|
| 主规划循环 | `src/planner/plan_manage/src/planner_manager.cpp` |
| FSM / 重规划入口 | `src/planner/plan_manage/src/ego_replan_fsm.cpp` |
| 代价与 L-BFGS | `src/planner/bspline_opt/src/bspline_optimizer.cpp`，`include/bspline_opt/lbfgs.hpp` |
| B 样条求值 | `src/planner/bspline_opt/src/uniform_bspline.cpp` |
| A\* | `src/planner/path_searching/src/dyn_a_star.cpp` |
| 占据地图 | `src/planner/plan_env/src/grid_map.cpp` |
| 多项式初值 | `src/planner/traj_utils/src/polynomial_traj.cpp` |

---

## 9. 与控制的接口

规划成功后 FSM 发布 `traj_utils/Bspline`（`pos_pts`、`knots`、`order`、`start_time`、`traj_id`）。`traj_server` 重建 `UniformBspline` 并采样为 `PositionCommand`。见 [02_control_math.md](02_control_math.md)。
