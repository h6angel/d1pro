# 规划数学原理

本文说明 EGO Planner 在本仓库中的 **轨迹表示、初值生成、弹性带优化与 D1 平面化改动**。系统级流程见 [00_overview.md](00_overview.md)。

---

## 1. 问题表述

给定：

- 起点状态 $(\mathbf{p}_s, \mathbf{v}_s, \mathbf{a}_s)$（来自 `/odom`）
- 局部目标 $(\mathbf{p}_g, \mathbf{v}_g)$（由 FSM `getLocalTarget()` 在全局路径上选取）
- 膨胀占据地图 $\mathcal{O}$

求一条 **分段多项式参数化的均匀 B 样条轨迹** $\mathbf{p}(t)$，满足：

1. 平滑（低 jerk）
2. 与障碍保持安全距离
3. 速度、加速度不超过 `max_vel`、`max_acc`
4. 终端接近局部目标

**决策变量**：均匀 B 样条的控制点矩阵 $\mathbf{Q} \in \mathbb{R}^{3 \times N}$，每列 $\mathbf{q}_i$ 为一个控制点。

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

`EGOPlannerManager::reboundReplan` 分三步（源码：`planner_manager.cpp`）：

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

**随机脱困**（`flag_randomPolyTraj`）：在 XY 中点沿垂直于起终点方向插入扰动点，再 `minSnapTraj`。D1 开启 `use_planning_z` 时扰动仅在 **水平面**，$z = z_{\text{ref}}$。

#### 3.1.2 Warm-start 初值（局部重规划）

当 `flag_polyInit=false`：

1. 取墙钟时间 $t_{cur} = \min(t_{\text{wall}}, T_{\text{dur}})$
2. 从 $t_{cur}$ 到轨迹末端按 $t_s$ 采样 B 样条，建 **伪弧长** 表
3. 必要时在末端接一段多项式延伸到 `local_target_pt`
4. 按弧长等距重采样得 `point_set`
5. 边界导数使用 **`start_vel`、`start_acc`（odom）**，而非轨迹上的值

**混合钉点（Hybrid warm-start，commit `3f316d5`）**：

参数化得到 `ctrl_pts` 后，强制前 3 个控制点：

$$
\mathbf{q}_i = \mathbf{p}_s + \mathbf{v}_s \cdot (i \cdot t_s), \quad i = 0,1,2
$$

这样在 L-BFGS 优化时，轨迹起点与 **实测 odom** 一致，减少重规划瞬间的跳变。L-BFGS 仍从第 `order_` 个控制点开始优化，但钉点会在每步 `enforcePlanningZ` 后保持 $z$ 与 $xy$ 锚定逻辑一致。

### 3.2 STEP 1 续：`initControlPoints` 与 A*

沿初值 B 样条检测碰撞段，对每段在自由空间运行 **动态 A\***，得到绕行折线 `a_star_pathes`。

对每个可能碰撞的控制点 $i$，构造：

- **基点** $\mathbf{b}_{i,j}$：障碍边界上的参考点
- **法向** $\mathbf{n}_{i,j}$：由 A* 路径方向确定的单位向量（弹性方向）

Signed distance：

$$
d_i = (\mathbf{q}_i - \mathbf{b}_{i,j}) \cdot \mathbf{n}_{i,j}
$$

若 $d_i < \text{clearance}$，产生反弹（rebound）代价梯度。

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

默认 D1 参数见 `config/d1_robot.yaml`（`single_run.launch.py` 加载）示例：`max_vel=0.6`，`max_acc=1.0`，`optimization/dist0=0.55`，`lambda_fitness=1.5`。

### 4.1 平滑项 $J_{\text{smooth}}$（Jerk）

离散 jerk（与三次 B 样条差分一致）：

$$
\mathbf{j}_i = \mathbf{q}_{i+3} - 3\mathbf{q}_{i+2} + 3\mathbf{q}_{i+1} - \mathbf{q}_i
$$

$$
J_{\text{smooth}} = \sum_i \|\mathbf{j}_i\|^2
$$

梯度链式分配到 $\mathbf{q}_i \ldots \mathbf{q}_{i+3}$（系数 $-1,3,-3,1$）。

### 4.2 避障反弹项 $J_{\text{dist}}$

对每个弹性约束：

$$
d = (\mathbf{q}_i - \mathbf{b}) \cdot \mathbf{n}, \quad d_{\text{err}} = \text{clearance} - d
$$

- $d_{\text{err}} < 0$：无惩罚
- $0 \le d_{\text{err}} < d_0$：$J \mathrel{+}= d_{\text{err}}^3$，$\nabla_{\mathbf{q}_i} \mathrel{+}= -3 d_{\text{err}}^2 \mathbf{n}$
- $d_{\text{err}} \ge d_0$：三次多项式延拓保证 $C^1$ 连续（代码中 $a=3d_0,\, b=-3d_0^2,\, c=d_0^3$）

迭代中若轨迹足够平滑，会 `check_collision_and_rebound()` 更新弹性方向并可能 **earlyExit** 重启 L-BFGS。

### 4.3 可行性项 $J_{\text{feas}}$

离散速度、加速度（均匀结点间隔 $t_s$）：

$$
\mathbf{v}_i = \frac{\mathbf{q}_{i+1} - \mathbf{q}_i}{t_s}, \quad
\mathbf{a}_i = \frac{\mathbf{q}_{i+2} - 2\mathbf{q}_{i+1} + \mathbf{q}_i}{t_s^2}
$$

默认分支（非 `SECOND_DERIVATIVE_CONTINOUS`）：超限时加 **平方惩罚**，例如

$$
J \mathrel{+}= \bigl(\max(0,\, v_{i,j} - v_{\max})\bigr)^2 \cdot t_s^{-2}
$$

加速度项类似，对 $\mathbf{q}_i,\mathbf{q}_{i+1},\mathbf{q}_{i+2}$ 分配梯度。

### 4.4 终端项 $J_{\text{term}}$

B 样条端点位置组合（与过点约束相同形式）：

$$
\mathbf{p}_{\text{end}} = \frac{1}{6}(\mathbf{q}_{N-3} + 4\mathbf{q}_{N-2} + \mathbf{q}_{N-1})
$$

$$
J_{\text{term}} = \|\mathbf{p}_{\text{end}} - \mathbf{p}_g\|^2
$$

### 4.5 拟合项 $J_{\text{fitness}}$（Refine）

沿参考路径切向 $\mathbf{v}$ 与法向误差（`calcFitnessCost`）：

$$
\mathbf{x} = \frac{\mathbf{q}_{i-1} + 4\mathbf{q}_i + \mathbf{q}_{i+1}}{6} - \mathbf{r}_i, \quad
f = \frac{(\mathbf{x}\cdot\mathbf{v})^2}{a^2} + \frac{\|\mathbf{x}\times\mathbf{v}\|^2}{b^2}
$$

用于时间重分配后的轨迹微调，$a^2=25,\, b^2=1$。

### 4.6 求解器

- **L-BFGS**（`lbfgs.hpp`），通过 `costFunctionRebound` / `costFunctionRefine` 回调
- 优化变量：控制点序列展平为 `double[n]`，从索引 `order_` 起（前若干点由边界/钉点约束）
- 每次迭代后：若启用 `use_planning_z`，对变量与梯度执行 **$z$ 约束**（见下节）

---

## 5. 动态 A\*

- 在 `100×100×100` 局部池内搜索
- 启发式：对角线距离 `getDiagHeu` 或曼哈顿 `getManhHeu`
- 邻居：26 连通（3D）

**平面模式**（`dyn_a_star.cpp`，commit `59066c3`）：

当 $\|z_{\text{start}} - z_{\text{end}}\| < 10^{-4}$ 时 `search_planar=true`：

- 扩展邻居时 **禁止 $\Delta z \neq 0$**
- 邻居 $z$ 索引固定为 `start_idx(2)`

等价于在固定高度层做 **8 邻域 2D A\***。

---

## 6. D1 改动：固定 $z$ 的 2.5D 规划

相对原版四旋翼 3D EGO，本仓库通过以下机制将问题退化为 **固定高度的平面规划**（核心 rebound 公式未改，约束方式改变）。

### 6.1 `setRobotPlanningZ` / `use_planning_z`

每次 `callReboundReplan`：

```cpp
planner_manager_->setRobotPlanningZ(odom_pos_(2));
start_pt_(2) = local_target_pt_(2) = odom_z;
start_vel_(2) = start_acc_(2) = local_target_vel_(2) = 0;
```

`BsplineOptimizer` 中：

| 操作 | 作用 |
|------|------|
| `checkOccupancy(pos)` | 查询前令 `pos(2) = planning_z_` |
| `enforcePlanningZOnControlPoints` | 所有 $\mathbf{q}_i(2) = z_{\text{ref}}$ |
| `enforcePlanningZOnGradient` | $\partial J / \partial z = 0$ |
| `enforcePlanningZOnSolverVars` | L-BFGS 迭代中强制 $q_z = z_{\text{ref}}$ |

`reboundReplan` 入口还对 `start_pt`、`local_target_pt`、`point_set`、`ctrl_pts` 调用 `flatten*Z`。

参数：`manager/use_robot_z_planning`（`single_run.launch.py` / `d1_robot.yaml`，默认 `true`）。

### 6.2 随机脱困仅在 XY

插入点：

$$
\mathbf{p}_{\text{mid}} = \frac{\mathbf{p}_s + \mathbf{p}_g}{2} + \mathbf{h} \cdot \text{rand\_scale}
$$

其中 $\mathbf{h} = (-d_y, d_x, 0)$ 为起终点水平法向，`rand_scale` 随连续失败次数衰减。

### 6.3 FSM：XY 到达与近目标逻辑

- `at_goal`：$\|\mathbf{p}_{\text{odom}}^{xy} - \mathbf{p}_{\text{goal}}^{xy}\| < \texttt{goal\_reach\_thresh}$
- `near_goal_phase`：局部目标在 XY 上接近全局目标，或 `dist_to_goal_xy < planning_horizon`
- 轨迹时间结束但 XY 未到目标 → `[goal_timeout]` → `REPLAN_TRAJ`（commit `d705b4f`）

---

## 7. 近期 commit 与行为变更

| Commit | 摘要 | 规划侧影响 |
|--------|------|------------|
| `59066c3` | 2D plan based on robot z | `use_planning_z`、平面 A*、`flatten*Z` |
| `caf7cfb` | FSM, local replan from odom | 重规划起点改为 odom；曾短暂去掉沿轨迹 warm-start |
| `3f316d5` | replan fix | 恢复 `REPLAN_TRAJ`→`planFromCurrentTraj`；前 3 CP 钉 odom；边界导数用 odom 速度 |
| `d705b4f` | parameter & plan fix | XY 到达判定、`near_goal_phase`、调试日志；`thresh_replan_time` 等 |

**当前推荐理解**：局部重规划 = **旧 B 样条几何 warm-start** + **odom 运动学锚定** + **固定 z 平面避障**。

---

## 8. 关键源码

| 内容 | 文件 |
|------|------|
| 主规划循环 | `src/planner/plan_manage/src/planner_manager.cpp` |
| FSM / 重规划入口 | `src/planner/plan_manage/src/ego_replan_fsm.cpp` |
| 代价与 L-BFGS | `src/planner/bspline_opt/src/bspline_optimizer.cpp` |
| B 样条求值 | `src/planner/bspline_opt/src/uniform_bspline.cpp` |
| A* | `src/planner/path_searching/src/dyn_a_star.cpp` |
| 占据地图 | `src/planner/plan_env/src/grid_map.cpp` |
| 多项式初值 | `src/planner/traj_utils/src/polynomial_traj.cpp` |

---

## 9. 与控制的接口

规划成功后 FSM 发布 `traj_utils/Bspline`，包含：

- `pos_pts[]`：控制点
- `knots[]`：结点向量
- `order`、`start_time`、`traj_id`

`traj_server` 重建 `UniformBspline` 并采样为 `PositionCommand`。采样与跟踪数学见 [02_control_math.md](02_control_math.md)。
