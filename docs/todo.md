# 规划算法学习与改进路线图

本文档基于当前 **EGO Planner × D1** 栈，整理值得学习的规划算法、与本仓库的对应关系，以及建议的学习与落地顺序。系统背景见 [00_overview.md](00_overview.md)，数学细节见 [01_planning_math.md](01_planning_math.md)。

---

## 1. 当前范式（基线）

EGO 在本仓库中属于 **「引导搜索 + 轨迹参数化 + 惩罚项优化」**，而非纯离散图搜索：

```
FSM 局部目标
    → STEP 1 INIT：多项式 / Warm-start 初值 + initControlPoints（动态 A* 弹性方向）
    → STEP 2 OPT：B 样条控制点 + combineCostRebound（L-BFGS）
    → STEP 3 REFINE：combineCostRefine
    → traj_server → d1_planner_bridge（cmd_vel）
```

| 模块 | 源码位置 | 作用 |
|------|----------|------|
| 重规划入口 | `src/planner/plan_manage/src/planner_manager.cpp` → `reboundReplan` | 三步流水线调度 |
| 动态 A* | `src/planner/path_searching/src/dyn_a_star.cpp` | 碰撞段绕行折线，构造 rebound 法向 |
| 弹性初值 | `src/planner/bspline_opt/src/bspline_optimizer.cpp` → `initControlPoints` | 控制点 + A* 引导 |
| 代价与梯度 | `bspline_optimizer.cpp` → `combineCostRebound` / `combineCostRefine` | 平滑、避障、可行性、终端 |
| 求解器 | `src/planner/bspline_opt/src/gradient_descent_optimizer.cpp` | L-BFGS / 梯度下降 |
| 执行层 | `src/d1_planner_bridge/` | 差速跟踪，非完整约束主要在此体现 |

**改进方向概览：** 更好的初值 → 更好的障碍表示 → 更好的优化器 → 更好的机器人运动学模型。

---

## 2. 学习优先级总览

```mermaid
flowchart LR
  subgraph P1["优先级 1"]
    OC["最优控制 iLQR / MPC"]
    HA["Hybrid A*"]
    ESDF["ESDF 距离场"]
  end
  subgraph P2["优先级 2"]
    TEB["TEB 弹性带"]
    MINCO["MINCO / GCOPTER"]
  end
  subgraph P3["优先级 3"]
    RRT["RRT* / BIT*"]
    MPPI["MPPI / DWA"]
  end
  P1 --> P2 --> P3
```

---

## 3. 待办清单（按优先级）

### 优先级 1 — 基础能力（建议先学、收益最大）

- [ ] **最优控制：LQR → iLQR → MPC**
  - **学什么：** 动力学递推、代价二次近似、滚动时域、约束处理（软/硬惩罚、QP 子问题）。
  - **为何重要：** 理解 EGO 惩罚项优化的理论上限；为替换 L-BFGS 或改进 `d1_planner_bridge` 跟踪打基础。
  - **可落地位置：**
    - 优化层：替代 `gradient_descent_optimizer` + `combineCostRebound` 的求解方式；
    - 执行层：在 bridge 做短 horizon MPC，减轻 B 样条与底盘模型不一致。
  - **参考资料：** Bertsekas《Dynamic Programming and Optimal Control》；MIT Underactuated Robotics（iLQR）；acados / OSQP 示例。
  - **验收：** 能用 iLQR 在简单 2D 点质量或差速模型上复现一段避障轨迹，并与当前 EGO 单步耗时对比。

- [ ] **非完整运动规划：Hybrid A* / State Lattice**
  - **学什么：** $(x,y,\theta)$ 搜索、Dubin / Reeds-Shepp、运动原语库、启发式设计。
  - **为何重要：** 当前 `dyn_a_star` 为 **几何栅格 A***，对 D1 差速「不能横移」约束弱；差初值会导致 B 样条优化陷局部极小。
  - **可落地位置：**
    - 替换或并联 `dyn_a_star.cpp`，为 `initControlPoints` 提供更可执行的折线；
    - FSM 全局航点链（`ego_replan_fsm.cpp`）上的全局层。
  - **参考资料：** Nav2 **SMAC Planner** 源码与文档；Hybrid A* 原始论文。
  - **验收：** 在相同占据地图上，对比几何 A* 与 Hybrid A* 初值下 `reboundReplan` 成功率与轨迹可行性。

- [ ] **ESDF（欧氏符号距离场）障碍表示**
  - **学什么：** 体素占据 → ESDF 增量更新；距离与梯度的查表。
  - **为何重要：** 当前 rebound 依赖 A* 折线方向 + signed distance 惩罚，梯度在障碍附近不如 ESDF 平滑稳定。
  - **可落地位置：**
    - `plan_env` / `GridMap`：维护 ESDF 层；
    - `calcDistanceCostRebound`：用 ESDF 梯度替代或辅助弹性法向。
  - **参考资料：** FIESTA、voxblox ESDF、nvblox 文档。
  - **验收：** 同场景下障碍代价收敛迭代次数减少，或贴障轨迹更平滑。

---

### 优先级 2 — 同场景对照与轨迹层升级

- [ ] **TEB（Timed Elastic Band）研读与对比实验**
  - **学什么：** 时间弹性带、障碍距离 + 速度轮廓联合优化、差速非完整约束进代价。
  - **为何重要：** 地面机器人局部规划事实标准之一，与 EGO 同属「连续优化」但 **显式考虑底盘运动学**。
  - **可落地位置：**
    - 概念对照：优化变量、障碍耦合、实时性；
    - 可选：Nav2 + TEB 与当前栈做 A/B（同地图、同目标点）。
  - **参考资料：** `teb_local_planner` ROS 包；TEB 原始论文。
  - **验收：** 整理一篇内部对比笔记（约束建模、实时性、贴障行为、参数敏感度）。

- [ ] **MINCO / GCOPTER 轨迹表示与约束**
  - **学什么：** 最小控制多项式、安全走廊、时空联合、与 B 样条代价设计的差异。
  - **为何重要：** 与 EGO 同属 FAST-Lab 一脉，适合 **改轨迹层** 而非推倒感知。
  - **可落地位置：**
    - 长期可替代 `UniformBspline` + 部分 `combineCost*` 项；
    - 短期可先读 GCOPTER 走廊约束，启发 rebound 设计。
  - **参考资料：** MINCO、GCOPTER 论文与开源实现。
  - **验收：** 在仿真中跑通官方 demo，并列出与 `reboundReplan` 三步的模块映射表。

---

### 优先级 3 — 扩展与兜底

- [ ] **采样法：RRT* / Informed RRT* / BIT***
  - **学什么：** 随机采样、渐近最优性、 informed 椭圆采样。
  - **适用场景：** 全局路断裂、几何/Hybrid A* 失败时的 **fallback**；复杂拓扑初值。
  - **可落地位置：** FSM 全局层或 `reboundReplan` 失败后的重试策略。
  - **验收：** 构造 A* 失败用例，验证 RRT* fallback 能否恢复规划。

- [ ] **反应式局部：MPPI / DWA**
  - **学什么：** 速度空间采样、轨迹 rollout 加权、短 horizon 反应。
  - **适用场景：** 优化器超时、动态障碍、安全壳。
  - **可落地位置：** 与 EGO 并联的短窗层，或 bridge 层应急。
  - **参考资料：** Nav2 MPPI Controller。
  - **验收：** 了解原理即可；有动态障碍需求时再做单模块原型。

- [ ] **学习式规划（扩散策略等）— 了解即可**
  - **说明：** 研究热点，可解释性与工程可控性仍弱于优化法；暂不作为 D1 实机主线。

---

## 4. 建议落地阶段（工程顺序）

与「先读书」并行时，推荐按 **改动风险从低到高** 推进：

| 阶段 | 目标 | 主要改动 | 状态 |
|------|------|----------|------|
| **Phase 0** | 熟悉基线 | 读 `01_planning_math.md`，跑通实机/仿真，记录失败 case | [ ] |
| **Phase 1** | 强化障碍与初值 | ESDF 或改进距离查询；评估 Hybrid A* 初值 | [ ] |
| **Phase 2** | 优化器升级 | iLQR/MPC 原型替换部分 rebound 求解；调 `lambda` 与约束 | [ ] |
| **Phase 3** | 执行层一致 | bridge 层 MPC / 非完整约束前馈 | [ ] |
| **Phase 4** | 架构对照 | TEB 或 MINCO 分支对比，决定是否替换整条局部链 | [ ] |

---

## 5. 模块级「算法 → 源码」替换对照

便于改代码时快速定位：

| 想改进的问题 | 推荐算法 | 优先改动文件 |
|--------------|----------|--------------|
| 初值路径不可执行、转弯怪 | Hybrid A* / Lattice | `dyn_a_star.cpp`，`initControlPoints` |
| 贴障抖动、优化不收敛 | ESDF + 平滑距离代价 | `GridMap`，`calcDistanceCostRebound` |
| 惩罚项调参困难、实时性差 | iLQR / MPC | `gradient_descent_optimizer.cpp`，`combineCostRebound` |
| 轨迹表示不够灵活 | MINCO / GCOPTER | `UniformBspline`，`planner_manager.cpp` |
| 全局断线 | RRT* / BIT* | `ego_replan_fsm.cpp`，全局航点逻辑 |
| 跟踪与规划模型脱节 | MPC @ bridge | `d1_planner_bridge_node.cpp` |
| 与业界地面栈对齐 | TEB / Nav2 SMAC | 新包或并行 launch 对比 |

---

## 6. 推荐阅读顺序（论文 / 代码）

1. CHOMP → 理解「轨迹优化 + 障碍梯度」鼻祖  
2. EGO-Planner 论文 → 对照本仓库 `reboundReplan`  
3. Hybrid A* + Nav2 SMAC 源码  
4. iLQR 讲义 + 简单 MPC 实现（acados）  
5. TEB 论文 + `teb_local_planner`  
6. GCOPTER / MINCO → 走廊与最小控制  
7. FIESTA / voxblox → ESDF 工程实现  

---

## 7. 记录区（自行补充）

### 失败 case 库

| 日期 | 场景描述 | 现象 | 归因假设 | 对应待办 |
|------|----------|------|----------|----------|
| | | | | |

### 实验记录

| 日期 | 实验 | 结论 | 下一步 |
|------|------|------|--------|
| | | | |

---

## 8. 相关文档

- [系统总览](00_overview.md)
- [规划数学原理](01_planning_math.md)
- [控制数学原理](02_control_math.md)
