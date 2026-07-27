# 规划算法学习与改进路线图

本文档基于当前 **D1 规划—控制栈**（动态 A\* 引导 + 均匀 B 样条 + L-BFGS rebound），整理值得学习的规划算法、与本仓库的对应关系，以及建议的学习与落地顺序。系统背景见 [00_overview.md](00_overview.md)，数学细节见 [01_planning_math.md](01_planning_math.md)。

---

## 1. 当前范式（基线）

本仓库局部规划属于 **「引导搜索 + 轨迹参数化 + 惩罚项优化」**，而非纯离散图搜索：

```
FSM 局部目标
    → STEP 1 INIT：多项式 / Warm-start 初值 + initControlPoints（动态 A* 弹性方向）
    → STEP 2 OPT：B 样条控制点 + combineCostRebound（L-BFGS）
    → STEP 3 REFINE：combineCostRefine
    → traj_server → d1_planner_bridge（cmd_vel）
```

| 模块 | 源码位置 | 作用 |
|------|----------|------|
| 重规划入口 | `planner_manager.cpp` → `reboundReplan` | 三步流水线调度 |
| 动态 A\* | `dyn_a_star.cpp` | 碰撞段绕行折线，构造 rebound 法向 |
| 弹性初值 | `bspline_optimizer.cpp` → `initControlPoints` | 控制点 + A\* 引导 |
| 代价与梯度 | `combineCostRebound` / `combineCostRefine` | 平滑、避障、可行性、终端 |
| 求解器 | `lbfgs.hpp` | L-BFGS 无约束优化 |
| 建图 / 柱碰撞 | `grid_map.cpp` | 在线占据 + 高度柱查询 |
| 执行层 | `d1_planner_bridge/` | 差速跟踪（航向 P + 速度投影） |

**改进方向概览：** 更好的初值 → 更好的障碍表示 / 调参 → 更好的优化器 → 更好的机器人运动学模型（含执行层）。

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
  - **学什么：** 动力学递推、代价二次近似、滚动时域、约束处理。
  - **为何重要：** 理解惩罚项优化的理论上限；改进 bridge 跟踪或替换 L-BFGS。
  - **可落地位置：** 优化层替代部分 `combineCostRebound`；执行层短 horizon MPC。
  - **验收：** 简单 2D / 差速模型上复现避障轨迹，对比当前单步耗时。

- [ ] **非完整运动规划：Hybrid A\* / State Lattice（建议按融合方案落地）**
  - **学什么：** $(x,y,\theta)$、运动原语；同时搞清与普通 A\* 的分工。
  - **本仓主张：** 全局避障用 A\*，去质点用 Hybrid 运动学逻辑——见 [07_astar_hybrid_fusion.md](07_astar_hybrid_fusion.md)。
  - **可落地位置：** 替换 `planGlobalTraj` 直线插点；局部 `dyn_a_star` 仍可服务 rebound。
  - **验收：** 同地图下急停恢复 / 首次规划成功率与初值可执行性。

- [ ] **ESDF（欧氏符号距离场）障碍表示**
  - **学什么：** 占据 → ESDF 增量更新；距离与梯度查表。
  - **为何重要：** 当前 rebound 依赖 A\* 折线方向 + signed distance，梯度不如 ESDF 平滑。
  - **可落地位置：** `GridMap` 增 ESDF 层；`calcDistanceCostRebound` 用 ESDF 梯度。
  - **验收：** 同场景障碍代价收敛迭代减少，或贴障更平滑。

### 优先级 2 — 同场景对照与轨迹层升级

- [ ] **TEB 研读与对比实验** — 显式差速非完整约束的连续优化对照。
- [ ] **MINCO / GCOPTER** — 最小控制多项式与安全走廊，适合改轨迹层。

### 优先级 3 — 扩展与兜底

- [ ] **RRT\* / BIT\*** — 全局断线 fallback。
- [ ] **MPPI / DWA** — 优化超时或动态障碍时的短窗反应层。
- [ ] **学习式规划** — 了解即可，暂不作为实机主线。

---

## 4. 建议落地阶段（工程顺序）

| 阶段 | 目标 | 主要改动 | 状态 |
|------|------|----------|------|
| **Phase 0** | 熟悉基线 | 读 `01`/`02`/`06`，跑通实机，记失败 case | 进行中 |
| **Phase 0.5** | 建图与安全 | 地面滤波、高度柱、`[SAFETY_TIER]`；**实机高度/膨胀参数已调妥** | **完成** |
| **Phase 1** | 强化障碍与初值 | ESDF；评估 Hybrid A\* 初值 | [ ] |
| **Phase 2** | 优化器升级 | iLQR/MPC 原型；调 `lambda` | [ ] |
| **Phase 3** | 执行层一致 | bridge 层 MPC / 非完整前馈；可选恢复横向纠偏 | [ ] |
| **Phase 4** | 架构对照 | TEB 或 MINCO 分支对比 | [ ] |

近期可合并工程项见 [REMAINING_PRS.md](../REMAINING_PRS.md)。

---

## 5. 模块级「算法 → 源码」替换对照

| 想改进的问题 | 推荐算法 | 优先改动文件 |
|--------------|----------|--------------|
| 初值路径不可执行、转弯怪 | Hybrid A\* / Lattice | `dyn_a_star.cpp`，`initControlPoints` |
| 贴障抖动、优化不收敛 | ESDF + 平滑距离代价 | `GridMap`，`calcDistanceCostRebound` |
| 惩罚项调参困难、实时性差 | iLQR / MPC | `lbfgs.hpp`，`bspline_optimizer.cpp` |
| 轨迹表示不够灵活 | MINCO / GCOPTER | `UniformBspline`，`planner_manager.cpp` |
| 全局断线 | RRT\* / BIT\* | `ego_replan_fsm.cpp` 全局航点逻辑 |
| 跟踪与规划模型脱节 | MPC @ bridge | `d1_planner_bridge_node.cpp` |
| 与业界地面栈对齐 | TEB / Nav2 SMAC | 新包或并行 launch 对比 |
| 矮障 / 抬升漏检 | 参数已调妥；换机身后再改 yaml | `d1_robot.yaml`，`grid_map.cpp` |

---

## 6. 推荐阅读顺序（论文 / 代码）

1. CHOMP → 「轨迹优化 + 障碍梯度」
2. 本仓库 `reboundReplan` + `01_planning_math.md`（对照实现，不必绑定外部项目名）
3. Hybrid A\* + Nav2 SMAC 源码  
4. iLQR 讲义 + 简单 MPC（acados）  
5. TEB + `teb_local_planner`  
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
- [地面障碍建模](06_ground_obstacle_modeling.md)
- [A\* × Hybrid 融合](07_astar_hybrid_fusion.md)
- [评估指标](03_planning_metrics.md)
