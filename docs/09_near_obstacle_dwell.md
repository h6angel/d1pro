# 近障贴障时间过长：原因与解决思路

本文说明：在 [08_near_obstacle_start_collision.md](08_near_obstacle_start_collision.md) 落地「近障防撞」门禁之后，真机上出现的新现象——**没有撞上障碍，但在障碍旁停留 / 磨蹭很久**——其机制原因、与「重规划禁速」的关系，以及在 **「不撞障优先、贴障时间次优」** 原则下的落地改动方案（见 §5–§6）。

相关文档：[系统总览](00_overview.md)、[近障起步撞障分析](08_near_obstacle_start_collision.md)、[A\* + Hybrid 融合](07_astar_hybrid_fusion.md)、[地面障碍建模](06_ground_obstacle_modeling.md)。

实机主证据：`ego_log/stack_20260729_164015`（完整 8 次到点）、`ego_log/stack_20260729_155448`（起跑贴障卡死）。对照优化前撞障：`ego_log/stack_20260729_145142`（见文档 08）。

---

## 1. 现象定义：什么叫「贴障时间长」

### 1.1 不是什么

| 误解 | 实际情况 |
|------|----------|
| 车贴着障碍蹭了很久然后撞上了 | 新门禁下多段 log **未发生** odom `z` 崩塌 / Depth Lost 导航坠毁 |
| 避障逻辑坏了、不起作用 | 相反：门禁在**反复打断危险下发**，安全目标基本达成 |
| 「凡重规划一律禁止速度」 | 只在部分状态**显式清零**；`publish_gate` 拒发时是**禁发新轨**，旧轨未必停 |

### 1.2 是什么

障碍在侧向或前方约 \(< 0.5\text{–}1\,\mathrm{m}\) 时，车出现：

1. **突然刹停**（`vel≡0` / `hold=1`）一小段；
2. 或 **挪几厘米 → 再停 → 再重规划**；
3. 绕开 / 离开障碍耗时明显变长；极端时目标在数米外，车在起点附近晃几十秒，净位移接近 0。

一句话：**用时间换安全**——不再假成功直接撞，但通行变「肉」、贴障磨蹭。

### 1.3 与文档 08 的边界

| 文档 | 核心问题 | 期望日志特征 | 状态 |
|------|----------|--------------|------|
| [08](08_near_obstacle_start_collision.md) | 近障起步 **撞障** | 擦障 `bspline_publish` → `z` 崩 → `Depth Lost` | 初版门禁已落地 |
| **本文 09** | 门禁生效后 **贴障停留过久** | `[near_obs]` / `[publish_gate]` / `odom_anomaly` / SAFETY 重规划密、前进慢 | 阶段 0 已落地（gate 停旧轨 + REPLAN 近障先停）；阶段 1+ 待做 |

不要把「贴障久」误判成停速闩锁失效：到点后的 `[stop_traj]` + `traj_hold_stop` 仍正常；贴障磨蹭发生在 **有目标、执行 / 重规划过程中**。

---

## 2. 实机证据摘要

启动配置（三份有规划痕迹的 log 一致）：

```text
[fsm] ... safety_slowdown=1 fail_estop_count=2 publish_gate=1
      near_obs_r=0.60 odom_anomaly_hold=1
```

### 2.1 `stack_20260729_164015`（主验收）

| 项 | 数值 / 现象 |
|----|-------------|
| 到点 | 8 次 `goal_reached` + `stop_traj` |
| 路径长度 | tracking_trace 约 \(31\,\mathrm{m}\)，odom \(z \in [-0.09, 0.06]\) |
| `[publish_gate]` | ~18 次拒发 |
| `[near_obs] stop then plan` | 7 次 |
| `hybrid` 缩 R / skip | shrink 多次；`start_yaw_blend skipped` ×2 |
| `odom_anomaly → EMERGENCY_STOP` | 10 次（持 bspline 零速） |
| `GEN_NEW` 失败 8 次放弃 | 1 次（大转角 `dyaw≈2.16`） |
| `traj_hit` / `SAFETY_HOLD` / `imminent` | 0（多靠 gate + 重规划 + odom hold） |

典型「贴障很久」段：G7→G8 约 **168 s**，含失败 8 次原地 hold、再起步后密集 REPLAN + gate 风暴、末段 ESTOP + near_obs。

### 2.2 `stack_20260729_155448`（卡死典型）

| 项 | 数值 / 现象 |
|----|-------------|
| 目标 | 约 \(3\,\mathrm{m}\) 外 |
| 运动 | path ≈ \(0.49\,\mathrm{m}\)，净位移 ≈ \(0.02\,\mathrm{m}\) |
| `[publish_gate]` | 5 次；常与同秒 `bspline_publish` 交替 |
| `[SAFETY]: EXEC→EXEC` | 5 次（轨迹段碰膨胀 → 重规划「成功」续跑） |
| `[near_obs]` | 0（本段未走 GEN_NEW 近障先停） |
| 结果 | **未到点**；bridge 仍发不大的 `cmd_vel`，局部 `cmd_pos` 只比机身远几厘米 |

说明：gate / SAFETY **在挡危险长轨**，但策略退化成「拒一条、再发一条短轨」，走廊不够时会无限磨蹭。

### 2.3 `stack_20260729_154916`（对照：门禁未上场）

有目标、约每 2.5 s 正常 publish，但净位移 ≈ \(0.04\,\mathrm{m}\)，**0 次 gate / near_obs / SAFETY**。更像执行 / 底盘未真正跟住，**不能用来评价避障门禁**，也不应归因于「贴障保护」。

### 2.4 与优化前 `145142` 对照

文档 08：近障假成功下发 → 物理撞击 → VIO `z` 崩 → Depth Lost。  
本文相关 log：**无同类坠毁**。安全维度成功；通行维度引入贴障停留。

---

## 3. 当前已落地的相关逻辑（为何会停）

参数见 `src/planner/plan_manage/config/d1_robot.yaml`；实现见 `ego_replan_fsm.cpp` / hybrid / traj_server。

| 机制 | 参数 / 日志关键字 | 对速度与时间的影响 |
|------|-------------------|-------------------|
| 发布碰撞门禁 | `publish_collision_gate_enable`；`[publish_gate]` | 擦障 B 样条**不下发**；旧轨可能仍播 |
| 近障先停再规划 | `near_obstacle_stop_before_plan` + `near_obstacle_check_radius`；`[near_obs] … stop then plan` | GEN_NEW 前若近障且 \(\|v\|>0.05\) → `callEmergencyStop` |
| 禁 random escape | `near_obstacle_block_escape`；`skip random poly escape` | 减少「假成功」；规划更易失败 → 停着重试 |
| Hybrid 圆弧占障 | `hybrid_blend_occ_check` + `r_shrink`；`occupied, try shrink` / `skipped` | 大转角初值变保守；失败则更难起步 |
| GEN_NEW 退避重试 | `gen_new_traj_max_failures: 8` + backoff | 失败期间 hold；满 8 次 → WAIT_TARGET 零速 |
| 执行期轨迹扫描 | `[SAFETY]: EXEC→EXEC` 或 `SAFETY_TIER` | 碰障则重规划 / hold / ESTOP |
| Odom 异常刹停 | `odom_anomaly_hold_enable`；`odom_anomaly → EMERGENCY_STOP` | 非纯避障，但会清零并拉长恢复 |
| 到点停速闩锁 | `[stop_traj]` / `traj_hold_stop` | 到点后 `vel≡0`（与贴障磨蹭无关） |

GEN_NEW 近障先停（代码意图）：

```text
// Near-obstacle: zero cmd first so a failed/escape plan cannot leave residual speed.
if (near_obstacle_stop_before_plan_ && 机身在障或半径内有障 && |v_xy| > 0.05)
  callEmergencyStop(odom_pos_);
然后 planFromGlobalTraj(...);
```

发布门禁：

```text
if (rebound 成功 && publish_gate 开启 && 整轨膨胀碰撞扫描不通过)
  打 [publish_gate] 日志，return false;  // 不 publish bspline
```

---

## 4. 因果链：贴障时间是怎么被拉长的

```text
车靠近障碍 / 大转角离开
        │
        ├─① [near_obs] GEN_NEW 前先停          ← 突然刹住
        │
        ├─② hybrid 缩 R / skip + rebound 难成功 ← 停着重试（backoff）
        │
        ├─③ [publish_gate] 拒发擦障新轨         ← 新速发不出去
        │     └─ 同一次 WithEscape 下一条 trial
        │        仍可能「刚好过门」并下发短/擦边轨
        │
        ├─④ 执行中 SAFETY 再重规划              ← 障边磨蹭、前进很慢
        │     （155448：EXEC→EXEC 密、净位移≈0）
        │
        └─⑤ 偶发 odom_anomaly 再急停            ← 又清零，再走 ①
                │
                ▼
        没撞上，但「贴障时间」被拉得很长
```

### 4.1 分步说明

**① 近障先停再规划**  
仅在 `GEN_NEW_TRAJ`：近障且仍在动 → emergency stop。设计目的是避免残速 + 失败规划把车拱进障。体感：障边突然停住。

**② 规划本身变难**  
近障 + 大 `dyaw`：圆弧占障 → 缩半径 → skip 折线；rebound 大量 `plan_success=0`。FSM 按 `0.25, 0.75, …` 秒退避，最多 8 次。重试窗口内基本是零速 hold。满 8 次则：

```text
GEN_NEW_TRAJ failed 8 times ... Holding stop; ... → WAIT_TARGET
```

安全放弃，但需要人工重发目标（`164015` 中 `dyaw≈2.16` 出现过）。

**③ Gate 是「单次 trial 否决」，不是「态势升级」**  
`callReboundReplan` 里一条被 gate 打掉后，外层 `WithEscape` 的下一次 trial 仍可能成功发布。日志上常见 **同一秒 `GATE` 紧跟 `PUB`**（`155448`、`164015` gate 风暴段）。  
因此：危险轨被挡了，但系统仍在「换一条勉强能发的轨」，而不是「认定走廊不够 → 停稳换全局策略」。

**④ SAFETY 成功重规划会掩盖「过不去」**  
`[SAFETY]: from EXEC_TRAJ to EXEC_TRAJ` 表示重规划成功并续跑。若新轨仍然贴障/很短，车几乎不前进，却不会触发「失败 streak → ESTOP / 放弃」。`155448` 整段任务失败即此模式。

**⑤ Odom 异常与避障抢控制权**  
`164015` 中 10 次 ESTOP 均为 `odom_anomaly`，不是 `traj_hit imminent`。每次 hold → GEN_NEW → 可能再触发 near_obs 先停，显著拉长贴障/恢复时间。易被误读成「重规划禁速」。

### 4.2 关于「重规划禁止速度分发」

| 状态 | 速度行为 |
|------|----------|
| `near_obs` stop_before_plan | **显式清零** 再规划 |
| `odom_anomaly` / 部分 SAFETY_HOLD | **显式 hold** bspline，`vel≡0` |
| GEN_NEW 失败退避 / 满 8 次放弃 | **保持 stop / hold** |
| `publish_gate` 失败 | **不发新轨**；旧轨可能仍在播（未必零速） |
| 普通 `REPLAN_TRAJ` 成功 | 发新轨，正常非零速度 |
| 普通 `REPLAN_TRAJ` 失败且未到点 | 留在 REPLAN 重试；旧轨可能仍播 |

结论：贴障久 **确实** 与「保护逻辑打断速度」有关，但不是单一的「REPLAN 全局禁速开关」。

---

## 5. 原则与链路重审

### 5.1 最高原则

| 优先级 | 要求 | 含义 |
|--------|------|------|
| **P0** | **绝不撞障** | 不得削弱文档 08 已落地的门禁；宁可停 / 放弃，不可假成功下发擦障轨 |
| **P1** | 贴障时间尽量短 | 在 P0 成立前提下，减少「拒一条再发一条短轨」空转、无谓 ESTOP、大转角干停 |

约束（硬）：

1. **不得关闭 / 放宽**：`publish_collision_gate_enable`、`near_obstacle_block_escape`、`near_obstacle_stop_before_plan`（GEN_NEW）、膨胀碰撞扫描标准。  
2. **通行优化只能走「更干净地停 → 换策略 → 再走」**，不能走「更容易过 gate / 更松 escape」。  
3. 任何「减 ESTOP / 加快通行」改动，必须能证明**不增加**近障起步撞崩风险（回归文档 08 场景）。

### 5.2 当前链路按原则审视

```text
目标到达
  │
  ├─ GEN_NEW: near_obs 先停 → planFromGlobalTraj → gate → publish
  │     ✅ 先停 = P0；❌ 只绑 GEN_NEW，REPLAN 仍可带残速
  │
  ├─ callReboundReplanWithEscape: warm → poly → (可选) random escape
  │     ✅ body 在障时 skip escape = P0；
  │     ❌ gate 否决单次 trial 后下一 trial 仍可「刚好过门」发短轨
  │
  ├─ EXEC: SAFETY 扫轨 → planFromCurrentTraj 成功则 EXEC→EXEC
  │     ✅ 碰障会拦；
  │     ❌ 「成功换轨」重置 fail streak，走廊不够时无限磨蹭（155448）
  │
  ├─ publish_gate 拒发
  │     ✅ 擦障新轨不下发 = P0；
  │     ❌ 旧轨可能仍播，近障时继续往障边蹭（P0 缺口）
  │
  └─ odom_anomaly ESTOP
        ✅ 撞前 z 崩兜底（文档 08）；
        ❌ 日常小跳也会 ESTOP，拉长贴障恢复（仅 P1，改时须保留硬 |Δz|）
```

| 环节 | 对 P0 | 对 P1（贴障时间） | 结论 |
|------|-------|-------------------|------|
| `publish_gate` | 关键保护 | 拒发本身会停顿 | **保留**；补「近障拒发时停旧轨」 |
| `near_obs` GEN_NEW 先停 | 防残速拱进障 | 突然刹住 | **保留**；扩展到 REPLAN |
| `block_escape` | 防假成功 | 规划更易失败 | **保留**，禁止为加速而重开 |
| SAFETY `EXEC→EXEC` | 碰障会拦 | 短轨续跑磨蹭 | **保留扫描**；加「无进展升级」 |
| GEN_NEW 8 次放弃 | 安全停等重发 | 大转角干停久 | **保留上限**；后期加可执行脱出（仍过 gate） |
| `odom_anomaly` | 撞崩兜底 | 误触发拉长 | **硬阈值保留**；只收紧误触发路径 |

一句话：当前最大 P0 缺口是 **gate 拒发后旧轨仍可能蹭障** + **REPLAN 近障未先停**；最大 P1 浪费是 **无进展仍「成功换轨」**。优化顺序必须先堵 P0 缺口，再用「停稳 + 全局重搜 / 放弃」吃掉磨蹭，而不是放松门禁。

---

## 6. 落地改动方案（具体逻辑）

目标表述：**先把「贴着障还在动」收干净，再把「停着空转」升级成可恢复策略**；全程新轨仍必须过 `publish_gate`。

### 6.1 阶段 0 — 安全收口（先做，允许略增「肉」感）

目的：堵住「新轨被拒但旧轨仍蹭」「REPLAN 带着速度钻近障」两类 P0 风险。贴障时间可能略升，可接受。

**状态：已落地**（`ego_replan_fsm` + `d1_robot.yaml`：`publish_gate_stop_old_traj_near_obs`）。

#### 改动 0-A：Gate 拒发且近障 → 停旧轨

**位置**：`callReboundReplan` 中 `[publish_gate]` 分支（约现有 `return false` 前）。

**逻辑**：

```text
if (publish_gate 开启 && !isLocalTrajCollisionFree(...))
  打 [publish_gate] 日志
  if (have_odom_ &&
      (isOdomBodyInObstacle() || isObstacleNearOdom(near_obstacle_check_radius_)))
    callEmergencyStop(odom_pos_)   // 或 publishStopTraj，与现有停速路径一致
    打 [publish_gate] near_obs — stop old traj
  return false;   // 仍当 plan 失败，外层 WithEscape / GEN_NEW 退避照旧
```

**为何安全**：新轨本就不发；额外保证执行侧不再沿旧擦边轨蠕动。  
**参数**：可加 `fsm/publish_gate_stop_old_traj_near_obs: true`（默认 true），便于对照实验。  
**不做**：放宽 gate 扫描、增大 `skip_start_m`。

#### 改动 0-B：近障时 REPLAN / SAFETY 重规划先钳速

**位置**：`planFromCurrentTraj`（`REPLAN_TRAJ` 与 SAFETY 回调共用）。

**逻辑**：

```text
start_pt_ = odom_pos_
start_vel_ = odom_vel_
start_acc_ = 0

if (near_obstacle_stop_before_plan_ &&
    (isOdomBodyInObstacle() || isObstacleNearOdom(near_obstacle_check_radius_)))
  if (|v_xy| > 0.05)
    callEmergencyStop(odom_pos_)          // 与 GEN_NEW 对称：先清零
    打 [near_obs] stop before REPLAN/SAFETY plan
  start_vel_.setZero()                    // 规划初值不用残速
  // 可选：短 hold 标志，本周期只停不 plan，下一周期再 planFromCurrentTraj

return callReboundReplanWithEscape(...)
```

**为何安全**：与 GEN_NEW 的 near_obs 意图对齐，避免「执行中全速试轨 → 钻进算不出的区」。  
**不做**：关闭 near_obs、缩小 `near_obstacle_check_radius` 来少停。

**阶段 0 验收**：文档 08 近障起步仍无假成功撞崩；gate 拒发时 `cmd_vel` / bspline 应迅速到零（可用 `155448` 同场景对照「拒发后是否还在蹭」）。

---

### 6.2 阶段 1 — 无进展升级（主治贴障磨蹭，且不放松门禁）

目的：走廊不够时，从「拒一条 / 换一条短轨」升级为 **停稳 → 全局重搜 → 再 GEN_NEW**；仍过不去则明确放弃。这是 **用策略升级换时间**，不是用更松的规划换时间。

#### 改动 1-A：贴障无进展计数器

**位置**：`ego_replan_fsm` 成员 + `callReboundReplan` / SAFETY 成功路径 / EXEC 周期。

**状态**（建议）：

```text
dwell_window_start_pos_xy
dwell_window_t0
dwell_gate_count          // 窗口内 publish_gate 次数
dwell_safety_replan_ok    // 窗口内 SAFETY 成功 EXEC→EXEC 次数
dwell_escalate_count      // 已触发「全局重搜」次数（封顶）
```

**触发条件**（初值可 yaml 化，示例）：

```text
窗口 W = 2.0 s 内：
  |Δxy| < dwell_min_progress_m   (如 0.10)
  AND (dwell_gate_count + dwell_safety_replan_ok) >= dwell_event_thresh  (如 3)
→ 触发 escalateNearObstacleDwell()
窗口滑动或进度足够则清零计数。
```

**注意**：只统计「近障相关」事件（gate / SAFETY），**不要**把正常开阔地 REPLAN 算进去；可用 `isObstacleNearOdom` 作门闩。

#### 改动 1-B：`escalateNearObstacleDwell()` 行为

```text
escalateNearObstacleDwell():
  1. callEmergencyStop(odom_pos_)           // P0：先停
  2. pending_safety_global_replan_ = true   // 复用已有「从 odom 强制全局重搜」
  3. resetGenNewTrajRetry()
  4. dwell_escalate_count++
  5. 打 [dwell_escalate] gate=N safety_ok=M dxy=.. escalate=K
  6. if (dwell_escalate_count >= max，如 2):
       have_target_ = false
       callEmergencyStop
       → WAIT_TARGET
       打 [dwell_escalate] give up — resend goal
     else:
       → GEN_NEW_TRAJ
```

**与现有 SAFETY 失败路径的关系**：`handleTrajHitAfterReplanFailed` 已是「失败 → hold → 全局重搜」；本改动补的是 **「成功换轨但几乎不前进」** 的对称升级（正是 `155448` 模式）。  
**不做**：升级时临时关 gate / 开 random escape。

#### 改动 1-C（可选，同阶段）：Gate 连续失败计入 GEN_NEW 失败语义

若当前已在 `GEN_NEW_TRAJ`，且同一次 `planFromGlobalTraj` / WithEscape 内多次 trial 均 gate 失败 → 本就走 `onGenNewTrajPlanFailed`；确认 **gate 的 `return false` 已计入失败 streak**（现状如此）。阶段 1 重点是 EXEC 侧无进展，不必重复造 GEN_NEW 轮子。

**阶段 1 验收（`155448` 类）**：侧向/前方障碍、前方目标 → 数秒内应出现 `[dwell_escalate]` 与全局重搜；不得再出现「分钟级短轨磨蹭、净位移≈0」；若两次 escalate 仍无走廊 → `WAIT_TARGET` + 明确日志，车保持零速。

---

### 6.3 阶段 2 — 减少无谓打断（仅 P1，且保撞崩兜底）

目的：缩短 `164015` 式 odom 误 ESTOP 拉长的恢复；**不得削弱真撞时的 z 崩检测**。

#### 改动 2-A：分层触发

```text
硬路径（保留，文档 08）：
  |Δz| >= odom_z_jump_thresh → 立即 pending ESTOP（可保持 0.15 或仅微调）

软路径（收紧误报）：
  SUSPECT_JUMP && implied_v >= odom_anomaly_implied_v
  → 改为连续 N 帧（如 2～3）才 pending；单帧只打日志
```

**不做**：为减 ESTOP 大幅提高 `|Δz|` 阈值到「撞了还不触发」；不做「贴障时禁用 odom_anomaly」。

**阶段 2 验收**：开阔多航点误 ESTOP 次数下降；近障真撞 / 大 `|Δz|` 仍能 ESTOP；文档 08 回归通过。

---

### 6.4 阶段 3 — 大转角可执行脱出（通行增强，仍过门禁）

目的：减少「hybrid skip → 8 次失败 → 只能重发」；**新策略产出的轨必须仍过 `publish_gate`**。

#### 改动 3-A（推荐先做）：skip 圆弧后「先转正再走」

**位置**：`applyHybridCurvatureL` skip 之后，或 FSM 在 GEN_NEW 检测到 `|dyaw| > thresh` 且 near_obs。

**逻辑**：

```text
if (start_yaw_blend skipped && |dyaw| > yaw_align_thresh):
  方案甲（FSM 子步骤）：
    发原地/极小半径转向指令对齐全局路径切向（或短 yaw-only 轨）
    对齐后再 planFromGlobalTraj（折线/直线初值）
  方案乙（规划侧）：
    全局 path 前插「当前位姿 → 航向对齐点」再 rebound
  无论甲乙：最终 bspline 仍走 publish_gate；失败则既有 backoff / 8 次放弃
```

#### 改动 3-B（可选）：自由空间侧向脱出点

近障时在膨胀 **free** 区选侧向点 → 接入 A\* 全局，再局部优化；**禁止**占用内 random escape。

**阶段 3 验收**：`|dyaw|>1.5` 近障离开少出现「8 次失败只能重发」；全程无擦障 publish；gate / near_obs 日志仍正常。

---

### 6.5 明确不做 / 延后

| 想法 | 原因 |
|------|------|
| 关 gate / 增大 skip_start / 近障开 random escape | 直接违反 P0，会回到文档 08 撞崩 |
| 为减贴障时间缩小 `near_obstacle_check_radius` | 削弱先停保护，优先级错误 |
| 无进展时「强制发最短可行轨」绕过 gate | 假成功风险 |
| 一上来大改 hybrid / 优化器权重「更敢转弯」 | 收益不确定且易引入擦障；放在阶段 3 且必须过 gate |

---

### 6.6 推荐落地顺序（按原则重排）

```text
阶段 0-A  Gate 近障拒发停旧轨     ← 堵 P0 缺口
阶段 0-B  REPLAN/SAFETY 近障先停   ← 堵 P0 缺口
阶段 1    无进展 escalate + 全局重搜 / 放弃  ← 主治 155448 磨蹭
阶段 2    odom 软路径减误触发      ← 治 164015 无谓打断（保硬 |Δz|）
阶段 3    大转角转正 / 侧向脱出    ← 治干停与 8 次放弃
```

与旧「A→B→D→C→E」的差异：**E（停旧轨）与 B（REPLAN 先停）升为阶段 0**，因为它们首先服务「不撞」；原 A（失败升级）仍是贴障时间主手段，但排在安全收口之后。

### 6.7 验收场景与日志

| 场景 | 期望（P0 优先） |
|------|-----------------|
| 近障起步（文档 08） | **不得**假成功撞崩；可见 `[publish_gate]` / `[near_obs]` |
| 起跑贴障（`155448`） | 先停干净；数秒内 `[dwell_escalate]` 或明确放弃；禁止无限短轨磨蹭 |
| 大转角近障（`164015` G7） | 尽量转正脱出；失败则零速等重发，不撞 |
| 开阔多航点 | 到点 `stop_traj` 正常；阶段 2 后误 ESTOP 下降 |

| 关键字 | 含义 |
|--------|------|
| `[near_obs] … stop then plan` | GEN_NEW /（改后）REPLAN 近障先停 |
| `[publish_gate]` | 发布前门禁拒发 |
| `[publish_gate] … stop old traj` | 阶段 0-A：拒发后停旧轨 |
| `[dwell_escalate]` | 阶段 1：无进展升级 |
| `[SAFETY]: EXEC→EXEC` | 执行期撞轨后重规划成功续跑 |
| `odom_anomaly → EMERGENCY_STOP` | 里程计异常刹停 |
| `GEN_NEW_TRAJ failed 8 times` | 规划放弃、零速等重发 |
| `goal_reached` / `stop_traj` / `traj_hold_stop` | 到点停速（对照） |

**主要改动文件**：`ego_replan_fsm.cpp` / `.h`、`d1_robot.yaml`；阶段 3 另及 `planner_manager.cpp`（`applyHybridCurvatureL`）。

---

## 7. 关键文件索引

| 模块 | 路径 |
|------|------|
| FSM / GEN_NEW / REPLAN / 安全 / gate | `src/planner/plan_manage/src/ego_replan_fsm.cpp` |
| Hybrid 起步圆弧 | `src/planner/plan_manage/src/planner_manager.cpp`（`applyHybridCurvatureL`） |
| Rebound 优化 | `src/planner/bspline_opt/src/bspline_optimizer.cpp` |
| 停速闩锁 | `src/planner/plan_manage/src/traj_server.cpp` |
| 参数 | `src/planner/plan_manage/config/d1_robot.yaml` |
| 撞障根因（前序） | [08_near_obstacle_start_collision.md](08_near_obstacle_start_collision.md) |
| 主 log | `ego_log/stack_20260729_164015/`、`ego_log/stack_20260729_155448/` |

---

## 8. 一句话结论

近障门禁把「假成功 → 撞障」改成了「拒发 / 先停 / 重试」；贴障时间长是保护副作用，外加 **gate 拒发后旧轨仍蹭、REPLAN 未近障先停、成功换轨却无进展、odom 误 ESTOP**。  
落地原则是 **P0 不撞 > P1 少磨蹭**：先做 gate 停旧轨 + REPLAN 近障先停，再用无进展升级（停稳 → 全局重搜 / 放弃）压缩空转；减误 ESTOP 与大转角脱出放后面，且 **绝不靠放宽门禁换时间**。
