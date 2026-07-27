# 地面障碍与世界坐标建模

本文说明 D1 实机在 **VIO 原点≈相机**、**机身可抬升** 条件下，如何把贴地矮障纳入建图与碰撞查询。  
**状态：主体已落地**（参数与代码见 `d1_robot.yaml` / `grid_map`）；下文区分「已实现」与「后续可调」。

相关：[系统总览](00_overview.md)、[规划数学 §6](01_planning_math.md#6-d1-改动固定-z-与高度柱)。

---

## 1. 问题背景

### 1.1 坐标系事实

OpenVINS 初始化后，世界系 `global` 原点通常落在 **当时的相机（IMU）位姿** 上：

- 初始化瞬间：相机世界坐标 ≈ `(0, 0, 0)`
- 若 Z 朝上：真实地面约在 `z ≈ -h_cam`
- 机身抬升后：相机 `z` 升高；地面障碍的世界 `z` 仍偏低

世界原点在相机 **可以保留**。关键是「地面高度」与「碰撞查询高度」的建模。

### 1.2 历史错误假设（已修正方向）

| 机制 | 旧问题 | 当前默认（`d1_robot.yaml`） |
|------|--------|---------------------------|
| `grid_map/ground_height` | 曾接近 0，矮障 out-of-map | **-0.5**，罩住地面带 |
| 地面滤波 | 曾关闭或阈值不当 | **`ground_filter_enable: true`**，`camera_to_ground: 0.40`，`obstacle_min_height: 0.10` |
| 碰撞查询 | 仅相机高度单层 | **`column_collision_enable: true`** 高度柱 |
| `inflate_xy_only` | 低层不胀到相机层 | 仍为 true；靠柱查询打通垂直语义 |
| `use_robot_z_planning` | 优化锁 z | 仍 true；柱上沿用查询 / planning_z |

---

## 2. 目标语义（三层）

```text
世界 z ≈ 0          初始化相机高度（随 VIO 漂移）
世界 z ≈ -h_cam     真实地面带
地图 Z 覆盖         [ground_height, ground_height + map_size_z]
障碍                相对地面凸起超过 obstacle_min_height
规划碰撞            (x,y) 高度柱内是否碰到膨胀占据
```

---

## 3. 已落地行为

### 3.1 地图 Z 范围

- `ground_height: -0.5`，`map_size_z: 3.0`，`virtual_ceil_height: 2.9`
- 原点仍可用相机 init；下界需罩住真实地面

### 3.2 地面滤波

估计地面：

$$
z_{\mathrm{g}} = z_{\mathrm{cam}} - \texttt{camera\_to\_ground}
$$

若 `pos.z < z_g + obstacle_min_height` → 当地板（不入库为障碍）；凸起保留。

`camera_to_ground` **过大** 会把矮障滤掉；过小则地板易铺墙。当前默认 0.40 m，需按实机测量微调。

### 3.3 高度柱查询（方案 A）

`getColumnInflateOccupancy` / `getInflateOccupancy`（开启柱时）：

对 $(x,y)$，扫描

$$
[z_{\mathrm{floor}}+\varepsilon,\ \max(z_{\mathrm{query}},\, z_{\mathrm{floor}}+\varepsilon)]
$$

任一膨胀格占用 → 障碍。机身抬升时柱上沿跟查询高度走，矮障仍可见。

### 3.4 Footprint

`robot_footprint_*` + `no_inflate` + `clear_margin`：水平向自占据豁免，与垂直柱语义正交。

---

## 4. 后续可调 / 仍待观察

1. **标定**：用卷尺复核 `camera_to_ground`、`obstacle_min_height`；抬升前后各测一轮矮盒。
2. **膨胀**：`obstacles_inflation` 当前 **0.20 m**（偏保守）；与 `robot_footprint_clear_margin` 联动，贴身真障被抹时可下调 clear。
3. **方案 B（未做）**：建图时把柱压成 2D 占据层——目前靠查询侧柱检查即可。
4. **ESDF / 更平滑障碍梯度**：见 [todo.md](todo.md)，与本文正交。

---

## 5. 验收标准（实机）

| 场景 | 期望 |
|------|------|
| 初始化后地面附近矮盒（高于 `obstacle_min_height`） | 占据可见；规划绕行或安全停障 |
| 机身抬升后同矮盒 | 柱查询仍当障碍 |
| 平坦地面 | 不因地面点铺满膨胀墙 |
| VIO `z` 略低于 0 | 不因 out-of-map 误判「整轨在障碍里」 |

---

## 6. 相关参数索引

配置源：`src/planner/plan_manage/config/d1_robot.yaml`

| 参数 | 角色 |
|------|------|
| `grid_map/ground_height` | 地图 Z 下界 |
| `planner/map_size_z` | 高度跨度 |
| `grid_map/camera_to_ground` | 相机离地估计 |
| `grid_map/obstacle_min_height` | 凸起阈值 |
| `grid_map/ground_filter_*` | 滤波开关与备用 margin |
| `grid_map/column_collision_*` | 高度柱 |
| `grid_map/inflate_xy_only` | XY 膨胀 |
| `grid_map/robot_footprint_*` | 机身水平豁免 |
| `manager/use_robot_z_planning` | 优化锁 z |
| `planner/obstacles_inflation` | 膨胀半径 |

---

## 7. 一句话

> 原点可以在相机；地图必须罩住真实地面；障碍按离地凸起选取；规划用「地面→查询高度」柱检查，而不是仅相机高度单层切片。
