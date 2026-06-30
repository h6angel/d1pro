#!/usr/bin/env python3
"""Generate PNG diagrams for docs/05_demo.md via Graphviz."""

import os
import subprocess

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
DOT = "/usr/bin/dot"

FONT = "Noto Sans CJK SC"
DPI = 160

COMMON = f"""
    graph [bgcolor="#F8FAFC", fontname="{FONT}", fontsize=14, pad=0.45, dpi={DPI}]
    node  [fontname="{FONT}", fontsize=11, shape=box, style="rounded,filled",
           fillcolor="#FFFFFF", color="#37474F", penwidth=1.8, margin="0.18,0.10"]
    edge  [fontname="{FONT}", fontsize=9,  color="#607D8B", penwidth=1.4]
"""


def render(name: str, body: str) -> None:
    dot = f"digraph G {{\n{COMMON}\n{body}\n}}"
    path = os.path.join(OUT_DIR, name)
    subprocess.run(
        [DOT, "-Tpng", "-o", path],
        input=dot.encode("utf-8"),
        check=True,
    )
    print(f"Saved {path}")


def draw_three_layer() -> None:
    body = """
    graph [rankdir=TB, nodesep=0.55, ranksep=0.75, compound=true,
           label="系统三层架构", labelloc=t, fontsize=14, fontcolor="#1A237E"]

    subgraph cluster_l1 {
        label="感知层（外部 + 可选本仓）"
        color="#2962FF"  style="rounded,dashed"  bgcolor="#E3F2FD"
        fontcolor="#1565C0"  fontsize=12

        RS [label="RealSense\\nD435i",  color="#2962FF"]
        OV [label="OpenVINS\\nov_msckf", color="#2962FF"]
        AT [label="AprilTag\\n（可选）", color="#2962FF"]
        { rank=same; RS; OV; AT }
    }

    subgraph cluster_l2 {
        label="规划层（本仓 src/planner/）"
        color="#2E7D32"  style="rounded,dashed"  bgcolor="#E8F5E9"
        fontcolor="#1B5E20"  fontsize=12

        GM  [label="GridMap\\n在线建图",       color="#2E7D32"]
        FSM [label="EGOReplanFSM\\n状态机",    color="#2E7D32"]
        PM  [label="B 样条 + A*\\n优化",       color="#2E7D32"]
        TS  [label="traj_server\\n轨迹采样",  color="#2E7D32"]
        { rank=same; GM; FSM; PM; TS }
    }

    subgraph cluster_l3 {
        label="控制层（本仓 d1_planner_bridge）"
        color="#F57C00"  style="rounded,dashed"  bgcolor="#FFF3E0"
        fontcolor="#E65100"  fontsize=12

        BR [label="TrajectoryTracker",              color="#F57C00"]
        D1 [label="D1 底盘\\n/command/cmd_twist",    color="#F57C00"]
        { rank=same; BR; D1 }
    }

    RS  -> GM  [label="深度",     color="#2962FF", ltail=cluster_l1, lhead=cluster_l2]
    OV  -> GM  [label="pose",     color="#2962FF"]
    OV  -> FSM [label="odomimu",  color="#2962FF"]
    AT  -> FSM [label="目标",     color="#2962FF"]
    GM  -> PM  [label="占据查询", color="#2E7D32"]
    FSM -> PM  [label="规划触发", color="#2E7D32"]
    PM  -> TS  [label="B-spline", color="#2E7D32"]
    TS  -> BR  [label="pos_cmd",  color="#2E7D32", ltail=cluster_l2, lhead=cluster_l3]
    OV  -> TS  [label="odom",     color="#5C6BC0", constraint=false]
    OV  -> BR  [label="odom",     color="#5C6BC0", constraint=false]
    BR  -> D1  [label="cmd_vel",  color="#F57C00"]
    """
    render("three_layer_architecture.png", body)


def draw_data_flow() -> None:
    body = """
    graph [rankdir=TB, nodesep=0.55, ranksep=0.6,
           label="端到端数据流", labelloc=t, fontsize=14, fontcolor="#1A237E"]
    node  [fontsize=10]

    RS  [label="RealSense",   color="#2962FF"]
    OV  [label="OpenVINS",    color="#2962FF"]
    GM  [label="GridMap",     color="#2E7D32"]
    EP  [label="ego_planner", color="#2E7D32"]
    TS  [label="traj_server", color="#2E7D32"]
    BR  [label="d1_bridge",   color="#F57C00"]
    D1  [label="D1 底盘",     color="#F57C00"]
    GOAL [label="RViz 2D Goal\\n/ AprilTag", shape=note,
          fillcolor="#FFF9C4", color="#FBC02D", fontsize=9]

    { rank=source;  RS; OV }
    { rank=same;    GOAL; GM; EP }
    { rank=same;    TS; BR }
    { rank=sink;    D1 }

    RS   -> OV  [label="IMU + 图像",    color="#2962FF"]
    RS   -> GM  [label="深度图",        color="#2962FF"]
    OV   -> GM  [label="pose_stamped",   color="#2962FF"]
    OV   -> EP  [label="odomimu",       color="#2962FF"]
    GOAL -> EP  [label="目标点",        color="#FBC02D", style=dashed]
    GM   -> EP  [label="占据查询",      color="#2E7D32"]
    EP   -> TS  [label="B-spline 轨迹", color="#2E7D32"]
    OV   -> TS  [label="odom",          color="#5C6BC0", constraint=false]
    TS   -> BR  [label="pos_cmd",       color="#2E7D32"]
    OV   -> BR  [label="odom",          color="#5C6BC0", constraint=false]
    BR   -> D1  [label="cmd_twist",     color="#F57C00"]
    """
    render("data_flow_sequence.png", body)


def draw_fsm() -> None:
    body = """
    graph [rankdir=TB, nodesep=0.85, ranksep=0.9,
           label="EGOReplanFSM 规划状态机（10 ms 周期）",
           labelloc=t, fontsize=14, fontcolor="#1A237E"]
    node  [shape=circle, fixedsize=true, width=1.45, height=1.45,
           style="filled", fillcolor="#FFFFFF", fontsize=9]

    start [shape=point, width=0.08, color="#37474F"]
    INIT            [label="INIT",            color="#78909C"]
    WAIT_TARGET     [label="WAIT\\nTARGET",    color="#5C6BC0"]
    GEN_NEW_TRAJ    [label="GEN\\nNEW TRAJ",   color="#2E7D32"]
    EXEC_TRAJ       [label="EXEC\\nTRAJ",      color="#2E7D32"]
    REPLAN_TRAJ     [label="REPLAN\\nTRAJ",    color="#388E3C"]
    EMERGENCY_STOP  [label="EMERGENCY\\nSTOP", color="#C62828", fontcolor="#C62828"]

    { rank=same; start; INIT; WAIT_TARGET; GEN_NEW_TRAJ }
    { rank=same; EXEC_TRAJ; REPLAN_TRAJ }

    start           -> INIT            [label="  启动  "]
    INIT            -> WAIT_TARGET     [label="收到 odom"]
    WAIT_TARGET     -> GEN_NEW_TRAJ    [label="有目标"]
    GEN_NEW_TRAJ    -> EXEC_TRAJ       [label="规划成功"]
    GEN_NEW_TRAJ    -> WAIT_TARGET     [label="连续失败", color="#78909C", constraint=false]
    EXEC_TRAJ       -> REPLAN_TRAJ     [label="定时 2.5s\\n/ 安全检测"]
    REPLAN_TRAJ     -> EXEC_TRAJ       [label="重规划成功"]
    REPLAN_TRAJ     -> WAIT_TARGET     [label="近目标且\\n规划失败", constraint=false]
    EXEC_TRAJ       -> WAIT_TARGET     [label="XY 距目标\\n< 0.3 m"]
    EXEC_TRAJ       -> EMERGENCY_STOP  [label="碰撞风险", color="#C62828"]
    EMERGENCY_STOP  -> GEN_NEW_TRAJ    [label="低速 +\\nfail_safe", color="#C62828"]
    """
    render("fsm_state_diagram.png", body)


def draw_planning_flow() -> None:
    body = """
    graph [rankdir=LR, nodesep=1.0, ranksep=0.4,
           label="规划算法流程 — reboundReplan()",
           labelloc=t, fontsize=14, fontcolor="#1A237E"]

    S1 [label="{STEP 1|INIT|多项式初值\\n+ A* 弹性方向}",
        shape=record, style="rounded,filled", fillcolor="#FFFFFF", color="#5C6BC0"]
    S2 [label="{STEP 2|OPT|L-BFGS 优化\\nB 样条控制点}",
        shape=record, style="rounded,filled", fillcolor="#FFFFFF", color="#2E7D32"]
    S3 [label="{STEP 3|REFINE|精修轨迹}",
        shape=record, style="rounded,filled", fillcolor="#FFFFFF", color="#2E7D32"]

    S1 -> S2 -> S3 [color="#2E7D32", penwidth=2.2, arrowsize=0.9]
    """
    render("planning_algorithm_flow.png", body)


def draw_control_chain() -> None:
    body = """
    graph [rankdir=TB, nodesep=0.55, ranksep=0.85,
           label="控制链路", labelloc=t, fontsize=14, fontcolor="#1A237E"]
    node  [fontsize=10]

    BS [label="B-spline",                              color="#2E7D32"]
    TS [label="traj_server\\nDe Boor 采样",             color="#2E7D32"]
    PC [label="PositionCommand\\np, v, yaw",           color="#2E7D32"]
    BR [label="d1_planner_bridge\\nTrajectoryTracker", color="#F57C00"]
    CV [label="cmd_vel\\nlinear.x + angular.z",       color="#F57C00"]
    OD [label="odom", color="#5C6BC0"]

    { rank=same; BS -> TS -> PC -> BR -> CV [color="#455A64", penwidth=1.8] }
    TS -> OD [style=invis]
    BR -> OD [style=invis]
    OD -> TS [label="进度采样", color="#5C6BC0", constraint=false]
    OD -> BR [label="位姿反馈", color="#5C6BC0", constraint=false]
    """
    render("control_chain.png", body)


if __name__ == "__main__":
    draw_three_layer()
    draw_data_flow()
    draw_fsm()
    draw_planning_flow()
    draw_control_chain()
    print("All diagrams generated.")
