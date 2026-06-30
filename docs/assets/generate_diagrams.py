#!/usr/bin/env python3
"""Generate polished PNG diagrams for docs/05_demo.md."""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
import matplotlib.font_manager as fm

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
DPI = 180

C_PERCEPTION = "#2962FF"
C_PLANNING = "#2E7D32"
C_CONTROL = "#F57C00"
C_ACCENT = "#5C6BC0"
C_BG = "#F8FAFC"
C_BOX = "#FFFFFF"
C_TEXT = "#1A237E"
C_MUTED = "#546E7A"
C_ARROW = "#78909C"

FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if not os.path.exists(FONT_PATH):
    FONT_PATH = "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"
FONT = fm.FontProperties(fname=FONT_PATH)
FONT_BOLD = fm.FontProperties(fname=FONT_PATH, weight="bold")

plt.rcParams.update({
    "axes.unicode_minus": False,
    "figure.facecolor": C_BG,
    "savefig.facecolor": C_BG,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.35,
})


def draw_text(ax, x, y, s, size=9, bold=False, color=C_TEXT, ha="center", va="center", **kw):
    ax.text(x, y, s, ha=ha, va=va, fontsize=size,
            color=color, fontproperties=FONT_BOLD if bold else FONT, **kw)


def rounded_box(ax, x, y, w, h, text, color, fontsize=9, bold=False):
    box = FancyBboxPatch(
        (x - w / 2, y - h / 2), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.08",
        facecolor=C_BOX, edgecolor=color, linewidth=2.2, zorder=3,
    )
    ax.add_patch(box)
    draw_text(ax, x, y, text, size=fontsize, bold=bold, zorder=4, linespacing=1.35)
    return box


def group_box(ax, x, y, w, h, label, color):
    box = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.01,rounding_size=0.12",
        facecolor=color + "18", edgecolor=color, linewidth=1.8,
        linestyle="--", zorder=1,
    )
    ax.add_patch(box)
    draw_text(ax, x + w / 2, y + h - 0.35, label, size=10, bold=True,
              color=color, va="top", zorder=2)


def arrow(ax, x1, y1, x2, y2, label="", color=C_ARROW, rad=0.0):
    style = f"arc3,rad={rad}" if rad else "arc3,rad=0"
    arr = FancyArrowPatch(
        (x1, y1), (x2, y2),
        arrowstyle="-|>", mutation_scale=12,
        color=color, linewidth=1.6,
        connectionstyle=style, zorder=2,
    )
    ax.add_patch(arr)
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        draw_text(ax, mx, my + 0.18, label, size=7.5, color=C_MUTED, va="bottom", zorder=5,
                  bbox=dict(boxstyle="round,pad=0.15", facecolor="white",
                            edgecolor="none", alpha=0.85))


def save(fig, name):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=DPI, transparent=False)
    plt.close(fig)
    print(f"Saved {path}")


def draw_three_layer():
    fig, ax = plt.subplots(figsize=(10, 7.5))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.axis("off")
    ax.set_title("系统三层架构", fontsize=16, color=C_TEXT, fontproperties=FONT_BOLD, pad=16)

    group_box(ax, 0.4, 5.6, 9.2, 2.0, "感知层（外部 + 可选本仓）", C_PERCEPTION)
    l1_y = 6.5
    rounded_box(ax, 2.0, l1_y, 2.2, 0.75, "RealSense\nD435i", C_PERCEPTION, 9, True)
    rounded_box(ax, 5.0, l1_y, 2.2, 0.75, "OpenVINS\nov_msckf", C_PERCEPTION, 9, True)
    rounded_box(ax, 8.0, l1_y, 2.0, 0.75, "AprilTag\n（可选）", C_PERCEPTION, 9, True)

    group_box(ax, 0.4, 2.8, 9.2, 2.4, "规划层（本仓 src/planner/）", C_PLANNING)
    l2_y = 3.9
    rounded_box(ax, 1.6, l2_y, 1.9, 0.7, "GridMap\n在线建图", C_PLANNING, 8.5)
    rounded_box(ax, 3.8, l2_y, 2.0, 0.7, "EGOReplanFSM\n状态机", C_PLANNING, 8.5)
    rounded_box(ax, 6.2, l2_y, 2.0, 0.7, "B 样条 + A*\n优化", C_PLANNING, 8.5)
    rounded_box(ax, 8.4, l2_y, 1.8, 0.7, "traj_server\n轨迹采样", C_PLANNING, 8.5)

    group_box(ax, 0.4, 0.5, 9.2, 1.9, "控制层（本仓 d1_planner_bridge）", C_CONTROL)
    l3_y = 1.35
    rounded_box(ax, 3.5, l3_y, 2.4, 0.75, "TrajectoryTracker", C_CONTROL, 9, True)
    rounded_box(ax, 7.0, l3_y, 2.6, 0.75, "D1 底盘\n/command/cmd_twist", C_CONTROL, 9, True)

    arrow(ax, 2.0, 6.1, 1.6, 4.25, "深度", C_PERCEPTION)
    arrow(ax, 5.0, 6.1, 1.6, 4.35, "pose", C_PERCEPTION, rad=0.15)
    arrow(ax, 5.0, 6.1, 3.8, 4.25, "odomimu", C_PERCEPTION)
    arrow(ax, 8.0, 6.1, 3.8, 4.35, "目标", C_PERCEPTION, rad=-0.12)
    arrow(ax, 1.6, 3.55, 6.2, 4.25, "占据查询", C_PLANNING)
    arrow(ax, 3.8, 3.55, 6.2, 3.95, "", C_PLANNING)
    arrow(ax, 6.2, 3.55, 8.4, 3.95, "B-spline", C_PLANNING)
    arrow(ax, 8.4, 3.55, 3.5, 1.75, "pos_cmd", C_PLANNING, rad=0.2)
    arrow(ax, 5.0, 6.1, 8.4, 4.0, "odom", C_ACCENT, rad=-0.25)
    arrow(ax, 5.0, 6.1, 3.5, 1.85, "odom", C_ACCENT, rad=0.3)
    arrow(ax, 3.5, 0.97, 7.0, 0.97, "cmd_vel", C_CONTROL)
    save(fig, "three_layer_architecture.png")


def draw_data_flow():
    fig, ax = plt.subplots(figsize=(12, 5.5))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 5.5)
    ax.axis("off")
    ax.set_title("端到端数据流", fontsize=16, color=C_TEXT, fontproperties=FONT_BOLD, pad=14)

    nodes = [
        (1.0, "RealSense", C_PERCEPTION),
        (2.8, "OpenVINS", C_PERCEPTION),
        (4.6, "GridMap", C_PLANNING),
        (6.4, "ego_planner", C_PLANNING),
        (8.2, "traj_server", C_PLANNING),
        (10.0, "d1_bridge", C_CONTROL),
        (11.5, "D1 底盘", C_CONTROL),
    ]
    ny = 1.2
    for x, label, color in nodes:
        rounded_box(ax, x, ny, 1.5, 0.85, label, color, 8.5, True)

    for x, _, color in nodes:
        ax.plot([x, x], [1.65, 4.8], color=color, linewidth=1.2,
                linestyle=":", alpha=0.45, zorder=0)

    def msg(x1, x2, y, label, color=C_ARROW):
        ax.annotate("", xy=(x2, y), xytext=(x1, y),
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=1.5))
        draw_text(ax, (x1 + x2) / 2, y + 0.12, label, size=7.5, color=C_MUTED,
                  bbox=dict(boxstyle="round,pad=0.12", facecolor="white",
                            edgecolor="none", alpha=0.9))

    msg(1.0, 2.8, 4.5, "IMU + 图像", C_PERCEPTION)
    msg(1.0, 4.6, 4.1, "深度图", C_PERCEPTION)
    msg(2.8, 4.6, 3.7, "pose_stamped", C_PERCEPTION)
    msg(2.8, 6.4, 3.3, "odomimu", C_PERCEPTION)
    draw_text(ax, 6.4, 3.55, "RViz 2D Goal\n或 AprilTag", size=7.5, color=C_MUTED,
              bbox=dict(boxstyle="round,pad=0.2", facecolor="#FFF9C4",
                        edgecolor="#FBC02D", alpha=0.9))
    msg(4.6, 6.4, 2.9, "占据查询", C_PLANNING)
    msg(6.4, 8.2, 2.5, "B-spline 轨迹", C_PLANNING)
    msg(2.8, 8.2, 2.1, "odom", C_ACCENT)
    msg(8.2, 10.0, 1.7, "pos_cmd", C_PLANNING)
    msg(2.8, 10.0, 1.3, "odom", C_ACCENT)
    msg(10.0, 11.5, 0.9, "cmd_twist", C_CONTROL)
    save(fig, "data_flow_sequence.png")


def draw_fsm():
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 7)
    ax.axis("off")
    ax.set_title("EGOReplanFSM 规划状态机（10 ms 周期）", fontsize=15,
                 color=C_TEXT, fontproperties=FONT_BOLD, pad=14)

    states = {
        "INIT": (1.5, 5.5),
        "WAIT_TARGET": (4.5, 5.5),
        "GEN_NEW_TRAJ": (7.5, 5.5),
        "EXEC_TRAJ": (5.5, 3.0),
        "REPLAN_TRAJ": (8.5, 3.0),
        "EMERGENCY_STOP": (2.5, 1.0),
    }
    colors = {
        "INIT": C_MUTED,
        "WAIT_TARGET": C_ACCENT,
        "GEN_NEW_TRAJ": C_PLANNING,
        "EXEC_TRAJ": C_PLANNING,
        "REPLAN_TRAJ": C_PLANNING,
        "EMERGENCY_STOP": "#C62828",
    }

    for name, (x, y) in states.items():
        c = Circle((x, y), 0.65, facecolor=C_BOX, edgecolor=colors[name],
                   linewidth=2.5, zorder=3)
        ax.add_patch(c)
        fs = 8 if len(name) > 10 else 9
        draw_text(ax, x, y, name.replace("_", "\n"), size=fs, bold=True, zorder=4)

    ax.plot(0.3, 5.5, "o", color=C_TEXT, markersize=8, zorder=3)
    arrow(ax, 0.45, 5.5, 0.85, 5.5, "启动", C_ARROW)

    transitions = [
        (1.5, 5.5, 3.85, 5.5, "收到 odom"),
        (5.15, 5.5, 6.85, 5.5, "有目标"),
        (7.5, 4.85, 5.5, 3.65, "规划成功"),
        (7.5, 5.15, 4.5, 5.5, "连续失败", -0.3),
        (6.15, 3.0, 7.85, 3.0, "定时 2.5s\n/ 安全检测"),
        (8.5, 3.65, 6.15, 3.0, "重规划成功", -0.25),
        (8.5, 2.35, 4.5, 5.15, "近目标且\n规划失败", 0.35),
        (5.5, 2.35, 4.5, 4.85, "XY 距目标\n< 0.3m", 0.3),
        (4.85, 2.65, 3.15, 1.65, "碰撞风险", 0.2),
        (2.5, 1.65, 7.5, 4.85, "低速 +\nfail_safe", 0.4),
    ]
    for tr in transitions:
        x1, y1, x2, y2, label = tr[:5]
        rad = tr[5] if len(tr) > 5 else 0.0
        arrow(ax, x1, y1, x2, y2, label, C_ARROW, rad=rad)

    legend_items = [("正常流转", C_PLANNING), ("等待/恢复", C_ACCENT), ("急停", "#C62828")]
    for i, (txt, col) in enumerate(legend_items):
        ax.add_patch(mpatches.Rectangle((0.5 + i * 2.8, 0.15), 0.3, 0.25,
                                         facecolor=col, edgecolor="none"))
        draw_text(ax, 0.9 + i * 2.8, 0.27, txt, size=8, color=C_MUTED, va="center", ha="left")
    save(fig, "fsm_state_diagram.png")


def draw_planning_flow():
    fig, ax = plt.subplots(figsize=(10, 3.8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 3.8)
    ax.axis("off")
    ax.set_title("规划算法流程 — reboundReplan()", fontsize=15,
                 color=C_TEXT, fontproperties=FONT_BOLD, pad=14)

    steps = [
        (2.0, "STEP 1\nINIT", "多项式初值\n+ A* 弹性方向", C_ACCENT),
        (5.0, "STEP 2\nOPT", "L-BFGS 优化\nB 样条控制点", C_PLANNING),
        (8.0, "STEP 3\nREFINE", "精修轨迹", C_PLANNING),
    ]
    for x, title, desc, color in steps:
        rounded_box(ax, x, 2.2, 2.4, 1.1, title, color, 10, True)
        draw_text(ax, x, 1.0, desc, size=9, color=C_MUTED, linespacing=1.4,
                  bbox=dict(boxstyle="round,pad=0.25", facecolor="#ECEFF1",
                            edgecolor="#CFD8DC", alpha=0.8))

    arrow(ax, 3.2, 2.2, 3.8, 2.2, "", C_PLANNING)
    arrow(ax, 6.2, 2.2, 6.8, 2.2, "", C_PLANNING)
    save(fig, "planning_algorithm_flow.png")


def draw_control_chain():
    fig, ax = plt.subplots(figsize=(11, 4.2))
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 4.2)
    ax.axis("off")
    ax.set_title("控制链路", fontsize=15, color=C_TEXT, fontproperties=FONT_BOLD, pad=14)

    chain = [
        (1.2, "B-spline", C_PLANNING),
        (3.2, "traj_server\nDe Boor 采样", C_PLANNING),
        (5.5, "PositionCommand\np, v, yaw", C_PLANNING),
        (7.8, "d1_planner_bridge\nTrajectoryTracker", C_CONTROL),
        (10.0, "cmd_vel\nlinear.x + angular.z", C_CONTROL),
    ]
    cy = 2.3
    for x, label, color in chain:
        rounded_box(ax, x, cy, 1.7, 0.95, label, color, 8.5)

    for i in range(len(chain) - 1):
        x1 = chain[i][0] + 0.85
        x2 = chain[i + 1][0] - 0.85
        arrow(ax, x1, cy, x2, cy, "", C_ARROW)

    rounded_box(ax, 5.5, 0.75, 1.4, 0.65, "odom", C_ACCENT, 9, True)
    arrow(ax, 5.5, 1.08, 3.2, 1.85, "", C_ACCENT, rad=0.15)
    arrow(ax, 5.5, 1.08, 7.8, 1.85, "", C_ACCENT, rad=-0.15)
    save(fig, "control_chain.png")


if __name__ == "__main__":
    draw_three_layer()
    draw_data_flow()
    draw_fsm()
    draw_planning_flow()
    draw_control_chain()
    print("All diagrams generated.")
