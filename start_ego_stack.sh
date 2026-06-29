#!/usr/bin/env bash
# 一键启动：RealSense + OpenVINS (rs_d435i) + EGO 规划 + D1 桥接 (+ 可选 RViz / AprilTag)
#
# 感知栈（与 Readme / docs/00_overview 一致）：
#   1. ros2 launch realsense2_camera rs_launch.py
#   2. ros2 launch ov_msckf subscribe.launch.py config:=rs_d435i use_stereo:=true max_cameras:=2
#
# 用法:
#   ./start_ego_stack.sh                        # 默认：RViz 手动设点
#   ./start_ego_stack.sh enable_tag_tracking=true  # AprilTag 追踪模式
#   ./start_ego_stack.sh --no-rviz              # 不启动 RViz
#   ./start_ego_stack.sh --skip-wait            # 跳过话题就绪检测（调试用）
#
# 日志目录: <本仓库>/ego_log/stack_YYYYMMDD_HHMMSS/

# 不用 set -u：ROS/colcon 的 setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
D1ROBOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REALSENSE_WS="${D1ROBOT_DIR}/realsense"
OPENVINS_WS="${D1ROBOT_DIR}/openvins"
EGO_WS="${SCRIPT_DIR}"

load_d1_robot_config() {
  local cfg
  for cfg in \
    "${EGO_WS}/install/ego_planner/share/ego_planner/config/d1_robot.yaml" \
    "${EGO_WS}/src/planner/plan_manage/config/d1_robot.yaml"; do
    if [[ -f "${cfg}" ]]; then
      D1_CONFIG="${cfg}"
      eval "$(python3 - "${cfg}" <<'PY'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
lim = c["limits"]
print(f"D1_MAX_VX={lim['max_vel']}")
print(f"D1_MAX_WZ={lim['max_wz']}")
print(f"D1_MAX_ACC={lim['max_acc']}")
PY
)"
      return 0
    fi
  done
  echo "[错误] 未找到 d1_robot.yaml（请先 colcon build ego_planner）" >&2
  exit 1
}

load_d1_robot_config

ENABLE_RVIZ=true
SKIP_WAIT=false
ENABLE_TAG_TRACKING=false

usage() {
  sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz) ENABLE_RVIZ=false; shift ;;
    --skip-wait) SKIP_WAIT=true; shift ;;
    enable_tag_tracking=true) ENABLE_TAG_TRACKING=true; shift ;;
    enable_tag_tracking=false) ENABLE_TAG_TRACKING=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage; exit 1 ;;
  esac
done

for ws in "${REALSENSE_WS}" "${OPENVINS_WS}" "${EGO_WS}"; do
  if [[ ! -f "${ws}/install/setup.bash" ]]; then
    echo "[错误] 未找到 ${ws}/install/setup.bash，请先在对应目录 colcon build" >&2
    exit 1
  fi
done

LOG_DIR="${EGO_WS}/ego_log/stack_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

PIDS=()
CLEANUP_DONE=false
INTERRUPTED=false

log() { echo "[$(date +%H:%M:%S)] $*"; }

launch_bg() {
  local name="$1"
  shift
  log "启动 ${name} ..."
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
  log "${name} PID=$!  日志: ${LOG_DIR}/${name}.log"
}

cleanup() {
  if [[ "${CLEANUP_DONE}" == "true" ]]; then
    return 0
  fi
  CLEANUP_DONE=true
  trap - EXIT INT TERM

  log "正在停止所有节点 ..."
  for pid in "${PIDS[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -KILL "${pid}" 2>/dev/null || true
  done
  log "已退出。日志保留在: ${LOG_DIR}"
}

on_interrupt() {
  INTERRUPTED=true
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_interrupt INT TERM

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${REALSENSE_WS}/install/setup.bash"
# shellcheck disable=SC1091
source "${OPENVINS_WS}/install/setup.bash"
# shellcheck disable=SC1091
source "${EGO_WS}/install/setup.bash"

# 本仓 apriltag_detect 优先于旧 ../apriltagdetect 工作区（同名包遮蔽会导致 launch 失败）
EGO_APRILTAG_PREFIX="${EGO_WS}/install/apriltag_detect"
if [[ ! -f "${EGO_APRILTAG_PREFIX}/share/apriltag_detect/launch/apriltag.launch.py" ]]; then
  log "[错误] 未找到 ${EGO_APRILTAG_PREFIX}/share/apriltag_detect/launch/apriltag.launch.py" >&2
  log "       请先执行: cd ${EGO_WS} && colcon build --symlink-install --packages-select apriltag_detect" >&2
  exit 1
fi
export AMENT_PREFIX_PATH="${EGO_APRILTAG_PREFIX}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"

if [[ -z "${RMW_IMPLEMENTATION:-}" ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  log "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
fi

wait_for_topic() {
  local topic="$1"
  local timeout="${2:-60}"
  log "等待话题出现: ${topic} (最多 ${timeout}s，Ctrl+C 可立即退出)"
  local i=0
  while (( i < timeout )); do
    if [[ "${INTERRUPTED}" == "true" ]]; then
      return 130
    fi
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      log "话题已出现: ${topic}"
      return 0
    fi
    sleep 1 || return 130
    ((i++))
  done
  log "超时: 未找到话题 ${topic}" >&2
  return 1
}

wait_for_message() {
  local topic="$1"
  local timeout="${2:-90}"
  log "等待首条消息: ${topic} (最多 ${timeout}s，Ctrl+C 可立即退出)"
  if [[ "${INTERRUPTED}" == "true" ]]; then
    return 130
  fi
  if timeout "${timeout}" ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    log "已收到: ${topic}"
    return 0
  fi
  if [[ "${INTERRUPTED}" == "true" ]]; then
    return 130
  fi
  log "超时: 未收到 ${topic} 数据（可轻微晃动相机帮助 OpenVINS 初始化）" >&2
  return 1
}

log "工作区: realsense=${REALSENSE_WS}"
log "        openvins=${OPENVINS_WS}"
log "        ego_control=${EGO_WS}"
log "日志目录: ${LOG_DIR}"
log "enable_tag_tracking=${ENABLE_TAG_TRACKING}"
log "D1 config: ${D1_CONFIG}"
log "D1 limits: max_vx=${D1_MAX_VX} m/s max_wz=${D1_MAX_WZ} rad/s max_acc=${D1_MAX_ACC} m/s^2"

# 1. RealSense D435i
launch_bg realsense \
  ros2 launch realsense2_camera rs_launch.py

if [[ "${SKIP_WAIT}" == "false" ]]; then
  wait_for_topic /camera/camera/imu 60 || exit $?
  wait_for_topic /camera/camera/depth/image_rect_raw 60 || exit $?
  wait_for_message /camera/camera/depth/image_rect_raw 30 || exit $?
fi

# 2. OpenVINS (rs_d435i)
launch_bg openvins \
  ros2 launch ov_msckf subscribe.launch.py \
  config:=rs_d435i use_stereo:=true max_cameras:=2

if [[ "${SKIP_WAIT}" == "false" ]]; then
  wait_for_topic /ov_msckf/pose_stamped 30 || exit $?
  wait_for_message /ov_msckf/pose_stamped 90 || exit $?
fi

# 3. AprilTag 感知（仅追踪模式）
if [[ "${ENABLE_TAG_TRACKING}" == "true" ]]; then
  resolved_apriltag="$(ros2 pkg prefix apriltag_detect 2>/dev/null || true)"
  log "apriltag_detect 包路径: ${resolved_apriltag}"
  if [[ "${resolved_apriltag}" != "${EGO_APRILTAG_PREFIX}" ]]; then
    log "[错误] apriltag_detect 仍解析到旧工作区: ${resolved_apriltag}" >&2
    log "       请检查 ~/.bashrc 是否 source 了 ../apriltagdetect/install/setup.bash" >&2
    exit 1
  fi
  launch_bg apriltag \
    ros2 launch apriltag_detect apriltag.launch.py
  if [[ "${SKIP_WAIT}" == "false" ]]; then
    wait_for_topic /apriltag/target_detected 30 || true
  fi
fi

# 4. EGO 规划（速度/话题默认来自 d1_robot.yaml）
launch_bg ego_planner \
  ros2 launch ego_planner single_run.launch.py \
  enable_tag_tracking:=${ENABLE_TAG_TRACKING}

sleep 2 || exit 130

# 5. D1 底盘桥接
launch_bg d1_bridge \
  ros2 launch d1_planner_bridge d1_planner_bridge.launch.py

# 6. 可选 RViz（Fixed Frame 选 global）
if [[ "${ENABLE_RVIZ}" == "true" ]]; then
  launch_bg rviz \
    ros2 launch ego_planner rviz.launch.py
fi

log "=========================================="
log "全部节点已启动。按 Ctrl+C 停止。"
log "  RealSense  -> ${LOG_DIR}/realsense.log"
log "  OpenVINS   -> ${LOG_DIR}/openvins.log"
if [[ "${ENABLE_TAG_TRACKING}" == "true" ]]; then
  log "  AprilTag   -> ${LOG_DIR}/apriltag.log"
fi
log "  EGO 规划   -> ${LOG_DIR}/ego_planner.log"
log "  D1 桥接    -> ${LOG_DIR}/d1_bridge.log"
if [[ "${ENABLE_RVIZ}" == "true" ]]; then
  log "  RViz       -> ${LOG_DIR}/rviz.log"
fi
log "=========================================="

wait
