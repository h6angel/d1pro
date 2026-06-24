#!/usr/bin/env bash
# 一键启动：RealSense 相机 + OpenVINS (rs_d435i) + EGO 规划 + D1 桥接 (+ 可选 RViz)
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

trap cleanup EXIT INT TERM

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${REALSENSE_WS}/install/setup.bash"
# shellcheck disable=SC1091
source "${OPENVINS_WS}/install/setup.bash"
# shellcheck disable=SC1091
source "${EGO_WS}/install/setup.bash"

if [[ -z "${RMW_IMPLEMENTATION:-}" ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  log "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
fi

wait_for_topic() {
  local topic="$1"
  local timeout="${2:-60}"
  log "等待话题出现: ${topic} (最多 ${timeout}s)"
  local i=0
  while (( i < timeout )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      log "话题已出现: ${topic}"
      return 0
    fi
    sleep 1
    ((i++))
  done
  log "超时: 未找到话题 ${topic}" >&2
  return 1
}

wait_for_message() {
  local topic="$1"
  local timeout="${2:-90}"
  log "等待首条消息: ${topic} (最多 ${timeout}s)"
  if timeout "${timeout}" ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    log "已收到: ${topic}"
    return 0
  fi
  log "超时: 未收到 ${topic} 数据（可轻微晃动相机帮助 OpenVINS 初始化）" >&2
  return 1
}

log "工作区: realsense=${REALSENSE_WS}"
log "        openvins=${OPENVINS_WS}"
log "        ego_control=${EGO_WS}"
log "日志目录: ${LOG_DIR}"
log "enable_tag_tracking=${ENABLE_TAG_TRACKING}"

# 1. RealSense D435i
launch_bg realsense \
  ros2 launch realsense2_camera rs_launch.py

if [[ "${SKIP_WAIT}" == "false" ]]; then
  wait_for_topic /camera/camera/imu 60 || exit 1
  wait_for_topic /camera/camera/depth/image_rect_raw 60 || exit 1
  wait_for_message /camera/camera/depth/image_rect_raw 30 || exit 1
fi

# 2. OpenVINS (rs_d435i)
launch_bg openvins \
  ros2 launch ov_msckf subscribe.launch.py \
  config:=rs_d435i use_stereo:=true max_cameras:=2

if [[ "${SKIP_WAIT}" == "false" ]]; then
  wait_for_topic /ov_msckf/pose_stamped 30 || exit 1
  wait_for_message /ov_msckf/pose_stamped 90 || exit 1
fi

# 3. AprilTag 感知（仅追踪模式）
if [[ "${ENABLE_TAG_TRACKING}" == "true" ]]; then
  launch_bg apriltag \
    ros2 launch apriltag_detect apriltag.launch.py
  if [[ "${SKIP_WAIT}" == "false" ]]; then
    wait_for_topic /apriltag/target_detected 30 || true
  fi
fi

# 4. EGO 规划
launch_bg ego_planner \
  ros2 launch ego_planner single_run.launch.py \
  enable_tag_tracking:=${ENABLE_TAG_TRACKING}

sleep 2

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
log "  EGO 规划   -> ${LOG_DIR}/ego_planner.log (+ ego_log/ 内 launch tee)"
log "  D1 桥接    -> ${LOG_DIR}/d1_bridge.log"
if [[ "${ENABLE_RVIZ}" == "true" ]]; then
  log "  RViz       -> ${LOG_DIR}/rviz.log"
fi
log "=========================================="

wait
