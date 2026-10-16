#!/bin/bash
# ============================================================================
# launch_rviz.sh — RViz 可视化仿真（宿主机原生，不用 Docker）
#
# 改好 pnc 的功能后，在本机跑完整版 autoware 的 planning_simulator(rviz:=true)，
# 可选自动设位姿/路线/engage 与分阶段行人投放，方便可视化验证新功能。
#
# 前提：先运行  ./sync_to_sim.sh <改动的包>  把 pnc 改动同步+编译进 autoware。
#
# 用法：
#   ./launch_rviz.sh                     # 地图路径自动取最新 sample-map-planning_local-*
#   ./launch_rviz.sh /path/to/map        # 指定地图目录
#   ./launch_rviz.sh --no-waypoint       # 不自动设位姿/路线/engage，纯开 RViz 手动跑
#   ./launch_rviz.sh --no-pedestrians    # 不投放行人障碍
#   MAP_PATH=/xxx ./launch_rviz.sh       # 或用环境变量指定地图
#
# 关于环境：install/setup.bash 是用 `builtin cd $(dirname $BASH_SOURCE)` 相对推导
# 自身前缀的，所以 install 树可重定位、宿主机上直接 source 即可。它会链式 source
# 容器基础镜像遗留的父前缀 /opt/autoware，宿主机上不存在——那条 "not found" 告警
# 无害，脚本已过滤掉。
# ============================================================================
set -euo pipefail

PNC_WS="${PNC_WS:-/home/jm/pnc}"
SIM_WS="${SIM_WS:-/home/jm/autoware}"
MAP_ROOT="${MAP_ROOT:-/home/jm/autoware_map}"

RMW="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
DOMAIN="${ROS_DOMAIN_ID:-17}"

# ---------- 参数解析 ----------
DO_WAYPOINT=1
DO_PEDESTRIANS=1
MAP_ARG=""

for arg in "$@"; do
  case "$arg" in
    --no-waypoint)    DO_WAYPOINT=0 ;;
    --no-pedestrians) DO_PEDESTRIANS=0 ;;
    --*) echo "[错误] 未知参数: $arg" >&2; exit 1 ;;
    *) MAP_ARG="$arg" ;;
  esac
done

# ---------- 确定地图路径（不写死，多地图参数化） ----------
if [ -n "$MAP_ARG" ]; then
  MAP_PATH="$MAP_ARG"
  # 若传的是目录名而非完整路径，自动拼到 MAP_ROOT 下
  [ ! -d "$MAP_PATH" ] && MAP_PATH="${MAP_ROOT}/${MAP_PATH}"
elif [ -n "${MAP_PATH:-}" ] && [ -d "${MAP_PATH:-}" ]; then
  : # 已用环境变量 MAP_PATH 指定
elif [ -n "${MAP_PATH_FROM_FILES:-}" ]; then
  MAP_PATH="$MAP_PATH_FROM_FILES"
else
  # 自动选最新的 sample-map-planning_local-*（若无则回退到任意 sample-map-planning*）
  newest=$(ls -dt "${MAP_ROOT}"/sample-map-planning_local-*/ 2>/dev/null | head -1)
  [ -z "$newest" ] && newest=$(ls -dt "${MAP_ROOT}"/sample-map-planning*/ 2>/dev/null | head -1)
  MAP_PATH="${newest%/}"
fi

# 校验地图可用
if [ -z "$MAP_PATH" ] || [ ! -d "$MAP_PATH" ]; then
  echo "[错误] 地图路径无效: '$MAP_PATH'
  请在 ${MAP_ROOT} 选择一个地图目录作为第一个参数，或设 MAP_PATH" >&2
  exit 1
fi

echo "==> 地图: $MAP_PATH"
echo "==> 仿直运行时: $SIM_WS  (请确保已运行 ./sync_to_sim.sh 同步过改动)"
echo "==> DDS: $RMW / ROS_DOMAIN_ID=$DOMAIN"

# ---------- 前置检查 ----------
[ -f "${SIM_WS}/install/setup.bash" ] || {
  echo "[错误] ${SIM_WS}/install/setup.bash 不存在，先确认 autoware 已构建" >&2
  exit 1
}

# ---------- 加载 ROS 环境 ----------
# set -u 会让 ROS 的 setup.bash 因未定义变量报错，source 期间临时关闭。
# 末尾的 grep 只过滤链式 source /opt/autoware 时那条无害的缺失告警。
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${SIM_WS}/install/setup.bash" \
  2> >(grep -vF 'not found: "/opt/autoware/local_setup.bash"' >&2) || true
set -u

command -v rviz2 >/dev/null || { echo "[错误] rviz2 不可用，检查 /opt/ros/humble" >&2; exit 1; }

export RMW_IMPLEMENTATION="$RMW"
export ROS_DOMAIN_ID="$DOMAIN"

cleanup() {
  echo "==> 停止仿真"
  pkill -INT -f "ros2 launch autoware_launch planning_simulator" 2>/dev/null || true
  pkill -f "spawn_pedestrians.py" 2>/dev/null || true
  for pid in ${LAUNCH_PID:-} ${SPAWN_PID:-} ${WAYPOINT_PID:-}; do
    kill "$pid" 2>/dev/null || true
  done
  exit 0
}
trap cleanup INT TERM

# ---------- 1. 启动 RViz 可视化仿真 ----------
echo "==> 启动 planning_simulator (rviz:=true, launch_api:=true)"
ros2 launch autoware_launch planning_simulator.launch.xml \
  vehicle_model:=cruise_vehicle \
  sensor_model:=sample_sensor_kit \
  map_path:="${MAP_PATH}" \
  rviz:=true \
  launch_api:=true &
LAUNCH_PID=$!

# ---------- 2. 可选：分阶段行人障碍 ----------
if [ "$DO_PEDESTRIANS" -eq 1 ]; then
  echo "==> 启动静态行人投放脚本"
  python3 "${PNC_WS}/spawn_pedestrians.py" --ros-args \
    -p trigger_distance:=8.0  -p first_distance:=6.0  -p first_lateral:=0.3 \
    -p bypass_distance:=10.0   -p second_distance:=7.0 -p second_lateral:=-0.3 \
    -p third_trigger_distance:=20.0 -p third_distance:=8.0 -p third_lateral:=0.4 &
  SPAWN_PID=$!
fi

# ---------- 3. 可选：自动设位姿/路线/auto/engage ----------
if [ "$DO_WAYPOINT" -eq 1 ]; then
  echo "==> 运行自动设位姿/路线/engage 脚本"
  python3 "${PNC_WS}/run_waypoint.py" --ros-args \
    -p input_file:="${PNC_WS}/waypoint_input.yaml" &
  WAYPOINT_PID=$!
else
  echo "==> 已跳过自动设路线/engage，请在 RViz 手动操作，或另行调用脚本"
fi

echo "==> 仿真运行中，按 Ctrl-C 停止"
# 保持脚本存活；launch 退出即被唤醒，交由 trap 清理
wait "${LAUNCH_PID}" 2>/dev/null || true
cleanup
