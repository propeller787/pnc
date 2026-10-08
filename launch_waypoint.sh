#!/bin/bash
# 免 RViz 自动启动：planning_simulator（rviz 关，launch_api 开保留定位接口/engage）+ 读取 waypoint_input.yaml 自动设位姿/路线/auto/engage。
#
# 用法：
#   cd /home/d123/autoware3 && ./launch_waypoint.sh
# 每次开新终端都要先导出 DDS 环境（本脚本已自动导出）：
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   export ROS_DOMAIN_ID=17

set -eo pipefail

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=17

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_ROOT=${MAP_ROOT:-${HOME}/autoware_map}

# 地图自由选择（三级）：命令行第一个参数 > MAP_PATH 环境变量 > 自动选最新
MAP_ARG=""
for arg in "$@"; do MAP_ARG="${MAP_ARG:-$arg}"; done
if [ -n "$MAP_ARG" ]; then
  MAP_PATH="$MAP_ARG"
  # 若传的是目录名而非完整路径，自动拼到 MAP_ROOT 下
  [ ! -d "$MAP_PATH" ] && MAP_PATH="${MAP_ROOT}/${MAP_PATH}"
elif [ -n "${MAP_PATH:-}" ] && [ -d "${MAP_PATH:-}" ]; then
  : # 已用环境变量 MAP_PATH 指定
else
  newest=$(ls -dt "${MAP_ROOT}"/sample-map-planning_local-*/ 2>/dev/null | head -1 || true)
  [ -z "$newest" ] && newest=$(ls -dt "${MAP_ROOT}"/sample-map-planning*/ 2>/dev/null | head -1 || true)
  MAP_PATH="${newest%/}"
fi
if [ -z "$MAP_PATH" ] || [ ! -d "$MAP_PATH" ]; then
  echo "[错误] 地图路径无效: '$MAP_PATH'
  请在 ${MAP_ROOT} 选择一个地图目录作为第一个参数，或设 MAP_PATH" >&2
  exit 1
fi

INPUT_FILE=${WS_DIR}/waypoint_input.yaml

source ${WS_DIR}/install/setup.bash

echo "==> 启动 planning_simulator（rviz:=false, launch_api:=true）"
ros2 launch autoware_launch planning_simulator.launch.xml \
  vehicle_model:=cruise_vehicle \
  sensor_model:=sample_sensor_kit \
  map_path:=${MAP_PATH} \
  rviz:=false \
  launch_api:=true &
LAUNCH_PID=$!

# 清理函数：Ctrl-C 时一并关闭 launch
cleanup() {
  echo "==> 停止 planning_simulator（pid=${LAUNCH_PID}）"
  kill -INT -- -${LAUNCH_PID} 2>/dev/null || kill -INT ${LAUNCH_PID} 2>/dev/null || true
  wait ${LAUNCH_PID} 2>/dev/null || true
  [ -n "${SPAWN_PID:-}" ] && kill ${SPAWN_PID} 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM

echo "==> 启动静态行人障碍物投放脚本（第1个 8m后前方6m偏左0.3m；第2个 绕过后前方7m偏右0.3m；第3个 再20m后前方8m偏左0.4m）"
python3 ${WS_DIR}/spawn_pedestrians.py \
  --ros-args -p trigger_distance:=8.0 -p first_distance:=6.0 -p first_lateral:=0.3 \
  -p bypass_distance:=10.0 -p second_distance:=7.0 -p second_lateral:=-0.3 \
  -p third_trigger_distance:=20.0 -p third_distance:=8.0 -p third_lateral:=0.4 &
SPAWN_PID=$!

echo "==> 运行自动设位姿/路线/auto/engage 脚本（读取 ${INPUT_FILE}）"
python3 ${WS_DIR}/run_waypoint.py --ros-args -p input_file:=${INPUT_FILE}

cleanup
