#!/bin/bash
# ============================================================================
# sync_to_sim.sh — 把 pnc(阉割版, 改动源) 的包同步进 autoware(完整版, 仿真器) 并在本机编译
#
# 背景：pnc 是从公司小车拉下来的阉割版(无 autoware_launch, 无法独立启动仿真器)，
#       autoware 是官方完整版。二者 autoware_universe 同为 0.48.0 且改动互不重叠，
#       因此可以整包覆盖。
#
# 全部在宿主机原生进行，不用 Docker：colcon 生成的 install/setup.bash 是以
# `builtin cd $(dirname $BASH_SOURCE)` 相对推导自身前缀的，所以 install 树可重定位，
# 宿主机上直接 source 即可（已实测：包解析、launch 文件解析、rviz2 均正常）。
#
# 用法：
#   ./sync_to_sim.sh <包名> [<包名>...]   同步指定包并编译
#   ./sync_to_sim.sh --changed            自动同步所有「pnc 与 autoware 有差异」的包
#   ./sync_to_sim.sh --dry-run <包名>     只显示会发生什么，不改动任何文件
#   ./sync_to_sim.sh --no-build <包名>    只同步，不编译
#   ./sync_to_sim.sh --build-only <包名>  只编译，不同步
#   ./sync_to_sim.sh --force  <包名>      强制同步受保护包（危险，见下）
#   ./sync_to_sim.sh --list-protected     列出受保护包
#
# 同步编译完成后即可跑 ./launch_rviz.sh 启动 RViz 仿真验证。
# ============================================================================
set -euo pipefail

PNC_WS="${PNC_WS:-/home/jm/pnc}"
SIM_WS="${SIM_WS:-/home/jm/autoware}"

# autoware 侧独有的改动（减速带那批：3 个提交只存在于 autoware，pnc 里是纯净 0.48.0）。
# 若被 pnc 的纯净版覆盖，减速带功能会被静默抹掉，故默认拒绝。
PROTECTED=(
  autoware_pid_longitudinal_controller
  autoware_behavior_velocity_speed_bump_module
)

# rsync 与 diff 分别用各自的排除语法（diff 只认 -x，不认 --exclude）
RSYNC_EXCLUDES=(--exclude .git --exclude build --exclude install --exclude log
                --exclude __pycache__ --exclude '*.pyc')
DIFF_EXCLUDES=(-x .git -x build -x install -x log -x __pycache__)

info() { echo "==> $*"; }
warn() { echo "[警告] $*" >&2; }
die()  { echo "[错误] $*" >&2; exit 1; }

usage() {
  cat <<'EOF'
用法：
  ./sync_to_sim.sh <包名> [<包名>...]   同步指定包并编译
  ./sync_to_sim.sh --changed            自动同步所有「pnc 与 autoware 有差异」的包
  ./sync_to_sim.sh --dry-run <包名>     只显示会发生什么，不改动任何文件
  ./sync_to_sim.sh --no-build <包名>    只同步，不编译
  ./sync_to_sim.sh --build-only <包名>  只编译，不同步
  ./sync_to_sim.sh --force  <包名>      强制同步受保护包（危险）
  ./sync_to_sim.sh --list-protected     列出受保护包
  ./sync_to_sim.sh --help

示例：
  ./sync_to_sim.sh autoware_behavior_path_static_obstacle_avoidance_module
  ./sync_to_sim.sh --changed
EOF
  exit "${1:-0}"
}

# ---------- 定位包 ----------
# 只认「真包」（目录内含 package.xml）。这一步必不可少：autoware_launch 下有大量
# 与包同名的参数目录（如 .../behavior_path_planner/autoware_behavior_path_xxx_module/），
# 实测 157 个包中有 38 个存在这种同名误导目录，若只按名字 find 会命中它们，
# 把包内容 rsync 进 launch 的配置目录、覆盖掉参数 yaml。
list_pkg_dirs() {
  find "$1/src" -type d -name "$2" \
    -not -path '*/build/*' -not -path '*/install/*' -not -path '*/log/*' \
    -exec test -f '{}/package.xml' \; -print 2>/dev/null | sort
}

is_protected() {
  local name="$1" p
  for p in "${PROTECTED[@]}"; do
    [ "$p" = "$name" ] && return 0
  done
  return 1
}

# 两目录是否有差异（0=有差异，1=无差异）
pkg_differs() {
  ! diff -rq "$1" "$2" "${DIFF_EXCLUDES[@]}" >/dev/null 2>&1
}

# ---------- 同步一个包；返回 2 表示被受保护名单拒绝，1 表示无差异 ----------
sync_one() {
  local name="$1" force="$2" src dst
  local src_hits=() dst_hits=()

  mapfile -t src_hits < <(list_pkg_dirs "${PNC_WS}" "${name}")
  mapfile -t dst_hits < <(list_pkg_dirs "${SIM_WS}" "${name}")

  [ "${#src_hits[@]}" -ge 1 ] || die "在 ${PNC_WS}/src 下找不到包：${name}"
  [ "${#dst_hits[@]}" -ge 1 ] || die "在 ${SIM_WS}/src 下找不到包：${name}（该包可能只存在于 autoware，如 autoware_launch）"
  [ "${#src_hits[@]}" -eq 1 ] || die "包名有歧义，pnc 侧命中多个真包：$(printf '\n  %s' "${src_hits[@]}")"
  [ "${#dst_hits[@]}" -eq 1 ] || die "包名有歧义，autoware 侧命中多个真包：$(printf '\n  %s' "${dst_hits[@]}")"

  src="${src_hits[0]}"
  dst="${dst_hits[0]}"

  if is_protected "${name}" && [ "${force}" != "1" ]; then
    warn "拒绝同步受保护包：${name}"
    warn "  该包在 autoware 侧有本地独有改动（减速带功能），pnc 里是纯净 0.48.0，"
    warn "  同步会用纯净版覆盖并静默抹掉那些改动。确实要覆盖请显式加 --force。"
    return 2
  fi

  if ! pkg_differs "${src}" "${dst}"; then
    info "无差异，跳过：${name}"
    return 1
  fi

  info "同步 ${name}"
  echo "      pnc  : ${src}"
  echo "      -> sim: ${dst}"
  if [ "${DRY_RUN}" = "1" ]; then
    echo "      [dry-run] rsync 会执行的动作："
    rsync -a --delete --dry-run --itemize-changes "${RSYNC_EXCLUDES[@]}" \
      "${src}/" "${dst}/" | sed 's/^/        /'
  else
    rsync -a --delete "${RSYNC_EXCLUDES[@]}" "${src}/" "${dst}/"
  fi
  return 0
}

# ---------- 前置检查：宿主编译需要 build/install 的写权限 ----------
check_writable() {
  local d owner
  for d in "${SIM_WS}/build" "${SIM_WS}/install"; do
    [ -e "$d" ] || continue
    if [ ! -w "$d" ]; then
      owner="$(stat -c %U "$d")"
      die "无写权限：${d}（属主 ${owner}）
  宿主机原生编译需要写权限，执行一次即可（之后长期有效）：
      sudo chown -R $(id -un):$(id -gn) ${SIM_WS}/build ${SIM_WS}/install ${SIM_WS}/log"
    fi
  done
}

# ---------- 清理容器遗留的 CMake 缓存 ----------
# 容器里编译时 CMAKE_HOME_DIRECTORY 写的是 /autoware_ws/src/...，宿主机上该路径不存在，
# cmake 会因「源目录与缓存不一致」直接报错。build/<包> 里有 /autoware_ws 残留就清掉重编。
purge_container_cache() {
  local name
  for name in "$@"; do
    local bd="${SIM_WS}/build/${name}"
    [ -d "${bd}" ] || continue
    if grep -rqs "/autoware_ws" "${bd}" \
         --include="CMakeCache.txt" --include="*.cmake" --include="*.sh" 2>/dev/null; then
      info "清理容器遗留的编译缓存：build/${name}（含 /autoware_ws 路径）"
      rm -rf "${bd}"
    fi
  done
}

# ---------- 宿主机原生编译 ----------
build_pkgs() {
  check_writable
  purge_container_cache "$@"

  info "编译：$*"
  # set -u 会让 ROS 的 setup.bash 因未定义变量报错，source 期间临时关闭。
  # 末尾的 grep 只过滤 install/setup.bash 链式 source /opt/autoware 时那条无害的
  # 缺失告警（容器基础镜像遗留的父前缀，宿主机上没有、也不需要）。
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${SIM_WS}/install/setup.bash" \
    2> >(grep -vF 'not found: "/opt/autoware/local_setup.bash"' >&2) || true
  set -u

  ( cd "${SIM_WS}" && colcon build --packages-select "$@" --allow-overriding "$@" )
  info "编译完成"
}

# ---------- 参数解析 ----------
FORCE=0; DO_BUILD=1; CHANGED=0; BUILD_ONLY=0; DRY_RUN=0; PKGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --force)          FORCE=1 ;;
    --no-build)       DO_BUILD=0 ;;
    --build-only)     BUILD_ONLY=1 ;;
    --dry-run)        DRY_RUN=1; DO_BUILD=0 ;;
    --changed)        CHANGED=1 ;;
    --list-protected) printf '%s\n' "${PROTECTED[@]}"; exit 0 ;;
    -h|--help)        usage 0 ;;
    --*)              die "未知参数：$1（用 --help 查看用法）" ;;
    *)                PKGS+=("$1") ;;
  esac
  shift
done

# --changed：挑出 pnc 与 autoware 有差异的包（受保护包稍后仍会被拦截）
if [ "${CHANGED}" = "1" ]; then
  info "扫描 ${PNC_WS}/src，找出与 autoware 有差异的包..."
  while IFS= read -r name; do
    [ -n "${name}" ] || continue
    mapfile -t src_hits < <(list_pkg_dirs "${PNC_WS}" "${name}")
    mapfile -t dst_hits < <(list_pkg_dirs "${SIM_WS}" "${name}")
    # 只比较双方都恰好命中一个真包的情况，有歧义的交给 sync_one 报错
    [ "${#src_hits[@]}" -eq 1 ] && [ "${#dst_hits[@]}" -eq 1 ] || continue
    if pkg_differs "${src_hits[0]}" "${dst_hits[0]}"; then
      PKGS+=("${name}")
    fi
  done < <(find "${PNC_WS}/src" -name package.xml \
             -not -path '*/build/*' -not -path '*/install/*' -not -path '*/log/*' \
             -exec dirname {} \; 2>/dev/null | xargs -n1 basename 2>/dev/null | sort -u)
  [ "${#PKGS[@]}" -gt 0 ] || die "未发现与 autoware 有差异的包"
  info "发现 ${#PKGS[@]} 个有差异的包"
fi

[ "${#PKGS[@]}" -gt 0 ] || usage 1

# ---------- 执行 ----------
info "pnc=${PNC_WS}  sim=${SIM_WS}  force=${FORCE}"

if [ "${BUILD_ONLY}" = "1" ] && [ "${DRY_RUN}" = "0" ]; then
  build_pkgs "${PKGS[@]}"
  info "完成（仅编译）。接着可运行：./launch_rviz.sh"
  exit 0
fi

SYNCED=(); REFUSED=()
for name in "${PKGS[@]}"; do
  rc=0
  sync_one "${name}" "${FORCE}" || rc=$?
  case "${rc}" in
    0) SYNCED+=("${name}") ;;
    1) : ;;                                    # 无差异，跳过
    *) REFUSED+=("${name}"); warn "已跳过：${name}" ;;   # 受保护名单拒绝
  esac
done

# 有包被拒绝却报成功会误导调用方，这里显式失败
if [ "${#REFUSED[@]}" -gt 0 ] && [ "${#SYNCED[@]}" -eq 0 ]; then
  die "请求的包全部被拒绝（受保护名单）。如确需覆盖请加 --force。"
fi

if [ "${#SYNCED[@]}" -eq 0 ]; then
  info "没有任何包需要同步（均无差异），无需编译。"
  exit 0
fi

if [ "${DRY_RUN}" = "1" ]; then
  info "dry-run 结束：以上为将要发生的事，未改动任何文件。"
  info "去掉 --dry-run 即真正执行："
  info "  ./sync_to_sim.sh ${SYNCED[*]}"
elif [ "${DO_BUILD}" = "1" ]; then
  build_pkgs "${SYNCED[@]}"
  info "完成。接着可运行：./launch_rviz.sh"
else
  info "--no-build：已跳过编译。需要编译时执行："
  info "  ./sync_to_sim.sh --build-only ${SYNCED[*]}"
fi
