#!/usr/bin/env bash
# SITL 链路存活检查。
#
# 这个脚本只回答一个问题：PX4 SITL + Gazebo + uXRCE-DDS + ROS 2 这条链路能不能
# 起来并让 ROS 2 看到 PX4 的 topic。它**不**验证任务能力、不验证起飞、不验证避障。
# 任务级验收请用 run_p2_sitl.py，它的结果不在这里冒充通过。
#
# 用法：scripts/ci/sitl_smoke.sh [超时秒数]
# 环境变量：
#   ROS_SETUP   默认 /opt/ros/jazzy/setup.bash
#   PX4_ROOT    默认 /home/pawn/drone-simulation
#   PX4_WORLD   默认 $PX4_ROOT/Tools/simulation/gz/worlds/default.sdf
#   AGENT_PORT  默认 8888
set -uo pipefail

TIMEOUT_S=${1:-180}
ROS_SETUP=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
PX4_ROOT=${PX4_ROOT:-/home/pawn/drone-simulation}
PX4_WORLD=${PX4_WORLD:-$PX4_ROOT/Tools/simulation/gz/worlds/default.sdf}
AGENT_PORT=${AGENT_PORT:-8888}
PX4_BIN=$PX4_ROOT/build/px4_sitl_default/bin/px4
PX4_ROOTFS=$PX4_ROOT/build/px4_sitl_default/tmp/rootfs

LOG_DIR=${LOG_DIR:-$(pwd)/log/ci_sitl_smoke}
mkdir -p "$LOG_DIR"

fail() { echo "::error::$*" >&2; exit 1; }
note() { echo "[smoke] $*"; }

note "=== preflight ==="
for f in "$ROS_SETUP" "$PX4_BIN"; do
  [ -f "$f" ] || fail "missing file: $f"
done
PX4_ETC=$PX4_ROOT/build/px4_sitl_default/etc
[ -d "$PX4_ETC" ] || fail "missing ROMFS source: $PX4_ETC (did the build finish?)"
[ -f "$PX4_WORLD" ] || fail "missing world file: $PX4_WORLD (set PX4_WORLD)"

# 全新构建后 tmp/rootfs 并不存在（本地机器上同样没有）：PX4 是在
# `make px4_sitl` 启动时才从 build/.../etc 铺出来的。这里自己铺一份，
# 这样 smoke 不依赖那条会顺带启动 Gazebo 的 make 目标。
if [ ! -d "$PX4_ROOTFS/etc" ]; then
  mkdir -p "$PX4_ROOTFS"
  cp -a "$PX4_ETC" "$PX4_ROOTFS/etc"
  note "prepared rootfs from $PX4_ETC"
fi
# 必须先 source 再查 ros2：/opt/ros/jazzy/setup.bash 才把它放进 PATH。
# shellcheck disable=SC1090
set +u; source "$ROS_SETUP"; set -u
note "ROS_DISTRO=${ROS_DISTRO:-<unset>}"

for c in gz MicroXRCEAgent ros2; do
  command -v "$c" >/dev/null 2>&1 || fail "command not found after sourcing $ROS_SETUP: $c"
done

PIDS=()
cleanup() {
  local rc=$?
  note "=== cleanup (rc=$rc) ==="
  for pid in "${PIDS[@]:-}"; do
    if [ -n "${pid}" ] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "${PIDS[@]:-}"; do
    if [ -n "${pid}" ] && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "MicroXRCEAgent" 2>/dev/null || true
  pkill -f "px4_sitl_default/bin/px4" 2>/dev/null || true
  exit "$rc"
}
trap cleanup EXIT INT TERM

export GZ_PARTITION="smoke-$$"
export GZ_IP=127.0.0.1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$(shuf -i 30-60 -n 1)}
export ROS2CLI_ENABLE_DAEMON=0
export PX4_GZ_STANDALONE=1

# Gazebo 需要知道模型和世界在哪。PX4 自己的 make px4_sitl 会设这两个变量，
# 我们直接拉起 gz 时必须自己设，否则 world 里的 file://x500_depth/model.sdf
# 解析不到（"Unable to find uri"）。
GZ_MODELS=$PX4_ROOT/Tools/simulation/gz/models
GZ_WORLDS=$PX4_ROOT/Tools/simulation/gz/worlds
[ -d "$GZ_MODELS" ] || fail "missing Gazebo models dir: $GZ_MODELS"
export GZ_SIM_RESOURCE_PATH="$GZ_MODELS:$GZ_WORLDS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
note "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"

# PX4 自己的 gz 系统插件（MotorFailurePlugin 等）编在 build 里，不在系统路径上。
# 不设这个变量时 Gazebo 会报 "Could not find shared library"，模型虽然解析成功，
# 但机体系统起不来，PX4 永远拿不到 IMU/气压计，也就不会发布 /fmu/out/*。
GZ_PLUGINS=$PX4_ROOT/build/px4_sitl_default/src/modules/simulation/gz_plugins
[ -d "$GZ_PLUGINS" ] || fail "missing PX4 gz plugins dir: $GZ_PLUGINS"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GZ_PLUGINS${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
note "GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH"
export PX4_GZ_MODEL=PX4
export PX4_SIM_MODEL=gz_x500_depth
export PX4_SYS_AUTOSTART=4002

note "=== start MicroXRCEAgent on udp/$AGENT_PORT ==="
setsid MicroXRCEAgent udp4 -p "$AGENT_PORT" >"$LOG_DIR/xrce.log" 2>&1 &
PIDS+=($!)

note "=== start Gazebo headless: $PX4_WORLD ==="
setsid gz sim -r -s "$PX4_WORLD" >"$LOG_DIR/gz.log" 2>&1 &
PIDS+=($!)

note "=== start PX4 SITL ==="
(cd "$PX4_ROOT" && setsid "$PX4_BIN" -d -i 0 -w "$PX4_ROOTFS" >"$LOG_DIR/px4.log" 2>&1 & echo $! >"$LOG_DIR/px4.pid")
[ -f "$LOG_DIR/px4.pid" ] && PIDS+=("$(cat "$LOG_DIR/px4.pid")")

note "=== wait for /fmu/out/vehicle_status (budget ${TIMEOUT_S}s) ==="
deadline=$(( $(date +%s) + TIMEOUT_S ))
found=0
attempt=0
# 用 60s 而不是 10s：ROS2CLI_ENABLE_DAEMON=0 时每次 ros2 topic list 都要现场
# 做一遍发现，10s 会周期性被 timeout 杀掉、返回空输出，看起来像"主题一直没出现"。
# PX4 新版会给消息加版本后缀，所以 vehicle_status 和 vehicle_status_v4 都要认。
while [ "$(date +%s)" -lt "$deadline" ]; do
  attempt=$((attempt + 1))
  if timeout 60 ros2 topic list 2>/dev/null | grep -qE '^/fmu/out/vehicle_status(_v[0-9]+)?$'; then
    found=1
    break
  fi
  sleep 3
done
note "wait finished after $attempt attempt(s)"

if [ "$found" -eq 1 ]; then
  note "topic /fmu/out/vehicle_status is visible"
  timeout 20 ros2 topic echo --once /fmu/out/vehicle_status >"$LOG_DIR/vehicle_status.txt" 2>&1 \
    || note "WARN: topic appeared but sample could not be read"
  head -20 "$LOG_DIR/vehicle_status.txt" 2>/dev/null || true
  echo "SMOKE PASS: simulation chain is alive"
  exit 0
fi

note "=== failure diagnostics ==="
ros2 topic list 2>/dev/null | head -30 || true
echo "--- px4.log (tail) ---"; tail -40 "$LOG_DIR/px4.log" 2>/dev/null || true
echo "--- gz.log (tail) ---"; tail -40 "$LOG_DIR/gz.log" 2>/dev/null || true
echo "--- xrce.log (tail) ---"; tail -20 "$LOG_DIR/xrce.log" 2>/dev/null || true
fail "simulation chain did not come up within ${TIMEOUT_S}s"
