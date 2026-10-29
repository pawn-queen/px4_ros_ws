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
for d in "$PX4_ROOTFS"; do
  [ -d "$d" ] || fail "missing directory: $d"
done
[ -f "$PX4_WORLD" ] || fail "missing world file: $PX4_WORLD (set PX4_WORLD)"
for c in gz MicroXRCEAgent ros2; do
  command -v "$c" >/dev/null 2>&1 || fail "command not found on runner: $c"
done

# shellcheck disable=SC1090
set +u; source "$ROS_SETUP"; set -u
note "ROS_DISTRO=${ROS_DISTRO:-<unset>}"

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
while [ "$(date +%s)" -lt "$deadline" ]; do
  if timeout 10 ros2 topic list 2>/dev/null | grep -qx "/fmu/out/vehicle_status"; then
    found=1
    break
  fi
  sleep 3
done

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
