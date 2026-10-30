#!/usr/bin/env bash
# SITL 链路存活检查。
#
# 这个脚本只回答一个问题：PX4 SITL + Gazebo + uXRCE-DDS + ROS 2 这条链路能不能
# 起来，并且让 ROS 2 **真正收到** PX4 发的一条 VehicleStatus 消息。
# 它不验证起飞、解锁、避障、任务状态机或 EKF 性能——那些属于 validation/sitl/run_acceptance.py。
#
# 判据（全部必须成立）：
#   1. MicroXRCEAgent / Gazebo / PX4 正常启动
#   2. 在 /fmu/out/vehicle_status 或 /fmu/out/vehicle_status_vN 中精确发现一个主题
#   3. 该主题能解析出消息类型，且至少有一个 publisher
#   4. `ros2 topic echo --once` 能收到一帧真实消息
# 只满足"名字出现在 ros2 topic list 里"不算通过。
#
# 用法：scripts/ci/sitl_smoke.sh [超时秒数]
# 环境变量：
#   ROS_SETUP         默认 /opt/ros/jazzy/setup.bash
#   WORKSPACE_SETUP   默认 <cwd>/install/setup.bash；source 之后才能解析
#                     px4_msgs 的消息类型（不 source 时 echo 必然失败）
#   PX4_ROOT          默认 /home/pawn/drone-simulation
#   PX4_WORLD         默认 $PX4_ROOT/Tools/simulation/gz/worlds/default.sdf
#   AGENT_PORT        默认 8888
set -uo pipefail

TIMEOUT_S=${1:-180}
ROS_SETUP=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
WORKSPACE_SETUP=${WORKSPACE_SETUP:-$(pwd)/install/setup.bash}
PX4_ROOT=${PX4_ROOT:-/home/pawn/drone-simulation}
PX4_WORLD=${PX4_WORLD:-$PX4_ROOT/Tools/simulation/gz/worlds/default.sdf}
AGENT_PORT=${AGENT_PORT:-8888}
PX4_BIN=$PX4_ROOT/build/px4_sitl_default/bin/px4
PX4_ROOTFS=$PX4_ROOT/build/px4_sitl_default/tmp/rootfs

LOG_DIR=${LOG_DIR:-$(pwd)/log/ci_sitl_smoke}
mkdir -p "$LOG_DIR"

fail() { echo "::error::$*" >&2; exit 1; }
note() { echo "[smoke] $*"; }

now_s() { date +%s; }

# 单次查询的超时不得超过剩余预算，避免最后一轮把整个作业拖过 timeout-minutes。
remaining_s() {
  local left=$(( DEADLINE - $(now_s) ))
  if [ "$left" -gt 0 ]; then echo "$left"; else echo 0; fi
}
clamp_s() {  # clamp_s <desired> <max>
  local d=$1 m=$2
  if [ "$m" -le 0 ]; then echo 0; return; fi
  if [ "$d" -lt "$m" ]; then echo "$d"; else echo "$m"; fi
}

note "=== preflight ==="
for f in "$ROS_SETUP" "$PX4_BIN"; do
  [ -f "$f" ] || fail "missing file: $f"
done
PX4_ETC=$PX4_ROOT/build/px4_sitl_default/etc
[ -d "$PX4_ETC" ] || fail "missing ROMFS source: $PX4_ETC (did the build finish?)"

# 全新构建后 tmp/rootfs 并不存在：PX4 是在 `make px4_sitl` 启动时才从
# build/.../etc 铺出来的。这里自己铺一份，避免依赖那条会顺带启动 Gazebo 的目标。
if [ ! -d "$PX4_ROOTFS/etc" ]; then
  mkdir -p "$PX4_ROOTFS"
  cp -a "$PX4_ETC" "$PX4_ROOTFS/etc"
  note "prepared rootfs from $PX4_ETC"
fi

set +u
# shellcheck disable=SC1090  # 路径来自环境变量，静态无法跟随
source "$ROS_SETUP"
set -u
note "ROS_DISTRO=${ROS_DISTRO:-<unset>}"

# 消息类型必须可解析，否则 ros2 topic echo 无论如何都读不出样本。
if [ -f "$WORKSPACE_SETUP" ]; then
  set +u
  # shellcheck disable=SC1090  # 路径来自环境变量，静态无法跟随
  source "$WORKSPACE_SETUP"
  set -u
  note "sourced workspace: $WORKSPACE_SETUP"
else
  note "WARN: workspace setup not found at $WORKSPACE_SETUP; px4_msgs types will not resolve"
fi

for c in gz MicroXRCEAgent ros2 awk; do
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

GZ_MODELS=$PX4_ROOT/Tools/simulation/gz/models
GZ_WORLDS=$PX4_ROOT/Tools/simulation/gz/worlds
[ -d "$GZ_MODELS" ] || fail "missing Gazebo models dir: $GZ_MODELS"
export GZ_SIM_RESOURCE_PATH="$GZ_MODELS:$GZ_WORLDS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
note "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"

# PX4 自己的 gz 系统插件编在 build 里，不在系统路径上；不设它 Gazebo 会
# 报 "Could not find shared library"，机体系统起不来，PX4 拿不到 IMU。
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

DEADLINE=$(( $(now_s) + TIMEOUT_S ))
note "=== wait for a VehicleStatus topic (budget ${TIMEOUT_S}s) ==="

# 精确匹配：$1 必须正好是未版本化名字，或者 _v<数字> 形式；不接受子串。
# 用命令替换 + awk 而不是 `ros2 topic list | grep -q`：后者在 pipefail 下会被
# grep 提前退出引发的 SIGPIPE 干扰，可能得到假阴性。
discover_status_topics() {
  local budget=$1 listing
  listing=$(timeout "$budget" ros2 topic list -t 2>/dev/null || true)
  printf '%s\n' "$listing" | awk '
    $1 == "/fmu/out/vehicle_status" || $1 ~ /^\/fmu\/out\/vehicle_status_v[0-9]+$/ {
      v = 0
      if ($1 != "/fmu/out/vehicle_status") {
        rest = $1
        sub(/^.*_v/, "", rest)
        v = rest + 0
      }
      print v "\t" $1 "\t" $2
    }' | sort -rn -k1,1
}

STATUS_TOPIC=""
STATUS_TOPIC_TYPE=""
ATTEMPT=0
CANDIDATES_LOG=$LOG_DIR/vehicle_status_candidates.txt
: >"$CANDIDATES_LOG"

while :; do
  left=$(remaining_s)
  [ "$left" -le 0 ] && break
  ATTEMPT=$(( ATTEMPT + 1 ))

  per=$(clamp_s 60 "$left")
  mapfile -t rows < <(discover_status_topics "$per")
  if [ "${#rows[@]}" -gt 0 ]; then
    {
      echo "--- attempt $ATTEMPT (remaining ${left}s) ---"
      printf '%s\n' "${rows[@]}"
    } >>"$CANDIDATES_LOG"

    for row in "${rows[@]}"; do
      cand=$(printf '%s' "$row" | cut -f2)
      ctype=$(printf '%s' "$row" | cut -f3)
      left=$(remaining_s)
      [ "$left" -le 0 ] && break

      # 类型必须可解析，否则 echo 一定读不出东西。
      case "$ctype" in
        ""|"[unknown"*|"[UnknownType]"*)
          note "skip $cand: message type not resolvable (is px4_msgs built and sourced?)"
          continue ;;
      esac

      info_budget=$(clamp_s 30 "$left")
      info_out=$LOG_DIR/vehicle_status_info.txt
      timeout "$info_budget" ros2 topic info "$cand" --verbose >"$info_out" 2>&1 || true
      pub_count=$(awk -F': *' '/^Publisher count:/ {print $2; exit}' "$info_out" | tr -dc '0-9')
      note "candidate $cand type=$ctype publishers=${pub_count:-unknown}"
      if [ -n "$pub_count" ] && [ "${pub_count:-0}" -eq 0 ]; then
        note "skip $cand: no publisher"
        continue
      fi

      left=$(remaining_s)
      [ "$left" -le 0 ] && break
      echo_left=$(clamp_s 20 "$left")
      if timeout "$echo_left" ros2 topic echo --once "$cand" >"$LOG_DIR/vehicle_status.txt" 2>&1 \
         && [ -s "$LOG_DIR/vehicle_status.txt" ]; then
        STATUS_TOPIC=$cand
        STATUS_TOPIC_TYPE=$ctype
        note "received one message on $cand"
        break
      fi
      note "no message received on $cand within ${echo_left}s"
    done

    [ -n "$STATUS_TOPIC" ] && break
  fi
  sleep 3
done

note "discovery attempts: $ATTEMPT"
note "STATUS_TOPIC=${STATUS_TOPIC:-<none>}"
note "STATUS_TOPIC_TYPE=${STATUS_TOPIC_TYPE:-<none>}"

if [ -n "$STATUS_TOPIC" ]; then
  head -20 "$LOG_DIR/vehicle_status.txt" 2>/dev/null || true
  {
    echo "### SITL smoke"
    echo ""
    echo "- ROS_DISTRO: ${ROS_DISTRO:-unset}"
    echo "- GZ_SIM_RESOURCE_PATH: \`$GZ_SIM_RESOURCE_PATH\`"
    echo "- GZ_SIM_SYSTEM_PLUGIN_PATH: \`$GZ_SIM_SYSTEM_PLUGIN_PATH\`"
    echo "- STATUS_TOPIC: \`$STATUS_TOPIC\`"
    echo "- STATUS_TOPIC_TYPE: \`$STATUS_TOPIC_TYPE\`"
    echo "- publisher: $(awk -F': *' '/^Publisher count:/ {print $2; exit}' "$LOG_DIR/vehicle_status_info.txt" 2>/dev/null | tr -dc '0-9')"
    echo "- sample: received"
  } >>"${GITHUB_STEP_SUMMARY:-/dev/null}"
  echo "SMOKE PASS: PX4-Gazebo-uXRCE-DDS-ROS2 data path is alive"
  exit 0
fi

note "=== failure diagnostics ==="
echo "--- ros2 topic list -t (first 40) ---"
timeout 30 ros2 topic list -t 2>/dev/null | head -40 || true
echo "--- px4.log (tail) ---"; tail -40 "$LOG_DIR/px4.log" 2>/dev/null || true
echo "--- gz.log (tail) ---"; tail -40 "$LOG_DIR/gz.log" 2>/dev/null || true
echo "--- xrce.log (tail) ---"; tail -20 "$LOG_DIR/xrce.log" 2>/dev/null || true
fail "no VehicleStatus message was received within ${TIMEOUT_S}s"
