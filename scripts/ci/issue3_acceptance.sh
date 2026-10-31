#!/usr/bin/env bash
# shellcheck disable=SC1091  # ROS/workspace setup files belong to the target runner.
# 手动运行实际电厂验收；只写本次日志和隔离运行资源，不修改 PX4/model 树。
# 用法：issue3_acceptance.sh prepare|scan|calibrate|mission
# PX4_ROOT、NATIVE_OUTPUT_ROOT 可显式配置；mission 必须给出 CONTROLLER_PROFILE。
set -euo pipefail

scope=${1:-prepare}
case "$scope" in
  prepare|scan|calibrate|mission) ;;
  -h|--help)
    echo 'usage: issue3_acceptance.sh prepare|scan|calibrate|mission'
    echo 'mission requires CONTROLLER_PROFILE pointing to an existing frozen profile'
    exit 0 ;;
  *) echo "unsupported acceptance scope: $scope" >&2; exit 2 ;;
esac

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
px4_root=${PX4_ROOT:-$HOME/drone-simulation}
output=${NATIVE_OUTPUT_ROOT:-$workspace/log/issue3_native/$(date -u +%Y%m%dT%H%M%SZ)-$$}
profile=${CONTROLLER_PROFILE:-$workspace/src/px4_powerplant_mission/config/p2_sitl_controller.yaml}
if [[ "$scope" == mission && -z "${CONTROLLER_PROFILE:-}" ]]; then
  echo 'mission requires an explicit frozen CONTROLLER_PROFILE' >&2
  exit 2
fi
[[ -d "$px4_root" && -f "$profile" ]] || { echo 'PX4 root or controller profile is missing' >&2; exit 2; }
[[ ! -e "$output" ]] || { echo "refusing to reuse evidence directory: $output" >&2; exit 2; }
mkdir -p "$output"
output=$(cd "$output" && pwd)
px4_root=$(cd "$px4_root" && pwd)

# 同一个用户的全部本仓库验收共用锁；额外查找其他用户/非本脚本进程。
# 不对已有进程发送信号；既有 smoke workflow 也不得与本任务并发执行。
lock_dir=$HOME/.cache/px4_ros_ws
mkdir -p "$lock_dir"
exec 9>"$lock_dir/flight.lock"
flock -n 9 || { echo 'another flight acceptance owns the runner lock' >&2; exit 2; }
python3 - "$output" <<'PY'
import json, os
from pathlib import Path
import sys
found=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit(): continue
    try:
        args=(proc/'cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
        if not args or not args[0]: continue
        name=Path(args[0]).name
        sim=(name in {'px4','MicroXRCEAgent','gzserver','gzclient'}
             or name.startswith(('gz-sim','ignition-gazebo'))
             or (any(Path(a).name=='gz' for a in args[:2]) and 'sim' in args[1:4]))
        if sim: found.append({'pid':int(proc.name),'uid':proc.stat().st_uid,'executable':name,'argv':args})
    except (FileNotFoundError,ProcessLookupError,PermissionError): continue
Path(sys.argv[1],'existing_simulation_processes.json').write_text(json.dumps(found,indent=2))
if found: raise SystemExit('existing Gazebo/PX4/Agent processes found; refusing to interfere')
PY

# Jazzy 的 setup 读取部分未定义变量；只在 source 期间关闭 nounset。
[[ -f /opt/ros/jazzy/setup.bash && -f "$workspace/install/setup.bash" ]] || {
  echo 'ROS Jazzy or workspace setup missing; build the workspace first' >&2; exit 2;
}
set +u
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
# 仅在系统缺少所需 DDS 时，用现有锁定包补齐用户态依赖。
# 缓存留在 checkout 外；不修改 apt、系统 ROS 或外部 PX4/model 树。
if [[ ! -f /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ]]; then
  export P0_ROS_PREFIX="$lock_dir/ros-deps/opt/ros/jazzy"
  if [[ ! -f "$P0_ROS_PREFIX/lib/librmw_cyclonedds_cpp.so" ]]; then
    if ! python3 "$workspace/tools/env/bootstrap_ros.py" --prefix "$lock_dir/ros-deps" \
      2>&1 | tee "$output/middleware_bootstrap.log"; then
      # 原工具显式禁用代理；仅更换下载通道，版本/字节锁保持不变。
      # curl 使用正常代理配置及 TLS 校验，所有包通过原 SHA 后才解压。
      python3 - "$workspace/tools/env/ros_dependencies.json" "$lock_dir/ros-deps" "$output" <<'PYDDS'
import hashlib, json, platform, subprocess, sys, tempfile
from pathlib import Path
lock, prefix, output = map(Path, sys.argv[1:])
release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
if release.get('VERSION_CODENAME', '').strip('"') != 'noble' or platform.machine() != 'x86_64':
    raise SystemExit('pinned DDS fallback requires Ubuntu noble amd64')
receipts = []
with tempfile.TemporaryDirectory(prefix='native-dds-') as temporary:
    verified = []
    for entry in json.loads(lock.read_text())['packages']:
        target = Path(temporary) / (entry['package'] + '.deb')
        urls = [entry['url'], entry['url'].replace(
            'https://mirrors.ustc.edu.cn/ros2/ubuntu/',
            'https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/')]
        for url in urls:
            result = subprocess.run(['curl', '-q', '--fail', '--location', '--silent', '--show-error',
                '--proto', '=https', '--proto-redir', '=https', '--connect-timeout', '10',
                '--max-time', '30', '--retry', '1', '--output', str(target), url])
            if result.returncode != 0:
                continue
            if hashlib.sha256(target.read_bytes()).hexdigest() != entry['sha256']:
                raise SystemExit('DDS fallback SHA256 mismatch: ' + entry['package'])
            verified.append(target)
            receipts.append({'package': entry['package'], 'version': entry['version'],
                             'url': url, 'sha256': entry['sha256']})
            break
        else:
            raise SystemExit('locked DDS dependency unavailable: ' + entry['package'])
    prefix.mkdir(parents=True, exist_ok=True)
    for target in verified:
        subprocess.run(['dpkg-deb', '-x', str(target), str(prefix)], check=True)
(output / 'middleware_fallback_receipts.json').write_text(json.dumps(receipts, indent=2))
PYDDS
    fi
  fi
fi
source "$workspace/tools/env/ros_env.sh"
set -u
python3 - "$output/middleware.json" <<'PYRMW'
import json, sys
from pathlib import Path
import rclpy
rclpy.init()
try:
    actual = rclpy.get_rmw_implementation_identifier()
    Path(sys.argv[1]).write_text(json.dumps({'actual_rmw': actual}, indent=2))
    if actual != 'rmw_cyclonedds_cpp':
        raise SystemExit('unexpected ROS middleware: ' + actual)
finally:
    rclpy.shutdown()
PYRMW
for command_name in gz MicroXRCEAgent ros2; do
  command -v "$command_name" >/dev/null || { echo "required command missing: $command_name" >&2; exit 2; }
done
# 不让 WSL/Mesa 软件渲染覆盖悄悄进入原生 GPU 对照实验。
for variable in GALLIUM_DRIVER MESA_D3D12_DEFAULT_ADAPTER_NAME LIBGL_ALWAYS_SOFTWARE; do
  [[ -z "${!variable:-}" ]] || { echo "incompatible renderer override is set: $variable" >&2; exit 2; }
done
# GPU 类型是待测事实；缺少 NVIDIA 工具不能阻断资源审计。
# 实际 renderer 仍由验收器捕获本次 Gazebo/Ogre 日志独立判定。
{
  if command -v nvidia-smi >/dev/null; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv || true
  else
    echo 'nvidia-smi: not installed'
  fi
  if command -v lspci >/dev/null; then
    lspci | awk '/VGA|3D controller|Display controller/'
  fi
  python3 - <<'PYDEV'
from pathlib import Path
for pattern in ('dri/*', 'nvidia*', 'dxg'):
    for path in sorted(Path('/dev').glob(pattern)):
        print(path)
PYDEV
} > "$output/gpu.txt" 2>&1
{
  printf 'host=%s\nuser=%s\nkernel=%s\ncpus=%s\n' "$(hostname)" "$(id -un)" "$(uname -sr)" "$(nproc)"
  printf 'workspace=%s\npx4_root=%s\nscope=%s\n' "$workspace" "$px4_root" "$scope"
  . /etc/os-release
  printf 'os=%s\nros_distro=%s\nrmw=%s\n' "$PRETTY_NAME" "${ROS_DISTRO:-}" "${RMW_IMPLEMENTATION:-}"
  free -m
  df -h "$output" "$px4_root"
  git -C "$workspace" rev-parse HEAD
  git -C "$workspace" status --short
  git -C "$px4_root" rev-parse HEAD
  git -C "$px4_root" status --short
  gz sim --versions
} > "$output/environment.txt" 2>&1

# 保留原始 manifest；只有所有锁定字节和完整机体审计 hash 都相同才允许
# 生成本次运行自己的版本锁。真实几何和支持域仍由 run_acceptance 再次审计。
python3 - "$workspace" "$px4_root" "$output" <<'PY'
import hashlib, json, subprocess, sys
from pathlib import Path
workspace,px4,out=map(Path,sys.argv[1:])
root=(px4/'Tools/simulation/gz').resolve()
source=workspace/'src/px4_powerplant_mission_cpp/config/powerplant_assets.json'
body_path=workspace/'validation/evidence/sitl_body_envelope.json'
payload=json.loads(source.read_text())
if payload.get('schema_version')!=1 or payload.get('asset_commit_pending'):
    raise SystemExit('asset lock is not a committed schema-1 baseline')
head=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
entries=payload.get('files')
if not isinstance(entries,list) or not entries: raise SystemExit('asset lock has no file list')
checked=[]
paths=set()
def verify(path,expected,kind):
    actual=path.resolve()
    if not actual.is_relative_to(root): raise SystemExit('asset path escapes model root')
    relative=str(actual.relative_to(root))
    if relative in paths: raise SystemExit('duplicate asset file')
    paths.add(relative)
    digest=hashlib.sha256(actual.read_bytes()).hexdigest()
    if digest!=expected: raise SystemExit('asset hash mismatch: '+relative)
    # 同时检查工作树内容与当前提交，避免把未提交资源绑定到 HEAD。
    tracked=subprocess.check_output(['git','-C',str(root),'show',head+':'+relative])
    if hashlib.sha256(tracked).hexdigest()!=digest:
        raise SystemExit('asset differs from checked-out model commit: '+relative)
    checked.append({'path':relative,'sha256':digest,'kind':kind})
for item in entries:
    if not isinstance(item,dict) or not isinstance(item.get('path'),str):
        raise SystemExit('invalid asset lock entry')
    relative=Path(item['path'])
    if relative.is_absolute() or '..' in relative.parts: raise SystemExit('invalid asset path')
    verify(root/relative,item.get('sha256'),'scene_asset')
body=json.loads(body_path.read_text())
hashes=body.get('source_sdf_sha256',{})
if {Path(p).parent.name for p in hashes}!={'x500_depth','x500','x500_base','OakD-Lite'}:
    raise SystemExit('incomplete body source audit')
for path,digest in hashes.items():
    verify(root/'models'/Path(path).parent.name/'model.sdf',digest,'body_sdf')
(out/'source_powerplant_assets.json').write_bytes(source.read_bytes())
audit={'source_manifest_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
       'source_model_commit':payload['model_git_commit'],'actual_model_commit':head,
       'rebound':payload['model_git_commit']!=head,
       'body_audit_sha256':hashlib.sha256(body_path.read_bytes()).hexdigest(),
       'verified_files':checked,
       'scope':'run-local version binding after exact scene/body byte checks; does not replace geometry, support or runtime acceptance'}
(out/'asset_binding_audit.json').write_text(json.dumps(audit,indent=2))
payload.update(model_git_commit=head,asset_root_hint=str(root))
(out/'powerplant_assets.json').write_text(json.dumps(payload,indent=2))
PY

common=(--px4-root "$px4_root" --detection-mode none --render-context egl
  --sensor-profile cpu --cpu-camera-rate-hz 5 --cpu-lidar-rate-hz 5
  --powerplant-asset-lock "$output/powerplant_assets.json"
  --ros-domain "${ACCEPTANCE_ROS_DOMAIN:-25}" --agent-port "${ACCEPTANCE_AGENT_PORT:-8889}"
  --timeout "${ACCEPTANCE_TIMEOUT:-300}" --startup-timeout "${ACCEPTANCE_STARTUP_TIMEOUT:-120}")
cd "$workspace"
case "$scope" in
  prepare|scan)
    args=(validation/sitl/run_acceptance.py --scenario powerplant --stop-after scan
      --controller-profile "$profile" --controller-calibration-candidate 2 --takeoff-policy A
      --output-dir "$output/$scope" "${common[@]}")
    [[ "$scope" != prepare ]] || args+=(--prepare-only)
    python3 "${args[@]}" ;;
  calibrate)
    python3 validation/sitl/calibrate_takeoff.py --profile "$profile" \
      --output-dir "$output/calibration" -- "${common[@]}" ;;
  mission)
    # 验收器自己核验冻结证据、代码/模型及控制回读；不自行伪造 frozen。
    python3 validation/sitl/run_acceptance.py --scenario powerplant --stop-after mission \
      --controller-profile "$profile" --output-dir "$output/mission" "${common[@]}" ;;
esac
