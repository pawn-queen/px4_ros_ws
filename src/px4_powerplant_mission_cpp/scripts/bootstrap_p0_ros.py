#!/usr/bin/env python3
"""Extract pinned ROS middleware packages locally, without root or apt changes."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import tempfile
import urllib.request


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, default=package.parents[1] / '.p0_deps')
    args = parser.parse_args()
    release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
    if release.get('VERSION_CODENAME', '').strip('"') != 'noble' or platform.machine() != 'x86_64':
        parser.error('pinned packages require Ubuntu noble amd64; install the matching ROS middleware for this platform')
    lock = json.loads((package / 'config/p0_ros_dependencies.json').read_text())
    # Use the repository's pinned SHA256 values; a changed mirror payload fails.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with tempfile.TemporaryDirectory(prefix='powerplant_p0_') as temporary:
        verified = []
        for entry in lock['packages']:
            data = opener.open(entry['url'], timeout=30).read()
            if hashlib.sha256(data).hexdigest() != entry['sha256']:
                raise RuntimeError('SHA256 mismatch for ' + entry['package'])
            path = Path(temporary) / (entry['package'] + '.deb')
            path.write_bytes(data)
            verified.append(path)
        args.prefix.mkdir(parents=True, exist_ok=True)
        for path in verified:
            subprocess.run(['dpkg-deb', '-x', str(path), str(args.prefix.resolve())], check=True)
    prefix = args.prefix / 'opt/ros/jazzy'
    (prefix / 'local_setup.bash').write_text(
        'P0_CURRENT_PREFIX="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"\n'
        'export AMENT_PREFIX_PATH="${P0_CURRENT_PREFIX}:${AMENT_PREFIX_PATH:-}"\n'
        'export LD_LIBRARY_PATH="${P0_CURRENT_PREFIX}/lib:${P0_CURRENT_PREFIX}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"\n'
        'unset P0_CURRENT_PREFIX\n')
    print('Middleware extracted to', args.prefix.resolve())


if __name__ == '__main__':
    main()
