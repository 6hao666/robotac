#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
"${workspace}/tools/build.sh"
set +u
source "${workspace}/devel/setup.bash"
set -u
for package in robotac_bringup robotac_examples robotac_localization robotac_servo; do
  rospack find "${package}" >/dev/null
done
echo "catkin 构建和项目包发现检查通过。"
