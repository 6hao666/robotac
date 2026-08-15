#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -f "${workspace}/devel/setup.bash" ]]; then
  echo "请先执行 tools/test_02_build.sh。" >&2
  exit 1
fi
set +u
source "${workspace}/devel/setup.bash"
set -u
rostest robotac_examples examples_sim.test
