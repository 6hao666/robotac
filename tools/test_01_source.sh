#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
python3 "${workspace}/tools/check_source.py"
bash -n "${workspace}"/tools/*.sh
if git -C "${workspace}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git -C "${workspace}" diff --check
fi
