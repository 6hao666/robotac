#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "用法：$0 <用户@主机:远端目录>" >&2
  exit 64
fi
workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
rsync -a --human-readable --progress \
  --exclude=.git --exclude=build --exclude=devel \
  --exclude=install --exclude=log --exclude=.ruff_cache \
  --exclude=__pycache__ --exclude='*.pyc' \
  "${workspace}/" "$1/"
