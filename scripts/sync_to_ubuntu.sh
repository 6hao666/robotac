#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 user@host[:/remote/path]"
  exit 64
fi

workspace_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
destination=$1

if [[ "${destination}" != *:* ]]; then
  destination="${destination}:~/robotac_ws"
fi

rsync -a --human-readable --progress \
  --exclude='.git' --exclude='build' --exclude='devel' --exclude='install' --exclude='log' \
  "${workspace_dir}/" "${destination}/"
