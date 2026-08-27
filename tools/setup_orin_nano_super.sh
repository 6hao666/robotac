#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec "${workspace}/tools/setup_jetson_orin.sh" "$@"
