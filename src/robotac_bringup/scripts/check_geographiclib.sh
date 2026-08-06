#!/usr/bin/env bash
set -euo pipefail

geoids_dir=/usr/share/GeographicLib/geoids
if [[ -d "${geoids_dir}" ]] && compgen -G "${geoids_dir}/*.pgm" >/dev/null; then
  echo "GeographicLib geoid datasets found in ${geoids_dir}."
  if [[ "${1:-}" == "--hold" ]]; then
    # Keep the required launch preflight alive after a successful validation.
    exec tail -f /dev/null
  fi
  exit 0
fi

echo "WARNING: GeographicLib geoid datasets are missing."
echo "Run src/mavros/mavros/scripts/install_geographiclib_datasets.sh on Ubuntu before using global-position MAVROS plugins."
exit 1
