#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="${workspace}/src/robotac_examples/src:${workspace}/src/robotac_localization/src:${workspace}/src/robotac_servo/src${PYTHONPATH:+:${PYTHONPATH}}"
python3 -m unittest discover -s "${workspace}/src/robotac_examples/test" -p 'test_*.py'
python3 -m unittest discover -s "${workspace}/src/robotac_localization/test" -p 'test_*.py'
python3 -m unittest discover -s "${workspace}/src/robotac_servo/test" -p 'test_*.py'
python3 -m unittest discover -s "${workspace}/tools/test" -p 'test_*.py'
