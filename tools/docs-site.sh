#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=../docs-site/scripts/docs-site-lib.sh
source "${workspace}/docs-site/scripts/docs-site-lib.sh"

docs_site_main "$@"
