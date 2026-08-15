site_dir="${workspace}/docs-site"

load_env_file() {
  local env_file=${DOCS_ENV_FILE:-${site_dir}/.env}
  local line key value
  [[ -f "${env_file}" ]] || return 0

  while IFS= read -r line || [[ -n "${line}" ]]; do
    line=${line%$'\r'}
    [[ "${line}" =~ ^[[:space:]]*$ || "${line}" =~ ^[[:space:]]*# ]] && continue
    if [[ "${line}" != *=* ]]; then
      echo "无效的环境配置行：${env_file}" >&2
      return 1
    fi
    key=${line%%=*}
    value=${line#*=}
    if [[ ! "${key}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
      echo "无效的环境变量名：${key}" >&2
      return 1
    fi
    if [[ "${value}" == \"*\" || "${value}" == \'*\' ]]; then
      value=${value:1:${#value}-2}
    fi
    if ! declare -p "${key}" >/dev/null 2>&1; then
      printf -v "${key}" '%s' "${value}"
      export "${key}"
    fi
  done <"${env_file}"
}

load_env_file
remote_target=${DOCS_DEPLOY_TARGET:-}
remote_base=${DOCS_REMOTE_BASE:-/srv/www/docs.yundrone.cn}
public_base=${DOCS_PUBLIC_BASE:-https://docs.yundrone.cn}
keep_releases=${DOCS_KEEP_RELEASES:-5}

usage() {
  cat <<'EOF'
用法：./tools/docs-site.sh <command> [args]

命令：
  preview                 启动本地预览并打开浏览器
  diagrams                更新已提交的 PlantUML SVG
  build                   生成并验证静态站
  check                   检查现有构建；过期时重新构建
  deploy                  增量上传并原子发布
  status                  对比本地与线上版本
  rollback <release-id>   回滚到指定线上版本

环境变量：
  DOCS_ENV_FILE           配置文件路径，默认 docs-site/.env
  DOCS_OPEN_BROWSER=0     本地预览时不自动打开浏览器
  DOCS_PORT=3000          本地预览端口
  DOCS_PLANTUML_BIN       PlantUML 命令，默认 plantuml
  DOCS_DEPLOY_TARGET      SSH config 中的发布目标，部署和回滚时必填
  DOCS_REMOTE_BASE        远端发布根目录
  DOCS_PUBLIC_BASE        公网站点根 URL
EOF
}

require_deploy_target() {
  if [[ -z "${remote_target}" ]]; then
    echo "缺少 DOCS_DEPLOY_TARGET，请从 docs-site/.env.template 创建 docs-site/.env。" >&2
    exit 1
  fi
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "缺少命令：$1" >&2
    exit 1
  fi
}

ensure_node() {
  require_command node
  require_command pnpm
  node -e '
    const [major, minor] = process.versions.node.split(".").map(Number)
    if (major < 20 || (major === 20 && minor < 9)) {
      console.error(`需要 Node.js >= 20.9，当前为 ${process.version}`)
      process.exit(1)
    }
  '
}

ensure_dependencies() {
  ensure_node
  pnpm --dir "${site_dir}" install --frozen-lockfile --prefer-offline
}

build_site() {
  local docs_before docs_after
  ensure_dependencies
  docs_before=$(node "${site_dir}/scripts/write-release.mjs" --docs-digest)
  python3 "${workspace}/tools/check_source.py"
  pnpm --dir "${site_dir}" build
  docs_after=$(node "${site_dir}/scripts/write-release.mjs" --docs-digest)
  if [[ "${docs_before}" != "${docs_after}" ]]; then
    echo "构建过程修改了 docs/，已停止。" >&2
    exit 1
  fi
  pnpm --dir "${site_dir}" check
}

ensure_current_build() {
  ensure_dependencies
  if node "${site_dir}/scripts/write-release.mjs" --check-content >/dev/null 2>&1 \
    && pnpm --dir "${site_dir}" check >/dev/null 2>&1; then
    if node "${site_dir}/scripts/write-release.mjs" --check-release >/dev/null 2>&1; then
      echo "复用内容与版本信息一致的现有构建。"
    else
      node "${site_dir}/scripts/write-release.mjs"
      echo "复用静态构建，仅刷新 Git 版本信息。"
    fi
  else
    build_site
  fi
}

release_value() {
  node -e '
    const fs = require("fs")
    const release = JSON.parse(fs.readFileSync(process.argv[1], "utf8"))
    const value = release[process.argv[2]]
    console.log(typeof value === "object" ? JSON.stringify(value) : value)
  ' "${site_dir}/out/release.json" "$1"
}

health_check() {
  local expected_release=${1:-}
  local mode=${2:-current}
  if [[ "${mode}" == "rollback" ]]; then
    node "${site_dir}/scripts/check-public.mjs" --basic "${public_base}" "${expected_release}"
  else
    node "${site_dir}/scripts/check-public.mjs" "${public_base}" "${expected_release}"
  fi
}

update_diagrams() {
  ensure_node
  node "${site_dir}/scripts/render-diagrams.mjs"
}

preview_site() {
  local port=${DOCS_PORT:-3000}
  ensure_current_build

  if [[ "${DOCS_OPEN_BROWSER:-1}" != "0" && -t 1 ]]; then
    (
      local url="http://127.0.0.1:${port}/robotac/"
      for _ in $(seq 1 60); do
        if curl -fsS --max-time 1 "${url}" >/dev/null 2>&1; then
          if command -v open >/dev/null 2>&1; then
            open "${url}"
          elif command -v xdg-open >/dev/null 2>&1; then
            xdg-open "${url}" >/dev/null 2>&1
          fi
          exit 0
        fi
        sleep 0.5
      done
    ) &
  fi

  DOCS_PORT="${port}" node "${site_dir}/scripts/serve-preview.mjs"
}

deploy_site() {
  local release_id release_root staging release_path previous
  local -a rsync_args=(-azc --delete --itemize-changes --stats)
  require_command rsync
  require_command ssh
  require_command curl
  require_deploy_target
  ensure_current_build

  release_id=$(release_value releaseId)
  if [[ ! "${release_id}" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "非法发布版本：${release_id}" >&2
    exit 1
  fi

  if node "${site_dir}/scripts/check-public.mjs" \
    --matches-local "${public_base}" "${site_dir}/out/release.json"; then
    health_check "${release_id}"
    echo "线上已是相同内容与 Git 版本：${release_id}"
    return 0
  fi

  release_root="${remote_base}/releases/robotac"
  staging="${release_root}/.incoming-${release_id}-$$"
  release_path="${release_root}/${release_id}"
  previous=$(ssh "${remote_target}" "readlink '${remote_base}/robotac' 2>/dev/null || true")
  if [[ -n "${previous}" ]]; then
    rsync_args+=(--link-dest="${remote_base}/robotac")
  fi

  ssh "${remote_target}" "set -eu; mkdir -p '${release_root}'; rm -rf '${staging}'; mkdir '${staging}'"
  rsync "${rsync_args[@]}" \
    -e ssh "${site_dir}/out/" "${remote_target}:${staging}/"

  ssh "${remote_target}" "set -eu
    if [ -d '${release_path}' ]; then
      rm -rf '${staging}'
    else
      mv '${staging}' '${release_path}'
    fi
    ln -sfn 'releases/robotac/${release_id}' '${remote_base}/.robotac.next'
    mv -Tf '${remote_base}/.robotac.next' '${remote_base}/robotac'"

  if ! health_check "${release_id}"; then
    echo "线上健康检查失败，恢复上一版本。" >&2
    if [[ -n "${previous}" ]]; then
      ssh "${remote_target}" "set -eu; ln -sfn '${previous}' '${remote_base}/.robotac.next'; mv -Tf '${remote_base}/.robotac.next' '${remote_base}/robotac'"
    else
      ssh "${remote_target}" "rm -f '${remote_base}/robotac'"
    fi
    exit 1
  fi

  ssh "${remote_target}" "set -eu
    find '${release_root}' -mindepth 1 -maxdepth 1 -type d ! -name '.*' -printf '%T@ %p\\n' \
      | sort -rn \
      | tail -n +$((keep_releases + 1)) \
      | cut -d' ' -f2- \
      | xargs -r rm -rf"
  echo "发布完成：${public_base}/robotac/ (${release_id})"
}

show_status() {
  local local_release current_state online_release
  if [[ -f "${site_dir}/out/release.json" ]]; then
    local_release=$(cat "${site_dir}/out/release.json")
  else
    local_release='{"status":"尚未构建"}'
  fi
  current_state=$(node "${site_dir}/scripts/write-release.mjs" --show)
  online_release=$(curl -fsS --max-time 10 "${public_base}/robotac/release.json" 2>/dev/null || printf '%s' '{"status":"无法读取"}')
  printf '%s\n' \
    '--- 本地构建 ---' "${local_release}" \
    '--- 当前工作树 ---' "${current_state}" \
    '--- 线上版本 ---' "${online_release}"
}

rollback_site() {
  local release_id=${1:-} release_path previous
  require_deploy_target
  if [[ -z "${release_id}" || ! "${release_id}" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "rollback 需要合法的 release-id。" >&2
    exit 1
  fi
  release_path="${remote_base}/releases/robotac/${release_id}"
  previous=$(ssh "${remote_target}" "readlink '${remote_base}/robotac' 2>/dev/null || true")
  ssh "${remote_target}" "set -eu; test -d '${release_path}'; ln -sfn 'releases/robotac/${release_id}' '${remote_base}/.robotac.next'; mv -Tf '${remote_base}/.robotac.next' '${remote_base}/robotac'"
  if ! health_check "${release_id}" rollback; then
    echo "回滚目标健康检查失败，恢复原版本。" >&2
    if [[ -n "${previous}" ]]; then
      ssh "${remote_target}" "set -eu; ln -sfn '${previous}' '${remote_base}/.robotac.next'; mv -Tf '${remote_base}/.robotac.next' '${remote_base}/robotac'"
    fi
    exit 1
  fi
  echo "已回滚到 ${release_id}。"
}

docs_site_main() {
  local command=${1:-}
  case "${command}" in
    preview) preview_site ;;
    diagrams) update_diagrams ;;
    build) build_site ;;
    check) ensure_current_build ;;
    deploy) deploy_site ;;
    status) show_status ;;
    rollback) rollback_site "${2:-}" ;;
    *) usage; [[ -n "${command}" ]] && return 1 ;;
  esac
}
