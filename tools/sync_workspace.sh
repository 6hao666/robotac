#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<EOF
用法：$0 [选项] <用户@主机:远端绝对目录>

选项：
  --delete       删除远端在本地已不存在的源码文件
  --clean-build  同步前删除远端 build/devel/install/log
  --include-git  同步 .git，使远端 Git 基线与本地一致
  --init-env     缺少 .env 时从同目录 .env.template 初始化
  --mirror       完整镜像，相当于以上四个选项
  --dry-run      只显示 rsync 变化，不删除目录或初始化 .env
  -h, --help     显示本帮助

已有 .env 和 .env.local 始终保留，不会被上传或删除。
EOF
}

delete_remote=false; clean_build=false; include_git=false; init_env=false; dry_run=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --delete) delete_remote=true ;;
    --clean-build) clean_build=true ;;
    --include-git) include_git=true ;;
    --init-env) init_env=true ;;
    --mirror)
      delete_remote=true
      clean_build=true
      include_git=true
      init_env=true
      ;;
    --dry-run) dry_run=true ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      echo "未知选项：$1" >&2
      usage >&2
      exit 64
      ;;
    *) break ;;
  esac
  shift
done

if [[ $# -ne 1 || "$1" != *:* ]]; then
  usage >&2
  exit 64
fi

target=$1
remote_host=${target%%:*}
remote_path=${target#*:}
if [[ -z "$remote_host" || "$remote_path" != /* || "$remote_path" == *[[:space:]]* ]]; then
  echo "目标必须使用 <用户@主机:/不含空格的绝对目录> 格式。" >&2
  exit 64
fi
workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
rsync_args=(
  -a
  --human-readable
  --progress
  --exclude=build
  --exclude=devel
  --exclude=devel_full
  --exclude=install
  --exclude=log
  --exclude=.ruff_cache
  --exclude=__pycache__
  --exclude='*.pyc'
  --exclude=.DS_Store
  --exclude=.env
  --exclude=.env.local
  --exclude=docs-site/node_modules
  --exclude=docs-site/.next
  --exclude=docs-site/out
  --exclude=docs-site/content
  --exclude=docs-site/test-artifacts
  --exclude=docs-site/.build-state.json
  --exclude=docs-site/.content-map.json
  --exclude=src/robotac_bringup/config/lidar/mid360s.json  # 现场硬件 IP 留飞机本地
)

if [[ "$include_git" != true ]]; then
  rsync_args+=(--exclude=.git)
fi
if [[ "$delete_remote" == true ]]; then
  rsync_args+=(--delete)
fi
if [[ "$dry_run" == true ]]; then
  rsync_args+=(--dry-run --itemize-changes)
fi

if [[ "$clean_build" == true ]]; then
  if [[ "$dry_run" == true ]]; then
    echo "[预演] 将删除远端 build/devel/install/log。"
  else
    ssh "$remote_host" bash -s -- "$remote_path" <<'REMOTE_CLEAN'
set -euo pipefail
workspace=$1
case "$workspace" in
  /|/home|/opt|/srv|/usr|/var)
    echo "拒绝清理高风险目录：$workspace" >&2
    exit 64
    ;;
esac
if [[ "$workspace" != /* || ! -d "$workspace" ]]; then
  echo "远端工作空间不存在或不是绝对目录：$workspace" >&2
  exit 66
fi
rm -rf -- "$workspace/build" "$workspace/devel" "$workspace/devel_full" \
  "$workspace/install" "$workspace/log" "$workspace/.ruff_cache" \
  "$workspace/docs-site/node_modules" "$workspace/docs-site/.next" \
  "$workspace/docs-site/out" "$workspace/docs-site/content" \
  "$workspace/docs-site/test-artifacts" \
  "$workspace/docs-site/.build-state.json" \
  "$workspace/docs-site/.content-map.json"
find "$workspace" -type d -name __pycache__ -prune -exec rm -rf {} +
find "$workspace" -type f -name '*.pyc' -delete
REMOTE_CLEAN
  fi
fi

rsync "${rsync_args[@]}" "${workspace}/" "${target}/"

if [[ "$init_env" == true ]]; then
  if [[ "$dry_run" == true ]]; then
    echo "[预演] 将为缺失的 .env.template 初始化 .env，已有 .env 保持不变。"
  else
    ssh "$remote_host" bash -s -- "$remote_path" <<'REMOTE_ENV'
set -euo pipefail
workspace=$1
while IFS= read -r -d '' template; do
  env_file=${template%.template}
  if [[ -e "$env_file" ]]; then
    echo "保留已有环境文件：$env_file"
  else
    install -m 600 "$template" "$env_file"
    echo "已从模板初始化：$env_file"
  fi
done < <(find "$workspace" -type f -name '.env.template' \
  -not -path '*/.git/*' -print0)
REMOTE_ENV
  fi
fi
