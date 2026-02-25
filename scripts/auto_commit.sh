#!/usr/bin/env bash
set -euo pipefail

# Auto-commit loop:
# - checks tracked/untracked changes every N seconds
# - commits only when there are real changes
# - pushes to current branch if remote exists

INTERVAL_SECONDS="${1:-10}"

while true; do
  if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "[auto-commit] not in a git repo, retrying..."
    sleep "${INTERVAL_SECONDS}"
    continue
  fi

  if [[ -n "$(git status --porcelain)" ]]; then
    ts="$(date +"%Y-%m-%d %H:%M:%S")"
    msg="auto: snapshot ${ts}"

    git add -A
    # Commit may fail if only ignored/empty changes remain after add.
    git commit -m "${msg}" >/dev/null 2>&1 || true

    # Push only if upstream exists.
    if git rev-parse --abbrev-ref --symbolic-full-name '@{u}' >/dev/null 2>&1; then
      git push >/dev/null 2>&1 || true
    fi

    echo "[auto-commit] ${msg}"
  fi

  sleep "${INTERVAL_SECONDS}"
done

