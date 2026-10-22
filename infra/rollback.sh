#!/usr/bin/env bash
set -euo pipefail
base=${DEPLOY_ROOT:-/srv/s15p31b205}
cd "$base"
exec 9>"$base/deploy.lock"
flock -w 180 9
target=$(readlink -f "$base/previous" || true)
current=$(readlink -f "$base/current" || true)
[[ -n "$target" && -f "$target/compose.yml" ]] || { echo 'No previous healthy release.' >&2; exit 1; }
if ! docker compose --project-name s15p31b205-app --env-file "$target/release.env" -f "$target/compose.yml" up -d --wait --wait-timeout 60; then
    if [[ -n "$current" && -f "$current/compose.yml" ]]; then
        docker compose --project-name s15p31b205-app --env-file "$current/release.env" -f "$current/compose.yml" up -d --wait --wait-timeout 60
    fi
    exit 1
fi
ln -sfn "$target" "$base/current.next"
mv -Tf "$base/current.next" "$base/current"
if [[ -n "$current" ]]; then
    ln -sfn "$current" "$base/previous.next"
    mv -Tf "$base/previous.next" "$base/previous"
fi
echo "Rollback healthy: $(basename "$target")"
