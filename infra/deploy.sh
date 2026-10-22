#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
base=${DEPLOY_ROOT:-/srv/s15p31b205}
image=${1:?Usage: deploy.sh IMAGE REVISION}
revision=${2:?Revision is required}
[[ $revision =~ ^[a-zA-Z0-9][a-zA-Z0-9._-]{0,79}$ ]] || { echo 'Invalid revision' >&2; exit 2; }
[[ $image =~ ^[a-zA-Z0-9][a-zA-Z0-9._/:@-]+$ ]] || { echo 'Invalid image' >&2; exit 2; }
mkdir -p "$base/releases"
cd "$base"
exec 9>"$base/deploy.lock"
flock -w 180 9
release="$base/releases/$revision"
old=$(readlink -f "$base/current" || true)
if [[ "$old" == "$release" ]]; then
    docker compose --project-name s15p31b205-app --env-file "$old/release.env" -f "$old/compose.yml" up -d --wait --wait-timeout 60
    curl --fail --silent --show-error --max-time 10 "${HEALTH_URL:-https://k15b205.p.ssafy.io/healthz}"
    exit 0
fi
mkdir -p "$release"
cp "$script_dir/app.compose.yml" "$release/compose.yml"
printf 'APP_IMAGE=%s\n' "$image" >"$release/release.env"
chmod 600 "$release/release.env"
if docker compose --project-name s15p31b205-app --env-file "$release/release.env" -f "$release/compose.yml" up -d --wait --wait-timeout 60 &&
    curl --fail --silent --show-error --retry 5 --retry-all-errors --retry-delay 2 --max-time 10 "${HEALTH_URL:-https://k15b205.p.ssafy.io/healthz}"; then
    if [[ -n "$old" && -d "$old" ]]; then
        ln -sfn "$old" "$base/previous.next"
        mv -Tf "$base/previous.next" "$base/previous"
    fi
    ln -sfn "$release" "$base/current.next"
    mv -Tf "$base/current.next" "$base/current"
    echo "Deployment healthy: $revision"
else
    echo 'Deployment failed; restoring the previous healthy release.' >&2
    if [[ -n "$old" && -f "$old/compose.yml" ]]; then
        docker compose --project-name s15p31b205-app --env-file "$old/release.env" -f "$old/compose.yml" up -d --wait --wait-timeout 60
    else
        docker compose --project-name s15p31b205-app --env-file "$release/release.env" -f "$release/compose.yml" down
    fi
    exit 1
fi
