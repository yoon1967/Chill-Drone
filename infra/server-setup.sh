#!/usr/bin/env bash
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'Run with sudo.' >&2; exit 1; }
. /etc/os-release
[[ $ID == ubuntu && $VERSION_ID == 24.04 ]] || { echo 'This setup targets Ubuntu 24.04.' >&2; exit 1; }
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y ca-certificates curl gnupg git jq
install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc
printf 'deb [arch=%s signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu %s stable\n' "$(dpkg --print-architecture)" "$VERSION_CODENAME" >/etc/apt/sources.list.d/docker.list
curl -fsSL https://packages.gitlab.com/runner/gitlab-runner/gpgkey | gpg --dearmor --yes -o /etc/apt/keyrings/gitlab-runner.gpg
printf 'deb [signed-by=/etc/apt/keyrings/gitlab-runner.gpg] https://packages.gitlab.com/runner/gitlab-runner/ubuntu/ noble main\n' >/etc/apt/sources.list.d/gitlab-runner.list
apt-get update -qq
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin gitlab-runner
install -m 0755 "$(dirname "$0")/docker-firewall.sh" /usr/local/sbin/s15p31b205-docker-firewall
install -d -m 0755 /etc/systemd/system/docker.service.d
cat >/etc/systemd/system/docker.service.d/20-private-ports.conf <<'UNIT'
[Service]
ExecStartPost=/usr/local/sbin/s15p31b205-docker-firewall
UNIT
systemctl daemon-reload
systemctl enable --now docker gitlab-runner
/usr/local/sbin/s15p31b205-docker-firewall
ufw allow 22/tcp
ufw allow 443/tcp
ufw --force enable
usermod -aG docker gitlab-runner
install -d -o gitlab-runner -g gitlab-runner -m 0750 /srv/s15p31b205
install -d -m 0750 /srv/s15p31b205-edge
install -m 0644 "$(dirname "$0")/Caddyfile" /srv/s15p31b205-edge/Caddyfile
install -m 0644 "$(dirname "$0")/edge.compose.yml" /srv/s15p31b205-edge/compose.yml
systemctl restart gitlab-runner
bash "$(dirname "$0")/install-status-snapshot.sh"
docker compose -p s15p31b205-edge -f /srv/s15p31b205-edge/compose.yml up -d
docker version --format '{{.Server.Version}}'
docker compose version
ufw status verbose
