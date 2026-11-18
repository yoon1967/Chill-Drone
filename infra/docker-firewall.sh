#!/usr/bin/env bash
set -euo pipefail
# Caddy uses host networking and is filtered by UFW. All bridge-published
# container ports remain private even if a future compose file binds 0.0.0.0.
external=$(ip -4 route show default | awk 'NR==1 {print $5}')
[[ -n "$external" ]] || { echo 'Default interface not found' >&2; exit 1; }
for firewall in iptables ip6tables; do
    if "$firewall" -w -nL DOCKER-USER >/dev/null 2>&1; then
        "$firewall" -w -N S15P31B205-PRIVATE 2>/dev/null || true
        "$firewall" -w -F S15P31B205-PRIVATE
        "$firewall" -w -A S15P31B205-PRIVATE -i "$external" -m conntrack --ctstate ESTABLISHED,RELATED -j RETURN
        "$firewall" -w -A S15P31B205-PRIVATE -i "$external" -j DROP
        "$firewall" -w -A S15P31B205-PRIVATE -j RETURN
        "$firewall" -w -C DOCKER-USER -j S15P31B205-PRIVATE 2>/dev/null || "$firewall" -w -I DOCKER-USER 1 -j S15P31B205-PRIVATE
    fi
done
