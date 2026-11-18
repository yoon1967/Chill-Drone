#!/usr/bin/env bash
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'Run with sudo.' >&2; exit 1; }
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
install -d -m 0755 /usr/local/lib/s15p31b205
install -m 0644 "$script_dir/status-snapshot.py" /usr/local/lib/s15p31b205/status-snapshot.py
install -m 0644 "$script_dir/status-snapshot.service" /etc/systemd/system/s15p31b205-status-snapshot.service
install -m 0644 "$script_dir/status-snapshot.timer" /etc/systemd/system/s15p31b205-status-snapshot.timer
install -m 0644 "$script_dir/status-snapshot.tmpfiles.conf" /etc/tmpfiles.d/s15p31b205-status.conf
systemd-tmpfiles --create /etc/tmpfiles.d/s15p31b205-status.conf
systemctl daemon-reload
systemctl start s15p31b205-status-snapshot.service
systemctl enable s15p31b205-status-snapshot.timer
systemctl restart s15p31b205-status-snapshot.timer
