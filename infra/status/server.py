"""Temporary deployment target until the application is added to this repository."""

import json
import math
import os
import re
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent
SNAPSHOT_PATH = Path('/run/status/runtime.json')
KNOWN_CONTAINERS = {
    'web': ('상태 확인 웹', 's15p31b205-app-status-1'),
    'edge': ('HTTPS 프록시', 's15p31b205-edge-edge-1'),
}
SERVICES = {'docker': 'Docker 엔진', 'gitlab-runner': '자동 배포 Runner'}
EVENT_ACTIONS = {'create', 'start', 'stop', 'die', 'destroy', 'restart', 'pause', 'unpause', 'rename', 'health_status: healthy', 'health_status: unhealthy', 'health_status: starting'}


def safe_name(value):
    return isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,254}', value) is not None


def public_snapshot(snapshot, now=None):
    """Expose a fixed public schema, never raw Docker or host output."""
    now = now or datetime.now(timezone.utc)
    try:
        collected = datetime.fromisoformat(snapshot['collected_at'])
        age = (now - collected).total_seconds()
        fresh = -5 <= age <= 15
    except (KeyError, TypeError, ValueError):
        fresh = False
    containers = []
    inventory_ok = snapshot.get('inventory_ok', isinstance(snapshot.get('containers'), list)) is True
    fresh = fresh and inventory_ok
    for item in snapshot.get('containers', []):
        if not isinstance(item, dict):
            continue
        key = item.get('id', '')
        known = KNOWN_CONTAINERS.get(key)
        name = item.get('name', known[1] if known else '')
        if not safe_name(key) or not safe_name(name):
            continue
        state = item.get('state', 'unknown')
        health = item.get('health', 'unknown')
        image = item.get('image', '')
        image = image if isinstance(image, str) and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:/@-]{0,511}', image) else ''
        container_id = item.get('container_id', '')
        container_id = container_id[:12] if isinstance(container_id, str) and re.fullmatch(r'[a-f0-9]{12,64}', container_id) else ''
        containers.append({
            'id': key, 'label': known[0] if known else name, 'name': name,
            'image': image, 'container_id': container_id,
            'state': state if state in ('running', 'exited', 'created', 'restarting', 'paused', 'dead', 'removing') else 'unknown',
            'health': health if health in ('healthy', 'unhealthy', 'starting', 'none') else 'unknown',
        })
    containers.sort(key=lambda item: (item['state'] != 'running', item['name']))
    events = []
    for event in snapshot.get('events', []):
        if not isinstance(event, dict) or not safe_name(event.get('name')) or event.get('action') not in EVENT_ACTIONS:
            continue
        try:
            at = datetime.fromisoformat(event['at'])
            if at.tzinfo is None:
                continue
        except (KeyError, TypeError, ValueError):
            continue
        events.append({'name': event['name'], 'action': event['action'], 'at': at.isoformat()})
    events = sorted(events, key=lambda event: event['at'], reverse=True)[:20]
    services = []
    raw_services = {item.get('id'): item for item in snapshot.get('services', []) if isinstance(item, dict)}
    for key, label in SERVICES.items():
        state = raw_services.get(key, {}).get('state', 'unknown')
        services.append({'id': key, 'label': label, 'state': state if state in ('active', 'inactive', 'failed', 'activating') else 'unknown'})
    resources = {}
    for key in ('cpu_percent', 'memory_percent', 'disk_percent'):
        value = snapshot.get('resources', {}).get(key)
        resources[key] = round(value, 1) if isinstance(value, (int, float)) and math.isfinite(value) and 0 <= value <= 100 else None
    healthy = fresh and bool(containers) and all(c['state'] == 'running' and c['health'] in ('healthy', 'none') for c in containers) and all(s['state'] == 'active' for s in services)
    running = sum(c['state'] == 'running' for c in containers)
    collected_at = snapshot.get('collected_at') if fresh else None
    uptime = snapshot.get('uptime_seconds')
    return {
        'status': 'ok' if healthy else 'degraded',
        'fresh': fresh,
        'checked_at': now.isoformat(),
        'collected_at': collected_at,
        'revision': os.environ.get('REVISION', 'unknown'),
        'containers': containers, 'services': services, 'resources': resources,
        'summary': {'total': len(containers), 'running': running, 'stopped': len(containers) - running, 'unhealthy': sum(c['health'] == 'unhealthy' for c in containers)},
        'inventory_ok': inventory_ok, 'events': events,
        'events_ok': snapshot.get('events_ok', False) is True,
        'refresh_seconds': 3,
        'uptime_seconds': int(uptime) if isinstance(uptime, (int, float)) and math.isfinite(uptime) and uptime >= 0 else None,
    }


class Handler(BaseHTTPRequestHandler):
    server_version = 'Status'
    sys_version = ''

    def respond(self, body, content_type, status=200):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/api/status':
            try:
                snapshot = json.loads(SNAPSHOT_PATH.read_text(encoding='utf-8'))
                if not isinstance(snapshot, dict):
                    snapshot = {}
                payload = public_snapshot(snapshot)
            except (OSError, ValueError, TypeError, AttributeError):
                payload = public_snapshot({})
            self.respond(json.dumps(payload, ensure_ascii=False).encode(), 'application/json; charset=utf-8')
            return
        if path == '/healthz':
            body = json.dumps({'status': 'ok', 'service': 'S15P31B205 deployment foundation', 'revision': os.environ.get('REVISION', 'unknown')}).encode()
            self.respond(body, 'application/json; charset=utf-8')
            return
        assets = {
            '/': ('index.html', 'text/html; charset=utf-8'),
            '/assets/status.css': ('status.css', 'text/css; charset=utf-8'),
            '/assets/status.js': ('status.js', 'text/javascript; charset=utf-8'),
            '/favicon.svg': ('favicon.svg', 'image/svg+xml'),
        }
        if path not in assets:
            self.respond(b'Not found', 'text/plain; charset=utf-8', 404)
            return
        filename, content_type = assets[path]
        self.respond((ROOT / filename).read_bytes(), content_type)


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", 18066), Handler).serve_forever()
