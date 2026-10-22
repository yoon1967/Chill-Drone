"""Collect Docker inventory and selected events without commands, labels, or logs."""

import json
import os
import shutil
import subprocess
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

OUTPUT = Path('/run/s15p31b205-status/runtime.json')
KNOWN_IDS = {'s15p31b205-app-status-1': 'web', 's15p31b205-edge-edge-1': 'edge'}
LIST_FORMAT = '{"container_id":{{json .ID}},"name":{{json .Names}},"image":{{json .Image}},"state":{{json .State}},"status_text":{{json .Status}}}'
EVENT_FORMAT = '{"container_id":{{json .Actor.ID}},"name":{{json (index .Actor.Attributes "name")}},"action":{{json .Action}},"time_nano":{{.TimeNano}}}'
EVENT_ACTIONS = ('create', 'start', 'stop', 'die', 'destroy', 'restart', 'pause', 'unpause', 'rename', 'health_status')


def docker_inventory():
    try:
        result = subprocess.run(['docker', 'ps', '--all', '--no-trunc', '--format', LIST_FORMAT], capture_output=True, text=True, timeout=5, check=True)
        containers = []
        for line in result.stdout.splitlines():
            item = json.loads(line)
            state_text = item.get('status_text', '')
            health = 'unhealthy' if '(unhealthy)' in state_text else 'healthy' if '(healthy)' in state_text else 'starting' if '(health: starting)' in state_text else 'none'
            containers.append({
                'id': KNOWN_IDS.get(item['name'], item['container_id'][:12]),
                'container_id': item['container_id'][:12], 'name': item['name'],
                'image': item['image'], 'state': item['state'],
                'health': health, 'status_text': state_text,
            })
        return sorted(containers, key=lambda item: (item['state'] != 'running', item['name'])), True
    except (subprocess.SubprocessError, OSError, ValueError, KeyError, TypeError):
        return [], False


def docker_events(previous, until):
    since = previous.get('event_cursor') or until
    command = ['docker', 'events', '--since', since, '--until', until, '--filter', 'type=container', '--format', EVENT_FORMAT]
    for action in EVENT_ACTIONS:
        command.extend(['--filter', f'event={action}'])
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=5, check=True)
        events = list(previous.get('events', []))
        for line in result.stdout.splitlines():
            item = json.loads(line)
            action = item.get('action', '')
            if action.split(':', 1)[0] not in EVENT_ACTIONS:
                continue
            if action.startswith('health_status:') and action not in ('health_status: healthy', 'health_status: unhealthy', 'health_status: starting'):
                continue
            timestamp = int(item['time_nano'])
            container_id = item['container_id'][:12]
            events.append({'id': f'{timestamp}:{container_id}:{action}', 'container_id': container_id, 'name': item['name'], 'action': action, 'at': datetime.fromtimestamp(timestamp / 1_000_000_000, timezone.utc).isoformat()})
        unique = {event['id']: event for event in events}
        return sorted(unique.values(), key=lambda event: event['at'], reverse=True)[:20], until, True
    except (subprocess.SubprocessError, OSError, ValueError, KeyError, TypeError):
        return previous.get('events', [])[:20], since, False


def service_state(name):
    try:
        result = subprocess.run(['systemctl', 'is-active', name], capture_output=True, text=True, timeout=3)
        state = result.stdout.strip()
        return {'id': name, 'state': state if state in ('active', 'inactive', 'failed', 'activating') else 'unknown'}
    except (subprocess.SubprocessError, OSError):
        return {'id': name, 'state': 'unknown'}


def cpu_times():
    values = [int(value) for value in Path('/proc/stat').read_text().splitlines()[0].split()[1:9]]
    return sum(values), values[3] + values[4]


def collect():
    try:
        previous = json.loads(OUTPUT.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        previous = {}
    if not isinstance(previous, dict):
        previous = {}
    until = datetime.now(timezone.utc).isoformat()
    containers, inventory_ok = docker_inventory()
    events, event_cursor, events_ok = docker_events(previous, until)
    first_total, first_idle = cpu_times()
    time.sleep(0.2)
    total, idle = cpu_times()
    cpu = 100 * (1 - (idle - first_idle) / max(total - first_total, 1))
    memory = {line.split(':')[0]: int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()}
    disk = shutil.disk_usage('/')
    return {
        'collected_at': datetime.now(timezone.utc).isoformat(),
        'containers': containers, 'inventory_ok': inventory_ok,
        'events': events, 'event_cursor': event_cursor, 'events_ok': events_ok,
        'services': [service_state(name) for name in ('docker', 'gitlab-runner')],
        'resources': {'cpu_percent': round(cpu, 1), 'memory_percent': round(100 * (1 - memory['MemAvailable'] / memory['MemTotal']), 1), 'disk_percent': round(100 * disk.used / disk.total, 1)},
        'uptime_seconds': int(float(Path('/proc/uptime').read_text().split()[0])),
    }


if __name__ == '__main__':
    OUTPUT.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
    payload = json.dumps(collect(), ensure_ascii=False)
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=OUTPUT.parent, delete=False) as file:
        file.write(payload)
        temporary = file.name
    os.chmod(temporary, 0o644)
    os.replace(temporary, OUTPUT)
