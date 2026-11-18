import importlib.util
import json
import subprocess
import unittest
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('snapshot_collector', Path(__file__).resolve().parents[1] / 'infra/status-snapshot.py')
collector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(collector)


class SnapshotTests(unittest.TestCase):
    def test_inventory_discovers_new_names_and_stopped_containers(self):
        rows = [
            {'container_id': 'a' * 64, 'name': 'new-database', 'image': 'postgres:17', 'state': 'running', 'status_text': 'Up 2 minutes (unhealthy)'},
            {'container_id': 'b' * 64, 'name': 'old-worker', 'image': 'alpine:latest', 'state': 'exited', 'status_text': 'Exited (0) 1 minute ago'},
        ]
        with patch.object(collector.subprocess, 'run', return_value=SimpleNamespace(stdout='\n'.join(json.dumps(row) for row in rows))) as run:
            inventory, okay = collector.docker_inventory()
        self.assertTrue(okay)
        self.assertEqual(len(inventory), 2)
        self.assertEqual(inventory[0]['health'], 'unhealthy')
        self.assertEqual(inventory[1]['state'], 'exited')
        command = run.call_args.args[0]
        self.assertIn('--all', command)
        template = command[-1]
        for excluded in ('.Command', '.Labels', '.Mounts', '{{json .}}'):
            self.assertNotIn(excluded, template)

    def test_docker_failure_is_distinct_from_empty_inventory(self):
        with patch.object(collector.subprocess, 'run', side_effect=subprocess.CalledProcessError(1, 'docker')):
            self.assertEqual(collector.docker_inventory(), ([], False))
        with patch.object(collector.subprocess, 'run', return_value=SimpleNamespace(stdout='')):
            self.assertEqual(collector.docker_inventory(), ([], True))

    def test_events_are_filtered_deduplicated_and_cursor_is_preserved_on_failure(self):
        until = datetime.now(timezone.utc).isoformat()
        event = {'container_id': 'a' * 64, 'name': 'new-worker', 'action': 'create', 'time_nano': 1792635000000000000}
        unsafe_event = {**event, 'action': 'exec_create: sensitive-command'}
        output = '\n'.join(json.dumps(row) for row in (event, event, unsafe_event))
        with patch.object(collector.subprocess, 'run', return_value=SimpleNamespace(stdout=output)) as run:
            events, cursor, okay = collector.docker_events({'event_cursor': 'previous'}, until)
        self.assertTrue(okay)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['action'], 'create')
        self.assertEqual(cursor, until)
        self.assertNotIn('event=exec_create', run.call_args.args[0])
        with patch.object(collector.subprocess, 'run', side_effect=OSError('unavailable')):
            preserved, cursor, okay = collector.docker_events({'events': events, 'event_cursor': 'previous'}, until)
        self.assertFalse(okay)
        self.assertEqual(cursor, 'previous')
        self.assertEqual(preserved, events)
