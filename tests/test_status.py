import importlib.util
import json
import threading
import unittest
import urllib.error
import urllib.request
from datetime import datetime, timedelta, timezone
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("status_server", Path(__file__).resolve().parents[1] / "infra/status/server.py")
server_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server_module)


class StatusTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), server_module.Handler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = f"http://127.0.0.1:{cls.server.server_port}"

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def test_health_reports_deployed_revision_without_caching(self):
        with patch.dict("os.environ", {"REVISION": "test-revision"}):
            with urllib.request.urlopen(self.url + "/healthz") as response:
                self.assertEqual(response.status, 200)
                self.assertEqual(response.headers["Cache-Control"], "no-store")
                self.assertEqual(json.load(response)["revision"], "test-revision")

    def test_unknown_route_does_not_pass_health_check(self):
        with self.assertRaises(urllib.error.HTTPError) as raised:
            urllib.request.urlopen(self.url + "/missing")
        self.assertEqual(raised.exception.code, 404)
        raised.exception.close()

    def test_home_serves_dashboard_and_same_origin_assets(self):
        with urllib.request.urlopen(self.url + '/') as response:
            self.assertEqual(response.headers.get_content_type(), 'text/html')
            self.assertIn("script-src 'self'", response.headers['Content-Security-Policy'])
            self.assertIn('서버의 지금을 한눈에.', response.read().decode())
        for path, content_type in (('/assets/status.css', 'text/css'), ('/assets/status.js', 'text/javascript')):
            with urllib.request.urlopen(self.url + path) as response:
                self.assertEqual(response.headers.get_content_type(), content_type)

    def test_static_routes_do_not_expose_source_files(self):
        for path in ('/server.py', '/.env', '/assets/../server.py'):
            with self.assertRaises(urllib.error.HTTPError) as raised:
                urllib.request.urlopen(self.url + path)
            self.assertEqual(raised.exception.code, 404)
            raised.exception.close()

    def snapshot(self):
        return {
            'collected_at': datetime.now(timezone.utc).isoformat(),
            'containers': [{'id': 'web', 'state': 'running', 'health': 'healthy'}, {'id': 'edge', 'state': 'running', 'health': 'none'}],
            'services': [{'id': 'docker', 'state': 'active'}, {'id': 'gitlab-runner', 'state': 'active'}],
            'resources': {'cpu_percent': 3.2, 'memory_percent': 9.1, 'disk_percent': 2.1},
            'uptime_seconds': 3600,
        }

    def test_public_snapshot_keeps_only_allowlisted_information(self):
        snapshot = self.snapshot()
        snapshot.update({'environment': {'TEST_PRIVATE_VALUE': 'not-public'}, 'hostname': 'private-hostname'})
        snapshot['containers'].append({'id': 'worker1', 'name': 'worker-1', 'image': 'alpine:latest', 'state': 'running', 'health': 'none', 'environment': {'TEST_PRIVATE_VALUE': 'not-public'}})
        result = server_module.public_snapshot(snapshot)
        self.assertEqual(result['status'], 'ok')
        self.assertEqual(len(result['containers']), 3)
        self.assertEqual(result['summary']['running'], 3)
        self.assertTrue(any(item['name'] == 'worker-1' for item in result['containers']))
        rendered = json.dumps(result)
        for private_value in ('not-public', 'private-hostname', 'environment'):
            self.assertNotIn(private_value, rendered)

    def test_stale_snapshot_is_never_reported_healthy(self):
        snapshot = self.snapshot()
        snapshot['collected_at'] = (datetime.now(timezone.utc) - timedelta(seconds=16)).isoformat()
        result = server_module.public_snapshot(snapshot)
        self.assertEqual(result['status'], 'degraded')
        self.assertFalse(result['fresh'])

    def test_inventory_changes_update_dynamic_counts(self):
        snapshot = self.snapshot()
        self.assertEqual(server_module.public_snapshot(snapshot)['summary']['total'], 2)
        snapshot['containers'].append({'id': 'new-worker', 'name': 'new-worker', 'image': 'redis:latest', 'state': 'exited', 'health': 'none'})
        result = server_module.public_snapshot(snapshot)
        self.assertEqual(result['summary'], {'total': 3, 'running': 2, 'stopped': 1, 'unhealthy': 0})
        self.assertEqual(result['status'], 'degraded')
        snapshot['containers'].pop()
        self.assertEqual(server_module.public_snapshot(snapshot)['summary']['total'], 2)

    def test_failed_inventory_and_empty_inventory_are_not_healthy(self):
        snapshot = self.snapshot()
        snapshot['inventory_ok'] = False
        self.assertFalse(server_module.public_snapshot(snapshot)['fresh'])
        snapshot['inventory_ok'] = True
        snapshot['containers'] = []
        self.assertEqual(server_module.public_snapshot(snapshot)['status'], 'degraded')

    def test_events_exclude_execution_commands_and_other_attributes(self):
        snapshot = self.snapshot()
        at = datetime.now(timezone.utc).isoformat()
        snapshot['events'] = [
            {'name': 'worker-1', 'action': 'create', 'at': at, 'attributes': {'secret': 'not-public'}},
            {'name': 'worker-1', 'action': 'exec_create: sensitive-command', 'at': at},
        ]
        events = server_module.public_snapshot(snapshot)['events']
        self.assertEqual(events, [{'name': 'worker-1', 'action': 'create', 'at': at}])

    def test_stopped_container_and_missing_snapshot_are_degraded(self):
        snapshot = self.snapshot()
        snapshot['containers'][0]['state'] = 'exited'
        self.assertEqual(server_module.public_snapshot(snapshot)['status'], 'degraded')
        with patch.object(server_module, 'SNAPSHOT_PATH', Path('/not-present/status-test.json')):
            with urllib.request.urlopen(self.url + '/api/status') as response:
                result = json.load(response)
                self.assertEqual(result['status'], 'degraded')
                self.assertFalse(result['fresh'])


if __name__ == "__main__":
    unittest.main()
