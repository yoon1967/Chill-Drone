"""Cover the saved-map Roomform cancellation endpoint without model dependencies."""
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from threading import Event, Thread
from time import monotonic, sleep
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from urllib.parse import urlencode
from urllib.request import Request, urlopen
from uuid import uuid4

from apps.console.roomform_jobs import RoomformJobs
from apps.console.serve import ViewerHandler


class ConsoleRoomformTests(unittest.TestCase):
    def setUp(self):
        root = Path(self.enterContext(TemporaryDirectory()))
        (root / 'scene.json').write_text('{}', encoding='utf-8')
        (root / 'status.json').write_text('{"status":"complete"}', encoding='utf-8')
        self.release_tile = Event()
        self.tile_started = Event()
        self.after_tile = Mock()
        self.infer = Mock(side_effect=self.reconstruct)
        self.enterContext(patch.dict(sys.modules, {
            'modules.mapping.data.roomform_surface': SimpleNamespace(reconstruct_roomform=self.infer),
        }))
        self.enterContext(patch('apps.console.roomform_jobs.find_spec', return_value=True))
        models = SimpleNamespace(roomform_assets=lambda: (root / 'model.pt', None))
        self.jobs = RoomformJobs(models)
        handler = partial(ViewerHandler, result=root, roomform=self.jobs)
        server = self.enterContext(ThreadingHTTPServer(('127.0.0.1', 0), handler))
        self.endpoint = f'http://127.0.0.1:{server.server_port}/api/saved/roomform'
        Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.shutdown)
        self.addCleanup(self.jobs.shutdown)
        self.addCleanup(self.release_tile.set)

    def reconstruct(self, scene, checkpoint, progress):
        progress(0, 2)
        self.tile_started.set()
        if not self.release_tile.wait(5):
            raise TimeoutError('Test did not release the active tile')
        progress(1, 2)
        self.after_tile()
        progress(2, 2)
        return {'surfaces': {}}

    def request(self, method, **query):
        req = Request(self.endpoint + '?' + urlencode(query), method=method, headers={'X-Mapping-Client': '1'})
        with urlopen(req, timeout=5) as response:
            return json.load(response)

    def test_cancel_arriving_before_start_prevents_inference(self):
        key = uuid4().hex
        self.assertEqual(self.request('POST', id=key, action='cancel')['status'], 'cancelled')
        self.assertEqual(self.request('POST', id=key)['status'], 'cancelled')
        self.infer.assert_not_called()

    def test_running_job_stops_at_checkpoint_and_same_map_can_restart(self):
        key = uuid4().hex
        self.assertEqual(self.request('POST', id=key)['status'], 'running')
        self.assertTrue(self.tile_started.wait(3))
        self.assertEqual(self.request('POST', id=key)['id'], key)

        with ThreadPoolExecutor(max_workers=1) as client:
            cancel = client.submit(self.request, 'POST', id=key, action='cancel')
            try:
                deadline = monotonic() + 3
                while self.request('GET', id=key)['status'] == 'running' and monotonic() < deadline:
                    sleep(.01)
                self.assertEqual(self.request('GET', id=key)['status'], 'cancelling')
                self.assertFalse(cancel.done())
            finally:
                self.release_tile.set()
            self.assertEqual(cancel.result(timeout=3)['status'], 'cancelled')

        self.after_tile.assert_not_called()
        cancelled = self.request('GET', id=key)
        self.assertEqual(cancelled['status'], 'cancelled')
        self.assertNotIn('result', cancelled)

        retry = uuid4().hex
        self.assertEqual(self.request('POST', id=retry)['id'], retry)
        self.jobs.worker.join(timeout=3)
        self.assertEqual(self.request('GET', id=retry)['status'], 'complete')
        self.assertEqual(self.request('POST', id=retry, action='cancel')['status'], 'complete')
        self.assertEqual(self.request('POST', id=uuid4().hex)['id'], retry)
        self.after_tile.assert_called_once()
