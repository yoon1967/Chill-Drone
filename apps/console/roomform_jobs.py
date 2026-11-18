"""Run one local Roomform job at a time and retain recent results for this console."""
from collections import OrderedDict, deque
from importlib.util import find_spec
import json
import logging
import sys
import threading
from uuid import uuid4

from modules.mapping.data.roomform_errors import RoomformError


def scene_version(path):
    stat = path.stat()
    return path, stat.st_mtime_ns, stat.st_size


class RoomformJobs:
    def __init__(self, models):
        self.models = models
        self.jobs = OrderedDict()
        self.cancelled = deque(maxlen=32)
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.worker = None

    def start(self, path, key=None):
        version = scene_version(path)
        with self.lock:
            if key in self.cancelled:
                return {'id': key, 'status': 'cancelled'}
            if key in self.jobs:
                source, job = self.jobs[key]
                if source != version:
                    raise RoomformError('roomform_scene_changed')
                self.jobs.move_to_end(key)
                return {k: v for k, v in job.items() if k != 'result'}
            assets = self.models.roomform_assets()
            if assets is None:
                raise RoomformError('roomform_checkpoint_missing')
            for source, job in self.jobs.values():
                reusable = job['status'] == 'complete' or (key is None and job['status'] == 'running')
                if source == version and reusable:
                    self.jobs.move_to_end(job['id'])
                    return {k: v for k, v in job.items() if k != 'result'}
            if self.stopping.is_set() or self.worker and self.worker.is_alive():
                raise RoomformError('roomform_busy')
            if any(find_spec(package) is None for package in ('numpy', 'torch', 'scipy', 'pydantic')):
                raise RoomformError('roomform_runtime_missing')
            key = key or uuid4().hex
            job = {'id': key, 'status': 'running', 'tiles_done': 0, 'tiles_total': 0}
            self.jobs[key] = (version, job)
            if len(self.jobs) > 8:
                self.jobs.popitem(last=False)
            self.worker = threading.Thread(target=self._run, args=(version, job, *assets), daemon=True, name='roomform')
            self.worker.start()
            return dict(job)

    def cancel(self, key):
        with self.lock:
            entry = self.jobs.get(key)
            if entry is None:
                # The cancel request can reach the server before the matching start.
                if key not in self.cancelled:
                    self.cancelled.append(key)
                return {'id': key, 'status': 'cancelled'}
            _, job = entry
            if job['status'] not in {'running', 'cancelling'}:
                return {'id': key, 'status': job['status']}
            job['status'] = 'cancelling'
            if key not in self.cancelled:
                self.cancelled.append(key)
            worker = self.worker

        # Release the lock so the worker can reach a cancellation checkpoint and exit.
        worker.join()
        return {'id': key, 'status': 'cancelled'}

    def status(self, key):
        with self.lock:
            if key not in self.jobs:
                raise RoomformError('roomform_job_missing')
            self.jobs.move_to_end(key)
            return dict(self.jobs[key][1])

    def _run(self, version, job, checkpoint, runtime):
        path = version[0]

        def progress(done, total):
            with self.lock:
                if job['status'] == 'cancelling' or self.stopping.is_set():
                    raise RoomformError('roomform_stopped')
                job.update(tiles_done=done, tiles_total=total)

        try:
            progress(0, 0)
            from modules.mapping.data.roomform_surface import reconstruct_roomform

            scene = json.loads(path.read_text(encoding='utf-8'))
            if scene_version(path) != version:
                raise RoomformError('roomform_scene_changed')
            if runtime is not None and str(runtime) not in sys.path:
                sys.path.insert(0, str(runtime))
            result = reconstruct_roomform(scene, checkpoint, progress)
        except RoomformError as error:
            outcome = {'status': 'failed', 'error': str(error)}
        except ImportError:
            logging.exception('Local Roomform runtime could not be imported')
            outcome = {'status': 'failed', 'error': 'roomform_runtime_missing'}
        except Exception as error:
            logging.exception('Local Roomform inference failed')
            reason = 'roomform_out_of_memory' if 'out of memory' in str(error).lower() else 'roomform_inference_failed'
            outcome = {'status': 'failed', 'error': reason}
        else:
            outcome = {'status': 'complete', 'result': result}
        with self.lock:
            if job['status'] == 'cancelling':
                job['status'] = 'cancelled'
            else:
                job.update(outcome)

    def shutdown(self):
        self.stopping.set()
        if self.worker is not None:
            self.worker.join(timeout=15)
