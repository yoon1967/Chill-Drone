"""Run one local Roomform job at a time and retain recent results for this console."""
from collections import OrderedDict
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
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.worker = None

    def start(self, path):
        version = scene_version(path)
        with self.lock:
            assets = self.models.roomform_assets()
            if assets is None:
                raise RoomformError('roomform_checkpoint_missing')
            for key, (source, job) in self.jobs.items():
                if source == version and job['status'] != 'failed':
                    self.jobs.move_to_end(key)
                    return {k: v for k, v in job.items() if k != 'result'}
            if self.stopping.is_set() or self.worker and self.worker.is_alive():
                raise RoomformError('roomform_busy')
            if any(find_spec(package) is None for package in ('numpy', 'torch', 'scipy', 'pydantic')):
                raise RoomformError('roomform_runtime_missing')
            key = uuid4().hex
            job = {'id': key, 'status': 'running', 'tiles_done': 0, 'tiles_total': 0}
            self.jobs[key] = (version, job)
            if len(self.jobs) > 8:
                self.jobs.popitem(last=False)
            self.worker = threading.Thread(target=self._run, args=(version, job, *assets), daemon=True, name='roomform')
            self.worker.start()
            return dict(job)

    def status(self, key):
        with self.lock:
            if key not in self.jobs:
                raise RoomformError('roomform_job_missing')
            self.jobs.move_to_end(key)
            return dict(self.jobs[key][1])

    def _run(self, version, job, checkpoint, runtime):
        path = version[0]

        def progress(done, total):
            if self.stopping.is_set():
                raise RoomformError('roomform_stopped')
            with self.lock:
                job.update(tiles_done=done, tiles_total=total)

        try:
            from modules.mapping.data.roomform_surface import reconstruct_roomform

            scene = json.loads(path.read_text(encoding='utf-8'))
            if scene_version(path) != version:
                raise RoomformError('roomform_scene_changed')
            if runtime is not None and str(runtime) not in sys.path:
                sys.path.insert(0, str(runtime))
            result = reconstruct_roomform(scene, checkpoint, progress)
        except RoomformError as error:
            with self.lock:
                job.update(status='failed', error=str(error))
        except ImportError:
            logging.exception('Local Roomform runtime could not be imported')
            with self.lock:
                job.update(status='failed', error='roomform_runtime_missing')
        except Exception as error:
            logging.exception('Local Roomform inference failed')
            reason = 'roomform_out_of_memory' if 'out of memory' in str(error).lower() else 'roomform_inference_failed'
            with self.lock:
                job.update(status='failed', error=reason)
        else:
            with self.lock:
                job.update(status='complete', result=result)

    def shutdown(self):
        self.stopping.set()
        if self.worker is not None:
            self.worker.join(timeout=15)
