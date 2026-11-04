"""One reusable depth process, with a presentation barrier for every output frame."""
import multiprocessing
import signal
import threading


class DepthModelCache:
    def __init__(self):
        self.estimator = None

    def get(self):
        if self.estimator is None:
            from modules.mapping.data.stream_mapping import DepthEstimator
            estimator = DepthEstimator()
            estimator.prepare()
            self.estimator = estimator
        return self.estimator


def _serve(connection, cancelled):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    cache = DepthModelCache()
    try:
        while True:
            request = connection.recv()
            if request[0] == 'shutdown':
                return
            _, source, output, options = request
            try:
                from modules.mapping.data.stream_mapping import run
                def published(snapshot):
                    connection.send(('preview', snapshot))
                    # No next frame until the parent finishes displaying this one.
                    if connection.recv() != 'continue':
                        raise InterruptedError('cancelled')
                def receive():
                    connection.send(('input', None))
                    return connection.recv()
                result = run(source, output, options, estimator_factory=cache.get,
                    stop_event=cancelled, on_preview=published,
                    live_receive=receive if request[0] == 'live' else None)
                connection.send(('complete', result))
            except Exception as error:
                connection.send(('error', str(error), isinstance(error, InterruptedError)))
    except (EOFError, BrokenPipeError):
        pass
    finally:
        connection.close()


class DepthWorker:
    def __init__(self):
        self.context = multiprocessing.get_context('spawn')
        self.process = self.connection = self.cancelled = None
        self.lock = threading.Lock()

    def run(self, source, output, options, stop_event, on_preview, next_frame=None):
        with self.lock:
            if self.process is None or not self.process.is_alive():
                self.close()
                self.connection, child = self.context.Pipe()
                self.cancelled = self.context.Event()
                self.process = self.context.Process(target=_serve, args=(child, self.cancelled), daemon=True)
                self.process.start()
                child.close()
            self.cancelled.clear()
            if stop_event.is_set():
                raise InterruptedError('cancelled')
            self.connection.send(('live' if next_frame is not None else 'run', str(source), str(output), options))
            while True:
                if stop_event.is_set():
                    self.cancelled.set()
                if not self.connection.poll(.1):
                    if not self.process.is_alive():
                        raise RuntimeError('depth_worker_stopped')
                    continue
                message = self.connection.recv()
                if message[0] == 'input':
                    try:
                        self.connection.send(next_frame())
                    except BaseException:
                        self.cancelled.set()
                        self.close()
                        raise
                elif message[0] == 'preview':
                    try:
                        on_preview(message[1])
                    except BaseException:
                        self.cancelled.set()
                        self.connection.send('cancel')
                        self.close()
                        raise
                    if stop_event.is_set():
                        self.cancelled.set()
                    self.connection.send('continue')
                elif message[0] == 'complete':
                    return message[1]
                else:
                    raise (InterruptedError if message[2] else RuntimeError)(message[1])

    def close(self):
        if self.process is not None:
            if self.cancelled is not None:
                self.cancelled.set()
            if self.process.is_alive():
                try:
                    self.connection.send(('shutdown',))
                except (OSError, EOFError):
                    pass
                self.process.join(timeout=10)
                if self.process.is_alive():
                    self.process.terminate()
                    self.process.join(timeout=5)
            self.process.close()
            self.process = None
        if self.connection is not None:
            self.connection.close()
            self.connection = None
