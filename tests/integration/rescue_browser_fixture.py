"""Temporary synthetic rescue map served by the actual console for browser tests."""
from pathlib import Path
import json
import socket
import subprocess
import sys
import tempfile
import time
from urllib.request import urlopen
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from modules.mapping.data.stream_mapping import run


def create_result(output):
    rgb = np.full((120, 160, 3), [120, 145, 150], np.uint8)
    class Depth:
        def estimate(self, image):
            depth = np.full(image.shape[:2], 2., np.float32)
            h, w = depth.shape; depth[h//4:3*h//4, 2*w//5:3*w//5] = 1.6
            return depth
    class Semantics:
        def estimate(self, image, frame_id, stamp):
            labels = np.full(image.shape[:2], 2, np.uint8)
            h, w = labels.shape; labels[h//4:3*h//4, 2*w//5:3*w//5] = 1
            return dict(frame_id=frame_id, timestamp_s=stamp, labels=labels,
                confidence=np.ones(labels.shape, np.float32), inference_ms=1.,
                people=[dict(mask=labels == 1, confidence=.95)])
    class Tracker:
        def __init__(self, *args): pass
        def track(self, image, stamp):
            h, w = image.shape[:2]
            return dict(timestamp_s=stamp, tracking_state=2, map_id=0, T_cw=np.eye(4).tolist(),
                depth_anchors=[[w*x, h*y, 3.] for x in (.1,.2,.7,.8) for y in (.3,.6)])
        def close(self): pass
    frames = [(i+1, i*.2, rgb, (160, 120)) for i in range(5)]
    with patch('modules.mapping.data.stream_mapping.frames', return_value=iter(frames)):
        run('unused', output, dict(frame_fps=5, sample_fps=5, semantic_mapping=True, depth_resolution=256),
            estimator_factory=Depth, tracker_factory=Tracker, semantic_factory=Semantics)


def main():
    with tempfile.TemporaryDirectory(prefix='rescue-browser-') as temporary:
        folder = Path(temporary)
        result = folder / 'result'; create_result(result)
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0)); port = listener.getsockname()[1]
        server = subprocess.Popen([sys.executable, '-B', str(ROOT/'apps/console/serve.py'),
            '--result', str(result), '--import-source', str(result), '--live-root', str(folder/'imports'),
            '--profiles-root', str(folder/'profiles'), '--port', str(port)],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            url = f'http://127.0.0.1:{port}'
            for _ in range(80):
                try:
                    with urlopen(url+'/api/import/library', timeout=1) as response:
                        library = json.load(response)
                    break
                except OSError:
                    if server.poll() is not None: raise RuntimeError('fixture_server_failed')
                    time.sleep(.1)
            else: raise RuntimeError('fixture_server_timeout')
            print(json.dumps(dict(url=url, library=library)), flush=True)
            sys.stdin.readline()
        finally:
            server.terminate(); server.wait(timeout=15)


if __name__ == '__main__': main()
