"""Compare saturated point budgets using recorded RGB-D and the production renderer."""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from urllib.parse import urlsplit, parse_qs

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from modules.mapping.data.point_fusion import SemanticSurface
from modules.mapping.data.rescue_mapping import unproject, person_points


def stats(values):
    a = np.asarray(values, float)
    return dict(n=len(a), mean=float(a.mean()), p50=float(np.percentile(a, 50)),
                p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--history', required=True, type=Path)
    parser.add_argument('--budgets', default='8000,16000,24000,32000,48000,64000,96000,128000,192000,256000')
    parser.add_argument('--playwright-module', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--stress-frames', type=int, default=80)
    parser.add_argument('--backend-only', action='store_true')
    parser.add_argument('--browser-only', action='store_true')
    args = parser.parse_args()
    budgets = sorted(set(map(int, args.budgets.split(','))))
    if args.output.exists() or min(budgets) < 1000 or max(budgets) > 512000:
        parser.error('new output file and point budgets in [1000,512000] required')
    entries = [json.loads(line) for line in (args.history/'history/index.jsonl').read_text().splitlines()]
    entries = [entry for entry in entries if entry['observation']]
    selected = [entries[i] for i in np.linspace(0, len(entries)-1, min(24, len(entries))).astype(int)]
    rows, clouds, classes, people = [], [], [], []
    for entry in selected:
        with np.load(args.history/'history/frames'/f"{entry['revision']:08d}.npz", allow_pickle=False) as source:
            row = {key: source[key].copy() for key in ('rgb','depth','world_from_camera','intrinsics','semantic_labels','semantic_confidence','depth_scale')}
            rows.append(row)
            depth = np.where(row['semantic_labels'] == 1, 0, row['depth'])
            xyz, v, u = unproject(depth, row['world_from_camera'], row['intrinsics'], stride=1)
            clouds.append(np.column_stack((xyz, row['rgb'][v,u])))
            classes.append(row['semantic_labels'][v,u])
            rescue = json.loads(source['rescue_json'].tobytes())
            if sum(len(p['voxels']) for p in rescue['people']) > sum(len(p['voxels']) for p in people):
                people = rescue['people']
    pool = np.concatenate(clouds); labels = np.concatenate(classes)
    if len(pool) < max(budgets):
        raise ValueError('not_enough_observed_points_for_requested_budgets')
    samples = {}
    for budget in budgets:
        selected = np.linspace(0, len(pool)-1, budget).astype(int)
        points = pool[selected]; points[:,:3] = np.round(points[:,:3],5)
        wire_points = points.tolist()
        for point in wire_points:
            point[3:] = [int(c) for c in point[3:]]
        human_points = person_points(dict(people=people, person_color='#ff3030'))
        snapshot = dict(points=wire_points+human_points, cameras=[[0.,0.,0.]], rescue=dict(
            schema_version='rescue-semantic-map/1.0', render_mode='points', map_id=0,
            point_revision=1, person_color='#ff3030', person_outline=True, people=people,
            point_labels=labels[selected].tolist()+[1]*len(human_points)))
        samples[budget] = json.dumps(snapshot, separators=(',',':'), allow_nan=False).encode()
    del pool, clouds, classes, labels
    backend = []
    for budget in ([] if args.browser_only else budgets):
        scale = float(rows[0]['depth_scale'])
        surface = SemanticSurface(max(1e-5,.04*scale), max_cells=budget, spacing_increment=max(1e-6,.005*scale))
        times, packing, counts = [], [], []
        for i in range(args.stress_frames):
            row = rows[i % len(rows)]; pose = row['world_from_camera'].copy()
            # Explicit saturation stress: real observations, translated poses.
            # This is not an accuracy test or a recorded long-flight trajectory.
            pose[0,3] += i * max(1e-5,.04*scale) * 100
            start = time.perf_counter()
            surface.integrate(row['rgb'],row['depth'],pose,row['intrinsics'],row['semantic_labels'],row['semantic_confidence'])
            points = surface.points(); surface.labels()
            elapsed = (time.perf_counter()-start)*1000
            start = time.perf_counter(); json.dumps(points,separators=(',',':'),allow_nan=False)
            encoded = (time.perf_counter()-start)*1000
            if i >= 10:
                times.append(elapsed); packing.append(encoded); counts.append(len(points))
        backend.append(dict(budget=budget, fusion_ms=stats(times), json_encode_ms=stats(packing),
                            points=stats(counts), metadata=surface.metadata()))
        surface.close()
        print(json.dumps(dict(phase='backend', **backend[-1])), flush=True)

    class Handler(SimpleHTTPRequestHandler):
        def log_message(self,*args): pass
        def do_GET(self):
            url = urlsplit(self.path)
            if url.path == '/benchmark/snapshot':
                body = samples[int(parse_qs(url.query)['budget'][0])]
                self.send_response(200); self.send_header('Content-Type','application/json')
                self.send_header('Content-Length',str(len(body))); self.end_headers(); self.wfile.write(body)
                return
            if url.path in ('/','/index.html'):
                body = (ROOT/'apps/console/index.html').read_text(encoding='utf-8').replace(
                    '<script type="module" src="/app.js"></script>',
                    '<script type="module">import {connectProgressiveMap} from "/progressive-map.js"; connectProgressiveMap();</script>').encode()
                self.send_response(200); self.send_header('Content-Type','text/html; charset=utf-8')
                self.end_headers(); self.wfile.write(body); return
            return super().do_GET()
        def translate_path(self,path):
            aliases={'/vendor/three.module.js':'three/build/three.module.js', '/vendor/three.core.js':'three/build/three.core.js',
                     '/vendor/OrbitControls.js':'three/examples/jsm/controls/OrbitControls.js'}
            name = urlsplit(path).path
            if name in aliases: return str(ROOT/'apps/console/node_modules'/aliases[name])
            return super().translate_path(path)

    if args.backend_only:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(dict(backend=backend),indent=2)+'\n',encoding='utf-8')
        return
    server = ThreadingHTTPServer(('127.0.0.1',0),partial(Handler,directory=str(ROOT/'apps/console')))
    thread = threading.Thread(target=server.serve_forever,daemon=True); thread.start()
    try:
        with tempfile.TemporaryDirectory(prefix='point-budget-profile-') as temporary:
            report_path = Path(temporary)/'browser.json'
            subprocess.run(['node',str(Path(__file__).with_suffix('.mjs')),f'http://127.0.0.1:{server.server_port}',
                            args.playwright_module,str(report_path),','.join(map(str,budgets))],check=True,cwd=ROOT)
            browser = json.loads(report_path.read_text(encoding='utf-8'))
    finally:
        server.shutdown(); server.server_close(); thread.join()
    report = dict(scope='recorded observed points at exact screen counts; backend uses explicitly translated recorded RGB-D saturation stress',
                  input_observations=len(rows), stress_frames=args.stress_frames, backend=backend, browser=browser)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(phase='complete',output=str(args.output))),flush=True)


if __name__ == '__main__': main()
