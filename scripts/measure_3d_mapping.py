"""Measure the real mapping implementation using recorded UDP input; no drone commands.

Runs an isolated console and spawned depth worker. Raw recordings/maps live only in
a TemporaryDirectory; the report contains aggregate numeric measurements only.
Use --implementation-root while the semantic mapping dependency is unmerged.
"""
from __future__ import annotations

import argparse
import atexit
from collections import defaultdict
from functools import partial, wraps
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import socket
import subprocess
import sys
import tempfile
import threading
import time

EVENTS = []
CONTEXT = threading.local()
LOCK = threading.Lock()
INSTALLED = False


def milliseconds():
    return time.perf_counter_ns() / 1e6


def event(kind, start, **fields):
    row = dict(kind=kind, start_ms=start, ms=milliseconds()-start,
               frame=getattr(CONTEXT, 'frame', None), stamp=getattr(CONTEXT, 'stamp', None), **fields)
    with LOCK:
        EVENTS.append(row)
    return row


def flush():
    folder = os.environ.get('MAPPING_PROFILE_EVENTS')
    if folder and Path(folder).is_dir():
        with LOCK:
            rows = list(EVENTS)
        Path(folder, f'{os.getpid()}.json').write_text(json.dumps(rows), encoding='utf-8')


def instrument():
    global INSTALLED
    if INSTALLED:
        return
    INSTALLED = True
    sys.path.insert(0, os.environ['MAPPING_PROFILE_IMPLEMENTATION'])
    from modules.mapping.data import stream_mapping as pipeline
    from modules.mapping.data import rescue_mapping as rescue
    from modules.mapping.data.mapping_history import HistoryWriter
    from modules.mapping.localization import pose_first
    from modules.mapping.localization import orbslam3, orbslam3_live
    if os.environ.get('MAPPING_PROFILE_IMAGE'):
        # Pin the verified local image digest; never build, pull or retag it.
        for module in (pose_first, orbslam3, orbslam3_live):
            module.IMAGE = os.environ['MAPPING_PROFILE_IMAGE']
    from multiprocessing.connection import _ConnectionBase
    from multiprocessing.util import Finalize
    from PIL import Image
    Finalize(None, flush, exitpriority=0)
    atexit.register(flush)

    def timed(owner, name, kind):
        if not hasattr(owner, name):
            return  # New point fusion has no mesh extraction stage.
        original = getattr(owner, name)
        @wraps(original)
        def call(*args, **kwargs):
            start = milliseconds()
            try:
                return original(*args, **kwargs)
            finally:
                event(kind, start)
        setattr(owner, name, call)

    timed(pipeline.DepthEstimator, '__init__', 'prepare_depth_load')
    timed(pipeline.DepthEstimator, 'prepare', 'prepare_depth_warmup')
    timed(pipeline.DepthEstimator, 'estimate', 'depth')
    timed(rescue.SemanticEstimator, '__init__', 'prepare_semantic_load')
    timed(rescue.SemanticEstimator, 'estimate', 'semantics')
    timed(pose_first.OrbPoseTracker, '__init__', 'prepare_tracker')
    timed(pose_first, 'depth_in_pose_units', 'scale_alignment')
    timed(rescue, 'smooth_structural_depth', 'structural_smoothing')
    timed(rescue.SemanticSurface, 'integrate', 'static_fusion')
    timed(rescue.SemanticSurface, 'mesh', 'mesh_extraction')
    timed(rescue.SemanticSurface, '__init__', 'static_volume_setup')
    timed(rescue.SemanticSurface, 'points', 'preview_points')
    timed(rescue.PersonSightings, 'update', 'person_3d')
    timed(rescue.RescueMap, 'update', 'rescue_update_total')
    timed(rescue, 'person_points', 'person_compat_points')
    if os.environ.get('MAPPING_PROFILE_WARMUP') == '1':
        original_semantic_init = rescue.SemanticEstimator.__init__
        def semantic_init(self, *pos, **kw):
            original_semantic_init(self, *pos, **kw)
            import numpy as np
            begin=milliseconds()
            self.estimate(np.zeros((288,384,3),np.uint8),'benchmark_warmup',0.)
            surface=rescue.SemanticSurface(.04); surface.close()
            event('benchmark_extra_warmup',begin)
        rescue.SemanticEstimator.__init__=semantic_init
    timed(HistoryWriter, 'append', 'history_write')

    original_track = pose_first.OrbPoseTracker.track
    tracked_at = {}
    @wraps(original_track)
    def track(self, rgb, stamp):
        start = milliseconds()
        result = original_track(self, rgb, stamp)
        row = event('pose', start, input_stamp=stamp, tracking_state=result.get('tracking_state'))
        tracked_at[stamp] = row['start_ms'] + row['ms']
        return result
    pose_first.OrbPoseTracker.track = track
    original_take = pipeline.LivePoseFrames.take
    @wraps(original_take)
    def take(self):
        result = original_take(self)
        if result is not None:
            CONTEXT.frame, CONTEXT.stamp = result[:2]
            now = milliseconds()
            event('candidate_wait', tracked_at.get(result[1], now))
        else:
            CONTEXT.frame = CONTEXT.stamp = None
        return result
    pipeline.LivePoseFrames.take = take

    original_write = pipeline.write_json
    @wraps(original_write)
    def write(path, value):
        start = milliseconds()
        try:
            return original_write(path, value)
        finally:
            event('state_write', start, file=Path(path).name)
    pipeline.write_json = write
    original_save = Image.Image.save
    @wraps(original_save)
    def save(self, path, *args, **kwargs):
        start = milliseconds()
        result = original_save(self, path, *args, **kwargs)
        if str(path).lower().endswith('.jpg'):
            event('frame_jpeg_write', start)
        return result
    Image.Image.save = save
    original_send = _ConnectionBase.send
    @wraps(original_send)
    def send(self, value):
        preview = isinstance(value, tuple) and value and value[0] == 'preview'
        start = milliseconds()
        if preview:
            value[1]['_benchmark_child_ready_ms'] = start
        result = original_send(self, value)
        if preview:
            event('preview_ipc_send', start)
        return result
    _ConnectionBase.send = send


if os.environ.get('MAPPING_PROFILE_IMPLEMENTATION'):
    instrument()


def distribution(values):
    values = sorted(float(v) for v in values if v is not None and math.isfinite(v))
    if not values:
        return dict(n=0, mean=None, p50=None, p95=None, maximum=None)
    return dict(n=len(values), mean=sum(values)/len(values),
                p50=values[(len(values)-1)//2], p95=values[math.ceil(.95*len(values))-1], maximum=values[-1])


def summarize_report(report):
    samples=report['paired_mapped_frame_samples']
    static=lambda d:d.get('scale_alignment',0)+d.get('rescue_update_total',0)-d.get('person_3d',0)-d.get('preview_points',0)
    history=lambda d:sum(d.get(k,0) for k in ('preview_points','person_compat_points','history_write','frame_jpeg_write','state_write','preview_ipc_send'))
    stage_values={
        '3_depth':[s['stages_ms'].get('depth',0) for s in samples],
        '4_semantics':[s['stages_ms'].get('semantics',0) for s in samples],
        '5_static':[static(s['stages_ms']) for s in samples],
        '6_person':[s['stages_ms'].get('person_3d',0) for s in samples],
        '7_server_record_pack_ipc':[history(s['stages_ms']) for s in samples]}
    means={k:sum(v)/len(v) if v else 0 for k,v in stage_values.items()}
    total=sum(means.values())
    return dict(units='milliseconds; wall-time work per matched mapped frame, not an additive end-to-end prediction',
        mapped_frame_stages_ms={k:distribution(v) for k,v in stage_values.items()},
        mapped_frame_work_share_percent={k:100*v/total if total else 0 for k,v in means.items()},
        precise_decode_clock_samples=sum(s['precise_decode_clock'] for s in samples),
        child_to_parent_ms=distribution([s['child_to_parent_ms'] for s in samples]),
        rendered_fps=1000/report['browser']['liveFrameIntervalMs']['mean'] if report['browser']['liveFrameIntervalMs']['mean'] else None,
        mapped_updates_per_replay_second=report['outcome']['mapped_previews']/report['input']['replay_wall_s'])


def display_history(args, root):
    """Paired display comparison of an existing observed person/mesh snapshot."""
    from types import SimpleNamespace
    sys.path.insert(0,str(root))
    from modules.mapping.data.mapping_history import HistoryReader
    from modules.mapping.data.session_import import ImportManager
    from apps.console.serve import ViewerHandler, ViewerServer, Timeline, ObservationCache
    reconstruction=args.display_history_root.resolve()
    snapshot=HistoryReader(reconstruction).snapshot(args.revision)
    snapshot['_benchmark']={}
    with tempfile.TemporaryDirectory(prefix='mapping-display-profile-') as temporary:
        scratch=Path(temporary); manager=ImportManager(scratch/'maps')
        key=manager.create('Observed display comparison')['id']
        manager.job(key).update(status='ready',stage='mapping',output=reconstruction,preview=snapshot,
            options={'mode':'depth_rgbd','render_window_s':None})
        def stats():
            return dict(source='live',session_id='benchmark-observed',session_status='receiving',status='receiving',
                backend='orb_slam3_depth',input='tello_camera',transport_ready=True,map_available=False,
                depth_job_id=key,depth_job=manager.status(key),recording={'status':'complete'})
        session=SimpleNamespace(id='benchmark-observed',path=reconstruction.parent,condition=threading.Condition(),stats=stats)
        class Handler(ViewerHandler):
            def log_message(self,*unused): pass
            def do_GET(self):
                if self.path.startswith('/benchmark/'):
                    self.json_response(dict(session_done=True,perf_ms=milliseconds()));return
                super().do_GET()
        server=ViewerServer(('127.0.0.1',0),partial(Handler,live=SimpleNamespace(current=session),imports=manager,
            timeline=Timeline(),observations=ObservationCache()))
        threading.Thread(target=server.serve_forever,daemon=True).start()
        try:
            subprocess.run(['node',str(Path(__file__).with_suffix('.mjs')),f'http://127.0.0.1:{server.server_port}',
                args.playwright_module,str(scratch/'browser.json'),'display-only'],check=True)
            report=json.loads((scratch/'browser.json').read_text(encoding='utf-8'))
            report.update(scope='fixed observed history snapshot; display-only, no inferred pipeline latency',
                implementation_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
                history_chunk_sha256=hashlib.sha256((reconstruction/'history/frames'/f'{args.revision:08d}.npz').read_bytes()).hexdigest())
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
            print(json.dumps(dict(event='display_report',path=str(args.report))))
        finally:
            server.shutdown();server.server_close();manager.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--implementation-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--recording', type=Path, help='Original .h264; sibling .jsonl contains UDP packet offsets/times')
    parser.add_argument('--camera', type=Path)
    parser.add_argument('--display-history-root', type=Path, help='Display-only comparison using a real reconstruction/history snapshot')
    parser.add_argument('--revision',type=int,default=1)
    parser.add_argument('--start', type=float, default=105)
    parser.add_argument('--duration', type=float, default=40)
    parser.add_argument('--warmup-inference', action='store_true', help='Measure steady state after an explicit synthetic inference/volume warmup, reported separately')
    parser.add_argument('--playwright-module', required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    root = args.implementation_root.resolve()
    if not (root/'modules/mapping/data/rescue_mapping.py').is_file():
        parser.error('--implementation-root must contain the semantic mapping implementation being measured')
    if args.display_history_root:
        if args.report.resolve().is_relative_to(args.display_history_root.resolve()):
            parser.error('display report must be outside the original reconstruction')
        display_history(args,root);return
    if not args.recording or not args.camera:
        parser.error('--recording and --camera are required for packet replay')
    if not math.isfinite(args.start) or not math.isfinite(args.duration) or args.start<0 or args.duration<=0:
        parser.error('start must be nonnegative and duration must be positive finite seconds')
    if args.report.resolve() in {args.recording.resolve(), args.camera.resolve(), args.recording.with_suffix('.jsonl').resolve()}:
        parser.error('report must not overwrite a recording or camera input')
    started = milliseconds()
    with tempfile.TemporaryDirectory(prefix='mapping-stage-profile-') as temporary:
        scratch = Path(temporary)
        os.environ['MAPPING_PROFILE_IMPLEMENTATION'] = str(root)
        os.environ['MAPPING_PROFILE_EVENTS'] = str(scratch)
        os.environ['MAPPING_PROFILE_WARMUP'] = '1' if args.warmup_inference else '0'
        image_resolution = 'tag'
        try:
            image_id = subprocess.check_output(['docker', 'image', 'inspect', '--format', '{{.Id}}', 'ssafy-orbslam3:local'], text=True, stderr=subprocess.DEVNULL).strip()
        except subprocess.CalledProcessError:
            listed = subprocess.check_output(['docker','image','ls','--no-trunc','--quiet','ssafy-orbslam3:local'],text=True).splitlines()
            if len(set(listed)) != 1:
                raise RuntimeError('local SLAM image cannot be resolved unambiguously')
            image = json.loads(subprocess.check_output(['docker','image','inspect',listed[0]],text=True))[0]
            if 'ssafy-orbslam3:local' not in image.get('RepoTags',[]):
                raise RuntimeError('resolved image has a different tag')
            image_id=image['Id']; image_resolution='verified_id_after_tag_lookup_failure'
        os.environ['MAPPING_PROFILE_IMAGE'] = image_id
        instrument()
        import cv2
        from modules.mapping.data.session_import import ImportManager
        from modules.mapping.localization.orbslam3_live import LiveManager
        from modules.mapping.localization.live_depth import LiveDepthSession
        from modules.mapping.localization.tello_input import DirectCamera
        from modules.autonomy.tello_link.tello_video import TelloVideo
        from apps.console.serve import ViewerHandler, ViewerServer, Timeline, ObservationCache

        camera = json.loads(args.camera.read_text(encoding='utf-8'))
        entries = [json.loads(line) for line in args.recording.with_suffix('.jsonl').read_text(encoding='utf-8').splitlines()]
        zero = entries[0]['timestamp_s']
        selected = [e for e in entries if args.start <= e['timestamp_s']-zero < args.start+args.duration]
        if len(selected) < 20:
            raise ValueError('selected recording interval has too few UDP packets')
        manager = ImportManager(scratch/'maps')
        live = LiveManager(scratch/'live', manager)
        published = []
        inputs = {}
        browser_ready, browser_done, input_done = threading.Event(), threading.Event(), threading.Event()
        readiness = dict(feeding_complete=False, session_done=False)

        original_published = LiveDepthSession._published
        def publish(session, snapshot):
            now = milliseconds()
            source = inputs.get(snapshot['frame_number'], {})
            snapshot['_benchmark'] = dict(parent_ready_ms=now, **source)
            original_published(session, snapshot)
            published.append(dict(frame=snapshot['frame_number'], stamp=snapshot['timestamp_s'],
                revision=snapshot['revision'], tracking=snapshot['tracking'], **snapshot['_benchmark'],
                child_ready_ms=snapshot.get('_benchmark_child_ready_ms'),
                people=len(snapshot.get('rescue', {}).get('people', [])),
                voxels=sum(len(p['voxels']) for p in snapshot.get('rescue', {}).get('people', [])),
                vertices=len(snapshot.get('rescue', {}).get('mesh', {}).get('vertices', [])),
                triangles=len(snapshot.get('rescue', {}).get('mesh', {}).get('triangles', []))))
        LiveDepthSession._published = publish

        class Handler(ViewerHandler):
            def log_message(self, *unused):
                pass
            def do_GET(self):
                if self.path == '/benchmark/clock':
                    self.json_response(dict(perf_ms=milliseconds())); return
                if self.path == '/benchmark/ready':
                    browser_ready.set(); self.json_response(dict(ok=True)); return
                if self.path == '/benchmark/done':
                    browser_done.set(); self.json_response(dict(ok=True)); return
                if self.path == '/benchmark/state':
                    self.json_response(readiness); return
                super().do_GET()
            def json_response(self, value, *pos, **kw):
                begin = milliseconds()
                result = super().json_response(value, *pos, **kw)
                if isinstance(value, dict) and value.get('frame_id') and 'points' in value:
                    event('preview_http_serialize_write', begin, http_frame=value['frame_id'],
                          mesh_sent=bool(value.get('rescue', {}).get('mesh')))
                return result
        server = ViewerServer(('127.0.0.1', 0), partial(Handler, live=live, imports=manager,
            timeline=Timeline(), observations=ObservationCache()))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        url = f'http://127.0.0.1:{server.server_port}'
        node = subprocess.Popen(['node', str(Path(__file__).with_suffix('.mjs')), url,
            args.playwright_module, str(scratch/'browser.json')])
        session = video = sink = None
        sender = receiver = None
        stop = threading.Event()
        source_errors = []
        initial_gpu = subprocess.check_output(['nvidia-smi','--query-gpu=name,memory.total,memory.used,driver_version','--format=csv,noheader'], text=True).strip()
        try:
            state = live.start(dict(width=camera['width'], height=camera['height'], fps=24, camera=camera,
                semantic_mapping=True, depth_resolution=384, live_policy='continuous'),
                input_kind='tello_camera', timestamp_basis='replayed_udp_receive_monotonic', image_id=image_id)
            session = live.require(state['session_id'])
            deadline = time.monotonic()+240
            while not session.ready:
                if session.done:
                    raise RuntimeError(session.error or str(session.job.get('error')))
                if time.monotonic()>deadline or node.poll() is not None:
                    raise RuntimeError('model_or_browser_start_timeout')
                time.sleep(.1)
            if not browser_ready.wait(30):
                raise RuntimeError('browser_not_ready')
            prepare_ms = milliseconds()-started
            print(json.dumps(dict(event='prepared', seconds=round(prepare_ms/1000,2), url=url)), flush=True)

            video = TelloVideo(port=0, bind_ip='127.0.0.1')
            sink = session.recording.open_part('h264', 'replayed_udp_receive_monotonic')
            video.packet_sink = sink.offer
            video.start()
            codec = video._codec
            codec_marks, jpeg_marks = {}, {}
            class Codec:
                accumulated_cpu_ms = 0.
                def parse(self, data):
                    begin = milliseconds(); result = codec.parse(data)
                    row=event('input_parse', begin); self.accumulated_cpu_ms+=row['ms']; return result
                def decode(self, packet):
                    begin = milliseconds(); result = codec.decode(packet)
                    decoded_at=milliseconds()
                    row=event('input_decode', begin, decoded=len(result)); self.accumulated_cpu_ms+=row['ms']
                    if result:
                        for frame in result: codec_marks[id(frame)]=dict(decoded_ms=decoded_at,codec_ms=self.accumulated_cpu_ms/len(result))
                        self.accumulated_cpu_ms=0.
                    return result
            video._codec = Codec()
            camera_input = DirectCamera()
            camera_input.video = video
            replay_begin = milliseconds()

            def receive():
                sequence = 0
                try:
                    for jpeg, stamp in camera_input.frames(lambda: stop.is_set() or session.done):
                        if jpeg is None:
                            continue
                        session.note_input_frame()
                        if not session.can_accept_frame:
                            session.note_skipped_frame(); continue
                        if sequence == 0:
                            base = stamp; session.recording.set_mapping_origin(base)
                        # Python 3.12 Windows monotonic and perf_counter can have
                        # different origins. Never subtract them directly.
                        mark=jpeg_marks.get(stamp)
                        decoded_at=mark['decoded_ms'] if mark else milliseconds()-(time.monotonic()-stamp)*1000
                        inputs[sequence+1] = dict(decoded_ms=decoded_at, admitted_ms=milliseconds(), jpeg_bytes=len(jpeg),
                            precise_decode_clock=bool(mark))
                        session.offer(jpeg, stamp-base, sequence)
                        sequence += 1
                except Exception as error:
                    source_errors.append(str(error)); stop.set()
            receiver = threading.Thread(target=receive, daemon=True)
            receiver.start()
            original_jpeg = video.jpeg_snapshot
            previous_jpeg = [None]
            def jpeg(*pos, **kw):
                with video._lock:
                    before_frame,before_stamp=video._frame,video._frame_mono
                begin = milliseconds(); result = original_jpeg(*pos, **kw)
                if result and result[1] != previous_jpeg[0]:
                    previous_jpeg[0] = result[1]
                    with video._lock:
                        after_frame,after_stamp=video._frame,video._frame_mono
                    frame=before_frame if before_stamp==result[1] else after_frame if after_stamp==result[1] else None
                    mark=codec_marks.get(id(frame))
                    if mark: jpeg_marks[result[1]]=mark
                    event('input_jpeg', begin, codec_ms=mark['codec_ms'] if mark else None, bytes=len(result[0]))
                return result
            video.jpeg_snapshot = jpeg
            destination = video._sock.getsockname()
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp, args.recording.open('rb') as recording:
                for entry in entries[:2]:
                    recording.seek(entry['offset']); udp.sendto(recording.read(entry['size']), destination)
                wall_zero = time.perf_counter()
                for entry in selected:
                    wait = wall_zero + entry['timestamp_s']-zero-args.start - time.perf_counter()
                    if wait>0: stop.wait(wait)
                    if stop.is_set() or session.done: break
                    recording.seek(entry['offset']); udp.sendto(recording.read(entry['size']), destination)
            stop.wait(.15)
            replay_end = milliseconds()
            stop.set(); receiver.join(3); video.stop(); sink.close(video.stats())
            session.recording.finish_input()
            session.stop('benchmark_replay_finished')
            readiness['feeding_complete'] = True
            session.worker.join(180)
            if not session.done:
                raise RuntimeError('finalization_timeout')
            readiness['session_done'] = True
            print(json.dumps(dict(event='mapped', received=session.raw_received, admitted=session.received,
                previews=len(published), final_status=session.job['status'], error=session.error)), flush=True)
            deadline=time.monotonic()+180
            while not browser_done.wait(.2):
                if node.poll() is not None: raise RuntimeError('browser_measurement_failed')
                if time.monotonic()>deadline: raise RuntimeError('browser_measurement_timeout')
            node.wait(timeout=15)
            if node.returncode:
                raise RuntimeError('browser_measurement_failed')
            manager.shutdown()
            flush()
            rows=[]
            for file in scratch.glob('[0-9]*.json'):
                rows.extend(json.loads(file.read_text(encoding='utf-8')))
            browser=json.loads((scratch/'browser.json').read_text(encoding='utf-8'))
            grouped=defaultdict(list)
            for row in rows:
                if (row.get('frame') is not None or row['kind']=='pose' or row['kind'].startswith('input_') or row['kind'].startswith('preview_http_')) and replay_begin<=row['start_ms']<=replay_end+3000:
                    grouped[row['kind']].append(row['ms'])
                    if row['kind']=='input_jpeg' and row.get('codec_ms') is not None:
                        grouped['input_frame_service'].append(row['ms']+row['codec_ms'])
            per_frame=defaultdict(lambda: defaultdict(float))
            for row in rows:
                if row.get('frame') is not None:
                    per_frame[row['frame']][row['kind']]+=row['ms']
            mapped=[p for p in published if p['tracking']=='tracking']
            def static_time(frame):
                durations=per_frame[frame]
                return durations['scale_alignment']+durations['rescue_update_total']-durations['person_3d']-durations['preview_points']
            stage7=['preview_points','person_compat_points','history_write','frame_jpeg_write','state_write','preview_ipc_send']
            report=dict(schema_version='mapping-stage-profile/1.0',
                implementation_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
                scope='recorded original Tello UDP packets replayed on loopback at original packet timing; no physical camera/radio latency',
                input=dict(sha256=hashlib.sha256(args.recording.read_bytes()).hexdigest(), start_s=args.start,
                    duration_requested_s=args.duration, replay_wall_s=(replay_end-replay_begin)/1000,
                    width=camera['width'], height=camera['height'], packet_count=len(selected)),
                environment=dict(python=sys.version, cpu=platform.processor(), logical_cpus=os.cpu_count(), gpu=initial_gpu,
                    docker_image=image_id, image_resolution=image_resolution,
                    viewport=browser.get('viewport'), preparation_wall_ms=prepare_ms, explicit_inference_warmup=args.warmup_inference),
                outcome=dict(raw_received=session.raw_received, admitted=session.received, submitted=session.submitted,
                    previews=len(published), mapped_previews=len(mapped), final_status=session.job['status'],
                    error=session.error, input_errors=source_errors, mapping_sampled_out=session.job.get('progress',{}).get('mapping_sampled_out')),
                stages_ms={key:distribution(values) for key,values in grouped.items()},
                startup_ms={key:distribution([r['ms'] for r in rows if r['kind']==key]) for key in
                    ['prepare_depth_load','prepare_depth_warmup','prepare_semantic_load','prepare_tracker','benchmark_extra_warmup']},
                mapped_frame_ms=dict(static_stage_5=distribution([static_time(p['frame']) for p in mapped]),
                    server_stage_7=distribution([sum(per_frame[p['frame']][k] for k in stage7) for p in mapped])),
                paired_mapped_frame_samples=[dict(frame=p['frame'],stages_ms=dict(per_frame[p['frame']]),
                    decoded_to_parent_ms=p['parent_ready_ms']-p['decoded_ms'],
                    child_to_parent_ms=p['parent_ready_ms']-p['child_ready_ms'] if p['child_ready_ms'] is not None else None,
                    precise_decode_clock=p['precise_decode_clock']) for p in mapped],
                geometry=dict(people=distribution([p['people'] for p in mapped]), voxels=distribution([p['voxels'] for p in mapped]),
                    vertices=distribution([p['vertices'] for p in mapped]),triangles=distribution([p['triangles'] for p in mapped])),
                browser=browser, limitations=[
                    'Stage 1 is PC loopback receive/decode/JPEG CPU cost. Sensor capture, drone encoder and radio transport are unmeasured.',
                    'Pose tracking runs concurrently with mapping; stage percentiles must not be added to estimate total latency.',
                    'Browser paint is a post-WebGL requestAnimationFrame estimate, not a physical monitor/photon measurement.',
                    'Decode timestamps and browser clock calibration use perf_counter; original coarse monotonic PTS is used only for SLAM sequence identity.',
                    'Ablations compare the same observed snapshot with inference idle; they measure display cost, not reconstruction accuracy.'])
            report['summary']=summarize_report(report)
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
            print(json.dumps(dict(event='report',path=str(args.report))),flush=True)
        finally:
            stop.set()
            if session and not session.done: session.stop('benchmark_cleanup')
            if video and video.running: video.stop()
            if sink: sink.close()
            if session: session.recording.finish_input()
            manager.shutdown()
            if node.poll() is None: node.terminate(); node.wait(timeout=15)
            server.shutdown();server.server_close()


if __name__=='__main__':
    main()
