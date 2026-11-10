"""JPG/PNG adapter and confirmed import runner for the existing video pipeline."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import time
from fractions import Fraction

from PIL import Image, ImageOps

from modules.mapping.data.session_import import IMAGES, VIDEO, inside, read_json
from modules.mapping.data.camera_profiles import validate_intrinsics


def frame_paths(root):
    paths = [p for p in Path(root).rglob('*') if p.suffix.lower() in IMAGES and p.is_file()]
    if not 3 <= len(paths) <= 10000:
        raise ValueError('need_3_to_10000_frames')
    # Natural filename ordering is explicit in the confirmation screen, never claimed as EXIF time.
    def key(path):
        return [int(s) if s.isdigit() else s.casefold() for s in re.split(r'(\d+)', path.relative_to(root).as_posix())]
    return sorted(paths, key=key)


def inspect_input(source):
    import av
    source = Path(source)
    if source.is_dir():
        videos = [p for p in source.iterdir() if p.is_file() and p.suffix.lower() in VIDEO]
        if len(videos) == 1 and not any(p.suffix.lower() in IMAGES for p in source.iterdir()):
            return inspect_input(videos[0])
        frames = frame_paths(source)
        sizes = set()
        for path in frames:
            path = inside(source, path.relative_to(source).as_posix())
            with Image.open(path) as image:
                image = ImageOps.exif_transpose(image)
                sizes.add(image.size)
        if len(sizes) != 1:
            raise ValueError('frame_dimensions_differ')
        width, height = sizes.pop()
        return {'kind': 'frames', 'name': source.name, 'width': width, 'height': height, 'frames': len(frames),
            'order': [p.relative_to(source).as_posix() for p in frames[:8]], 'last_frame': frames[-1].relative_to(source).as_posix(),
            'timestamp_policy': 'filename_order_user_fps', 'warnings': ['파일명 숫자 순서로 정렬합니다. 촬영 시각이 없으므로 입력한 FPS로 시간축을 만듭니다.']}
    if source.suffix.lower() not in VIDEO:
        raise ValueError('unsupported_input_use_session_zip_video_or_frames')
    with av.open(str(source)) as container:
        streams = container.streams.video
        if not streams:
            raise ValueError('no_video_stream')
        stream = streams[0]
        first = next(container.decode(stream), None)
        if first is None:
            raise ValueError('empty_video')
        from modules.mapping.data.video_input import display_rotation
        rotation = display_rotation(first, stream.metadata)
        width, height = first.width, first.height
        if rotation in {90, 270}:
            width, height = height, width
        duration = float(stream.duration * stream.time_base) if stream.duration else (container.duration or 0) / av.time_base
        return {'kind': 'video', 'name': source.name, 'width': width, 'height': height,
            'frames': stream.frames or None, 'duration_s': duration or None,
            'source_fps': float(stream.average_rate) if stream.average_rate else None,
            'rotation': rotation, 'warnings': ['새 복원은 저장된 실시간 지도와 다른 결과가 나올 수 있습니다.'],
            'timestamp_policy': 'source_pts'}


def validate_options(options, summary):
    from modules.mapping.data.mapping_limits import DEFAULT_VIDEO_SAMPLE_FPS

    if not isinstance(options, dict) or options.get('confirmed') is not True:
        raise ValueError('confirm_input_order_and_camera')
    result = {}
    causal_video = options.get('causal_video') is True
    full_video = options.get('all_video_frames') is True
    for key, default, lo, hi in [('sample_fps', DEFAULT_VIDEO_SAMPLE_FPS if causal_video else 2, .1, 30),
                                 ('frame_fps', 2, .1, 240)]:
        value = options.get(key, default)
        if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError('invalid_' + key)
        result[key] = value
    camera = options.get('camera')
    if camera is not None:
        if not isinstance(camera, dict):
            raise ValueError('invalid_camera_intrinsics')
        camera = validate_intrinsics(camera)
        if any(type(camera.get(k)) not in (int, float) or not math.isfinite(camera[k]) for k in ('width', 'height', 'fx', 'fy', 'cx', 'cy')):
            raise ValueError('camera_values_must_be_finite_numbers')
        if (camera['width'], camera['height']) != (summary['width'], summary['height']):
            raise ValueError('calibration_dimensions_must_match_input')
        if not (camera['fx'] > 0 and camera['fy'] > 0 and 0 <= camera['cx'] <= camera['width'] and 0 <= camera['cy'] <= camera['height']):
            raise ValueError('invalid_camera_intrinsics')
        distortion = camera.get('distortion', [0., 0., 0., 0.])
        if not isinstance(distortion, list) or len(distortion) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) for v in distortion):
            raise ValueError('invalid_camera_distortion')
        camera['distortion'] = distortion
        camera.setdefault('calibration_status', 'user_supplied; accuracy_not_independently_verified')
        result['camera'] = camera
    result['confirmed'] = True
    mode = options.get('mode', 'sfm')
    if mode not in {'sfm', 'depth_rgbd'}:
        raise ValueError('invalid_reconstruction_mode')
    result['mode'] = mode
    if 'pose_model' in options or 'depth_model' in options:
        from .reconstruction_models import model_for
        if mode != 'depth_rgbd' or summary.get('kind') != 'video':
            raise ValueError('video_required')
        for kind in ('pose', 'depth'):
            field = kind + '_model'
            model_for(options.get(field), kind)
            result[field] = options[field]
    if causal_video or full_video:
        if mode != 'depth_rgbd' or summary.get('kind') != 'video':
            raise ValueError('video_required')
        result.update(causal_video=True, all_video_frames=full_video)
        result['frame_fps'] = min(240., max(1., summary.get('source_fps') or 30.))
    if mode == 'depth_rgbd':
        from modules.mapping.data.rescue_mapping import mapping_sample_fps
        result['sample_fps'] = mapping_sample_fps(result)
    window = options.get('render_window_s')
    if window is not None and (type(window) not in (int, float) or not math.isfinite(window) or not 1 <= window <= 120):
        raise ValueError('invalid_render_window_s')
    result['render_window_s'] = window
    from modules.mapping.data.rescue_mapping import mapping_options
    result.update(mapping_options(options))
    return result


def prepare_frames(source, output, run_id, fps):
    from modules.mapping.data.video_input import write_json, sha256
    started = time.perf_counter()
    output = Path(output)
    output.mkdir()
    (output / 'frames').mkdir()
    frames = []
    decisions = []
    size = None
    paths = frame_paths(source)
    last_progress = 0.
    for index, path in enumerate(paths):
        path = inside(source, path.relative_to(source).as_posix())
        with Image.open(path) as raw:
            image = ImageOps.exif_transpose(raw).convert('RGB')
            if size is not None and image.size != size:
                raise ValueError('frame_dimensions_differ')
            size = image.size
            name = f'frame_{index + 1:07d}.jpg'
            target = output / 'frames' / name
            image.save(target, quality=95, subsampling=0)
        stamp = index / fps
        decision = {'frame_id': name, 'selected': True, 'source_pts': index, 'source_time_base': str(1 / Fraction(str(fps))),
            'source_pts_s': stamp, 'timestamp_s': stamp, 'timestamp_source': 'user_fps_filename_order',
            'decoded_frame_index': index, 'reason': 'provided_frame'}
        decisions.append(decision)
        frames.append({**decision, 'path': 'frames/' + name, 'width': size[0], 'height': size[1],
            'sha256': sha256(target), 'bytes': target.stat().st_size,
            'original_name': path.relative_to(source).as_posix(), 'original_sha256': sha256(path)})
        if time.monotonic() - last_progress >= .25:
            write_json(output / 'status.json', {'status': 'preparing', 'phase': 'copy_frames',
                       'completed': index + 1, 'total': len(paths)})
            last_progress = time.monotonic()
    audit = output / 'frame_decisions.jsonl'
    audit.write_text(''.join(json.dumps(r) + '\n' for r in decisions), encoding='utf-8')
    write_json(output / 'manifest.json', {'schema_version': 'phone-video-input/1.0', 'run_id': run_id,
        'status': 'frames_ready', 'frames': frames, 'selected_frames': len(frames), 'decoded_frames': len(frames),
        'source': {'kind': 'frame_folder', 'filename': Path(source).name, 'timestamp_policy': 'user_fps_filename_order'},
        'frame_decisions': audit.name, 'frame_decisions_sha256': sha256(audit), 'metric_reference': None})
    write_json(output / 'status.json', {'status': 'frames_ready'})
    write_json(output / 'timing.json', {'elapsed_seconds': time.perf_counter() - started})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--options', type=Path, required=True)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    from modules.mapping.data.run_video_pipeline import run_pipeline
    source = args.source
    if source.is_dir():
        videos = [p for p in source.iterdir() if p.suffix.lower() in VIDEO]
        if len(videos) == 1 and not any(p.suffix.lower() in IMAGES for p in source.iterdir()):
            source = videos[0]
    options = validate_options(read_json(args.options), inspect_input(source))
    camera = options.get('camera')
    result = run_pipeline(source, args.output, sample_fps=options['sample_fps'],
        frame_fps=options['frame_fps'], camera=camera)
    return result['exit_code']


if __name__ == '__main__':
    raise SystemExit(main())
