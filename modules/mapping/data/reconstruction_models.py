"""Pinned model catalog and on-demand asset preparation for saved-video mapping."""
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path, PurePosixPath
import logging
import re
import subprocess
from tempfile import TemporaryDirectory
import threading
from urllib.request import urlopen
from zipfile import ZipFile

from .model_assets import DEPTH_MODEL
from modules.mapping.localization.pose_models import (
    ORB, DPVO, HYBRID, DPV, DPV_PLUS_PLUS, VISTA, POSE_IMAGES, BUILD_TARGETS, VISTA_CHECKPOINT, VISTA_VOCABULARY,
)

ROOT = Path(__file__).resolve().parents[3]
MODEL_ROOT = ROOT / 'data/models/reconstruction'
POSE_SOURCE = ROOT / 'modules/mapping/localization/learned_pose'
POSE_PREPARATION = threading.Lock()
# These builds have the same dependencies as this Dockerfile. Mounting the
# current worker lets existing installations keep their downloaded runtimes.
PREVIEW_RUNTIME_DOCKERFILE = '5d3bc51414ff39444e0258c160f7f18017eedc78b03eb3d1e11fc8279e018e2e'
COMPATIBLE_POSE_IMAGES = frozenset({
    '829f1825370cebd70d48c01763615636b93ff3c33a4f7a74d8ccaa83515b99fa',
    'aba470e0b029269137ffaf949f24bf019fe9e067894b4e4565be12136bf32715',
    '6ecb3f0d6e5ed69dc35b76316e794b19b94366ed85e166a752484b2f9256a940',
    'ddc07678e9200b947951ffaa846b42041c896c34b020581f2511bfd8097393fd',
})
DEPTH_V2 = 'depth-anything-v2-metric-indoor-small'
VIDEO_DEPTH = 'metric-video-depth-anything-small'
MOGE = 'moge-2-small'


@dataclass(frozen=True)
class Asset:
    name: str
    url: str
    size: int
    checksum: str
    # An archive contributes only these package directories or files.
    packages: tuple[str, ...] = ()
    strip_root: bool = False


@dataclass(frozen=True)
class Model:
    kind: str
    name: str
    size: int
    vram_gb: int
    repo: str = ''
    revision: str = ''
    assets: tuple[Asset, ...] = ()
    runtime_files: tuple[str, ...] = ()

    @property
    def install_version(self):
        if not self.runtime_files:
            return self.revision
        runtime_hash = sha256(''.join(asset.checksum for asset in self.assets).encode()).hexdigest()[:12]
        return self.revision + '-' + runtime_hash


def weights(repo, revision, name, size, checksum):
    return Asset(name, f'https://huggingface.co/{repo}/resolve/{revision}/{name}', size, checksum)


def source(repo, revision, size, checksum, *packages):
    return Asset(repo.split('/')[-1] + '.zip', f'https://codeload.github.com/{repo}/zip/{revision}',
                 size, checksum, packages, strip_root=True)


V2_REVISION = '8078d68a9c75a972131914f6afd0c1723be0da7f'
VIDEO_REPO = 'depth-anything/Metric-Video-Depth-Anything-Small'
VIDEO_REVISION = '273d090f2ce17df50c2872d82c8322c45da5b4dd'
MOGE_REPO = 'Ruicheng/moge-2-vits-normal'
MOGE_REVISION = '26b477f41595707c5db6770294c0d1721e8ed4ed'
VIDEO_WEIGHT = 'metric_video_depth_anything_vits.pth'
MOGE_WEIGHT = 'model.pt'
VISTA_REPO = 'zhangganlin/vista_slam'
VISTA_REVISION = '734e79548d29adf930903bb87d2fcaff8f05b60b'
VISTA_ASSETS = (
    weights(VISTA_REPO, VISTA_REVISION, VISTA_CHECKPOINT, 1754072333,
            '8b8cc1c49172731f8c8de928810aa969502411b7f263d6013951207d27fa06a3'),
    weights(VISTA_REPO, VISTA_REVISION, VISTA_VOCABULARY, 145250924,
            'f8dd027f7a6cb88129821341194d7f2c75b77b3394257ddd0d2229863d1a3570'),
)
EINOPS = Asset('einops.whl',
    'https://files.pythonhosted.org/packages/2a/09/f8d8f8f31e4483c10a906437b4ce31bdf3d6d417b73fe33f1a8b59e34228/einops-0.8.2-py3-none-any.whl',
    65638, '54058201ac7087911181bfec4af6091bb59380360f069276601256a76af08193', ('einops',))
EASYDICT = Asset('easydict.whl',
    'https://files.pythonhosted.org/packages/05/ec/fa6963f1198172c2b75c9ab6ecefb3045991f92f75f5eb41b6621b198123/easydict-1.13-py3-none-any.whl',
    6804, '6b787daf4dcaf6377b4ad9403a5cee5a86adbc0ca9a5bcf5410e9902002aeac2', ('easydict',))
# Sizes are for the vocabulary/checkpoint, excluding runtime files. VRAM figures
# allow headroom above the CUDA FP16 384 px benchmark and are recommendations.
CATALOG = {
    ORB: Model('pose', 'ORB-SLAM3', 145250924, 0),
    DPVO: Model('pose', 'DPVO', 14167743, 3),
    HYBRID: Model('pose', 'ORB-SLAM3 + DPVO', 159418667, 3),
    DPV: Model('pose', 'DPV-SLAM', 14167743, 3),
    DPV_PLUS_PLUS: Model('pose', 'DPV-SLAM++', 211405341, 4),
    VISTA: Model('pose', 'ViSTA-SLAM', sum(asset.size for asset in VISTA_ASSETS),
                 6, VISTA_REPO, VISTA_REVISION, VISTA_ASSETS),
    DEPTH_V2: Model('depth', 'Depth Anything V2 Metric Indoor Small', 99173660, 1, DEPTH_MODEL, V2_REVISION, (
        weights(DEPTH_MODEL, V2_REVISION, 'model.safetensors', 99173660,
                'e990eb82fbf11b05b7813261196a2b841bdcf5a05f64396724a8987fa90504a3'),
        weights(DEPTH_MODEL, V2_REVISION, 'config.json', 1063,
                '0b1d9fc591693d16b864249a9cfcf5264c8c00be999c1643fa1c64dc105d55f8'),
        weights(DEPTH_MODEL, V2_REVISION, 'preprocessor_config.json', 437,
                '533b16a60445d7cab5086d39b45f92be45624b977972c62d5984d93e98366063'),
    )),
    VIDEO_DEPTH: Model('depth', 'Metric Video Depth Anything Small', 116444063, 2, VIDEO_REPO, VIDEO_REVISION, (
        weights(VIDEO_REPO, VIDEO_REVISION, VIDEO_WEIGHT, 116444063,
                '3c28432b4e1f0d7bb31cad5151b6313b49457db5aa58d82e85bfb0f8b1311b33'),
        source('DepthAnything/Video-Depth-Anything', '4f5ae23172ba60fd7bc11ef671cca678842c7072', 7905704,
               '012dc88e5feb7e51f5794f9b8013f4c786aa3d61c60b8e0c3c5a45e1e0feb7c5', 'video_depth_anything', 'utils'),
        EINOPS, EASYDICT,
    ), ('video_depth_anything/video_depth_stream.py', 'video_depth_anything/util/transform.py',
        'utils/util.py', 'einops/__init__.py', 'easydict/__init__.py')),
    MOGE: Model('depth', 'MoGe-2 Small', 140550416, 2, MOGE_REPO, MOGE_REVISION, (
        weights(MOGE_REPO, MOGE_REVISION, MOGE_WEIGHT, 140550416,
                '79a16621928c2bf0ed04659218c55c01075e950507f40bb3332fb4c873d3e1dc'),
        source('microsoft/MoGe', '07444410f1e33f402353b99d6ccd26bd31e469e8', 11064671,
               '00a3f6b908ce4a9a1aa314e590dd1596d482511c1b2fa204fa8dade3b1cffd98', 'moge'),
        source('EasternJournalist/utils3d', '3fab839f0be9931dac7c8488eb0e1600c236e183', 172304,
               'ad4dbdf7605c31806194374189051cf4c82a3b6bd186ec2bc9b53c81d301d520', 'utils3d'),
    ), ('moge/model/v2.py', 'utils3d/__init__.py')),
}


def catalog():
    return [dict(id=key, kind=model.kind, name=model.name, size_bytes=model.size, vram_gb=model.vram_gb)
            for key, model in CATALOG.items()]


def model_for(key, kind=None):
    model = CATALOG.get(key) if isinstance(key, str) else None
    if model is None or (kind is not None and model.kind != kind):
        raise ValueError('invalid_reconstruction_model')
    return model


def cached_asset(model, asset):
    if asset.packages:
        return None
    from huggingface_hub import try_to_load_from_cache
    cached = try_to_load_from_cache(model.repo, asset.name, revision=model.revision)
    if isinstance(cached, str):
        return Path(cached)
    return None


def ready_file(path, size=None):
    try:
        return path.is_file() and (path.stat().st_size == size if size is not None else path.stat().st_size > 0)
    except OSError:
        return False


def ready_install(folder, model):
    return (all(ready_file(folder / asset.name, asset.size) for asset in model.assets if not asset.packages)
            and all(ready_file(folder / 'runtime' / name) for name in model.runtime_files))


def depth_location(key, root=MODEL_ROOT):
    return asset_location(key, model_for(key, 'depth'), root)


def pose_location(key, root=MODEL_ROOT):
    model = model_for(key, 'pose')
    return asset_location(key, model, root) if model.assets else None


def asset_location(key, model, root):
    folder = root / key / model.install_version
    if ready_install(folder, model):
        return folder
    # Transformers already uses this exact cache for the built-in V2 model.
    if not model.runtime_files:
        paths = [cached_asset(model, asset) for asset in model.assets]
        if all(path and ready_file(path, asset.size) for path, asset in zip(paths, model.assets)):
            return paths[0].parent
    return None


def copy_asset(model, asset, target, report):
    cached = cached_asset(model, asset)
    digest = sha256()
    received = 0
    if cached and not ready_file(cached, asset.size):
        cached = None
    with (cached.open('rb') if cached else urlopen(asset.url, timeout=45)) as reader, target.open('wb') as writer:
        for chunk in iter(lambda: reader.read(512 * 1024), b''):
            received += len(chunk)
            if received > asset.size:
                raise OSError('unexpected_model_file_size')
            digest.update(chunk)
            writer.write(chunk)
            report(received)
    if received != asset.size or digest.hexdigest() != asset.checksum:
        raise OSError('model_checksum_mismatch')


def extract_packages(archive, runtime, asset):
    # Only verified archives and declared package paths may populate the runtime.
    with ZipFile(archive) as zipped:
        for member in zipped.infolist():
            path = PurePosixPath(member.filename)
            parts = path.parts[1:] if asset.strip_root else path.parts
            if member.is_dir() or not parts or parts[0] not in asset.packages:
                continue
            if any(part in {'..', '/'} or '\\' in part or ':' in part for part in path.parts):
                raise OSError('invalid_model_archive_path')
            target = runtime.joinpath(*parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(zipped.read(member))


def install_assets(key, root, progress):
    model = model_for(key)
    target = root / key / model.install_version
    target.parent.mkdir(parents=True, exist_ok=True)
    total = sum(asset.size for asset in model.assets)
    completed = 0
    with TemporaryDirectory(prefix='.download-', dir=target.parent) as temporary:
        staging = Path(temporary) / 'install'
        staging.mkdir()
        for asset in model.assets:
            path = staging / asset.name
            copy_asset(model, asset, path, lambda received: progress(min(99, (completed + received) * 100 // total)))
            completed += asset.size
            if asset.packages:
                extract_packages(path, staging / 'runtime', asset)
                path.unlink()
        if not ready_install(staging, model):
            raise OSError('model_runtime_incomplete')
        # Publish only a complete install. A second local console may have
        # published this revision while the first was downloading.
        try:
            staging.rename(target)
        except OSError:
            if ready_install(target, model):
                return
            previous = Path(temporary) / 'incomplete'
            try:
                target.rename(previous)
            except FileNotFoundError:
                pass
            try:
                staging.rename(target)
            except OSError:
                if not ready_install(target, model):
                    raise


def require_pose_image(image):
    from apps.launcher import docker_assets
    from modules.mapping.localization.orbslam3_live import require_slam_image
    if image == docker_assets.ORB_IMAGE:
        require_slam_image()
        return
    require_slam_image(image)
    installed = docker_assets.image_source(image)
    if installed == docker_assets.source_hash(POSE_SOURCE):
        return
    if installed in COMPATIBLE_POSE_IMAGES:
        dockerfile = (POSE_SOURCE / 'Dockerfile').read_bytes().replace(b'\r\n', b'\n')
        if sha256(dockerfile).hexdigest() == PREVIEW_RUNTIME_DOCKERFILE:
            return
    raise RuntimeError('slam_image_missing')


def install_pose(key, root, progress):
    from apps.launcher import docker_assets
    model = model_for(key, 'pose')
    images = POSE_IMAGES[key]
    total = len(images) + bool(model.assets)
    with POSE_PREPARATION:
        for index, image in enumerate(images):
            try:
                require_pose_image(image)
            except RuntimeError as error:
                if str(error) != 'slam_image_missing':
                    raise
                target = BUILD_TARGETS.get(image)
                build_pose_image(image, POSE_SOURCE if target else docker_assets.ORB_SOURCE, target,
                                 lambda percent: progress((index * 100 + percent) // total))
            progress(min(99, (index + 1) * 100 // total))
        if model.assets and pose_location(key, root) is None:
            install_assets(key, root, lambda percent: progress((len(images) * 100 + percent) // total))


def build_pose_image(image, source_path, target, progress):
    from apps.launcher import docker_assets
    total = sum(line.startswith(('COPY ', 'RUN ')) for line in (source_path / 'Dockerfile').read_text().splitlines())
    operations, completed = set(), set()
    command = docker_assets.image_build_command(image, source_path, docker_assets.source_hash(source_path))
    if target:
        command[-1:-1] = ['--target', target]
    with subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, encoding='utf-8', errors='replace',
                          creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0)) as build:
        for line in build.stdout:
            step = re.match(r'^(#\d+) (?:\[.*?\] (RUN |COPY )|(DONE|CACHED)\b)', line)
            if not step:
                continue
            if step[2]:
                operations.add(step[1])
            elif step[1] in operations:
                completed.add(step[1])
                progress(min(99, len(completed) * 100 // max(1, total)))
        if build.wait() != 0:
            raise OSError('slam_build_failed')


class ReconstructionModels:
    """Share one download per model and broadcast its progress to local tabs."""
    def __init__(self, root=MODEL_ROOT):
        self.root = Path(root)
        self.changed = threading.Condition()
        self.jobs = {}

    def local_status(self, key):
        model = model_for(key)
        if model.kind == 'pose':
            from apps.launcher.docker_assets import probe
            if key != ORB and not probe(['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'], timeout=4):
                return dict(status='blocked', error='pose_gpu_unavailable')
            try:
                for image in POSE_IMAGES[key]:
                    require_pose_image(image)
            except RuntimeError as error:
                return dict(status='missing') if str(error) == 'slam_image_missing' else dict(status='blocked', error=str(error))
            if model.assets and pose_location(key, self.root) is None:
                return dict(status='missing')
        elif depth_location(key, self.root) is None:
            return dict(status='missing')
        return dict(status='ready', percent=100)

    def status(self, key):
        model_for(key)
        with self.changed:
            active = self.jobs.get(key)
            if active and active['status'] == 'downloading':
                return active
        status = self.local_status(key)
        with self.changed:
            active = self.jobs.get(key)
            if active and (active['status'] == 'downloading'
                           or (active['status'] == 'failed' and status['status'] == 'missing')):
                return active
        return status

    def update(self, key, **state):
        with self.changed:
            self.jobs[key] = state
            self.changed.notify_all()

    def download(self, key):
        model = model_for(key)
        with self.changed:
            observed = self.jobs.get(key)
        status = self.status(key)
        if status['status'] not in {'missing', 'failed'}:
            return status
        with self.changed:
            active = self.jobs.get(key)
            if active and (active['status'] == 'downloading'
                           or (active is not observed and active['status'] == 'ready')):
                return active
            method = 'build' if model.kind == 'pose' else 'transfer'
            pending = dict(status='downloading', percent=0, method=method)
            self.update(key, **pending)

            def work():
                def report(percent):
                    with self.changed:
                        if percent > self.jobs[key]['percent']:
                            self.update(key, status='downloading', percent=percent, method=method)
                try:
                    if method == 'build':
                        install_pose(key, self.root, report)
                    else:
                        install_assets(key, self.root, report)
                    if self.local_status(key)['status'] != 'ready':
                        raise OSError('model_runtime_incomplete')
                except Exception:
                    logging.getLogger(__name__).exception('Model preparation failed for %s', key)
                    self.update(key, status='failed', error='slam_build_failed' if method == 'build' else 'model_download_failed')
                else:
                    self.update(key, status='ready', percent=100)
            threading.Thread(target=work, name=f'prepare-{key}', daemon=True).start()
            return pending

    def events(self, key):
        state = self.status(key)
        yield state
        while state['status'] == 'downloading':
            with self.changed:
                updated = self.changed.wait_for(lambda: self.jobs.get(key) is not state, timeout=8)
                if updated:
                    state = self.jobs[key]
            yield state if updated else None
