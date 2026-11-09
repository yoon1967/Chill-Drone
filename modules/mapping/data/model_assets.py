"""Resolve prepared weights from disk without invoking a model hub download."""
from pathlib import Path

DEPTH_MODEL = 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf'
SCENE_MODEL = 'nvidia/segformer-b0-finetuned-ade-512-512'
PERSON_MODEL = 'yolo26n-seg.pt'


class ModelAssetsMissing(RuntimeError):
    def __init__(self, name):
        super().__init__(f'model_assets_missing: {name} · 모델 파일이 준비되지 않았습니다. '
            '인터넷 연결 상태에서 scripts/start.cmd prepare를 실행한 뒤 드론에 연결하세요. '
            '지도 실행 중에는 모델을 다운로드하지 않습니다.')


def cached_snapshot(model_id):
    from huggingface_hub import try_to_load_from_cache
    config = try_to_load_from_cache(model_id, 'config.json')
    if not isinstance(config, str):
        raise ModelAssetsMissing(model_id)
    # Keep the snapshot path: resolving a symlink would select the blobs folder.
    folder = Path(config).parent
    ready = lambda name: (folder/name).is_file() and (folder/name).stat().st_size > 0
    if not all(ready(name) for name in ('config.json','preprocessor_config.json')) or not any(
            ready(name) for name in ('model.safetensors','pytorch_model.bin')):
        raise ModelAssetsMissing(model_id)
    return folder


def person_checkpoint():
    import torch
    return Path(torch.hub.get_dir()) / 'checkpoints' / PERSON_MODEL


def require_local_models():
    result = dict(depth=cached_snapshot(DEPTH_MODEL), scene=cached_snapshot(SCENE_MODEL), person=person_checkpoint())
    if not result['person'].is_file() or result['person'].stat().st_size == 0:
        raise ModelAssetsMissing(PERSON_MODEL)
    return result
