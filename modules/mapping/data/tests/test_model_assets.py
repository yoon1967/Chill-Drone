from pathlib import Path
import tempfile
import types
import os
import unittest
from unittest.mock import patch, MagicMock

from modules.mapping.data.model_assets import cached_snapshot, ModelAssetsMissing


class ModelAssetsTests(unittest.TestCase):
    def test_inference_worker_enables_offline_mode_before_model_imports(self):
        from modules.mapping.data.depth_worker import _serve
        connection = MagicMock(); connection.recv.return_value = ('shutdown',)
        with patch.dict(os.environ), patch('modules.mapping.data.depth_worker.signal.signal'):
            _serve(connection, MagicMock(), MagicMock())
            self.assertEqual(os.environ['HF_HUB_OFFLINE'], '1')
            self.assertEqual(os.environ['TRANSFORMERS_OFFLINE'], '1')
        connection.close.assert_called_once()

    def test_prepared_snapshot_uses_its_local_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            for name in ('config.json','preprocessor_config.json','model.safetensors'):
                (folder/name).write_bytes(b'local file')
            hub = types.SimpleNamespace(try_to_load_from_cache=lambda *args: str(folder/'config.json'))
            with patch.dict('sys.modules', {'huggingface_hub': hub}):
                self.assertEqual(cached_snapshot('scene-model'), folder)

    def test_missing_or_partial_cache_fails_without_starting_a_download(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            (folder/'config.json').write_bytes(b'{}')
            for config in (None, str(folder/'config.json')):
                hub = types.SimpleNamespace(try_to_load_from_cache=lambda *args: config)
                with self.subTest(config=config), patch.dict('sys.modules', {'huggingface_hub': hub}):
                    with self.assertRaisesRegex(ModelAssetsMissing, '--prepare-models'):
                        cached_snapshot('scene-model')
