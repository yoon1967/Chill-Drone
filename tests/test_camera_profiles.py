import json
from pathlib import Path
import tempfile
import unittest

from modules.mapping.data.camera_profiles import CameraProfiles, BUILTIN_ID, validate_intrinsics


class CameraProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = CameraProfiles(Path(self.temp.name))
        self.camera = {'width': 640, 'height': 480, 'fx': 600., 'fy': 601., 'cx': 320., 'cy': 240., 'distortion': [0, 0, 0, 0]}
        self.body = {'camera': self.camera, 'name': 'Test camera', 'camera_identity': 'camera A',
            'image_geometry': 'full frame landscape', 'input_modes': ['camera', 'file'], 'confirmed': True}

    def selection(self, camera):
        return {k: camera['profile'][k] for k in ('id', 'version', 'camera_sha256')} | {'confirmed': True}

    def test_save_restart_export_and_reregister_preserve_values(self):
        camera = self.store.register(self.body)
        key = camera['profile']['id']
        self.store.set_default('camera', key)
        restarted = CameraProfiles(self.store.root)
        self.assertEqual(restarted.list()['defaults']['camera'], key)
        self.assertEqual(restarted.get(key), camera)
        imported = restarted.register({**self.body, 'name': 'Imported', 'camera': json.loads(json.dumps(camera))})
        self.assertEqual(imported['fx'], camera['fx'])
        self.assertEqual(imported['profile']['imported_from']['id'], key)
        self.assertNotEqual(imported['profile']['id'], key)

    def test_duplicate_retry_is_idempotent_and_new_name_never_overwrites(self):
        first = self.store.register(self.body)
        self.assertEqual(self.store.register(self.body), first)
        second = self.store.register({**self.body, 'name': 'new version'})
        self.assertNotEqual(first['profile']['id'], second['profile']['id'])
        self.assertEqual(self.store.get(first['profile']['id']), first)

    def test_mode_binding_dimensions_and_explicit_confirmation_are_required(self):
        camera = self.store.register(self.body)
        selection = self.selection(camera)
        self.assertEqual(self.store.resolve(selection, 'file', 640, 480), camera)
        for mode, width, height, config in [('tello', 640, 480, selection), ('file', 1280, 960, selection),
                ('file', 640, 480, {**selection, 'confirmed': False}), ('file', 640, 480, {**selection, 'version': 2})]:
            with self.subTest(mode=mode, config=config), self.assertRaises(ValueError):
                self.store.resolve(config, mode, width, height)

    def test_bad_numbers_dimensions_distortion_and_unknown_paths_are_rejected(self):
        for values in ({'fx': float('nan')}, {'fy': True}, {'width': 640.5}, {'cx': -1}, {'distortion': [0, 0, 0]}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                validate_intrinsics({**self.camera, **values})
        with self.assertRaises(ValueError):
            self.store.get('../private')

    def test_builtin_remains_measured_default_and_cannot_be_overwritten(self):
        camera = self.store.get(BUILTIN_ID)
        self.assertTrue(camera['profile']['readonly'])
        self.assertEqual(camera['fx'], 927.5521214084725)
        self.assertEqual(self.store.list()['defaults']['tello'], BUILTIN_ID)
        with self.assertRaises(ValueError):
            self.store.set_default('camera', BUILTIN_ID)

    def test_tampered_profile_is_not_exposed_as_valid(self):
        camera = self.store.register(self.body)
        key = camera['profile']['id']
        path = self.store.root / key / 'camera.json'
        camera['fx'] += 1
        path.write_text(json.dumps(camera), encoding='utf-8')
        self.assertTrue(self.store.list()['warnings'])
        with self.assertRaises(ValueError):
            self.store.get(key)


if __name__ == '__main__':
    unittest.main()
