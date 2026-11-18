"""Recovery must use the recorded source identity and must preserve existing data."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from modules.mapping.data.sensor_recovery import recover_empty_sensor_log


class SensorRecoveryTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        self.session, self.gateway = root / 'capture', root / 'gateway'
        self.session.mkdir()
        run = self.gateway / 'tello_source'
        run.mkdir(parents=True)
        self.source = run / 'sensor.jsonl'
        (run / 'manifest.json').write_text(json.dumps({'run_id': 'tello_source'}))
        self.samples = [{'received_mono': 10. + i / 10, 'sequence': i + 1,
                         'raw': {'yaw': i}, 'si': {}, 'valid': True} for i in range(5)]
        self.source.write_text(''.join(json.dumps(row) + '\n' for row in self.samples))
        (self.session / 'sensor.jsonl').write_bytes(b'')
        (self.session / 'frames.jsonl').write_text(''.join(json.dumps({'event': 'input',
            'source': {'run_id': 'tello_source', 'timestamp_s': stamp}}) + '\n' for stamp in (10.1, 10.3)))
        self.manifest = {'recording': {'finalized': True, 'log_records': {'sensor': 0}}, 'asset_sha256': {}}
        self.write_manifest()

    def write_manifest(self):
        (self.session / 'session.json').write_text(json.dumps(self.manifest))

    def test_recovers_original_packets_and_updates_hash_and_provenance(self):
        original = self.source.read_bytes()
        result = recover_empty_sensor_log(self.session, self.gateway)
        target = self.session / 'sensor.jsonl'
        rows = [json.loads(row) for row in target.read_text().splitlines()]
        self.assertEqual(len(rows), 5)
        self.assertEqual([{k: row[k] for k in sample} for row, sample in zip(rows, self.samples)], self.samples)
        self.assertEqual(self.source.read_bytes(), original)
        manifest = json.loads((self.session / 'session.json').read_text())
        self.assertEqual(manifest['recording']['sensor_samples'], 5)
        self.assertEqual(manifest['asset_sha256']['sensor.jsonl'], hashlib.sha256(target.read_bytes()).hexdigest())
        self.assertEqual(result['source_sha256'], hashlib.sha256(original).hexdigest())
        self.assertEqual(manifest['capture_metadata']['sensor_recovery']['source_run_id'], 'tello_source')

    def test_never_overwrites_existing_sensor_records_or_an_active_capture(self):
        target = self.session / 'sensor.jsonl'
        target.write_bytes(b'user-records')
        with self.assertRaisesRegex(ValueError, 'existing_empty'):
            recover_empty_sensor_log(self.session, self.gateway)
        self.assertEqual(target.read_bytes(), b'user-records')
        target.write_bytes(b'')
        self.manifest['recording']['finalized'] = False
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, 'finalized'):
            recover_empty_sensor_log(self.session, self.gateway)
        self.assertEqual(target.read_bytes(), b'')

    def test_rejects_wrong_run_or_sensor_sequence_gaps(self):
        manifest = self.gateway / 'tello_source/manifest.json'
        manifest.write_text(json.dumps({'run_id': 'different_source'}))
        with self.assertRaisesRegex(ValueError, 'identity_mismatch'):
            recover_empty_sensor_log(self.session, self.gateway)
        manifest.write_text(json.dumps({'run_id': 'tello_source'}))
        self.samples.pop(2)
        self.source.write_text(''.join(json.dumps(row) + '\n' for row in self.samples))
        with self.assertRaisesRegex(ValueError, 'discontinuous'):
            recover_empty_sensor_log(self.session, self.gateway)
        self.assertEqual((self.session / 'sensor.jsonl').read_bytes(), b'')


if __name__ == '__main__':
    unittest.main()
