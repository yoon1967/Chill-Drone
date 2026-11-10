"""실행기 Docker 이미지 준비(apps/launcher/docker_assets.py): 이미지를 만든 코드(Dockerfile과 COPY·ADD한 파일)의
해시를 라벨로 붙이고, 지금 코드와 다르면 다시 빌드한다. 문서·시험 스크립트처럼 이미지에 안 들어가는 파일은 보지 않는다."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from apps.launcher import docker_assets


def write(root, files):
    for name, body in files.items():
        path = Path(root) / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(body.encode('utf-8') if isinstance(body, str) else body)


DOCKERFILE = """FROM python:3.12 AS base
COPY --from=builder /opt/tool /opt/tool
COPY requirements.txt ./
COPY --chown=root:root lock.json \\
     prepare.py ./
ADD ["data dir/", "/opt/data/"]
ADD https://example.invalid/model.bin /opt/
COPY runtime/ /opt/runtime/
COPY conf/*.yaml /opt/conf/
RUN python -c "print('COPY not_a_source /x')"
"""

CONTEXT = {
    'Dockerfile': DOCKERFILE,
    'requirements.txt': 'numpy\n',
    'lock.json': '{}',
    'prepare.py': 'print(1)\n',
    'data dir/a.txt': 'a',
    'runtime/main.py': 'print("run")\n',
    'runtime/tests/test_main.py': 'pass\n',
    'conf/a.yaml': 'a: 1\n',
    'conf/b.txt': 'not copied\n',
    'README.md': '# docs\n',
    'run.ps1': 'Write-Output hi\n',
}


class ImageInputsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        write(self.root, CONTEXT)

    def tearDown(self):
        self.tmp.cleanup()

    def names(self):
        return [path.relative_to(self.root).as_posix() for path in docker_assets.image_inputs(self.root)]

    def test_reads_copy_and_add_sources_only(self):
        self.assertEqual(self.names(), ['Dockerfile', 'conf/a.yaml', 'data dir/a.txt', 'lock.json', 'prepare.py',
                                        'requirements.txt', 'runtime/main.py', 'runtime/tests/test_main.py'])

    def test_hash_follows_image_files_but_not_docs_scripts_or_pycache(self):
        before = docker_assets.source_hash(self.root)
        write(self.root, {'README.md': '# changed\n', 'run.ps1': 'changed\n', 'conf/b.txt': 'changed\n',
                          'runtime/__pycache__/main.cpython-312.pyc': b'\x00\x01', 'runtime/old.pyc': b'\x02'})
        self.assertEqual(docker_assets.source_hash(self.root), before)
        for name, body in (('runtime/main.py', 'print("new")\n'), ('Dockerfile', DOCKERFILE + 'ENV A=1\n'),
                           ('lock.json', '{"rev": 2}'), ('runtime/new.py', 'x = 1\n')):
            with self.subTest(changed=name):
                write(self.root, {name: body})
                after = docker_assets.source_hash(self.root)
                self.assertNotEqual(after, before)
                before = after

    def test_missing_source_falls_back_to_whole_folder(self):   # 못 읽으면 덜 다시 빌드하기보다 더 다시 빌드한다
        write(self.root, {'Dockerfile': 'FROM scratch\nCOPY gone.txt /x\n'})
        self.assertIn('README.md', self.names())

    def test_real_build_folders(self):   # 저장소의 MoNA·ORB-SLAM3 Dockerfile을 실제로 읽는다
        mona = {p.relative_to(docker_assets.MONA_SOURCE).as_posix() for p in docker_assets.image_inputs(docker_assets.MONA_SOURCE)}
        self.assertTrue({'Dockerfile', 'requirements.txt', 'upstream.lock.json', 'prepare_workspace.py',
                         'runtime/gateway_bridge.py', 'runtime/live.sh'} <= mona, mona)
        self.assertFalse({'README.md', 'run.ps1', 'export_da2_onnx.py'} & mona)
        orb = {p.relative_to(docker_assets.ORB_SOURCE).as_posix() for p in docker_assets.image_inputs(docker_assets.ORB_SOURCE)}
        self.assertTrue({'Dockerfile', 'CMakeLists.txt', 'bridge.cpp', 'safe_sim3.hpp', 'apply_guard.cmake'} <= orb, orb)


class EnsureImageTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        write(self.root, CONTEXT)
        self.want = docker_assets.source_hash(self.root)

    def tearDown(self):
        self.tmp.cleanup()

    def ensure(self, labels, build_ok=True):
        """labels: None = 이미지 없음, dict/None JSON = 이미지 라벨."""
        reply = None if labels == 'missing' else json.dumps(labels)
        with patch.object(docker_assets, 'probe', return_value=reply) as probe, \
                patch.object(docker_assets.subprocess, 'call', return_value=0 if build_ok else 1) as build, \
                patch('builtins.print') as said:
            ok = docker_assets.ensure_image('img:local', self.root)
        self.assertEqual(probe.call_args.args[0][:4], ['docker', 'image', 'inspect', 'img:local'])
        return ok, build, ' '.join(str(c.args[0]) for c in said.call_args_list)

    def test_same_code_is_not_rebuilt(self):
        ok, build, _ = self.ensure({docker_assets.SOURCE_LABEL: self.want, 'other': 'x'})
        self.assertTrue(ok)
        build.assert_not_called()

    def test_missing_changed_or_unlabeled_image_is_built_with_the_label(self):
        for labels, says in (('missing', '빌드합니다'), ({docker_assets.SOURCE_LABEL: 'old'}, '지금 코드와 달라'),
                             (None, '라벨 없음'), ({'org.opencontainers.image.version': '22.04'}, '라벨 없음')):
            with self.subTest(labels=labels):
                ok, build, said = self.ensure(labels)
                self.assertTrue(ok)
                command = build.call_args.args[0]
                self.assertEqual(command[:2], ['docker', 'build'])
                self.assertIn(f'{docker_assets.SOURCE_LABEL}={self.want}', command)
                self.assertEqual(command[-3:], ['-t', 'img:local', str(self.root)])
                self.assertIn(says, said)

    def test_failed_rebuild_keeps_the_old_image_but_missing_image_fails(self):
        ok, _, said = self.ensure({docker_assets.SOURCE_LABEL: 'old'}, build_ok=False)
        self.assertTrue(ok)
        self.assertIn('이전 코드로 만든 이미지로 켭니다', said)
        ok, _, _ = self.ensure('missing', build_ok=False)
        self.assertFalse(ok)


if __name__ == '__main__':
    unittest.main()
