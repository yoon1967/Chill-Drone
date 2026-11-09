"""통합 실행기(apps/launcher): 1·2번만, 묻지 않고 모두 켜기, 드론 포트 충돌, 안전한 끄기. 드론·창·PowerShell 없이."""
from pathlib import Path
import os
import tempfile
import subprocess
import threading
import unittest
from unittest.mock import patch

from apps.launcher import cli, drone, winproc
from apps.launcher.winproc import Proc

def tello(mode, **extra):
    return {'type': 'drone_state', 'source': 'tello', 'mode': mode, 'connected': True, **extra}


def done(stdout=b''):
    return subprocess.CompletedProcess([], 0, stdout=stdout, stderr=b'')


class SafetyTests(unittest.TestCase):
    def test_only_a_real_drone_in_the_air_blocks_stopping(self):
        self.assertFalse(drone.airborne(None))
        self.assertFalse(drone.airborne({'type': 'drone_state', 'source': 'fake', 'mode': 'manual'}))
        self.assertFalse(drone.airborne({'type': 'drone_state', 'source': 'replay', 'mode': 'auto'}))
        for mode in ('grounded', 'disconnected', 'emergency'):
            self.assertFalse(drone.airborne(tello(mode)), mode)
        for mode in ('manual', 'auto', 'paused', 'landing'):
            self.assertTrue(drone.airborne(tello(mode)), mode)
        self.assertTrue(drone.airborne(tello('grounded', flight_busy='takeoff')))


class CommandLineTests(unittest.TestCase):
    def test_matching_ignores_quotes_and_path_separators(self):
        quoted = Proc(1, 0, r'"C:\v\python.exe" "-B" "-m" "modules.perception.inference.gateway_ai_client" '
                            r'"--gateway" "http://127.0.0.1:8010"')
        plain = Proc(2, 0, r'python -B modules\autonomy\tello_link\gateway_server.py --port 8030 --fake')
        self.assertEqual(drone.match([quoted, plain], 'gateway_ai_client', '--gateway http://127.0.0.1:8010'), [quoted])
        self.assertEqual(drone.match([quoted, plain], 'modules/autonomy/tello_link/gateway_server.py'), [plain])
        self.assertEqual(drone.match([quoted, plain], 'gateway_server.py', exclude=('--fake',)), [])
        self.assertEqual(drone.gateway_port(plain.cmd), 8030)
        self.assertEqual(drone.gateway_port('python gateway_server.py'), 8010)

    def test_wifi_names_skip_bssid_lines(self):
        text = ('    Name : Wi-Fi 2\n    SSID                   : TELLO-59FA6D\n'
                '    BSSID                  : 60:60:1f:59:fa:6d\n  SSID : 2F-1_SSAFY \n')
        self.assertEqual(winproc.wifi_ssids(text), ['TELLO-59FA6D', '2F-1_SSAFY'])

    def test_services_hide_their_console_and_write_logs_without_cmd(self):
        argv = ['python.exe', '-B', 'a&b.py']
        with tempfile.TemporaryDirectory() as directory, patch.object(winproc.subprocess, 'Popen') as popen:
            root = Path(directory)
            winproc.start_service('gateway-8010', root, argv)
            self.assertTrue(winproc.service_log(root, 'gateway-8010').is_file())
        self.assertEqual(popen.call_args.args[0], argv)
        self.assertEqual(popen.call_args.kwargs['stdin'], subprocess.DEVNULL)
        self.assertEqual(popen.call_args.kwargs['stderr'], subprocess.STDOUT)
        if os.name == 'nt':
            self.assertEqual(popen.call_args.kwargs['creationflags'], winproc.CREATE_NEW_CONSOLE)
            self.assertEqual(popen.call_args.kwargs['startupinfo'].wShowWindow, subprocess.SW_HIDE)

    def test_mona_runs_in_flight_mode(self):
        cmd = drone.mona_cmd()
        self.assertNotIn('-AutoStart', cmd)   # 경로는 운용자가 이륙 뒤 "자율"을 눌러야 시작
        self.assertEqual(cmd[cmd.index('-Seconds') + 1], '3600')


class FakeModeTests(unittest.TestCase):
    def test_fake_drone_starts_everything_without_asking(self):
        with patch.object(drone, 'show_code'), \
                patch.object(drone.prepare, 'ensure_all', return_value=True) as ready, \
                patch.object(drone, 'state', return_value=None), \
                patch.object(drone, 'wait_for', return_value=True), \
                patch('modules.mapping.data.model_assets.person_checkpoint', return_value=Path('yolo.pt')), \
                patch.object(winproc, 'port_owners', return_value=[]), \
                patch.object(winproc, 'processes', return_value=[]), \
                patch.object(winproc, 'get_json', return_value=None), \
                patch.object(drone, 'start_console') as console, \
                patch.object(winproc, 'start_service') as window:
            self.assertEqual(drone.run_fake(no_browser=True, interactive=False), 0)
        ready.assert_called_once_with(mona=False)   # 영상 복원용 지도 콘솔(8796) 웹 의존성까지
        argvs = [c.args[2] for c in window.call_args_list]
        self.assertEqual(len(argvs), 4)
        self.assertEqual(argvs[0][-3:], ['--port', '8030', '--fake'])
        self.assertEqual(argvs[1][-4:], ['--gateway', 'http://127.0.0.1:8030', '--yolo-model', 'yolo.pt'])
        self.assertEqual(argvs[2][-3:], ['--port', '8795', '--start'])
        self.assertEqual(argvs[3][-4:], ['--map', 'http://127.0.0.1:8795', '--gateway', 'http://127.0.0.1:8030'])
        console.assert_called_once_with('http://127.0.0.1:8030', port=8796)

    def test_running_programs_are_not_started_twice(self):
        running = [Proc(5, 0, 'python -m modules.perception.inference.gateway_ai_client --gateway http://127.0.0.1:8030'),
                   Proc(6, 0, 'python gateway_map_bridge.py --map http://127.0.0.1:8795 --gateway http://127.0.0.1:8030')]
        with patch.object(drone, 'show_code'), patch.object(drone.prepare, 'ensure_all', return_value=True), \
                patch.object(drone, 'state', return_value={'type': 'drone_state', 'source': 'fake'}), \
                patch.object(winproc, 'processes', return_value=running), \
                patch.object(winproc, 'get_json', return_value={'enabled': True}), \
                patch.object(winproc, 'start_service') as window:
            self.assertEqual(drone.run_fake(no_browser=True, interactive=False), 0)
        window.assert_not_called()


class RealModeTests(unittest.TestCase):
    def run_real(self, ready=True, first_state=None, owners=()):
        states = iter([first_state])
        console_up = iter([None])   # 처음 확인은 꺼짐, 켠 뒤에는 응답

        def get_json(target, timeout=2.0):
            if target.endswith('/api/drone/state'):
                return next(states, tello('grounded', battery_pct=70, temp_high_c=60))
            return next(console_up, {'enabled': True})
        with patch.object(drone, 'show_code'), patch.object(drone, 'firewall_hint'), \
                patch.object(drone.prepare, 'ensure_all', return_value=ready), \
                patch.object(winproc, 'wifi_ssids', return_value=['TELLO-59FA6D']), \
                patch.object(winproc, 'get_json', side_effect=get_json), \
                patch.object(winproc, 'port_owners', return_value=list(owners)), \
                patch.object(winproc, 'processes', return_value=[Proc(4242, 0, 'python check_server.py')]), \
                patch('modules.mapping.data.model_assets.person_checkpoint', return_value=Path('yolo.pt')), \
                patch.object(drone, 'start_mona_when_connected') as mona, \
                patch.object(drone.mapkeeper, 'MapKeeper') as keeper, \
                patch.object(drone, 'open_page') as page, \
                patch.object(winproc, 'start_service') as window:
            code = drone.run_real(interactive=False)
        return code, [c.args[2] for c in window.call_args_list], mona, keeper, page

    def test_everything_is_started_without_asking(self):
        code, argvs, mona, keeper, page = self.run_real()
        self.assertEqual(code, 0)
        self.assertEqual(len(argvs), 4)
        self.assertEqual(argvs[0][-2:], ['--port', '8010'])
        self.assertNotIn('--fake', argvs[0])
        self.assertIn('modules.perception.inference.gateway_ai_client', argvs[1])
        self.assertEqual(argvs[2][-2:], ['--tello-gateway', 'http://127.0.0.1:8010'])
        self.assertEqual(argvs[3][-4:], ['--map', 'http://127.0.0.1:8790', '--gateway', 'http://127.0.0.1:8010'])
        mona.assert_called_once()
        keeper.assert_called_once_with('http://127.0.0.1:8010', 'http://127.0.0.1:8790', drone.say)
        self.assertEqual(page.call_args.args[0], 'http://127.0.0.1:8010/#/live')

    def test_failed_preparation_stops_before_starting_anything(self):
        code, argvs, mona, keeper, page = self.run_real(ready=False)
        self.assertEqual((code, argvs), (1, []))
        mona.assert_not_called()
        keeper.assert_not_called()
        page.assert_not_called()

    def test_drone_ports_in_use_stop_before_starting_anything(self):
        code, argvs, mona, _, _ = self.run_real(owners=[('udp', 8889, 4242)])
        self.assertEqual((code, argvs), (1, []))
        mona.assert_not_called()

    def test_non_real_gateway_on_8010_is_refused(self):
        code, argvs, _, _, _ = self.run_real(first_state={'type': 'drone_state', 'source': 'fake'})
        self.assertEqual((code, argvs), (1, []))


class MonaStarterTests(unittest.TestCase):
    def test_mona_starts_once_after_the_drone_connects(self):
        states = iter([tello('disconnected', connected=False), tello('grounded')])
        stop = threading.Event()
        with patch.object(drone, 'state', side_effect=lambda base: next(states, tello('grounded'))), \
                patch.object(drone, 'mona_running', return_value=False), \
                patch.object(winproc, 'start_service') as window:
            window.return_value.poll.return_value = 0
            drone.start_mona_when_connected('http://127.0.0.1:8010', stop, interval=0.01).join(2)
        self.assertEqual(window.call_count, 1)
        self.assertEqual(window.call_args.args[2], drone.mona_cmd())


class StopTests(unittest.TestCase):
    def run_stop(self, procs, states, keeper=None):
        sent = []
        stop_event = threading.Event()
        with patch.object(drone.signal, 'signal'), \
                patch.object(winproc, 'processes', return_value=procs), \
                patch.object(drone, 'state', side_effect=lambda base: states.get(base)), \
                patch.object(drone, 'mona_running', return_value=False), \
                patch.object(winproc, 'alive', side_effect=lambda pid: pid not in sent), \
                patch.object(winproc, 'send_ctrl_c', side_effect=sent.append), \
                patch.object(winproc, 'kill_tree') as kill:
            code = drone.run_stop(keeper=keeper, stop_event=stop_event)
        return code, sent, kill, stop_event

    def test_nothing_is_stopped_while_a_real_drone_is_in_the_air(self):
        procs = [Proc(10, 0, 'python gateway_server.py --port 8010'),
                 Proc(20, 0, 'python -m modules.perception.inference.gateway_ai_client --gateway http://127.0.0.1:8010')]
        keeper = unittest.mock.Mock()
        code, sent, kill, stop_event = self.run_stop(procs, {'http://127.0.0.1:8010': tello('auto')}, keeper)
        self.assertEqual((code, sent), (1, []))
        kill.assert_not_called()
        keeper.stop.assert_not_called()
        self.assertFalse(stop_event.is_set())

    def test_map_console_gateway_in_the_air_also_blocks(self):
        procs = [Proc(30, 0, r'python C:\repo\apps\console\serve.py --enable-camera --port 8790 '
                             r'--tello-gateway http://127.0.0.1:8011')]
        code, sent, _, _ = self.run_stop(procs, {'http://127.0.0.1:8011': tello('manual')})
        self.assertEqual((code, sent), (1, []))

    def test_on_the_ground_map_is_saved_then_helpers_stop_first_gateway_last(self):
        procs = [Proc(10, 0, 'python gateway_server.py --port 8010'),
                 Proc(21, 0, 'python gateway_map_bridge.py --map http://127.0.0.1:8790 --gateway http://127.0.0.1:8010'),
                 Proc(20, 0, 'python -m modules.perception.inference.gateway_ai_client --gateway http://127.0.0.1:8010')]
        keeper = unittest.mock.Mock()
        code, sent, kill, stop_event = self.run_stop(procs, {'http://127.0.0.1:8010': tello('grounded')}, keeper)
        self.assertEqual(code, 0)
        keeper.stop.assert_called_once_with()
        self.assertTrue(stop_event.is_set())
        self.assertEqual(sent, [20, 21, 10])
        kill.assert_not_called()


class CliTests(unittest.TestCase):
    def setUp(self):
        self.host = self.enterContext(patch.object(cli.prepare, 'check_host', return_value=True))
        self.enterContext(patch.object(cli.sys, 'prefix', 'test-venv'))

    def test_only_fake_and_real_commands(self):
        with patch.object(cli.drone, 'run_fake', return_value=0) as fake:
            self.assertEqual(cli.main(['fake', '--no-browser']), 0)
        fake.assert_called_once_with(no_browser=True)
        with patch.object(cli.drone, 'run_real', return_value=0) as real:
            self.assertEqual(cli.main(['real', '--phone']), 0)
        real.assert_called_once_with(phone=True, no_browser=False)
        for removed in (['map'], ['prepare'], ['stop'], ['team-autonomy', 'docker'], ['real', '--map']):
            with self.subTest(removed=removed), self.assertRaises(SystemExit):
                cli.main(removed)

    def test_menu_has_two_choices(self):
        self.assertEqual([key for key, _, _ in cli.MENU], ['1', '2'])
        for choice, target, options in (('1', 'run_fake', {'no_browser': False}),
                                        ('2', 'run_real', {'phone': False, 'no_browser': False})):
            with self.subTest(choice=choice), patch.object(cli.drone, 'show_code'), \
                    patch.object(cli.drone, target, return_value=0) as run, patch('builtins.input', return_value=choice):
                self.assertEqual(cli.main([]), 0)
            run.assert_called_once_with(**options)

    def test_failed_host_check_does_not_initialize_project_or_start_services(self):
        self.host.return_value = False
        with patch.object(cli.sys, 'prefix', cli.sys.base_prefix), \
                patch.object(cli.prepare, 'create_project_environment') as venv, \
                patch.object(cli.drone, 'run_real') as real:
            self.assertEqual(cli.main(['real', '--no-browser']), 1)
        self.host.assert_called_once_with(mona=True)
        venv.assert_not_called()
        real.assert_not_called()


if __name__ == '__main__':
    unittest.main()
