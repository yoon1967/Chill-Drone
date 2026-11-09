"""통합 실행기(apps/launcher): 안전한 끄기, 드론 포트 충돌, 띄울 명령, 메뉴·명령 연결. 드론·창·PowerShell 없이."""
from pathlib import Path
import subprocess
import sys
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

    def test_new_windows_quote_every_argument(self):
        with patch.object(winproc.subprocess, 'Popen') as popen:
            winproc.start_window('Chill-Drone 게이트웨이 (8010)', Path('C:/repo'), ['python.exe', '-B', 'a&b.py'])
        line = popen.call_args.args[0]
        self.assertTrue(line.startswith('cmd /d /c start "Chill-Drone 게이트웨이 (8010)" /D '))
        self.assertTrue(line.endswith('"python.exe" "-B" "a&b.py"'))


class FakeModeTests(unittest.TestCase):
    def test_fake_drone_starts_gateway_detection_map_and_bridge(self):
        with patch.object(drone, 'show_code'), \
                patch.object(drone, 'state', return_value=None), \
                patch.object(drone, 'wait_for', return_value=True), \
                patch.object(drone, 'detect_ready', return_value=(Path('yolo.pt'), None)), \
                patch.object(winproc, 'port_owners', return_value=[]), \
                patch.object(winproc, 'processes', return_value=[]), \
                patch.object(winproc, 'get_json', return_value=None), \
                patch.object(winproc, 'start_window') as window:
            self.assertEqual(drone.run_fake(no_browser=True), 0)
        argvs = [c.args[2] for c in window.call_args_list]
        self.assertEqual(len(argvs), 4)
        self.assertEqual(argvs[0][-3:], ['--port', '8030', '--fake'])
        self.assertEqual(argvs[1][-4:], ['--gateway', 'http://127.0.0.1:8030', '--yolo-model', 'yolo.pt'])
        self.assertEqual(argvs[2][-3:], ['--port', '8795', '--start'])
        self.assertEqual(argvs[3][-4:], ['--map', 'http://127.0.0.1:8795', '--gateway', 'http://127.0.0.1:8030'])

    def test_running_programs_are_not_started_twice(self):
        running = [Proc(5, 0, 'python -m modules.perception.inference.gateway_ai_client --gateway http://127.0.0.1:8030'),
                   Proc(6, 0, 'python gateway_map_bridge.py --map http://127.0.0.1:8795 --gateway http://127.0.0.1:8030')]
        with patch.object(drone, 'show_code'), \
                patch.object(drone, 'state', return_value={'type': 'drone_state', 'source': 'fake'}), \
                patch.object(winproc, 'processes', return_value=running), \
                patch.object(winproc, 'get_json', return_value={'enabled': True}), \
                patch.object(winproc, 'start_window') as window:
            self.assertEqual(drone.run_fake(no_browser=True), 0)
        window.assert_not_called()


class RealModeTests(unittest.TestCase):
    def common(self):
        return [patch.object(drone, 'show_code'), patch.object(winproc, 'wifi_ssids', return_value=['TELLO-59FA6D']),
                patch.object(drone, 'firewall_hint')]

    def test_drone_ports_in_use_stop_before_starting_anything(self):
        with self.common()[0], self.common()[1], self.common()[2], \
                patch.object(drone, 'state', return_value=None), \
                patch.object(winproc, 'port_owners', return_value=[('udp', 8889, 4242)]), \
                patch.object(winproc, 'processes', return_value=[Proc(4242, 0, 'python check_server.py')]), \
                patch.object(winproc, 'start_window') as window:
            self.assertEqual(drone.run_real(interactive=False, no_browser=True), 1)
        window.assert_not_called()

    def test_non_real_gateway_on_8010_is_refused(self):
        with self.common()[0], self.common()[1], self.common()[2], \
                patch.object(drone, 'state', return_value={'type': 'drone_state', 'source': 'fake'}), \
                patch.object(winproc, 'start_window') as window:
            self.assertEqual(drone.run_real(interactive=False, no_browser=True), 1)
        window.assert_not_called()

    def test_running_real_gateway_is_reused_and_choices_pick_the_preset(self):
        with self.common()[0], self.common()[1], self.common()[2], \
                patch.object(drone, 'state', return_value=tello('grounded', battery_pct=70, temp_high_c=60)), \
                patch.object(winproc, 'processes', return_value=[]), \
                patch.object(drone, 'start_mona') as mona, \
                patch.object(drone, 'open_page') as page, \
                patch.object(winproc, 'start_window') as window:
            self.assertEqual(drone.run_real(detect=False, map_=False, mona='hand', interactive=False), 0)
        window.assert_not_called()
        mona.assert_called_once_with(True)
        self.assertEqual(page.call_args.args[0], 'http://127.0.0.1:8010/?preset=autonomy#/live')

    def test_new_real_gateway_uses_port_8010_without_fake(self):
        states = iter([None])
        with self.common()[0], self.common()[1], self.common()[2], \
                patch.object(drone, 'state', side_effect=lambda base: next(states, tello('grounded'))), \
                patch.object(winproc, 'port_owners', return_value=[]), \
                patch.object(winproc, 'processes', return_value=[]), \
                patch.object(drone, 'open_page'), \
                patch.object(winproc, 'start_window') as window:
            self.assertEqual(drone.run_real(detect=False, map_=False, interactive=False, phone=False), 0)
        argv = window.call_args.args[2]
        self.assertEqual(argv[-2:], ['--port', '8010'])
        self.assertNotIn('--fake', argv)


class StopTests(unittest.TestCase):
    def run_stop(self, procs, states, **kwargs):
        sent = []
        with patch.object(drone.signal, 'signal'), \
                patch.object(winproc, 'processes', return_value=procs), \
                patch.object(drone, 'state', side_effect=lambda base: states.get(base)), \
                patch.object(drone.subprocess, 'run', return_value=done()), \
                patch.object(winproc, 'alive', side_effect=lambda pid: pid not in sent), \
                patch.object(winproc, 'send_ctrl_c', side_effect=sent.append), \
                patch.object(winproc, 'kill_tree') as kill, \
                patch.object(drone, 'ask', return_value=kwargs.pop('answer', 'y')):
            code = drone.run_stop(**kwargs)
        return code, sent, kill

    def test_nothing_is_stopped_while_a_real_drone_is_in_the_air(self):
        procs = [Proc(10, 0, 'python gateway_server.py --port 8010'),
                 Proc(20, 0, 'python -m modules.perception.inference.gateway_ai_client --gateway http://127.0.0.1:8010')]
        code, sent, kill = self.run_stop(procs, {'http://127.0.0.1:8010': tello('auto')}, yes=True)
        self.assertEqual(code, 1)
        self.assertEqual(sent, [])
        kill.assert_not_called()

    def test_map_console_gateway_in_the_air_also_blocks(self):
        procs = [Proc(30, 0, r'python C:\repo\apps\console\serve.py --enable-camera --port 8790 '
                             r'--tello-gateway http://127.0.0.1:8011')]
        code, sent, _ = self.run_stop(procs, {'http://127.0.0.1:8011': tello('manual')}, yes=True)
        self.assertEqual((code, sent), (1, []))

    def test_grounded_programs_get_ctrl_c_helpers_first_gateway_last(self):
        procs = [Proc(10, 0, 'python gateway_server.py --port 8010'),
                 Proc(21, 0, 'python gateway_map_bridge.py --map http://127.0.0.1:8790 --gateway http://127.0.0.1:8010'),
                 Proc(20, 0, 'python -m modules.perception.inference.gateway_ai_client --gateway http://127.0.0.1:8010')]
        code, sent, kill = self.run_stop(procs, {'http://127.0.0.1:8010': tello('grounded')}, yes=True)
        self.assertEqual(code, 0)
        self.assertEqual(sent, [20, 21, 10])
        kill.assert_not_called()

    def test_start_cmd_window_left_asking_to_terminate_is_closed(self):
        procs = [Proc(40, 0, r'"C:\WINDOWS\system32\cmd.exe" /c C:\repo\scripts\start.cmd map --fake'),
                 Proc(41, 40, r'"C:\repo\.venv\Scripts\python.exe" -B -m apps.launcher map --fake'),
                 Proc(42, 41, r'C:\repo\.venv\Scripts\python.exe -B C:\repo\apps\console\serve.py --enable-camera '
                              r'--port 8790 --tello-gateway http://127.0.0.1:8010')]
        fake = {'type': 'drone_state', 'source': 'fake', 'mode': 'grounded'}
        code, sent, kill = self.run_stop(procs, {'http://127.0.0.1:8010': fake}, yes=True)
        self.assertEqual((code, sent), (0, [42, 41]))
        kill.assert_called_once_with(40)

    def test_connected_drone_asks_first_and_no_keeps_everything(self):
        procs = [Proc(10, 0, 'python gateway_server.py --port 8010')]
        code, sent, _ = self.run_stop(procs, {'http://127.0.0.1:8010': tello('grounded')}, answer='n')
        self.assertEqual((code, sent), (0, []))


class CliTests(unittest.TestCase):
    def test_commands_reach_the_right_launcher(self):
        with patch.object(cli.mapping, 'main', return_value=0) as mapping, \
                patch.object(cli, 'ensure_console_web', return_value=True):
            self.assertEqual(cli.main(['map', '--fake', '--no-browser']), 0)
            mapping.assert_called_with(['--fake', '--no-browser'])
            self.assertEqual(cli.main(['prepare']), 0)
            mapping.assert_called_with(['--prepare-models'])
        with patch.object(cli.mapping, 'main') as mapping, patch.object(cli, 'ensure_console_web', return_value=False):
            self.assertEqual(cli.main(['prepare']), 1)
        mapping.assert_not_called()
        with patch.object(cli.subprocess, 'call', return_value=0) as call:
            self.assertEqual(cli.main(['team-autonomy', 'wsl', '--port', '8081']), 0)
        self.assertEqual(call.call_args.args[0], [sys.executable, '-B', str(cli.TEAM_AUTONOMY), 'wsl', '--port', '8081'])
        with patch.object(cli.drone, 'run_stop', return_value=0) as stop:
            cli.main(['stop', '--yes'])
        stop.assert_called_once_with(yes=True)

    def test_real_options_given_on_the_command_line_skip_questions(self):
        with patch.object(cli.drone, 'run_real', return_value=0) as real:
            cli.main(['real', '--no-detect', '--mona', 'hand', '--no-browser'])
        self.assertEqual(real.call_args.kwargs, dict(phone=False, detect=False, map_=None, mona='hand',
                                                     no_browser=True, interactive=False))

    def test_menu_choices(self):
        with patch.object(cli.drone, 'show_code'), patch.object(cli.mapping, 'main', return_value=0) as mapping, \
                patch('builtins.input', side_effect=['3', 'f', '']):
            self.assertEqual(cli.main([]), 0)
        mapping.assert_called_once_with(['--fake'])
        with patch.object(cli.drone, 'show_code'), patch.object(cli.drone, 'run_stop', return_value=0) as stop, \
                patch('builtins.input', side_effect=['0', '']):
            self.assertEqual(cli.main([]), 0)
        stop.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
