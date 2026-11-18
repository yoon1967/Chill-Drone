"""실행기 start.cmd sim(자율 비행 시뮬레이터 시연, WSL). WSL·드론·창·PowerShell 없이 명령줄과 켜는 순서만 본다."""
import subprocess
import unittest
from contextlib import ExitStack
from unittest.mock import Mock, patch

from apps.launcher import cli, drone, sim, winproc
from apps.launcher.winproc import Proc

GATEWAY = 'http://127.0.0.1:8020'


class ScriptTests(unittest.TestCase):
    def test_wsl_path(self):
        self.assertEqual(sim.wsl_path(r'C:\Users\me\S15P31B205'), '/mnt/c/Users/me/S15P31B205')
        self.assertEqual(sim.wsl_path('D:/work'), '/mnt/d/work')
        self.assertEqual(sim.wsl_path('/home/me'), '/home/me')

    def test_simulation_is_started_like_the_autonomy_readme(self):   # 자율 비행 쪽은 README 명령 그대로
        s = sim.sim_script()
        for part in ('source /opt/ros/jazzy/setup.bash', 'ros2 launch launch/mission.launch.py',
                     'world_dir:=$PWD/worlds/s7_stage3', 'headless:=true', 'web:=true', 'detect:=true'):
            self.assertIn(part, s)
        v = sim.view_script()
        self.assertIn('modules/autonomy/tello_link/ros_view_node.py', v)
        self.assertIn('people:=$HOME/ttrepo/simulation/tt-sim/worlds/s7_stage3/people.json', v)   # 시뮬레이션은 세계 폴더에 쓴다
        for part in ('discovered:=$HOME/ttrepo/simulation/tt-sim/worlds/s7_stage3/discovered.json',    # 찾은 장애물·높이 층도 세계 폴더
                     'camera_map:=$HOME/ttrepo/simulation/tt-sim/worlds/s7_stage3/camera_map.npz',
                     'map:=$HOME/ttrepo/simulation/tt-sim/worlds/s7_stage3/known_map.json', 'truth:=true'):   # 웹처럼 세계 지도·정답 위치
            self.assertIn(part, v)
        self.assertNotIn('truth:=', sim.view_script(world=False))   # 실기체는 정답 위치가 없다
        w = sim.world_script('20261115_120000')
        self.assertIn('generate_world.py --seed 7 --stage 3', w)
        self.assertIn('mv worlds/s7_stage3/$f worlds/s7_stage3/$f.20261115_120000', w)   # 지난 찾은 사람은 옮겨 둔다
        for script in (s, v, w, sim.sync_script('/mnt/c/repo/.git', 'refs/heads/x'), sim.STOP_SCRIPT):
            self.assertNotIn('set -u', script)   # ROS setup.bash가 멈춘다

    def test_copy_is_fetched_from_the_common_git_folder(self):   # worktree .git 파일은 Windows 경로라 WSL git이 못 읽는다
        out = {('rev-parse', '--path-format=absolute', '--git-common-dir'): 'C:/Users/me/S15P31B205/.git',
               ('symbolic-ref', '-q', 'HEAD'): 'refs/heads/feat/final-integration'}
        with patch.object(sim, 'git_out', side_effect=lambda *a: out.get(a, '')):
            repo, ref = sim.head_source()
        self.assertEqual((repo, ref), ('/mnt/c/Users/me/S15P31B205/.git', 'refs/heads/feat/final-integration'))
        script = sim.sync_script(repo, ref)
        self.assertIn('git fetch -q --depth 1 "file:///mnt/c/Users/me/S15P31B205/.git" "refs/heads/feat/final-integration"', script)
        self.assertIn('git checkout -q -B launcher-sim FETCH_HEAD', script)
        out[('symbolic-ref', '-q', 'HEAD')] = ''   # 분리 HEAD면 커밋으로
        out[('rev-parse', 'HEAD')] = 'abc123'
        with patch.object(sim, 'git_out', side_effect=lambda *a: out.get(a, '')):
            self.assertEqual(sim.head_source()[1], 'abc123')

    def test_stop_touches_only_the_simulation(self):   # 모든 launch를 잡으면 같은 PC의 실기체 자율 비행까지 끈다
        self.assertIn("'[m]ission.launch.py'", sim.STOP_SCRIPT)
        self.assertIn("'[g]z sim'", sim.STOP_SCRIPT)
        self.assertNotIn('real.launch', sim.STOP_SCRIPT)
        self.assertNotIn('ros2 launch', sim.STOP_SCRIPT)

    def test_gateway_is_the_ros_view_on_its_own_port(self):
        self.assertEqual(sim.gateway_cmd()[-4:], ['--ros-view', 'http://127.0.0.1:8094', '--port', '8020'])
        self.assertEqual(sim.gateway_cmd(sensor_pose=True)[-5:],   # 실시간으로 도는 드론만 지도용 센서 자세
                         ['--ros-view', 'http://127.0.0.1:8094', '--port', '8020', '--ros-view-sensor-pose'])
        self.assertEqual(sim.gateway_cmd(sensor_pose=True, phone=True)[-3:], ['--ros-view-sensor-pose', '--host', '0.0.0.0'])
        self.assertNotIn('--host', sim.gateway_cmd())   # 폰을 고르지 않으면 이 PC에서만
        self.assertNotIn(sim.SIM_PORT, (drone.REAL_PORT, drone.FAKE_PORT))
        self.assertNotIn(sim.SIM_CONSOLE_PORT, (drone.CONSOLE_PORT, drone.FAKE_CONSOLE_PORT))


class RunSimTests(unittest.TestCase):
    def run_sim(self, distros=('Ubuntu-24.04',), check='has-ros has-repo has-venv user=ssafy\n', simulator=True,
                running=False, launches=''):
        said, scripts = [], []

        def get_json(url, timeout=2.0):   # running: 자율 비행 웹·보기 노드가 이미 답한다
            return {'ok': True} if running and url.startswith((sim.AUTONOMY_WEB, sim.VIEW_URL)) else None

        def say(text=''):
            said.append(text)

        def wsl(script, timeout=60):
            scripts.append(script)
            if script == sim.CHECK_SCRIPT:
                return 0, check
            if script == sim.WHICH_SCRIPT:   # WSL에서 도는 launch(running이면)
                return 0, launches
            if 'git fetch' in script:
                return 0, 'abc1234 feat: 지금 커밋\n'
            if 'generate_world.py' in script:
                return 0, 'world-ok\n'
            return 0, ''
        with patch.object(drone, 'show_code'), patch.object(drone.prepare, 'ensure_all', return_value=True), \
                patch.object(sim, 'say', side_effect=say), patch.object(drone, 'say', side_effect=say), \
                patch.object(sim, 'warn', side_effect=say), patch.object(drone, 'warn', side_effect=say), \
                patch.object(sim, 'has_simulator', return_value=simulator), \
                patch.object(sim, 'distros', return_value=list(distros)), patch.object(sim, 'wsl', side_effect=wsl), \
                patch.object(sim.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, stdout=b'', stderr=b'')), \
                patch.object(winproc, 'get_json', side_effect=get_json), patch.object(drone, 'wait_for', return_value=True), \
                patch.object(winproc, 'port_owners', return_value=[]), patch.object(winproc, 'processes', return_value=[]), \
                patch('modules.mapping.data.model_assets.person_checkpoint') as checkpoint, \
                patch.object(drone, 'start_console', return_value=True) as console, \
                patch.object(sim.mapkeeper, 'MapKeeper') as keeper, patch.object(drone, 'open_page') as page, \
                patch.object(winproc, 'start_service') as window:
            checkpoint.return_value.is_file.return_value = True
            checkpoint.return_value.__str__.return_value = 'yolo.pt'
            code = sim.run_sim(no_browser=True, interactive=False)
        return code, window.call_args_list, console, keeper, page, said, scripts

    def test_everything_is_started_without_asking_in_order(self):
        code, calls, console, keeper, page, _, scripts = self.run_sim()
        self.assertEqual(code, 0)
        names = [c.args[0] for c in calls]
        self.assertEqual(names, ['sim-autonomy', 'sim-view-node', 'gateway-8020', 'people-8020', 'map-bridge-8020'])
        self.assertEqual(calls[0].args[2], sim.wsl_argv(sim.sim_script()))
        self.assertEqual(calls[1].args[2], sim.wsl_argv(sim.view_script()))
        self.assertEqual(calls[2].args[2], sim.gateway_cmd())
        self.assertEqual(calls[3].args[2][-4:], ['--gateway', GATEWAY, '--yolo-model', 'yolo.pt'])
        self.assertEqual(calls[4].args[2][-4:], ['--map', 'http://127.0.0.1:8797', '--gateway', GATEWAY])
        console.assert_called_once_with(GATEWAY, port=8797)
        self.assertEqual(keeper.call_args.args[:2], (GATEWAY, 'http://127.0.0.1:8797'))
        self.assertEqual(page.call_args.args[0], GATEWAY + '/?preset=autonomy#/live')
        self.assertTrue(any('git fetch' in s for s in scripts))   # WSL 사본을 지금 커밋으로 맞췄다

    def test_branch_without_the_simulator_keeps_the_wsl_copy(self):   # 자율 비행 코드가 안 합쳐진 브랜치
        code, calls, _, _, _, said, scripts = self.run_sim(simulator=False)
        self.assertEqual(code, 0)
        self.assertEqual(len(calls), 5)
        self.assertFalse(any('git fetch' in s for s in scripts))
        self.assertTrue(any('사본을 그대로' in line for line in said))

    def test_running_simulation_is_left_as_it_is(self):   # 다시 실행해도 도는 시뮬레이터의 코드·찾은 사람 파일은 그대로
        code, calls, _, _, page, _, scripts = self.run_sim(running=True)
        self.assertEqual(code, 0)
        self.assertEqual([c.args[0] for c in calls], ['gateway-8020', 'people-8020', 'map-bridge-8020'])
        self.assertFalse(any('git fetch' in s or 'mv worlds' in s for s in scripts))
        page.assert_called_once()

    def test_running_real_autonomy_is_not_taken_for_the_simulator(self):   # 11/16 점검 A6
        code, calls, _, _, page, said, _ = self.run_sim(running=True, launches='real-on\n')
        self.assertEqual((code, calls), (1, []))
        page.assert_not_called()
        self.assertTrue(any('실기체를 띄울 수' in line for line in said))

    def test_missing_wsl_explains_and_starts_nothing(self):
        code, calls, _, _, page, said, _ = self.run_sim(distros=())
        self.assertEqual((code, calls), (1, []))
        page.assert_not_called()
        self.assertTrue(any('wsl --install -d Ubuntu-24.04' in line for line in said))

    def test_missing_ros_shows_the_setup_command(self):
        code, calls, _, _, _, said, _ = self.run_sim(check='has-repo user=ssafy\n')
        self.assertEqual((code, calls), (1, []))
        self.assertTrue(any('wsl_sim_setup.sh ssafy' in line and '-u root' in line for line in said))


class CliSimTests(unittest.TestCase):
    def setUp(self):
        self.enterContext(patch.object(cli.prepare, 'check_host', return_value=True))
        self.enterContext(patch.object(cli.sys, 'prefix', 'test-venv'))

    def test_sim_is_a_command_not_a_menu_item(self):
        with patch.object(cli.sim, 'run_sim', return_value=0) as run:
            self.assertEqual(cli.main(['sim', '--no-browser']), 0)
        run.assert_called_once_with(no_browser=True, launch_args=[], phone=False)
        self.assertEqual([key for key, _, _ in cli.MENU], ['1', '2'])   # 메뉴는 1·2 그대로
        with patch.object(cli.sim, 'run_sim', return_value=0) as run:
            self.assertEqual(cli.main(['sim', '--phone']), 0)
        run.assert_called_once_with(no_browser=False, launch_args=[], phone=True)


REAL_WRAPPER = Proc(50, 0, 'wsl.exe -d Ubuntu-24.04 --exec bash -c "... exec ros2 launch launch/real.launch.py front_tof:=true"')
RELAY = Proc(51, 0, 'python -B modules/autonomy/tello_driver/tello_relay.py')
VIEW_GATEWAY = Proc(10, 0, 'python gateway_server.py --ros-view http://127.0.0.1:8094 --port 8020')


class StopSimTests(unittest.TestCase):
    def run_stop(self, procs, st=None, code=0, view=None, launches=(), stop_ok=True, force=False):
        self.groups = []
        with patch.object(drone, 'say'), patch.object(drone, 'warn'), patch.object(drone.signal, 'signal'), \
                patch.object(winproc, 'processes', return_value=procs), patch.object(winproc, 'get_json', return_value=None), \
                patch.object(drone, 'state', return_value=st), patch.object(winproc, 'alive', return_value=False), \
                patch.object(winproc, 'send_ctrl_c'), patch.object(winproc, 'kill_tree'), \
                patch.object(drone, 'stop_group', side_effect=lambda name, ps, w: self.groups.append((name, ps)) or bool(ps)), \
                patch.object(sim, 'view_state', return_value=view), \
                patch.object(sim, 'running_launches', return_value=set(launches)), \
                patch.object(sim, 'stop_sim', return_value=stop_ok) as stop:
            self.assertEqual(drone.run_stop(force=force), code)
        return stop

    def test_real_autonomy_not_known_to_be_on_the_ground_needs_a_second_enter(self):   # 11/16 점검 A5
        procs = [REAL_WRAPPER, RELAY, VIEW_GATEWAY]
        lost = {'type': 'drone_state', 'source': 'ros', 'mode': 'disconnected', 'flight_busy': None}
        for st in (None, lost):                                    # 게이트웨이가 답하지 않음·끊김, 보기 노드도 모름
            self.run_stop(procs, st, code=2).assert_not_called()
            self.assertEqual(self.groups, [])
        self.run_stop(procs, None, force=True).assert_called_once_with(real=True)   # 사람이 보고 한 번 더 누름
        fs = {'flying': False, 'busy': None, 'connected': True, 'age_s': 0.3}
        self.run_stop(procs, lost, view={'flight_status': fs}).assert_called_once_with(real=True)   # 드라이버가 땅이라고 함
        self.run_stop(procs, lost, view={'flight_status': {**fs, 'age_s': 5.0}}, code=2)          # 오래된 드라이버 상태는 모름

    def test_view_node_driver_in_the_air_stops_nothing(self):
        fs = {'flying': True, 'busy': None, 'connected': True, 'age_s': 0.2}
        self.run_stop([REAL_WRAPPER, RELAY, VIEW_GATEWAY], None, code=1, view={'flight_status': fs}).assert_not_called()
        self.assertEqual(self.groups, [])

    def test_relay_stays_until_wsl_is_down(self):   # WSL이 다 안 꺼지면 드라이버가 드론과 이야기하게 중계는 둔다
        ground = {'type': 'drone_state', 'source': 'ros', 'mode': 'grounded', 'flight_busy': None}
        self.run_stop([REAL_WRAPPER, RELAY, VIEW_GATEWAY], ground, stop_ok=False)
        self.assertNotIn('자율 비행 중계(Windows)', dict(self.groups))

    def test_real_launch_started_by_hand_counts_as_real(self):   # 이 실행기가 켜지 않은 real.launch도 공중이면 안 끈다
        flying = {'type': 'drone_state', 'source': 'ros', 'mode': 'auto', 'flight_busy': None}
        self.run_stop([RELAY, VIEW_GATEWAY], flying, code=1, launches=('real',)).assert_not_called()
        ground = {'type': 'drone_state', 'source': 'ros', 'mode': 'grounded', 'flight_busy': None}
        self.run_stop([RELAY, VIEW_GATEWAY], ground, launches=('real',)).assert_not_called()   # 손으로 켠 것은 안 끈다
        self.assertNotIn('자율 비행 중계(Windows)', dict(self.groups))                           # 그 드라이버의 중계도 둔다

    def test_second_enter_within_the_window_forces_the_stop(self):
        calls = []
        with patch.object(drone, 'say'), patch('builtins.input', side_effect=['', '']), \
                patch.object(drone.time, 'sleep'), \
                patch.object(drone, 'run_stop', side_effect=lambda keeper=None, force=False: calls.append(force) or (0 if force else 2)):
            self.assertEqual(drone.wait_and_stop(), 0)
        self.assertEqual(calls, [False, True])

    def test_stop_also_stops_the_simulation_this_launcher_started(self):
        wrapper = Proc(50, 0, 'wsl.exe -d Ubuntu-24.04 --exec bash -c "source /opt/ros/jazzy/setup.bash && cd '
                              '~/ttrepo/simulation/tt-sim && exec ros2 launch launch/mission.launch.py world_dir:=..."')
        self.run_stop([wrapper, Proc(10, 0, 'python gateway_server.py --ros-view http://127.0.0.1:8094 --port 8020')]) \
            .assert_called_once_with(real=False)

    def test_simulation_started_elsewhere_is_left_alone(self):
        self.run_stop([Proc(10, 0, 'python gateway_server.py --port 8010')]).assert_not_called()

    def test_real_autonomy_in_the_air_stops_nothing(self):   # 실기체 자율 비행이 떠 있으면 아무것도 끄지 않는다(시뮬레이터는 끔)
        procs = [Proc(50, 0, 'wsl.exe -d Ubuntu-24.04 --exec bash -c "... exec ros2 launch launch/real.launch.py front_tof:=false"'),
                 Proc(51, 0, 'python -B modules/autonomy/tello_driver/tello_relay.py'),
                 Proc(10, 0, 'python gateway_server.py --ros-view http://127.0.0.1:8094 --port 8020')]
        flying = {'type': 'drone_state', 'source': 'ros', 'mode': 'auto', 'flight_busy': None}
        self.run_stop(procs, flying, code=1).assert_not_called()
        self.assertEqual(self.groups, [])

    def test_real_autonomy_on_the_ground_stops_wsl_then_the_relay(self):
        relay = Proc(51, 0, 'python -B modules/autonomy/tello_driver/tello_relay.py')
        procs = [Proc(50, 0, 'wsl.exe -d Ubuntu-24.04 --exec bash -c "... exec ros2 launch launch/real.launch.py front_tof:=true"'),
                 relay, Proc(10, 0, 'python gateway_server.py --ros-view http://127.0.0.1:8094 --port 8020')]
        ground = {'type': 'drone_state', 'source': 'ros', 'mode': 'grounded', 'flight_busy': None}
        self.run_stop(procs, ground).assert_called_once_with(real=True)
        names = [name for name, ps in self.groups if ps]
        self.assertEqual(names[-1], '자율 비행 중계(Windows)')   # 게이트웨이 → (WSL 자율 비행) → 중계
        self.assertEqual(dict(self.groups)['자율 비행 중계(Windows)'], [relay])


class AutonomyTests(unittest.TestCase):
    """start.cmd autonomy: 자율 비행 README의 실기체 순서(Windows 중계 → WSL real.launch.py) + 관제 화면 쪽은 sim과 같다."""

    def test_real_launch_like_the_autonomy_readme(self):
        self.assertEqual(sim.real_script(False), 'source /opt/ros/jazzy/setup.bash && cd ~/ttrepo/simulation/tt-sim && '
                                                 'exec ros2 launch launch/real.launch.py front_tof:=false')
        self.assertIn('front_tof:=true', sim.real_script(True))   # 확장 키트(RMTT)일 때만 전방 ToF
        self.assertNotIn('people:=', sim.view_script(world=False))  # 실기체: 보기 노드 기본(가장 새 비행 폴더)
        self.assertEqual(sim.relay_cmd()[-1], 'modules/autonomy/tello_driver/tello_relay.py')
        self.assertEqual(sim.relay_cmd(fake_tello=True)[-8:], sim.FAKE_TELLO_ARGS)
        self.assertIn("'[r]eal.launch.py'", sim.REAL_STOP_SCRIPT)
        self.assertNotIn('gz sim', sim.REAL_STOP_SCRIPT)
        self.assertNotIn('real.launch', sim.STOP_SCRIPT)          # 시뮬레이터 끄기는 실기체를 안 건드린다

    def run_autonomy(self, ssids=('TELLO-59FA6D',), fake=False, procs=(), web=False, launches=(), launch_args=(), phone=False,
                     state=None):
        said = []

        def get_json(url, timeout=2.0):   # web: 자율 비행 웹(8080)이 이미 답한다
            return {'ok': True} if web and url.startswith(sim.AUTONOMY_WEB) else None
        with ExitStack() as stack:   # 바꿔 끼울 것이 많아 with 한 줄에 다 넣으면 파이썬 블록 수(20)를 넘는다
            for target, name, kwargs in ((drone, 'show_code', {}), (drone, 'get_ready', {'return_value': True}),
                                         (sim, 'check_wsl', {'return_value': 'ssafy'}), (sim, 'sync_copy', {}),
                                         (sim, 'say', {'side_effect': lambda t='': said.append(t)}), (drone, 'say', {}),
                                         (sim, 'warn', {'side_effect': said.append}),
                                         (winproc, 'wifi_ssids', {'return_value': list(ssids)}),
                                         (winproc, 'get_json', {'side_effect': get_json}), (drone, 'wait_for', {'return_value': True}),
                                         (sim, 'running_launches', {'return_value': set(launches)}),
                                         (winproc, 'port_owners', {'return_value': []}),
                                         (winproc, 'processes', {'return_value': list(procs)}), (drone, 'start_detect', {}),
                                         (drone, 'start_console', {'return_value': True}), (drone, 'start_bridge', {}),
                                         (sim.mapkeeper, 'MapKeeper', {}), (drone, 'open_page', {}),
                                         (drone, 'state', {'return_value': state})):
                stack.enter_context(patch.object(target, name, **kwargs))
            firewall = stack.enter_context(patch.object(drone, 'ensure_firewall'))
            lines = stack.enter_context(patch.object(drone, 'phone_lines',
                                                     return_value=['  폰 조작      http://10.0.0.2:8020/#/control  (내 핫스팟 · 공용)']))
            window = stack.enter_context(patch.object(winproc, 'start_service'))
            code = sim.run_autonomy(no_browser=True, fake_tello=fake, interactive=False, launch_args=launch_args, phone=phone)
        self.phone_lines = lines
        return code, [c.args[0] for c in window.call_args_list], window.call_args_list, firewall, said

    def test_basic_tello_starts_relay_then_autonomy_then_the_screen(self):
        code, names, calls, firewall, _ = self.run_autonomy()
        self.assertEqual(code, 0)
        self.assertEqual(names, ['tello-relay', 'autonomy-real', 'autonomy-view-node', 'gateway-8020'])
        self.assertEqual(calls[0].args[2], sim.relay_cmd())
        self.assertEqual(calls[1].args[2], sim.wsl_argv(sim.real_script(False)))   # 기본 Tello: 전방 ToF 끔
        self.assertEqual(calls[2].args[2], sim.wsl_argv(sim.view_script(world=False)))
        self.assertEqual(calls[3].args[2], sim.gateway_cmd(sensor_pose=True))       # 실기체: 지도용 센서 자세
        firewall.assert_called_once_with()                                          # 드론 상태·영상 방화벽 안내

    def test_rmtt_uses_the_front_tof(self):
        _, _, calls, _, _ = self.run_autonomy(ssids=('SSAFY', 'RMTT-9B10B2'))
        self.assertEqual(calls[1].args[2], sim.wsl_argv(sim.real_script(True)))

    def test_without_drone_wifi_nothing_starts(self):
        code, names, _, _, said = self.run_autonomy(ssids=('SSAFY',))
        self.assertEqual((code, names), (1, []))
        self.assertTrue(any('--fake-tello' in line for line in said))

    def test_fake_tello_tries_the_whole_path_without_a_drone(self):
        code, names, calls, firewall, _ = self.run_autonomy(ssids=(), fake=True)
        self.assertEqual(code, 0)
        self.assertEqual(names[:3], ['fake-tello', 'tello-relay', 'autonomy-real'])
        self.assertEqual(calls[1].args[2], sim.relay_cmd(fake_tello=True))
        firewall.assert_not_called()

    def test_other_kind_already_running_is_not_reused(self):   # 11/16 점검 A6: "가짜"라면서 실기체에 붙지 않는다
        fake_relay = Proc(52, 0, 'python -B modules/autonomy/tello_driver/tello_relay.py ' + ' '.join(sim.FAKE_TELLO_ARGS))
        code, names, _, _, said = self.run_autonomy(ssids=(), fake=True, procs=[RELAY], web=True, launches=('real',))
        self.assertEqual((code, names), (1, []))
        self.assertTrue(any('실기체를 띄울 수' in line for line in said))
        code, names, _, _, _ = self.run_autonomy(procs=[fake_relay], web=True, launches=('real',))   # 실기체를 켜려는데 가짜용
        self.assertEqual((code, names), (1, []))
        code, names, _, _, _ = self.run_autonomy(web=True, launches=('sim',))                        # 시뮬레이터가 켜져 있음
        self.assertEqual((code, names), (1, []))
        code, names, _, _, _ = self.run_autonomy(procs=[RELAY], web=True, launches=('real',))          # 같은 종류는 그대로 쓴다
        self.assertEqual(code, 0)
        self.assertNotIn('tello-relay', names)
        self.assertNotIn('autonomy-real', names)

    def test_cli(self):
        with patch.object(cli.prepare, 'check_host', return_value=True), patch.object(cli.sys, 'prefix', 'test-venv'), \
                patch.object(cli.sim, 'run_autonomy', return_value=0) as run:
            self.assertEqual(cli.main(['autonomy', '--fake-tello', '--no-browser']), 0)
        run.assert_called_once_with(no_browser=True, fake_tello=True, launch_args=[], phone=False)

    def test_launch_args_go_to_real_launch_as_they_are(self):   # 11/17: 탐색 시간 등은 자율 비행 파트와 맞춘 값을 시연 때 넘긴다
        with patch.object(cli.prepare, 'check_host', return_value=True), patch.object(cli.sys, 'prefix', 'test-venv'), \
                patch.object(cli.sim, 'run_autonomy', return_value=0) as run:
            self.assertEqual(cli.main(['autonomy', 'search_time:=150', '--fake-tello']), 0)
        run.assert_called_once_with(no_browser=False, fake_tello=True, launch_args=['search_time:=150'], phone=False)
        self.assertTrue(sim.real_script(False, ['search_time:=150']).endswith('front_tof:=false search_time:=150'))
        self.assertTrue(sim.sim_script(['search_time:=90']).endswith('detect:=true search_time:=90'))
        self.assertEqual(sim.real_script(True, ()), sim.real_script(True))   # 안 주면 지금과 같다
        code, names, calls, _, said = self.run_autonomy(launch_args=['search_time:=150', 'return_home:=true'])
        self.assertEqual(code, 0)
        self.assertEqual(calls[1].args[2], sim.wsl_argv(sim.real_script(False, ['search_time:=150', 'return_home:=true'])))
        self.assertTrue(any('search_time:=150 return_home:=true' in line for line in said))

    def test_phone_opens_the_screen_on_the_same_network(self):   # 11/17 송승우 "자율 비행 모드에서도 폰으로"
        with patch.object(cli.prepare, 'check_host', return_value=True), patch.object(cli.sys, 'prefix', 'test-venv'), \
                patch.object(cli.sim, 'run_autonomy', return_value=0) as run:
            self.assertEqual(cli.main(['autonomy', 'search_time:=150', '--phone']), 0)   # 인자 순서 상관없이
        run.assert_called_once_with(no_browser=False, fake_tello=False, launch_args=['search_time:=150'], phone=True)
        code, names, calls, _, said = self.run_autonomy(phone=True)
        self.assertEqual(code, 0)
        self.assertEqual(calls[3].args[2], sim.gateway_cmd(sensor_pose=True, phone=True))   # 같은 망에도 연다
        self.phone_lines.assert_called_once_with(8020, live='?preset=autonomy#/live')
        self.assertIn('  폰 조작      http://10.0.0.2:8020/#/control  (내 핫스팟 · 공용)', said)
        self.assertTrue(any('손 조종은 없어요' in line for line in said))   # 자율 비행이 조종한다
        _, _, calls, _, said = self.run_autonomy()
        self.assertEqual(calls[3].args[2], sim.gateway_cmd(sensor_pose=True))   # 고르지 않으면 이 PC에서만
        self.phone_lines.assert_not_called()

    def test_phone_cannot_reopen_a_gateway_running_only_here(self):   # 이미 켜진 게이트웨이는 바꾸지 못한다고 알린다
        running = {'type': 'drone_state', 'source': 'ros'}
        code, names, _, _, said = self.run_autonomy(phone=True, state=running, procs=[VIEW_GATEWAY])
        self.assertEqual(code, 0)
        self.assertNotIn('gateway-8020', names)
        self.assertTrue(any('폰 접속으로 바꾸지 못해요' in line for line in said))
        self.phone_lines.assert_not_called()
        open_gateway = Proc(11, 0, 'python gateway_server.py --ros-view http://127.0.0.1:8094 --port 8020 --host 0.0.0.0')
        _, _, _, _, said = self.run_autonomy(phone=True, state=running, procs=[open_gateway])
        self.assertFalse(any('바꾸지 못해요' in line for line in said))   # 이미 같은 망에 열려 있으면 주소만 알린다
        self.phone_lines.assert_called_once()

    def test_unsafe_launch_args_start_nothing(self):   # bash 명령줄에 붙으므로 이름:=값과 안전한 글자만
        for bad in (['search_time=150'], ['x:=1;reboot'], ['x:=$HOME'], ['x:=a b'], ['--ros-args'], ["x:='1'"]):
            code, names, _, _, said = self.run_autonomy(launch_args=bad)
            self.assertEqual((code, names), (1, []), bad)
            self.assertTrue(any('이름:=값' in line for line in said), bad)

    def test_running_autonomy_ignores_new_launch_args_and_says_so(self):
        code, names, _, _, said = self.run_autonomy(procs=[RELAY], web=True, launches=('real',), launch_args=['search_time:=150'])
        self.assertEqual(code, 0)
        self.assertNotIn('autonomy-real', names)
        self.assertTrue(any('적용되지 않아요' in line for line in said))


if __name__ == '__main__':
    unittest.main()
