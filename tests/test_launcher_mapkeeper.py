"""실기체 3D 지도 유지(apps/launcher/mapkeeper.py): 영상이 켜지면 시작, heartbeat, 끝나면 다시, 실패 제한, 끌 때 저장,
사람이 콘솔에서 마무리한 지도는 다음 이륙까지 그대로, 콘솔에서 시작한 지도는 기다림, 영상 끊김은 실패 아님."""
import unittest
from unittest.mock import patch

from apps.launcher import mapkeeper

GW, CONSOLE = 'http://127.0.0.1:8010', 'http://127.0.0.1:8790'


class FakeServers:
    def __init__(self, video=True):
        self.video = video
        self.mode = 'grounded'
        self.posts = []
        self.start_replies = []
        self.beat_reply = {}
        self.live = {'session_status': 'receiving'}   # /api/live/state?session=
        self.current = None                          # /api/live 의 current(콘솔이 지금 가진 세션)

    def get_json(self, url, timeout=2.0):
        if url.startswith(GW):
            return {'type': 'drone_state', 'mode': self.mode, 'video': {'available': self.video}}
        if url == CONSOLE + '/api/live':
            return {'enabled': True, 'tello_map_mode': 'gateway', 'current': self.current}
        return dict(self.live)

    def post_json(self, url, body, headers=None, timeout=5.0):
        self.posts.append((url[len(CONSOLE):], body, headers))
        if url.endswith('/api/live/tello/start'):
            return self.start_replies.pop(0) if self.start_replies else {'session_id': f'live_{self.starts()}'}
        if url.endswith('/api/live/tello/heartbeat'):
            return self.beat_reply
        return {}

    def starts(self):
        return [path for path, _, _ in self.posts].count('/api/live/tello/start')


class MapKeeperTests(unittest.TestCase):
    def setUp(self):
        self.servers = FakeServers()
        self.now = 0.0
        self.said = []
        self.patches = [patch.object(mapkeeper.winproc, 'get_json', side_effect=self.servers.get_json),
                        patch.object(mapkeeper.winproc, 'post_json', side_effect=self.servers.post_json)]
        for p in self.patches:
            p.start()
        self.keeper = mapkeeper.MapKeeper(GW, CONSOLE, self.said.append, clock=lambda: self.now)

    def tearDown(self):
        for p in self.patches:
            p.stop()

    def step(self, at=None):
        if at is not None:
            self.now = at
        self.keeper.step()

    def test_waits_for_video_then_starts_with_console_defaults_and_keeps_alive(self):
        self.servers.video = False
        self.step()
        self.assertEqual(self.servers.posts, [])
        self.servers.video = True
        self.step()
        path, body, headers = self.servers.posts[0]
        self.assertEqual(path, '/api/live/tello/start')
        self.assertEqual(body, mapkeeper.START_CONFIG)
        self.assertEqual(headers, {'X-Mapping-Client': '1'})
        self.assertEqual(self.keeper.session, 'live_1')
        self.step()
        self.assertEqual(self.servers.posts[-1][:2], ('/api/live/tello/heartbeat', {'session_id': 'live_1'}))

    def test_finished_session_restarts_when_video_is_back(self):
        self.step()
        self.servers.live = {'session_status': 'complete', 'stop_reason': 'tello_input_ended'}
        self.step()
        self.assertIsNone(self.keeper.session)
        self.step()                       # 5초 기다림
        self.assertEqual(self.servers.starts(), 1)
        self.step(6)
        self.assertEqual(self.keeper.session, 'live_2')

    def test_gives_up_after_three_failures(self):
        self.servers.start_replies = [{'error': 'slam_docker_unavailable'}] * 5
        for t in (0, 11, 32, 100, 200):
            self.step(t)
        self.assertEqual(self.servers.starts(), 3)
        self.assertTrue(self.keeper.gave_up)

    def test_busy_console_is_not_a_failure(self):
        self.servers.start_replies = [{'error': 'another_camera_session_is_active'},
                                      {'error': 'tello_input_cleanup_pending'}] * 3
        for t in (0, 6, 12, 18, 24, 30):
            self.step(t)
        self.assertEqual(self.servers.starts(), 6)
        self.assertEqual(self.keeper.failures, 0)
        self.assertFalse(self.keeper.gave_up)
        self.step(36)
        self.assertEqual(self.keeper.session, 'live_7')

    def test_failed_session_counts_as_failure(self):
        self.step()
        self.servers.live = {'session_status': 'failed', 'last_error': '[WinError 5] access denied'}
        self.step()
        self.assertIsNone(self.keeper.session)
        self.assertEqual(self.keeper.failures, 1)
        self.assertTrue(any('WinError 5' in line for line in self.said))

    def test_video_lost_for_30s_is_not_a_failure_and_restarts_when_video_is_back(self):
        self.step()
        self.servers.video = False
        self.servers.live = {'session_status': 'failed', 'last_error': 'tello_reconnect_timeout',
                             'stop_reason': 'tello_input_disconnected'}
        self.step()
        self.assertIsNone(self.keeper.session)
        self.assertEqual(self.keeper.failures, 0)
        self.assertTrue(any('30초 넘게 끊겨' in line for line in self.said))
        self.step(60)
        self.assertEqual(self.servers.starts(), 1)   # 영상이 없으면 시작하지 않는다
        self.servers.video = True
        self.servers.live = {'session_status': 'receiving'}
        self.step(62)
        self.assertEqual(self.keeper.session, 'live_2')

    def test_user_finishing_map_on_ground_waits_for_next_takeoff(self):
        self.step()
        self.servers.live = {'session_status': 'stopping', 'stop_reason': 'user_stop'}
        self.step()
        self.assertIsNone(self.keeper.session)
        self.assertEqual(self.keeper.held, 'live_1')
        self.assertTrue(any('마무리' in line for line in self.said))
        self.servers.live = {'session_status': 'complete', 'stop_reason': 'user_stop'}
        for t in (10, 60, 300):                       # 땅에서 영상이 켜져 있어도 새로 시작하지 않는다(저장할 시간)
            self.step(t)
        self.assertEqual(self.servers.starts(), 1)
        self.servers.mode = 'manual'                  # 다음 이륙
        self.servers.live = {'session_status': 'receiving'}
        self.step(302)
        self.assertEqual(self.keeper.session, 'live_2')
        self.assertIsNone(self.keeper.held)
        self.assertTrue(any('이륙해서' in line for line in self.said))

    def test_user_finishing_map_in_flight_waits_for_landing_and_next_takeoff(self):
        self.servers.mode = 'manual'
        self.step()
        self.servers.live = {'session_status': 'complete', 'stop_reason': 'user_stop'}
        self.step(2)
        for t, mode in ((10, 'manual'), (20, 'auto'), (30, 'landing'), (40, 'grounded'), (50, 'grounded')):
            self.servers.mode = mode
            self.step(t)
        self.assertEqual(self.servers.starts(), 1)    # 같은 비행 중·착륙 뒤에는 다시 시작하지 않는다
        self.servers.mode = 'manual'
        self.step(60)
        self.assertEqual(self.servers.starts(), 2)

    def test_user_finishing_map_then_video_off_and_on_starts_new_map(self):
        self.step()
        self.servers.live = {'session_status': 'complete', 'stop_reason': 'user_stop'}
        self.step(2)
        self.servers.video = False
        self.step(10)
        self.assertEqual(self.servers.starts(), 1)
        self.servers.video = True
        self.step(12)
        self.assertEqual(self.servers.starts(), 2)
        self.assertTrue(any('영상이 다시 켜져서' in line for line in self.said))

    def test_waits_while_console_runs_its_own_map(self):
        self.servers.current = {'session_id': 'live_console', 'session_status': 'receiving'}
        for t in (0, 2, 4):
            self.step(t)
        self.assertEqual(self.servers.starts(), 0)
        self.assertEqual(sum('live_console' in line for line in self.said), 1)   # 알림은 한 번만
        self.servers.current = {'session_id': 'live_console', 'session_status': 'complete'}
        self.step(6)
        self.assertEqual(self.keeper.session, 'live_1')

    def test_lets_go_when_console_replaced_the_session(self):
        self.step()
        self.servers.beat_reply = {'error': 'camera_session_mismatch'}
        self.step(2)
        self.assertIsNone(self.keeper.session)
        self.assertEqual(self.keeper.failures, 0)

    def test_stop_saves_the_running_session(self):
        self.step()
        self.keeper.stop()
        self.assertEqual(self.servers.posts[-1][:2], ('/api/live/stop', {'session_id': 'live_1', 'reason': 'user_stop'}))
        self.assertIsNone(self.keeper.session)


if __name__ == '__main__':
    unittest.main()
