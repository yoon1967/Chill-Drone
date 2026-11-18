"""관제 화면(apps/controller): 새로 연 화면(새 탭의 탐색 결과 등)이 이번 회차 비행 기록을 채운다.

전에는 화면이 열린 뒤 받은 알림만 쌓아, 새 탭은 "아직 기록이 없습니다"였다. 게이트웨이가 화면으로 보낸 알림을
GET /api/events로 내주고, 화면은 처음 연결할 때 그것과 받아 둔 발견(처음 본 시각)으로 기록을 채운 뒤 실시간으로 이어 간다.
"""
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP = (ROOT / 'apps' / 'controller' / 'app.js').read_text(encoding='utf-8')
SERVER = (ROOT / 'modules' / 'autonomy' / 'tello_link' / 'gateway_server.py').read_text(encoding='utf-8')


def function_body(name, prefix='function'):
    start = APP.index(f'\n{prefix} {name}(')
    return APP[start:APP.index('\n}\n', start)]


class FlightLogHistoryTest(unittest.TestCase):
    def test_gateway_keeps_what_the_screen_logs(self):
        self.assertIn('@app.get("/api/events")', SERVER)
        self.assertIn('HISTORY_TYPES = ("event", "detection_review")', SERVER)

    def test_first_open_fills_the_log_then_goes_live(self):
        onopen = APP[APP.index('ws.onopen = async'):APP.index('ws.onclose')]
        self.assertIn('if (first) had = await loadHistory();', onopen)
        self.assertIn('if (!had.has(historyKey(m))) onMessage(m);', onopen)   # 잡아 둔 메시지 중 기록에 이미 있는 알림은 건너뛴다
        self.assertIn('if (ui.hold) ui.hold.push(m); else onMessage(m);', onopen + APP[APP.index('ws.onmessage'):][:300])
        body = function_body('loadHistory', 'async function')
        self.assertIn('fetch("/api/events"', body)
        self.assertIn('ui.findings.values()', body)   # 사람 발견 줄은 받아 둔 발견으로

    def test_same_words_as_live_alerts(self):
        self.assertIn('eventLine(msg)', function_body('onMessage'))
        self.assertIn('eventLine(x.m)', function_body('loadHistory', 'async function'))
        self.assertIn('logFinding(f);', function_body('onDetection'))
        self.assertIn('logFinding(x.f)', function_body('loadHistory', 'async function'))


if __name__ == '__main__':
    unittest.main()
