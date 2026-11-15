"""관제 화면(apps/controller) 알림 띠 버튼: 게이트웨이가 지금 받는 명령(drone_state.allowed)일 때만 보인다."""
import re
import unittest
from pathlib import Path

APP = (Path(__file__).resolve().parents[1] / 'apps' / 'controller' / 'app.js').read_text(encoding='utf-8')


def function_body(name):
    start = APP.index(f'\nfunction {name}(')
    return APP[start:APP.index('\n}\n', start)]


class AlertButtonTest(unittest.TestCase):
    def test_alert_buttons_follow_allowed(self):
        # 자율 연결 끊김·자율 명령 끊김의 넘겨받기, 임무 끝의 착륙은 게이트웨이가 그 명령을 받을 때만 보인다.
        # 착륙하는 동안이나 명령을 받지 않는 모드에서 누르면 거절 알림만 떴다.
        body = function_body('alertBox')
        self.assertRegex(body, r'const act = \(name, label\) => \(st\.allowed\.includes\(name\) \?')
        self.assertEqual(re.findall(r'act\("(\w+)"', body), ['takeover', 'takeover', 'land'])
        self.assertEqual(re.findall(r'data-cmd="(\w+)"', body), ['return_home'])   # 복귀는 아직 화면 자리만(누르면 안내)

    def test_person_alert_stop_follows_allowed(self):   # 사람 발견의 "여기서 멈추기"는 자율 탐색 중에만
        self.assertIn('st.allowed.includes("pause") ?', function_body('personAlert'))

    def test_both_pages_pass_state(self):   # 실시간 감독·현장 조작
        self.assertEqual(re.findall(r'\balertBox\(([^)]*)\)', APP), ['sc, events, st', 'ctx.sc, ctx.events, ctx.st', 'sc, events, st'])


if __name__ == '__main__':
    unittest.main()
