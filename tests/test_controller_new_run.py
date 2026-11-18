"""관제 화면(apps/controller): 화면을 연 채 게이트웨이를 새 회차로 다시 켜면 앞 회차 발견·지나온 길을 화면에서 지운다.

전에는 화면 발견(ui.findings)이 회차가 바뀌어도 남아, 새 회차 finding-0001이 앞 회차 1번과 같은 번호로 섞였다.
앞 회차 것은 그 회차 폴더(data/outputs/tello/<회차>)에 남는다.
"""
import unittest
from pathlib import Path

APP = (Path(__file__).resolve().parents[1] / 'apps' / 'controller' / 'app.js').read_text(encoding='utf-8')


def function_body(name):
    start = APP.index(f'\nfunction {name}(')
    return APP[start:APP.index('\n}\n', start)]


class NewRunTest(unittest.TestCase):
    def test_state_with_another_run_id_starts_over(self):
        self.assertIn('if (ui.state?.run_id && msg.run_id && msg.run_id !== ui.state.run_id) newRun(msg);',
                      function_body('onMessage'))

    def test_previous_run_is_cleared_and_the_new_one_loaded(self):
        body = function_body('newRun')
        for part in ('ui.findings.clear()', 'ui.selFinding = null', 'track: []', 'people: null',
                     'ui.goto.pick = null', '게이트웨이 새 회차', 'loadFindings()'):
            self.assertIn(part, body)


if __name__ == '__main__':
    unittest.main()
