"""관제 화면(apps/controller): 자율 비행이 지도에 찍은 사람 수를 사람 탐지 발견(사진)과 따로 센다.

시뮬레이터 시연에서 자율 비행이 찾은 사람(지도의 원)은 2D 지도 범례에만 보였다. 발견 칸 머리와 위 "발견" 카드에도 적는다.
수는 2D 지도 범례와 같다(확정한 사람, 다시 봤는데 없음은 뺌). 사람 탐지 발견 건수에는 더하지 않는다.
"""
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP = (ROOT / 'apps' / 'controller' / 'app.js').read_text(encoding='utf-8')
CSS = (ROOT / 'apps' / 'controller' / 'style.css').read_text(encoding='utf-8')


def function_body(name):
    start = APP.index(f'\nfunction {name}(')
    return APP[start:APP.index('\n}\n', start)]


class FoundPeopleTest(unittest.TestCase):
    def test_same_count_as_the_map_legend(self):
        line = APP[APP.index('const autonomyPeople = '):].split('\n', 1)[0]
        self.assertIn('ui.autonomy.people?.people', line)
        self.assertIn('!q.gone', line)

    def test_shown_apart_from_detections(self):
        self.assertIn('자율 비행이 찾은 사람 ${k}명', function_body('detectionList'))
        strip = function_body('statusStrip')
        self.assertIn('자율 비행 ${k}명', strip)
        self.assertIn('statCard("발견", n ? `${n}건` : "없음"', strip)   # 큰 글자는 사람 탐지 발견 건수 그대로
        self.assertIn('.chip.person{background:var(--person-soft);color:var(--person-strong)}', CSS)


if __name__ == '__main__':
    unittest.main()
