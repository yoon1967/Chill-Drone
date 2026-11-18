"""관제 화면(apps/controller) 화면 바꾸기: 다른 화면으로 가면 맨 위에서 시작하고, 같은 화면을 다시 그릴 때는 스크롤을 둔다."""
import unittest
from pathlib import Path

APP = (Path(__file__).resolve().parents[1] / 'apps' / 'controller' / 'app.js').read_text(encoding='utf-8')


def function_body(name):
    start = APP.index(f'\nfunction {name}(')
    return APP[start:APP.index('\n}\n', start)]


class PageChangeTest(unittest.TestCase):
    def test_new_page_starts_at_top(self):
        # 네 화면(#/live·#/map·#/review·#/control)은 한 문서라 주소의 # 뒤만 바뀌고 스크롤은 남는다.
        # 폰에서 탐색 결과를 내린 채 현장 조작으로 가면 위 상태 카드가 머리줄 밑에 가려져 열렸다.
        # 맨 위로 보내는 것은 화면이 바뀔 때뿐이다(0.1초마다 다시 그릴 때·처음 열 때는 그대로). 지난 화면과 비교하려면 ui.view를 바꾸기 전에 본다.
        self.assertRegex(function_body('render'),
                         r'if \(ui\.view && ui\.view !== v\) window\.scrollTo\(0, 0\);.*\n\s*ui\.view = v;')

    def test_preset_in_the_address_picks_its_map_view(self):
        # 실행기는 ?preset=autonomy로 연다. 프리셋 고르기처럼 그 프리셋이 정한 지도 보기(자율 탐색은 2D)로 시작해야 한다
        # (전에는 이 브라우저에 기억한 보기, 처음엔 3D 점군으로 열렸다).
        presets = (Path(__file__).resolve().parents[1] / 'apps' / 'controller' / 'presets.js').read_text(encoding='utf-8')
        autonomy = presets[presets.index('id: "autonomy"'):]
        self.assertIn('{ id: "map", x: 0, y: 0, w: 7, h: 4, mode: "2d" }', autonomy[:autonomy.index('\n  },')])
        start = APP[APP.index('// ---------- 시작 ----------'):]
        self.assertRegex(start, r'if \(new URLSearchParams\(location\.search\)\.get\("preset"\)\) \{\s*'
                                r'const mode = livePreset\(\)\.panels\.find\(\(c\) => c\.id === "map"\)\?\.mode;\s*'
                                r'if \(MAP_MODES\[mode\]\) ui\.map3d\.mode = mode;')


if __name__ == '__main__':
    unittest.main()
