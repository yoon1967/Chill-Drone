"""관제 화면(apps/controller) 색 토큰과 테마: 규칙이 쓰는 토큰이 다 있고, 두 테마 모두 대비가 충분한지 본다."""
import re
import unittest
from pathlib import Path

CONTROLLER = Path(__file__).resolve().parents[1] / 'apps' / 'controller'
AUTONOMY_WEB = Path(__file__).resolve().parents[1] / 'modules' / 'autonomy' / 'autonomy_web' / 'static' / 'index.html'
CSS = (CONTROLLER / 'style.css').read_text(encoding='utf-8')
LAYOUT_VARS = {'--o', '--x', '--y', '--w', '--h', '--cols', '--rows',   # 칸 배치: app.js가 칸마다 style로 넣는다
               '--map-rot'}   # 2D 지도 회전 각(app.js·mapview.js가 그림 묶음에 넣고 글자를 똑바로 세움)

# (글자, 바탕): WCAG AA 보통 글자 4.5:1
TEXT_PAIRS = [
    ('--text', '--bg'), ('--text', '--panel'), ('--text', '--neutral-soft'), ('--text', '--brand-soft'),
    ('--muted', '--bg'), ('--muted', '--panel'), ('--muted', '--accent-soft'),
    ('--accent', '--panel'), ('--accent', '--accent-soft'), ('--on-accent', '--accent'),
    ('--success', '--success-soft'), ('--warning', '--warning-soft'),
    ('--danger', '--danger-soft'), ('--danger', '--panel'),
    ('--success', '--panel'), ('--warning', '--panel'), ('--person-strong', '--panel'),   # 상태 카드 값, 연결 카드 짧은 말
    ('--person-strong', '--person-soft'), ('--text', '--person-soft'),   # 사람 발견 알림
    ('--header-text', '--header'), ('--header-text', '--header-active'), ('--header-selected', '--header-active'),
    ('--danger', '--map'), ('--danger', '--map-seen'),   # 2D 지도 찾은 장애물 이름표(의자·닿음 등)
    ('--board-route', '--map'), ('--warning', '--map'), ('--board-route', '--map-seen'),   # 지도 그림 글(동선 끝·진입로 이름, 막힌 까닭)
]
# (그림, 바탕): 글자가 아닌 표시 3:1
GRAPHIC_PAIRS = [('--map-path', '--map'), ('--map-plan', '--map'), ('--person', '--map'), ('--person-strong', '--map'),
                 ('--header-ok', '--header-active'), ('--header-warn', '--header-active'), ('--header-bad', '--header-active'),
                 ('--success', '--panel'), ('--warning', '--panel'), ('--danger', '--panel'), ('--muted', '--panel'),   # 머리줄·연결 카드 점
                 ('--on-strong', '--person'),   # 진한 분홍 위 흰 글자
                 ('--map-wall', '--map-seen'), ('--map-blocked', '--map-seen'),   # 2D 지도 벽·높은 것, 낮은 물체 테두리
                 ('--map3d-low', '--map'), ('--map3d-mid', '--map'), ('--map3d-high', '--map'),   # 3D 점군 높이 색
                 ('--map-next', '--map'), ('--map-next', '--map-seen'),   # 2D 지도 계획 경로(본 곳 위에도)
                 # 2D 지도 드론·찍은 자리: 바탕과 본 곳 위 둘 다. 지나온 길·찾은 사람(주황)은 자율 비행 웹과 똑같은 값을 쓴다
                 # (11/17 송승우 "똑같이 맞추기", 아래 WebMapColorTest). 그 둘은 밝게에서 바탕에 2.5~3:1이라 이 목록에서 뺐다
                 ('--map-drone', '--map'), ('--map-drone', '--map-seen'), ('--map-pick', '--map'), ('--map-pick', '--map-seen')]


def block(selector):
    found = re.search(re.escape(selector) + r'\{([^}]*)\}', CSS)
    return {name: value.strip() for name, value in re.findall(r'(--[\w-]+):([^;]+);', found.group(1))}


LIGHT = block(':root')
DARK = {**LIGHT, **block(':root[data-theme=dark]')}


def resolve(tokens, name):
    value = tokens[name]
    for _ in range(5):
        ref = re.fullmatch(r'var\((--[\w-]+)\)', value)
        if not ref:
            break
        value = tokens[ref.group(1)]
    return value


def luminance(color):
    hex_digits = color.lstrip('#')
    if len(hex_digits) == 3:
        hex_digits = ''.join(c * 2 for c in hex_digits)
    channels = [int(hex_digits[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    r, g, b = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in channels]
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def contrast(a, b):
    high, low = sorted((luminance(a), luminance(b)), reverse=True)
    return (high + 0.05) / (low + 0.05)


def over(rgba, background):
    """반투명 토큰 rgba(r,g,b,a)를 바탕(#hex) 위에 섞은 #hex."""
    r, g, b, a = (float(v) for v in re.fullmatch(r'rgba\(([^)]*)\)', rgba.replace(' ', '')).group(1).split(','))
    base = [int(background.lstrip('#')[i:i + 2], 16) for i in (0, 2, 4)]
    return '#' + ''.join(f'{round(a * c + (1 - a) * d):02x}' for c, d in zip((r, g, b), base))


class TokenTest(unittest.TestCase):
    def test_rules_use_only_defined_tokens(self):
        used = set(re.findall(r'var\((--[\w-]+)', CSS))
        self.assertEqual(sorted(used - set(LIGHT) - LAYOUT_VARS), [])

    def test_dark_theme_overrides_known_tokens(self):
        self.assertEqual(sorted(set(block(':root[data-theme=dark]')) - set(LIGHT)), [])

    def test_rules_have_no_raw_colors(self):
        rules = CSS.split(':root[data-theme=dark]{', 1)[1].split('}', 1)[1]
        self.assertEqual(re.findall(r'#[0-9a-fA-F]{3,8}\b|rgba?\(', rules), [])

    def test_text_contrast_both_themes(self):
        for theme, tokens in (('light', LIGHT), ('dark', DARK)):
            for fg, bg in TEXT_PAIRS:
                with self.subTest(theme=theme, fg=fg, bg=bg):
                    self.assertGreaterEqual(contrast(resolve(tokens, fg), resolve(tokens, bg)), 4.5)

    def test_graphic_contrast_both_themes(self):
        for theme, tokens in (('light', LIGHT), ('dark', DARK)):
            for fg, bg in GRAPHIC_PAIRS:
                with self.subTest(theme=theme, fg=fg, bg=bg):
                    self.assertGreaterEqual(contrast(resolve(tokens, fg), resolve(tokens, bg)), 3)

    def test_map_height_colors_both_themes(self):   # 11/17 2D 지도 높이 층: 반투명 토큰은 본 곳 위에 섞은 색으로 잰다
        for theme, tokens in (('light', LIGHT), ('dark', DARK)):
            seen = resolve(tokens, '--map-seen')
            unsure, low = over(resolve(tokens, '--map-unsure'), seen), over(resolve(tokens, '--map-low'), seen)
            with self.subTest(theme=theme):
                self.assertGreaterEqual(contrast(unsure, seen), 3)                 # 막힘(높이 모름)
                self.assertGreater(contrast(unsure, seen), contrast(low, seen))    # 낮은 물체 면은 더 옅다(3:1은 테두리가 맡는다)


class PageTest(unittest.TestCase):
    def test_theme_is_chosen_before_styles_load(self):
        html = (CONTROLLER / 'index.html').read_text(encoding='utf-8')
        self.assertLess(html.index('<script src="theme.js"></script>'), html.index('href="style.css"'))
        self.assertEqual(re.findall(r'<option value="(\w+)">', html.split('id="theme"', 1)[1].split('</select>', 1)[0]),
                         ['auto', 'light', 'dark'])

    def test_canvas_map_follows_tokens(self):
        theme = (CONTROLLER / 'theme.js').read_text(encoding='utf-8')
        map3d = (CONTROLLER / 'map3d.js').read_text(encoding='utf-8')
        self.assertIn('"themechange"', theme)
        self.assertIn('"themechange"', map3d)
        self.assertIn('"--map-path"', map3d)
        self.assertIn('"--map3d-low", "--map3d-mid", "--map3d-high"', map3d)   # 높이 색도 토큰(테마가 바뀌면 다시 읽음)
        self.assertIn('this.heightColors = null', map3d)

    def test_theme_picker_gives_focus_back_to_flight_keys(self):
        # 선택 상자에 포커스가 남으면 app.js가 키 입력을 무시한다(Esc 비상 정지·L 착륙·WASD). 프리셋 상자와 같이 고른 뒤 뺀다.
        theme = (CONTROLLER / 'theme.js').read_text(encoding='utf-8')
        self.assertRegex(theme, r'addEventListener\("change", function \(\) \{ choose\(select\.value\); select\.blur\(\); \}\)')



# 2D 지도 색은 자율 비행 웹(autonomy_web/static/index.html)과 똑같다(11/17 송승우 "똑같이 맞추기"). 우리 토큰 → 웹 토큰
WEB_MAP_TOKENS = {'--map': '--map-floor', '--map-grid': '--map-grid', '--map-wall': '--map-wall', '--map-obst': '--map-obst',
                  '--map-next': '--map-path', '--map-trail': '--map-trail', '--map-drone': '--map-drone', '--map-goal': '--map-goal',
                  '--map-found': '--map-found', '--map-blocked': '--map-cam', '--map-suspect': '--map-cam-sus',
                  '--map-seen': '--map-seen', '--map-person': '--map-person', '--map-pick': '--map-pick', '--map-truth': '--map-truth'}


@unittest.skipUnless(AUTONOMY_WEB.exists(), '자율 비행 웹 파일이 없는 브랜치')
class WebMapColorTest(unittest.TestCase):
    def test_2d_map_uses_the_autonomy_web_colors(self):
        web = AUTONOMY_WEB.read_text(encoding='utf-8')
        tokens = lambda text: {n: v.strip().replace(' ', '') for n, v in re.findall(r'(--[\w-]+):\s*([^;]+);', text)}
        light = tokens(re.search(r':root\s*\{([^}]*)\}', web).group(1))
        dark = {**light, **tokens(re.search(r':root:not\(\[data-theme="light"\]\)\s*\{([^}]*)\}', web).group(1))}
        for ours, theirs in WEB_MAP_TOKENS.items():
            for theme, mine, web_tokens in (('light', LIGHT, light), ('dark', DARK, dark)):
                with self.subTest(token=ours, theme=theme):
                    self.assertEqual(resolve(mine, ours).replace(' ', '').lower(), web_tokens[theirs].lower())

if __name__ == '__main__':
    unittest.main()
