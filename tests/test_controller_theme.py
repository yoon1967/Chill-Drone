"""관제 화면(apps/controller) 색 토큰과 테마: 규칙이 쓰는 토큰이 다 있고, 두 테마 모두 대비가 충분한지 본다."""
import re
import unittest
from pathlib import Path

CONTROLLER = Path(__file__).resolve().parents[1] / 'apps' / 'controller'
CSS = (CONTROLLER / 'style.css').read_text(encoding='utf-8')
LAYOUT_VARS = {'--o', '--x', '--y', '--w', '--h', '--cols', '--rows'}   # 칸 배치: app.js가 칸마다 style로 넣는다

# (글자, 바탕): WCAG AA 보통 글자 4.5:1
TEXT_PAIRS = [
    ('--text', '--bg'), ('--text', '--panel'), ('--text', '--neutral-soft'), ('--text', '--brand-soft'),
    ('--muted', '--bg'), ('--muted', '--panel'), ('--muted', '--accent-soft'),
    ('--accent', '--panel'), ('--accent', '--accent-soft'), ('--on-accent', '--accent'),
    ('--success', '--success-soft'), ('--warning', '--warning-soft'),
    ('--danger', '--danger-soft'), ('--danger', '--panel'),
    ('--success', '--panel'), ('--warning', '--panel'), ('--person-strong', '--panel'),   # 상태 카드 값, 연결 카드 짧은 말
    ('--header-text', '--header'), ('--header-text', '--header-active'), ('--header-selected', '--header-active'),
]
# (그림, 바탕): 글자가 아닌 표시 3:1
GRAPHIC_PAIRS = [('--map-path', '--map'), ('--person', '--map'), ('--person-strong', '--map'), ('--person-off', '--map'),
                 ('--header-ok', '--header-active'), ('--header-warn', '--header-active'), ('--header-bad', '--header-active'),
                 ('--success', '--panel'), ('--warning', '--panel'), ('--danger', '--panel'), ('--muted', '--panel')]   # 머리줄·연결 카드 점


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


if __name__ == '__main__':
    unittest.main()
