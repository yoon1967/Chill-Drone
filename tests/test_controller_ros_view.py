"""관제 화면(apps/controller)의 자율 비행 보기 버튼(3단계).

게이트웨이(ros_view.py)와 같은 명령 이름을 쓰고, 자율 비행 웹과 같은 흐름(두 번 눌러 시작, 찍은 자리는 탐색 비행 중에만)을 따르는지 본다.
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP = (ROOT / "apps" / "controller" / "app.js").read_text(encoding="utf-8")
GATEWAY = (ROOT / "modules" / "autonomy" / "tello_link" / "ros_view.py").read_text(encoding="utf-8")


def function_body(name):
    start = APP.index(f"\nfunction {name}(")
    return APP[start:APP.index("\n}\n", start)]


class RosViewButtonsTest(unittest.TestCase):
    def test_same_command_names_as_the_gateway(self):
        screen = re.search(r"const ROS_CMDS = \[([^\]]*)\]", APP).group(1)
        gateway = re.search(r"ROS_CMDS = \{(.*?)\}", GATEWAY, re.S).group(1)
        self.assertEqual(sorted(re.findall(r'"(\w+)"', screen)), sorted(re.findall(r'"(\w+)":', gateway)))

    def test_start_needs_two_presses_like_the_web(self):   # 웹: 한 번 누르면 4초 동안 "한 번 더"
        body = function_body("requestTakeoff")
        self.assertIn('st.source === "ros"', body)
        self.assertIn("Date.now() + 4000", body)
        self.assertIn("rosStartBlocker(st)", function_body("takeoffBlocker"))

    def test_estop_opens_when_the_gateway_allows_it(self):
        self.assertIn('st.source === "ros" && !st.allowed.includes("emergency")', function_body("estopButton"))

    def test_goto_sends_only_when_allowed(self):   # 웹: 지도 클릭으로 찍고(아직 안 감), 보내기는 탐색 비행 단계에서만
        body = function_body("sendGoto")
        self.assertIn('st.allowed.includes("goto")', body)
        self.assertIn('sendCmd("goto"', body)
        self.assertIn('st.allowed.includes("goto_cancel")', function_body("clearGoto"))


class RosViewSummaryTest(unittest.TestCase):
    """시뮬레이터 시연에서 어색하던 요약·지도(4단계): 모르는 값은 지어내지 않고, 시간·출발은 게이트웨이가 센 것."""

    def test_unknown_interventions_are_not_zero(self):   # 자율 비행 보기는 개입 수를 받지 않는다: "0회"로 지어내지 않는다
        self.assertIn('"operator_interventions": None', GATEWAY)
        self.assertIn('st.operator_interventions == null ? "모름"', function_body("detailItems"))
        self.assertIn('n == null ? "개입 모름"', function_body("renderReview"))

    def test_flight_time_counted_by_the_gateway(self):   # 시뮬레이터 Tello는 모터 시간이 늘 0이다
        self.assertIn('"flight_time_s": self._flight_time()', GATEWAY)
        self.assertIn("ui.state?.flight_time_s != null", APP)   # 게이트웨이가 센 비행 시간이면 "비행", 실기체는 "모터"
        self.assertIn("timeLabel(true)", function_body("renderReview"))

    def test_start_is_the_takeoff_point(self):   # 시뮬레이터 위치는 지도 좌표라 원점이 출발이 아니다
        self.assertIn('"start_pose": self._start_pose', GATEWAY)
        body = function_body("autonomyMapPanel")
        self.assertIn("s.start_pose", body)
        self.assertIn('sp0 = wm?.spawn ? [wm.spawn.x, wm.spawn.y] : o', body)   # 출발(웹 spawn 네모): 지도 파일, 없으면 이륙한 자리
        self.assertIn('spawnSvg(X(sp0[0]), Y(sp0[1])', body)

    def test_phone_page_in_autonomy_view(self):   # 11/17 실행기 --phone: 폰 화면(#/control)을 자율 비행 보기에 맞춘다
        body = function_body("renderControl")
        self.assertIn('const ros = st.source === "ros";', body)
        for hidden in ('big("takeover", "넘겨받기")', 'big("return_home", "복귀")', 'data-part="sticks"'):
            self.assertIn(hidden, body)
        self.assertIn('${ros ? "" : big("takeover", "넘겨받기")}', body)   # 손 조종은 자율 비행에 없는 명령
        self.assertIn('<div data-part="estop">${estopButton(st, true)}</div>', body)   # 허용이 바뀌면 다시 그린다
        self.assertIn('${armed ? "한 번 더" : "이륙"}', function_body("takeoffPanel"))   # 폰에서 버튼이 밀려나지 않게 짧게


if __name__ == "__main__":
    unittest.main()
