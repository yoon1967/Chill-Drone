"""관제 화면 비행 기록이 게이트웨이가 보내는 까닭 코드를 쉬운 말로 적는지 (apps/controller/app.js의 WHY).

게이트웨이(조종권 관리자·비행 제어)가 모드를 바꾼 까닭을 새로 만들면 화면 표(WHY)에도 넣어야
비행 기록에 영어 코드(takeoff, route_done 등)가 그대로 찍히지 않는다.
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LINK = ROOT / "modules" / "autonomy" / "tello_link"


def gateway_reasons() -> set[str]:
    """mode_changed·safety_landing 이벤트에 실려 화면에 오는 까닭 코드."""
    arbiter = (LINK / "mode_arbiter.py").read_text(encoding="utf-8")
    gateway = (LINK / "gateway.py").read_text(encoding="utf-8")
    flight = (LINK / "flight_control.py").read_text(encoding="utf-8")
    found = set(re.findall(r'_set_mode\("\w+", [^,]+, "(\w+)"\)', arbiter))   # 까닭이 글자 그대로인 모드 바꿈
    found |= set(re.findall(r'_take_over\([^,]+, "(\w+)"\)', arbiter))        # 넘겨받기
    found |= set(re.findall(r'on_landed\("(\w+)"\)', gateway))                # 착륙 끝·비상 정지 뒤
    if '"mission_" + msg["phase"]' in arbiter:                                  # 임무 끝(자율이 도착·실패를 알림)
        found |= {"mission_arrived", "mission_failed"}
    for fn in ("_safety_reason", "_crash_check"):   # 자동 착륙 까닭(충돌·뒤집힘·스스로 내려앉음 포함)
        body = flight.split(f"def {fn}", 1)[1].split("\n    def ", 1)[0]
        found |= set(re.findall(r'return "(\w+)"', body))
    ros_view = (LINK / "ros_view.py").read_text(encoding="utf-8")                # 자율 비행 보기: 드라이버 상태로 정한 모드
    for fn in ("_derive_mode", "_landing_reason"):
        body = ros_view.split(f"def {fn}", 1)[1].split("\n    def ", 1)[0]
        found |= set(re.findall(r'return "\w+", "(\w+)"', body)) | set(re.findall(r'return "(\w+)"\n', body))
    # 보기 모드는 자율 비행 드라이버가 스스로 내린 까닭도 그대로 싣는다(드라이버 landing_reason)
    driver = (ROOT / "modules" / "autonomy" / "tello_driver" / "driver_core.py")
    if driver.exists():
        found |= set(re.findall(r'self\.landing_reason = "(\w+)"', driver.read_text(encoding="utf-8")))
    return found


def screen_words() -> set[str]:
    app = (ROOT / "apps" / "controller" / "app.js").read_text(encoding="utf-8")
    block = app.split("const WHY = {", 1)[1].split("};", 1)[0]
    return set(re.findall(r'(\w+):\s*"', block))


class LogWordsTest(unittest.TestCase):
    def test_every_gateway_reason_has_plain_words(self):
        reasons = gateway_reasons()
        self.assertGreater(len(reasons), 15)   # 찾는 규칙이 깨져 빈 목록으로 통과하지 않게
        missing = sorted(reasons - screen_words())
        self.assertEqual(missing, [], f"apps/controller/app.js WHY에 쉬운 말을 넣으세요: {missing}")


if __name__ == "__main__":
    unittest.main()
