"""실행기 방화벽 맞추기(apps/launcher/drone.py ensure_firewall): 실기체에서 Python이 드론 상태·영상(UDP 8890·11111)을 받게.

실사용자 PC에서는 처음 켤 때 뜨는 Windows 허용 창에서 취소하면 차단 규칙이 생겨 드론 상태·영상이 말없이 안 들어온다.
실행기가 드론 Wi-Fi 네트워크 종류에서 허용·차단을 보고, 모자라면 Windows 관리자 확인 창(사람이 '예')으로 그 두 포트만 연다.
실제 방화벽은 건드리지 않는다(PowerShell은 모두 가짜).
"""
import base64
import unittest
from unittest.mock import patch

from apps.launcher import drone, winproc

EXE = r'C:\Users\me\AppData\Local\Programs\Python\Python312\python.exe'


def rule(action='Allow', profile='Public', protocol='UDP', ports=('Any',)):
    return {'action': action, 'profile': profile, 'protocol': protocol, 'ports': list(ports)}


class FirewallStateTests(unittest.TestCase):
    def test_windows_prompt_rule_covers_a_public_drone_wifi(self):   # 11/16 이 PC: 허용 창에서 공용만 허용
        rules = [rule(protocol='TCP'), rule()]
        self.assertEqual(drone.firewall_state(rules, 'Public'), (True, False))
        self.assertEqual(drone.firewall_state(rules, 'Private'), (False, False))
        self.assertEqual(drone.firewall_state(rules, None), (False, False))   # 드론 Wi-Fi 연결 전: 공용·개인 둘 다 필요

    def test_ports_profiles_and_protocols(self):
        self.assertEqual(drone.firewall_state([rule(profile='Any', ports=('8890', '11111'))], None), (True, False))
        self.assertEqual(drone.firewall_state([rule(profile='Private, Public', ports=('8000-12000',))], 'Private'), (True, False))
        self.assertEqual(drone.firewall_state([rule(ports=('8890',))], 'Public'), (False, False))           # 영상 포트 없음
        self.assertEqual(drone.firewall_state([rule(protocol='TCP')], 'Public'), (False, False))
        self.assertEqual(drone.firewall_state([rule(profile='Domain')], 'DomainAuthenticated'), (True, False))

    def test_block_rule_wins(self):   # 허용 창에서 취소하면 생기는 차단 규칙은 허용 규칙보다 먼저다
        rules = [rule(profile='Any'), rule(action='Block', profile='Public')]
        self.assertEqual(drone.firewall_state(rules, 'Public'), (True, True))
        self.assertEqual(drone.firewall_state(rules, 'Private'), (True, False))


class PhoneTests(unittest.TestCase):
    """폰 접속(--phone): 같은 망 주소를 네트워크 이름과 함께, 방화벽이 막을 것 같을 때만 넣을 명령을 보여 준다(규칙은 만들지 않음)."""

    def test_tcp_state(self):
        rules = [rule(protocol='TCP'), rule()]                       # 11/17 이 PC: Python 3.12 허용 창 규칙(공용, TCP·UDP 전체)
        self.assertEqual(drone.tcp_state(rules, 'Public', 8020), (True, False))
        self.assertEqual(drone.tcp_state(rules, 'Private', 8020), (False, False))
        self.assertEqual(drone.tcp_state(rules, '', 8020), (False, False))          # 종류를 모르면 Any 규칙만
        self.assertEqual(drone.tcp_state([rule(profile='Any', protocol='TCP', ports=('8020',))], '', 8020), (True, False))
        self.assertEqual(drone.tcp_state([rule(protocol='UDP')], 'Public', 8020), (False, False))
        self.assertEqual(drone.tcp_state(rules + [rule('Block', protocol='TCP')], 'Public', 8020), (True, True))

    def lines(self, nets, rules, port_rule=''):
        with patch.object(drone, 'phone_networks', return_value=nets), patch.object(drone, 'firewall_rules', return_value=rules), \
                patch.object(winproc, 'powershell', return_value=port_rule):
            return drone.phone_lines(8020, live='?preset=autonomy#/live')

    def test_hotspot_address_with_network_name(self):
        out = self.lines([('10.172.42.215', '내 핫스팟', 'Public')], [rule(protocol='TCP')])
        self.assertEqual(out[0], '  폰 조작      http://10.172.42.215:8020/#/control  (내 핫스팟 · 공용)')
        self.assertIn('/?preset=autonomy#/live', out[1])
        self.assertFalse(any('New-NetFirewallRule' in line for line in out))       # 이미 열려 있으면 명령을 안 보인다
        self.assertIn('인증이 없어', out[-1])

    def test_closed_network_shows_the_command(self):
        out = self.lines([('192.168.42.10', 'USB 테더링', 'Private')], [rule(protocol='TCP')])
        self.assertIn('방화벽에 막힐 수 있음', out[0])
        self.assertTrue(any('-LocalPort 8020 -RemoteAddress LocalSubnet' in line for line in out))
        out = self.lines([('192.168.42.10', 'USB 테더링', 'Private')], [], port_rule='Drone gateway 8020 (test)')
        self.assertNotIn('막힐 수 있음', out[0])                                   # 포트 허용 규칙이 있으면 열림
        self.assertTrue(any('허용 규칙 있음: Drone gateway 8020 (test)' in line for line in out))

    def test_no_shared_network(self):
        out = self.lines([], [])
        self.assertEqual(len(out), 1)
        self.assertIn('폰 핫스팟', out[0])


class EnsureFirewallTests(unittest.TestCase):
    def run_ensure(self, before, after=None, category='Public', elevated='ok'):
        said = []
        reads = iter([before, after if after is not None else before])
        with patch.object(drone.sys, '_base_executable', EXE, create=True), \
                patch.object(drone, 'say', side_effect=said.append), patch.object(drone, 'warn', side_effect=said.append), \
                patch.object(drone, 'firewall_rules', side_effect=lambda exe: next(reads)), \
                patch.object(drone, 'drone_network_category', return_value=category), \
                patch.object(winproc, 'powershell_elevated', return_value=elevated) as elevate:
            ok = drone.ensure_firewall()
        return ok, elevate, said

    def test_allowed_needs_no_admin_window(self):
        ok, elevate, said = self.run_ensure([rule()])
        self.assertTrue(ok)
        elevate.assert_not_called()
        self.assertIn('받을 수 있어요', said[-1])

    def test_missing_rule_is_added_through_the_admin_window(self):
        ok, elevate, said = self.run_ensure([], after=[rule(profile='Any', ports=('8890', '11111'))], category='Private')
        self.assertTrue(ok)
        script = elevate.call_args.args[0]
        self.assertIn(f"$p = '{EXE}'", script)
        self.assertIn('-Protocol UDP -LocalPort 8890,11111 -Action Allow -Profile Any', script)
        self.assertIn('Disable-NetFirewallRule', script)        # 차단 규칙은 끈다(지우지 않음)
        self.assertNotIn('Remove-NetFirewallRule -DisplayName', script.split('Disable-NetFirewallRule')[0])
        self.assertTrue(any('관리자 확인 창' in line for line in said))

    def test_no_on_the_admin_window_shows_the_command(self):
        ok, _, said = self.run_ensure([rule(action='Block')], elevated='canceled')
        self.assertFalse(ok)
        self.assertTrue(any('차단 규칙' in line for line in said))
        self.assertTrue(any(line.strip().startswith('New-NetFirewallRule') and EXE in line for line in said))

    def test_unreadable_rules_show_the_command(self):
        ok, elevate, said = self.run_ensure(None)
        self.assertFalse(ok)
        elevate.assert_not_called()
        self.assertTrue(any('New-NetFirewallRule' in line for line in said))


class ScriptTests(unittest.TestCase):
    def test_fix_touches_only_this_python(self):
        script = drone.firewall_fix_script(EXE)
        disable = script[:script.index('Disable-NetFirewallRule')]
        for part in ("Where-Object { $_.Program -eq $p }", "$_.Action -eq 'Block'", "$_.Direction -eq 'Inbound'",
                     "Protocol -in @('UDP', 'Any')"):
            self.assertIn(part, disable)
        self.assertIn(f"-DisplayName '{drone.FIREWALL_RULE}'", script)

    def test_elevated_command_asks_windows_for_admin(self):
        outer = winproc.elevated_argv("exit 0")
        self.assertIn('-Verb RunAs', outer)
        encoded = outer.split("'-EncodedCommand','")[1].split("'")[0]
        self.assertEqual(base64.b64decode(encoded).decode('utf-16-le'), 'exit 0')
        with patch.object(winproc, 'powershell', return_value='exit=0\r\n'):
            self.assertEqual(winproc.powershell_elevated('x'), 'ok')
        with patch.object(winproc, 'powershell', return_value='canceled'):
            self.assertEqual(winproc.powershell_elevated('x'), 'canceled')
        with patch.object(winproc, 'powershell', return_value='exit=1'):
            self.assertEqual(winproc.powershell_elevated('x'), 'failed')


if __name__ == '__main__':
    unittest.main()
