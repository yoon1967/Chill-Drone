import unittest
from unittest.mock import Mock, patch

from modules.autonomy.tello_link.wifi import windows_networks, linux_networks, current_wifi
from modules.autonomy.tello_link.discovery import DroneDiscovery


class WifiTests(unittest.TestCase):
    def test_windows_multiple_bssids_korean_signal_and_hidden_ssid(self):
        rows = windows_networks('SSID 1 : TELLO-a\n BSSID 1 : aa:bb\n 신호 : 35%\n BSSID 2 : cc:dd\n Signal : 82%\nSSID 2 : Home\n Signal : 101%\nSSID 3 : \n Signal : 90%')
        self.assertEqual(rows, [{'ssid': 'TELLO-a', 'signal_pct': 82}, {'ssid': 'Home', 'signal_pct': None}])

    def test_linux_escaped_ssids_and_active_bssid_win_over_stronger_other_bssid(self):
        rows = linux_networks('*:TELLO\\:a:40\n:TELLO\\:a:90\n:Other:70\n')
        self.assertEqual(rows[0], {'ssid': 'TELLO:a', 'signal_pct': 40, 'connected': True})

    def test_current_windows_interface_changes_and_disconnects(self):
        runner = Mock(return_value=' State : connected\n SSID : TELLO-a\n BSSID : aa:bb\n Signal : 68%\n')
        self.assertEqual(current_wifi(runner, 'win32')['signal_pct'], 68)
        runner.return_value = ' 상태 : 연결 끊김\n'
        self.assertFalse(current_wifi(runner, 'win32')['connected'])

    def test_background_monitor_reports_failure_and_stale_values_without_sdk_commands(self):
        manager = DroneDiscovery(runner=Mock(side_effect=RuntimeError('os denied')))
        manager.refresh_wifi()
        manager.wifi_worker.join(2)
        self.assertEqual(manager.wifi_state()['status'], 'unavailable')
        manager.wifi_sample = {'status': 'ok', 'connected': True, 'ssid': 'TELLO-a', 'signal_pct': 80}
        with patch('modules.autonomy.tello_link.discovery.time.monotonic', return_value=manager.wifi_checked + 20), \
             patch.object(manager, 'refresh_wifi'):
            state = manager.wifi_state()
        self.assertEqual(state['status'], 'stale')
        self.assertIsNone(state['connected'])
        self.assertIsNone(state['signal_pct'])


if __name__ == '__main__':
    unittest.main()
