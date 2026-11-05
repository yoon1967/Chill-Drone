import ctypes
import unittest
from unittest.mock import Mock, patch

from modules.autonomy.tello_link.wifi import windows_networks, linux_networks, current_wifi, scan_wifi
from modules.autonomy.tello_link import windows_wifi
from modules.autonomy.tello_link.discovery import DroneDiscovery, run_os


class WifiTests(unittest.TestCase):
    def test_windows_refresh_requests_driver_scan_before_reading_networks(self):
        calls = []
        with patch('modules.autonomy.tello_link.wifi.request_windows_scan', side_effect=lambda: calls.append('scan')):
            rows = scan_wifi(lambda command: calls.append('read') or 'SSID 1 : TELLO-new\n', 'win32')
        self.assertEqual(calls, ['scan', 'read'])
        self.assertEqual(rows[0]['ssid'], 'TELLO-new')

    def test_failed_driver_scan_never_reports_a_cached_list_as_fresh(self):
        runner = Mock()
        with patch('modules.autonomy.tello_link.wifi.request_windows_scan',
                   side_effect=RuntimeError('wifi_scan_request_failed')):
            with self.assertRaisesRegex(RuntimeError, 'wifi_scan_request_failed'):
                scan_wifi(runner, 'win32')
        runner.assert_not_called()

    def test_native_scan_waits_for_driver_and_frees_handles_on_success_and_failure(self):
        class Buffer(ctypes.Structure):
            _fields_ = [('header', windows_wifi.InterfaceList), ('rows', windows_wifi.InterfaceInfo * 2)]
        buffer = Buffer(); buffer.header.count = 2
        for results in ([0, 0], [5, 5], [0, 5023]):
            api = Mock()
            api.WlanOpenHandle.return_value = 0
            def enumerate_interfaces(handle, reserved, out):
                ctypes.cast(out, ctypes.POINTER(ctypes.c_void_p)).contents.value = ctypes.addressof(buffer)
                return 0
            api.WlanEnumInterfaces.side_effect = enumerate_interfaces
            api.WlanScan.side_effect = results
            with self.subTest(results=results), patch.object(windows_wifi.ctypes, 'WinDLL', return_value=api, create=True), \
                 patch.object(windows_wifi.time, 'sleep') as wait:
                if all(results):
                    with self.assertRaisesRegex(RuntimeError, 'wifi_location_permission_required'):
                        windows_wifi.request_windows_scan()
                    wait.assert_not_called()
                else:
                    windows_wifi.request_windows_scan()
                    wait.assert_called_once_with(4)
                self.assertEqual(api.WlanScan.call_count, 2)
                api.WlanFreeMemory.assert_called_once()
                api.WlanCloseHandle.assert_called_once()

    def test_scan_returns_only_real_networks_and_failure_never_invents_a_drone(self):
        manager = DroneDiscovery(runner=Mock())
        items = [{'id': 'Home', 'ssid': 'Home', 'kind': 'wifi'},
                 {'id': 'TELLO-a', 'ssid': 'TELLO-a', 'kind': 'wifi'}]
        with patch.object(manager, 'refresh_wifi'), patch(
                'modules.autonomy.tello_link.discovery.wifi_candidates', return_value=items):
            self.assertEqual(manager.scan(), {'items': items, 'warning': None})
        with patch.object(manager, 'refresh_wifi'), patch(
                'modules.autonomy.tello_link.discovery.wifi_candidates',
                side_effect=RuntimeError('wifi_location_permission_required')):
            self.assertEqual(manager.scan(), {'items': [], 'warning': 'wifi_location_permission_required'})
        self.assertEqual(manager.candidates, {})

    def test_windows_location_denial_is_actionable_even_with_zero_exit_code(self):
        for code in (0, 1):
            with patch('modules.autonomy.tello_link.discovery.subprocess.run', return_value=Mock(
                    returncode=code, stdout=b'Location permission required: ms-settings:privacy-location')):
                with self.assertRaisesRegex(RuntimeError, 'wifi_location_permission_required'):
                    run_os(['netsh', 'wlan', 'show', 'networks', 'mode=bssid'])

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
