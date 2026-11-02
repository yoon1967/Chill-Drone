"""Parse OS Wi-Fi observations without inferring SDK reachability from an SSID."""
import re
import sys


def percent(text):
    match = re.fullmatch(r'\s*(\d{1,3})\s*%?\s*', text)
    return int(match[1]) if match and 0 <= int(match[1]) <= 100 else None


def windows_networks(text):
    rows, current = {}, None
    for line in text.splitlines():
        match = re.match(r'^\s*SSID\s+\d+\s*:\s*(.*?)\s*$', line)
        if match:
            ssid = match[1]
            current = rows.setdefault(ssid, {'ssid': ssid, 'signal_pct': None}) if ssid else None
        elif current is not None:
            match = re.match(r'^\s*(?:Signal|신호)\s*:\s*(.*?)\s*$', line, re.I)
            if match:
                signal = percent(match[1])
                if signal is not None:
                    current['signal_pct'] = max(current['signal_pct'] or 0, signal)
    return list(rows.values())


def split_nmcli(line):
    # nmcli escapes literal colons and backslashes in SSIDs.
    parts, value, escaped = [], '', False
    for char in line:
        if escaped:
            value += char; escaped = False
        elif char == '\\':
            escaped = True
        elif char == ':':
            parts.append(value); value = ''
        else:
            value += char
    parts.append(value)
    return parts


def linux_networks(text):
    rows = {}
    for line in text.splitlines():
        values = split_nmcli(line)
        if len(values) != 3 or not values[1]:
            continue
        active, ssid, raw = values
        signal = percent(raw)
        previous = rows.get(ssid)
        if previous is None or active == '*' or (not previous['connected'] and (signal or 0) > (previous['signal_pct'] or 0)):
            rows[ssid] = {'ssid': ssid, 'signal_pct': signal, 'connected': active == '*'}
    return list(rows.values())


def scan_wifi(runner, platform=None, rescan=True):
    platform = platform or sys.platform
    if platform == 'win32':
        rows = windows_networks(runner(['netsh', 'wlan', 'show', 'networks', 'mode=bssid']))
    elif platform.startswith('linux'):
        rows = linux_networks(runner(['nmcli', '-t', '--escape', 'yes', '-f', 'IN-USE,SSID,SIGNAL',
            'device', 'wifi', 'list', '--rescan', 'yes' if rescan else 'no']))
    else:
        raise RuntimeError('wifi_scan_not_supported_on_this_os')
    return [dict(row, id=row['ssid'], kind='wifi', verified=False,
                 drone_candidate=bool(re.match(r'^(TELLO|RMTT|ROBOMASTER)', row['ssid'], re.I)))
            for row in sorted(rows, key=lambda row: (-(row['signal_pct'] or 0), row['ssid']))]


def current_wifi(runner, platform=None, preferred=None):
    platform = platform or sys.platform
    if platform == 'win32':
        text = runner(['netsh', 'wlan', 'show', 'interfaces'])
        # A disconnected interface has no exact SSID field. BSSID is not an SSID.
        rows, current = [], None
        for line in text.splitlines():
            match = re.match(r'^\s*SSID\s*:\s*(.*?)\s*$', line)
            if match:
                current = {'ssid': match[1], 'signal_pct': None, 'connected': True}
                if match[1]:
                    rows.append(current)
            elif current:
                match = re.match(r'^\s*(?:Signal|신호)\s*:\s*(.*?)\s*$', line, re.I)
                if match:
                    current['signal_pct'] = percent(match[1])
    else:
        rows = [row for row in scan_wifi(runner, platform, rescan=False) if row.get('connected')]
    return next((row for row in rows if row['ssid'] == preferred), rows[0] if rows else
                {'ssid': None, 'signal_pct': None, 'connected': False})
