"""Request a Native Wi-Fi driver scan before reading Windows' network list."""
import ctypes
import time


class InterfaceInfo(ctypes.Structure):
    _fields_ = [('guid', ctypes.c_ubyte * 16), ('description', ctypes.c_wchar * 256),
                ('state', ctypes.c_uint32)]


class InterfaceList(ctypes.Structure):
    _fields_ = [('count', ctypes.c_uint32), ('index', ctypes.c_uint32)]


def request_windows_scan():
    # netsh show networks only reads the WLAN service's current cache. WlanScan
    # asks the driver to refresh it. Microsoft documents a four-second scan
    # deadline; read the cache after that bounded wait, without switching Wi-Fi.
    api = ctypes.WinDLL('wlanapi.dll')
    dword, pointer = ctypes.c_uint32, ctypes.c_void_p
    for name, args in {
        'WlanOpenHandle': [dword, pointer, ctypes.POINTER(dword), ctypes.POINTER(pointer)],
        'WlanEnumInterfaces': [pointer, pointer, ctypes.POINTER(pointer)],
        'WlanScan': [pointer, pointer, pointer, pointer, pointer],
        'WlanCloseHandle': [pointer, pointer],
        'WlanFreeMemory': [pointer],
    }.items():
        fn = getattr(api, name); fn.argtypes = args; fn.restype = None if name == 'WlanFreeMemory' else dword

    def check(code):
        if code:
            raise RuntimeError('wifi_location_permission_required' if code == 5 else 'wifi_scan_request_failed')

    handle, version, interfaces = pointer(), dword(), pointer()
    check(api.WlanOpenHandle(2, None, ctypes.byref(version), ctypes.byref(handle)))
    try:
        check(api.WlanEnumInterfaces(handle, None, ctypes.byref(interfaces)))
        header = ctypes.cast(interfaces, ctypes.POINTER(InterfaceList)).contents
        if not header.count:
            raise RuntimeError('wifi_adapter_not_found')
        rows = (InterfaceInfo * header.count).from_address(interfaces.value + ctypes.sizeof(InterfaceList))
        results = [api.WlanScan(handle, ctypes.byref(row.guid), None, None, None) for row in rows]
        if all(results):
            check(5 if 5 in results else results[0])
        time.sleep(4)
    finally:
        if interfaces:
            api.WlanFreeMemory(interfaces)
        api.WlanCloseHandle(handle, None)
