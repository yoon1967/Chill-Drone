"""Windows Shell dialogs for the local mapping console."""

from contextlib import ExitStack
import ctypes as ct
from ctypes import wintypes as wt
import os
from uuid import UUID


def _guid(value):
    return (ct.c_ubyte * 16).from_buffer_copy(UUID(value).bytes_le)


def _method(instance, slot, restype, *argtypes):
    vtable = ct.cast(instance, ct.POINTER(ct.POINTER(ct.c_void_p)))[0]
    return ct.WINFUNCTYPE(restype, ct.c_void_p, *argtypes)(vtable[slot])


def _check_hresult(result):
    if result < 0:
        raise ct.WinError(result)


def pick_folder(initial):
    """Open the Explorer-style folder picker on the HTTP request thread."""
    if os.name != 'nt':
        raise NotImplementedError
    ole, shell, user = ct.windll.ole32, ct.windll.shell32, ct.windll.user32
    user.GetForegroundWindow.restype = wt.HWND
    clsid = _guid('DC1C5A9C-E88A-4DDE-A5A1-60F82A20AEF7')  # FileOpenDialog
    iid_dialog = _guid('42F85136-DB7E-439C-85F1-E4075D135FC8')  # IFileDialog
    iid_item = _guid('43826D1E-E718-42EE-BC55-A1E261C37BFE')  # IShellItem
    dialog, item, selected, path = (ct.c_void_p() for _ in range(4))
    pointer = ct.POINTER(ct.c_void_p)

    _check_hresult(ole.CoInitializeEx(None, 0x2 | 0x4))  # STA, disable OLE1 DDE
    with ExitStack() as cleanup:
        cleanup.callback(ole.CoUninitialize)
        _check_hresult(ole.CoCreateInstance(ct.byref(clsid), None, 1, ct.byref(iid_dialog), ct.byref(dialog)))
        cleanup.callback(_method(dialog, 2, wt.ULONG), dialog)  # IUnknown.Release
        options = wt.DWORD()
        _check_hresult(_method(dialog, 10, ct.c_long, ct.POINTER(wt.DWORD))(dialog, ct.byref(options)))  # GetOptions
        pick_folders, force_file_system = 0x20, 0x40
        _check_hresult(_method(dialog, 9, ct.c_long, wt.DWORD)(dialog, options.value | pick_folders | force_file_system))
        if shell.SHCreateItemFromParsingName(ct.c_wchar_p(str(initial)), None, ct.byref(iid_item), ct.byref(item)) >= 0:
            cleanup.callback(_method(item, 2, wt.ULONG), item)
            _check_hresult(_method(dialog, 12, ct.c_long, ct.c_void_p)(dialog, item))  # SetFolder
        result = _method(dialog, 3, ct.c_long, wt.HWND)(dialog, user.GetForegroundWindow())  # Show
        if (result & 0xffffffff) == 0x800704C7:  # HRESULT_FROM_WIN32(ERROR_CANCELLED)
            return None
        _check_hresult(result)
        _check_hresult(_method(dialog, 20, ct.c_long, pointer)(dialog, ct.byref(selected)))  # GetResult
        cleanup.callback(_method(selected, 2, wt.ULONG), selected)
        file_system_path = 0x80058000  # SIGDN_FILESYSPATH
        _check_hresult(_method(selected, 5, ct.c_long, wt.DWORD, pointer)(selected, file_system_path, ct.byref(path)))
        cleanup.callback(ole.CoTaskMemFree, path)
        return ct.wstring_at(path)


def open_folder(folder):
    """Open the folder using Explorer's normal show mode."""
    if os.name != 'nt':
        raise NotImplementedError
    os.startfile(str(folder), 'open', show_cmd=1)  # SW_SHOWNORMAL
