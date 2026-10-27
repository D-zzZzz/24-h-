"""Minimal Windows HID CMSIS-DAP memory access; no reset or flash operations."""

import ctypes as c
import struct
import time
from ctypes import wintypes as w
from enum import Enum


class State(Enum):
    RUNNING = 1
    HALTED = 2


class GUID(c.Structure):
    _fields_ = [('data1', w.DWORD), ('data2', w.WORD), ('data3', w.WORD),
                ('data4', w.BYTE * 8)]


class Interface(c.Structure):
    _fields_ = [('size', w.DWORD), ('guid', GUID), ('flags', w.DWORD),
                ('reserved', c.c_void_p)]


class Overlapped(c.Structure):
    _fields_ = [('internal', c.c_void_p), ('internal_high', c.c_void_p),
                ('offset', w.DWORD), ('offset_high', w.DWORD), ('event', w.HANDLE)]


def hid_paths(vendor_filter='vid_c251&pid_f001'):
    hid = c.WinDLL('hid', use_last_error=True)
    setup = c.WinDLL('setupapi', use_last_error=True)
    hid.HidD_GetHidGuid.argtypes = [c.POINTER(GUID)]
    setup.SetupDiGetClassDevsW.argtypes = [c.POINTER(GUID), w.LPCWSTR, w.HWND, w.DWORD]
    setup.SetupDiGetClassDevsW.restype = w.HANDLE
    setup.SetupDiEnumDeviceInterfaces.argtypes = [w.HANDLE, c.c_void_p,
        c.POINTER(GUID), w.DWORD, c.POINTER(Interface)]
    setup.SetupDiGetDeviceInterfaceDetailW.argtypes = [w.HANDLE,
        c.POINTER(Interface), c.c_void_p, w.DWORD, c.POINTER(w.DWORD), c.c_void_p]
    setup.SetupDiDestroyDeviceInfoList.argtypes = [w.HANDLE]
    guid = GUID()
    hid.HidD_GetHidGuid(c.byref(guid))
    handle = setup.SetupDiGetClassDevsW(c.byref(guid), None, None, 0x12)
    if handle == c.c_void_p(-1).value:
        raise c.WinError(c.get_last_error())
    try:
        index = 0
        while True:
            interface = Interface()
            interface.size = c.sizeof(interface)
            if not setup.SetupDiEnumDeviceInterfaces(handle, None, c.byref(guid),
                    index, c.byref(interface)):
                error = c.get_last_error()
                if error != 259:
                    raise c.WinError(error)
                break
            index += 1
            required = w.DWORD()
            setup.SetupDiGetDeviceInterfaceDetailW(handle, c.byref(interface),
                None, 0, c.byref(required), None)
            detail = c.create_string_buffer(required.value)
            c.cast(detail, c.POINTER(w.DWORD))[0] = 8 if c.sizeof(c.c_void_p) == 8 else 6
            if not setup.SetupDiGetDeviceInterfaceDetailW(handle, c.byref(interface),
                    detail, required.value, c.byref(required), None):
                raise c.WinError(c.get_last_error())
            path = c.wstring_at(c.addressof(detail) + 4)
            if vendor_filter is None or vendor_filter in path.lower():
                yield path
    finally:
        setup.SetupDiDestroyDeviceInfoList(handle)


class HidLink:
    def __init__(self, path):
        self.kernel = c.WinDLL('kernel32', use_last_error=True)
        self.hid = c.WinDLL('hid', use_last_error=True)
        k = self.kernel
        k.CreateFileW.argtypes = [w.LPCWSTR, w.DWORD, w.DWORD, c.c_void_p,
                                  w.DWORD, w.DWORD, w.HANDLE]
        k.CreateFileW.restype = w.HANDLE
        k.CloseHandle.argtypes = [w.HANDLE]
        k.CreateEventW.argtypes = [c.c_void_p, w.BOOL, w.BOOL, w.LPCWSTR]
        k.CreateEventW.restype = w.HANDLE
        k.WaitForSingleObject.argtypes = [w.HANDLE, w.DWORD]
        k.GetOverlappedResult.argtypes = [w.HANDLE, c.POINTER(Overlapped),
                                         c.POINTER(w.DWORD), w.BOOL]
        k.CancelIoEx.argtypes = [w.HANDLE, c.POINTER(Overlapped)]
        for api in (k.ReadFile, k.WriteFile):
            api.argtypes = [w.HANDLE, c.c_void_p, w.DWORD, c.POINTER(w.DWORD),
                            c.POINTER(Overlapped)]
        self.handle = k.CreateFileW(path, 0xC0000000, 3, None, 3, 0x40000000, None)
        if self.handle == c.c_void_p(-1).value:
            raise c.WinError(c.get_last_error())
        self.input_size = self.output_size = 65
        self.hid.HidD_GetPreparsedData.argtypes = [w.HANDLE, c.POINTER(c.c_void_p)]
        self.hid.HidP_GetCaps.argtypes = [c.c_void_p, c.c_void_p]
        self.hid.HidP_GetCaps.restype = w.LONG
        self.hid.HidD_FreePreparsedData.argtypes = [c.c_void_p]
        parsed = c.c_void_p()
        if self.hid.HidD_GetPreparsedData(self.handle, c.byref(parsed)):
            try:
                caps = c.create_string_buffer(64)
                if self.hid.HidP_GetCaps(parsed, caps) == 0x110000:
                    self.input_size, self.output_size = struct.unpack_from('<HH', caps, 4)
            finally:
                self.hid.HidD_FreePreparsedData(parsed)

    def close(self):
        self.kernel.CloseHandle(self.handle)

    def io(self, api, buffer, length):
        event = self.kernel.CreateEventW(None, True, False, None)
        overlap = Overlapped()
        overlap.event = event
        count = w.DWORD()
        try:
            ok = api(self.handle, buffer, length, c.byref(count), c.byref(overlap))
            if not ok:
                error = c.get_last_error()
                if error != 997:
                    raise c.WinError(error)
                if self.kernel.WaitForSingleObject(event, 3000) != 0:
                    self.kernel.CancelIoEx(self.handle, c.byref(overlap))
                    self.kernel.WaitForSingleObject(event, 3000)
                    raise TimeoutError('CMSIS-DAP HID transfer timed out')
                if not self.kernel.GetOverlappedResult(self.handle, c.byref(overlap),
                        c.byref(count), False):
                    raise c.WinError(c.get_last_error())
            return count.value
        finally:
            self.kernel.CloseHandle(event)

    def command(self, command):
        raw = bytes([0]) + command
        if len(raw) > self.output_size:
            raise ValueError('DAP packet exceeds HID report length')
        output = c.create_string_buffer(raw.ljust(self.output_size, b'\0'))
        self.io(self.kernel.WriteFile, output, self.output_size)
        response = c.create_string_buffer(self.input_size)
        length = self.io(self.kernel.ReadFile, response, self.input_size)
        data = response.raw[1:length]
        if not data or data[0] != command[0]:
            raise RuntimeError('Unexpected CMSIS-DAP response')
        return data


class NativeTarget:
    State = State

    def __init__(self):
        self.link = None
        for path in hid_paths():
            candidate = HidLink(path)
            try:
                info = candidate.command(b'\x00\x02')
                product = info[2:2 + info[1]].decode('ascii', errors='replace')
                if 'CMSIS' in product.upper() or 'DAP' in product.upper():
                    self.link = candidate
                    self.product = product
                    break
            finally:
                if self.link is not candidate:
                    candidate.close()
        if self.link is None:
            raise RuntimeError('No matching CMSIS-DAP HID probe found')

    def __enter__(self):
        try:
            self.link.command(b'\x11' + struct.pack('<I', 500000))
            response = self.link.command(b'\x02\x01')
            if response[1] != 1:
                raise RuntimeError('Probe could not select SWD')
            self.link.command(b'\x04\x00\x64\x00\x00\x00')
            self.link.command(b'\x13\x00')
            for sequence in (b'\x12\x40' + b'\xff' * 8,
                             b'\x12\x10\x9e\xe7',
                             b'\x12\x38' + b'\xff' * 7,
                             b'\x12\x08\x00'):
                self.link.command(sequence)
            self.dpidr = self.transfer(2)
            self.transfer(0, 0x1E)  # Clear sticky DP errors.
            self.transfer(8, 0)
            self.transfer(4, 0x50000000)  # Debug/system power request; no reset.
            deadline = time.monotonic() + 2
            while self.transfer(6) & 0xA0000000 != 0xA0000000:
                if time.monotonic() > deadline:
                    raise RuntimeError('SWD target did not acknowledge debug power')
            self.transfer(1, 0x23000052)  # MEM-AP, 32-bit access, address increment.
            return self
        except Exception:
            self.link.close()
            raise

    def __exit__(self, *_):
        try:
            self.link.command(b'\x03')
        finally:
            self.link.close()

    def transfer(self, request, value=None):
        packet = bytes([5, 0, 1, request])
        if value is not None:
            packet += struct.pack('<I', value)
        response = self.link.command(packet)
        if response[1] != 1 or response[2] != 1:
            raise RuntimeError(f'SWD transfer failed: count={response[1]}, status=0x{response[2]:02X}')
        return struct.unpack_from('<I', response, 3)[0] if request & 2 else None

    def read32(self, address):
        self.transfer(5, address)
        return self.transfer(15)

    def write32(self, address, value):
        self.transfer(5, address)
        self.transfer(13, value)

    def flush(self):
        pass  # Every HID transaction is synchronous.

    def get_state(self):
        return State.HALTED if self.read32(0xE000EDF0) & 0x20000 else State.RUNNING

    def halt(self):
        self.saved_debug_control = self.read32(0xE000EDF0) & 0xF
        self.write32(0xE000EDF0, 0xA05F0003)
        deadline = time.monotonic() + 1
        while self.get_state() != State.HALTED:
            if time.monotonic() > deadline:
                raise RuntimeError('MCU did not halt')

    def resume(self):
        self.write32(0xE000EDF0, 0xA05F0000 | self.saved_debug_control)

    def read_core_register(self, name):
        if name != 'pc':
            raise ValueError('Only PC is implemented')
        self.write32(0xE000EDF4, 15)
        deadline = time.monotonic() + 1
        while not self.read32(0xE000EDF0) & 0x10000:
            if time.monotonic() > deadline:
                raise RuntimeError('Core register read timed out')
        return self.read32(0xE000EDF8)


if __name__ == '__main__':
    probe = NativeTarget()
    print('Probe product:', probe.product)
    probe.link.close()
