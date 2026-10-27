"""Compare actual old/new ARM OLED code with an emulated, ACK-capable SSD1306.

Requires unicorn. Arguments: baseline.axf optimized.axf.
No real hardware is accessed; OLED_I2C_Write is replaced with a bus model.
"""
import json
import struct
import sys
from pathlib import Path

from unicorn import Uc, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0,
    UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "JDY31PidTuner" / "tests"))
from test_pid_protocol import load_elf


class Display:
    def __init__(self, firmware):
        segments, self.symbols = load_elf(firmware)
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.cpu.mem_map(0x08000000, 0x20000)
        self.cpu.mem_map(0x20000000, 0x10000)
        for address, data in segments:
            self.cpu.mem_write(address, data)
        self.cpu.reg_write(UC_ARM_REG_SP, 0x2000FFF0)
        self.cpu.emu_start(self.symbols["__main"] | 1, self.symbols["main"] & ~1,
                           timeout=2000000, count=500000)
        self.pixels = bytearray(1024)
        self.page = self.column = 0
        self.packets = []
        self.fail_next_data = False
        bus = self.symbols["OLED_I2C_Write"] & ~1
        self.cpu.hook_add(UC_HOOK_CODE, self.transfer, begin=bus, end=bus)

    def transfer(self, cpu, address, size, _):
        control = cpu.reg_read(UC_ARM_REG_R0)
        payload = bytes(cpu.mem_read(cpu.reg_read(UC_ARM_REG_R1),
                        cpu.reg_read(UC_ARM_REG_R2) & 0xFFFF))
        self.packets.append((control, payload))
        failed = self.fail_next_data and control == 0x40
        if failed:
            self.fail_next_data = False
            payload = payload[:len(payload) // 2]
        for byte in payload:
            if control == 0:
                if 0xB0 <= byte <= 0xB7:
                    self.page = byte - 0xB0
                elif byte & 0xF0 == 0x10:
                    self.column = (self.column & 0x0F) | ((byte & 0x0F) << 4)
                elif byte <= 0x0F:
                    self.column = (self.column & 0xF0) | byte
            elif control == 0x40:
                assert self.column < 128
                self.pixels[self.page * 128 + self.column] = byte
                self.column = (self.column + 1) % 128
            else:
                raise AssertionError(f"Unknown OLED control byte {control}")
        cpu.reg_write(UC_ARM_REG_R0, 1 if failed else 0)
        cpu.reg_write(UC_ARM_REG_PC, cpu.reg_read(UC_ARM_REG_LR))

    def call(self, function, *args):
        stack = 0x2000FFF0
        self.cpu.reg_write(UC_ARM_REG_SP, stack)
        for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1,
                                   UC_ARM_REG_R2, UC_ARM_REG_R3), args):
            self.cpu.reg_write(register, value)
        for i, value in enumerate(args[4:]):
            self.cpu.mem_write(stack + i * 4, struct.pack("<I", value))
        self.cpu.reg_write(UC_ARM_REG_LR, 0x0801FF01)
        self.cpu.emu_start(self.symbols[function] | 1, 0x0801FF00,
                           timeout=2000000, count=1000000)

    def text(self, page, text, x=0, font=8):
        self.cpu.mem_write(0x20008000, text + b"\0")
        self.call("OLED_ShowString", x, page, 0x20008000, font)

    def stats(self):
        return {"transactions": len(self.packets),
                "bus_bytes": sum(len(data) + 2 for _, data in self.packets)}


def main():
    old, new = Display(sys.argv[1]), Display(sys.argv[2])
    checks = 0

    def same():
        nonlocal checks
        assert old.pixels == new.pixels, "OLED pixel output changed"
        checks += 1

    for display in (old, new):
        display.call("OLED_Clear")
        display.packets.clear()
    same()
    lines = [b"ver P:   1.0 D:   0.0", b"vel P: 0.123 I: 0.004",
             b"pitch:1.60", b"EL:    12 ER:    34"]
    lines = [line.ljust(21)[:21] for line in lines]
    for page, text in enumerate(lines):
        old.text(page, text)
        new.text(page, text)
        same()
    before, after = old.stats(), new.stats()
    assert after["bus_bytes"] < before["bus_bytes"] / 3
    old.packets.clear()
    new.packets.clear()
    for page, text in enumerate(lines):
        old.text(page, text)
        new.text(page, text)
    same()
    assert not new.packets, "Unchanged frame must not write the bus"
    checks += 1
    for display in (old, new):
        display.text(2, b"pitch:1.61".ljust(21))
    same()
    for display in (old, new):
        display.text(4, b"ABC", x=120, font=16)
    same()
    for page, text, x, font in [(7, b"not on a two-page font", 0, 16),
                               (6, b"ABC", 127, 16),
                               (0, b"Z", 127, 8)]:
        for display in (old, new):
            display.text(page, text, x=x, font=font)
        same()
    # armlink removes unused public APIs; exercise these when both images retain them.
    for name, args in [("OLED_ShowCHinese", (40, 5, 0)),
                       ("OLED_ShowNum", (0, 7, 12345, 5, 8))]:
        if name in old.symbols and name in new.symbols:
            old.call(name, *args)
            new.call(name, *args)
            same()
    old.text(2, b"pitch:99.99".ljust(21))
    new.fail_next_data = True
    new.text(2, b"pitch:99.99".ljust(21))
    assert not new.fail_next_data
    new.text(2, b"pitch:99.99".ljust(21))
    same()
    if all(name in old.symbols and name in new.symbols
           for name in ("OLED_Set_Pos", "OLED_WR_DATA")):
        for display in (old, new):
            display.call("OLED_Set_Pos", 0, 0)
            display.call("OLED_WR_DATA", 0x55)
            display.text(0, lines[0])
        same()
    for display in (old, new):
        display.call("OLED_Clear")
    same()
    print(json.dumps({"checks_passed": checks, "first_frame_old": before,
                      "first_frame_optimized": after,
                      "unchanged_frame_optimized_transactions": 0}))


if __name__ == "__main__":
    main()
