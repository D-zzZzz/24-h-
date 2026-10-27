"""Exercise the actual Keil-built parser in an emulated Cortex-M3, without hardware.

Requires Python unicorn. Usage: python test_pid_protocol.py path/to/Blance_car.axf
"""
import struct
import sys
from pathlib import Path

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP


def load_elf(path):
    data = Path(path).read_bytes()
    assert data[:6] == b"\x7fELF\x01\x01", "Expected little-endian ELF32"
    header = struct.unpack_from("<HHIIIIIHHHHHH", data, 16)
    assert header[1] == 40, "Expected ARM firmware"
    segments = []
    for i in range(header[9]):
        segment = struct.unpack_from("<8I", data, header[4] + i * header[8])
        kind, offset, address, _, size, _, _, _ = segment
        if kind == 1 and size:
            segments.append((address, data[offset:offset + size]))
    sections = [struct.unpack_from("<10I", data, header[5] + i * header[10])
                for i in range(header[11])]
    symbols = {}
    for section in sections:
        if section[1] != 2:
            continue
        strings = sections[section[6]]
        names = data[strings[4]:strings[4] + strings[5]]
        for offset in range(section[4], section[4] + section[5], section[9]):
            name, value, _, _, _, _ = struct.unpack_from("<IIIBBH", data, offset)
            symbols[names[name:].split(b"\0", 1)[0].decode("ascii", errors="replace")] = value
    assert "PID_ProtocolParse" in symbols, "PID_ProtocolParse symbol not found"
    return segments, symbols


def parse(segments, symbols, line):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.mem_map(0x08000000, 0x20000)
    cpu.mem_map(0x20000000, 0x10000)
    for address, data in segments:
        cpu.mem_write(address, data)
    # Initialize scatter-loaded RAM and the real ARM C library, stopping before
    # main so none of the MCU peripheral initialization or motor code executes.
    cpu.reg_write(UC_ARM_REG_SP, 0x2000FFF0)
    cpu.emu_start(symbols["__main"] | 1, symbols["main"] & ~1,
                  timeout=2000000, count=500000)
    line_address, values_address, return_address = 0x20008000, 0x20008200, 0x0801FF00
    initial = [11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0]
    cpu.mem_write(line_address, line.encode("ascii") + b"\0")
    cpu.mem_write(values_address, struct.pack("<9f", *initial))
    cpu.reg_write(UC_ARM_REG_SP, 0x2000FFF0)
    cpu.reg_write(UC_ARM_REG_R0, line_address)
    cpu.reg_write(UC_ARM_REG_R1, values_address)
    cpu.reg_write(UC_ARM_REG_LR, return_address | 1)
    cpu.emu_start(symbols["PID_ProtocolParse"] | 1, return_address,
                  timeout=2000000, count=500000)
    response_address = cpu.reg_read(UC_ARM_REG_R0)
    response = bytes(cpu.mem_read(response_address, 32)).split(b"\0", 1)[0].decode("ascii")
    values = struct.unpack("<9f", cpu.mem_read(values_address, 36))
    return response, values, initial


def main():
    segments, symbols = load_elf(sys.argv[1])
    cases = [
        ("PING", "PONG\r\n", None),
        ("PID,380,0.396,0.13,0.0044,2,0", "OK\r\n", [380, .396, .13, .0044, 2, 0]),
        ("PID,550,0,0,0,1.6,1", "OK\r\n", [550, 0, 0, 0, 1.6, 1]),
        ("PID,550,0,0,0,1.6,-7", "OK\r\n", [550, 0, 0, 0, 1.6, -7]),
        ("PID,1000,10,20,1,45,1000", "OK\r\n", [1000, 10, 20, 1, 45, 1000]),
        ("PID,-1000,-10,-20,-1,-45,-1000", "OK\r\n", [-1000, -10, -20, -1, -45, -1000]),
        ("PID,550,0,0,0,1.6", "OK\r\n", [550, 0, 0, 0, 1.6, 16]),
        ("PID,550,0,0,0,1.6,1e1", "OK\r\n", [550, 0, 0, 0, 1.6, 10]),
        ("PID,350,0.4762,0.11,0.0029,2,0,20,0,0", "OK\r\n", [350, .4762, .11, .0029, 2, 0, 20, 0, 0]),
        ("PID,350,0.47,0.11,0.0029,2,0,20,-0.2,-5", "OK\r\n", [350, .47, .11, .0029, 2, 0, 20, -.2, -5]),
        ("PID,1000,10,20,1,45,1000,200,5,100", "OK\r\n", [1000, 10, 20, 1, 45, 1000, 200, 5, 100]),
        ("PID,-1000,-10,-20,-1,-45,-1000,-200,-5,-100", "OK\r\n", [-1000, -10, -20, -1, -45, -1000, -200, -5, -100]),
    ]
    cases.extend([
        ("PID,350,0.47,100,0.0029,2", "OK\r\n", [350, .47, 100, .0029, 2]),
        ("PID,350,0.47,100,0.0029,2,0", "OK\r\n", [350, .47, 100, .0029, 2, 0]),
        ("PID,350,0.47,100,0.0029,2,0,20,0,0", "OK\r\n", [350, .47, 100, .0029, 2, 0, 20, 0, 0]),
        ("PID,350,0.47,100.0001,0.0029,2,0", "ERR,RANGE\r\n", None),
        ("PID,350,0.47,-20.0001,0.0029,2,0", "ERR,RANGE\r\n", None),
    ])
    for line in ["", "PONG", "PID,1,2,3,4", "PID,1,2,3,4,5,",
                 "PID,1,2,3,0,5,1,2", "PID,1,2,3,0,5,1,2,0",
                 "PID,1,2,3,0,5,1,2,0,0,0", "PID,1,2,3,0,5,1,2,0,",
                 "PID,1,2,3,0,5,abc", "PID,1,,3,0,5,1"]:
        cases.append((line, "ERR,FORMAT\r\n", None))
    for line in ["PID,1001,0,0,0,0,0", "PID,1,11,0,0,0,0", "PID,1,0,101,0,0,0",
                 "PID,1,0,0,2,0,0", "PID,1,0,0,0,46,0", "PID,1,0,0,0,0,1001",
                 "PID,1,0,0,0,0,-1001", "PID,1,0,0,0,0,0.5", "PID,1,0,0,0,0,nan",
                 "PID,nan,0,0,0,0,0", "PID,1,0,0,0,0,inf", "PID,1,0,0,0,0,1e999",
                 "PID,1,0,0,0,0,0,201,0,0", "PID,1,0,0,0,0,0,-201,0,0",
                 "PID,1,0,0,0,0,0,20,5.1,0", "PID,1,0,0,0,0,0,20,-5.1,0",
                 "PID,1,0,0,0,0,0,20,0,101", "PID,1,0,0,0,0,0,20,0,-101",
                 "PID,1,0,0,0,0,0,20,0,0.5", "PID,1,0,0,0,0,0,nan,0,0",
                 "PID,1,0,0,0,0,0,20,inf,0", "PID,1,0,0,0,0,0,20,0,nan"]:
        cases.append((line, "ERR,RANGE\r\n", None))
    for line, expected_response, expected_values in cases:
        response, values, initial = parse(segments, symbols, line)
        assert response == expected_response, (line, response, expected_response)
        expected_values = expected_values + initial[len(expected_values):] if expected_values is not None else initial
        for actual, expected in zip(values, expected_values):
            assert abs(actual - expected) <= 1e-5, (line, values, expected_values)
    print(f"PASS: {len(cases)} protocol cases against actual ARM firmware")


if __name__ == "__main__":
    main()
