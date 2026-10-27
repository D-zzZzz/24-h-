"""Test actual ARM steering/motor mixing and UART application without hardware."""
import struct
import sys

from unicorn import Uc, UC_ARCH_ARM, UC_HOOK_CODE, UC_HOOK_MEM_WRITE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0,
    UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_SP)
from test_pid_protocol import load_elf


class Firmware:
    def __init__(self, filename):
        segments, self.symbols = load_elf(filename)
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for address, size in [(0x08000000, 0x20000), (0x20000000, 0x10000),
                              (0x40000000, 0x40000), (0xE000E000, 0x2000)]:
            self.cpu.mem_map(address, size)
        for address, data in segments:
            self.cpu.mem_write(address, data)
        self.cpu.reg_write(UC_ARM_REG_SP, 0x2000FFF0)
        self.cpu.emu_start(self.symbols["__main"] | 1, self.symbols["main"] & ~1,
                           timeout=2000000, count=500000)
        self.tx = []
        self.produced = 0
        self.cpu.hook_add(UC_HOOK_MEM_WRITE,
            lambda cpu, access, address, size, value, unused: self.tx.append(value & 255),
            begin=0x40004804, end=0x40004807)
        self.cpu.mem_write(0x40004800, struct.pack("<I", 0xC0))

    def call(self, name):
        self.cpu.reg_write(UC_ARM_REG_SP, 0x2000FFF0)
        self.cpu.reg_write(UC_ARM_REG_LR, 0x0801FF01)
        self.cpu.emu_start(self.symbols[name] | 1, 0x0801FF00,
                           timeout=2000000, count=1000000)
        return self.cpu.reg_read(UC_ARM_REG_R0)

    def read(self, name, fmt):
        return struct.unpack(fmt, self.cpu.mem_read(self.symbols[name], struct.calcsize(fmt)))[0]

    def write(self, name, fmt, value):
        self.cpu.mem_write(self.symbols[name], struct.pack(fmt, value))

    def stub(self, name, handler=None):
        def invoke(cpu, address, size, unused):
            value = handler() if handler else 0
            cpu.reg_write(UC_ARM_REG_R0, value or 0)
            cpu.reg_write(UC_ARM_REG_PC, cpu.reg_read(UC_ARM_REG_LR))
        address = self.symbols[name] & ~1
        self.cpu.hook_add(UC_HOOK_CODE, invoke, begin=address, end=address)

    def receive(self, text):
        payload = text.encode("ascii") + b"\n"
        assert self.produced + len(payload) < 512, "Test transport uses one DMA period"
        ring = struct.unpack("<I", self.cpu.mem_read(0x4002003C, 4))[0]
        self.cpu.mem_write(ring + self.produced, payload)
        self.produced += len(payload)
        self.cpu.mem_write(0x40020034, struct.pack("<I", 512 - self.produced))
        self.tx.clear()
        self.call("Bluetooth_Process")
        return bytes(self.tx)


def main():
    firmware = Firmware(sys.argv[1])
    assert firmware.call("Bluetooth_Init") == 0
    base = "PID,350,0.47,0.11,0.0029,1.6"
    assert firmware.receive(base + ",0,20,0,0") == b"OK\r\n"
    firmware.write("stop", "<B", 0)
    assert firmware.receive(base + ",0,20,-0.2,3") == b"OK\r\n"
    assert firmware.read("stop", "<B") == 0, "Steering must preserve speed integrator"
    assert abs(firmware.read("Turn_Kd", "<f") + .2) < 1e-6
    assert firmware.read("Target_Turn", "<i") == 3
    assert firmware.receive(base + ",7") == b"OK\r\n"
    assert firmware.read("Target_Speed", "<i") == 7
    assert firmware.read("Target_Turn", "<i") == 3, "Six-value legacy frame must preserve steering"
    assert firmware.read("stop", "<B") == 1, "Changed speed must reset its integrator"
    firmware.write("stop", "<B", 0)
    assert firmware.receive(base) == b"OK\r\n"
    assert firmware.read("Target_Speed", "<i") == 7, "Five-value legacy frame must preserve speed"
    assert firmware.read("Target_Turn", "<i") == 3
    assert firmware.receive(base + ",0,20,0,0.5") == b"ERR,RANGE\r\n"
    assert firmware.read("Target_Speed", "<i") == 7, "Rejected frame must not partially update"
    assert firmware.read("Target_Turn", "<i") == 3
    assert firmware.receive("PING") == b"PONG\r\n"

    sample = {"balance": 0, "gyro": 0}
    motors = []

    def attitude():
        firmware.cpu.mem_write(firmware.cpu.reg_read(UC_ARM_REG_R0),
                              struct.pack("<f", sample["balance"] / 1000.0))

    def gyro():
        for register, value in [(UC_ARM_REG_R0, 0), (UC_ARM_REG_R1, 0),
                                (UC_ARM_REG_R2, sample["gyro"])]:
            firmware.cpu.mem_write(firmware.cpu.reg_read(register), struct.pack("<h", value))

    def load():
        motors.clear()
        for register in (UC_ARM_REG_R0, UC_ARM_REG_R1):
            value = firmware.cpu.reg_read(register)
            motors.append(value if value < 0x80000000 else value - 0x100000000)

    for name in ("Read_Speed", "MPU_Get_Accelerometer"):
        firmware.stub(name)
    firmware.stub("mpu_dmp_get_data", attitude)
    firmware.stub("MPU_Get_Gyroscope", gyro)
    firmware.stub("Load", load)
    for name, value in [("Vertical_Kp", 1000), ("Vertical_Kd", 0), ("Velocity_Kp", 0),
                        ("Velocity_Ki", 0), ("Med_Angle", 0)]:
        firmware.write(name, "<f", value)
    firmware.write("Target_Speed", "<i", 0)
    cases = [
        (1000, 20, 0, 0, 32767, 0, [1000, 1000]),
        (1000, 20, 0, 5, 0, 100, [900, 1100]),
        (1000, 20, 0, -5, 0, -100, [1100, 900]),
        (0, 20, .2, 0, 164, 32, [-32, 32]),
        (0, 20, -.2, 0, 164, -32, [32, -32]),
        (0, 200, 5, 100, 32767, 1000, [-1000, 1000]),
        (0, 200, 5, -100, -32768, -1000, [1000, -1000]),
        (4800, 200, 0, 100, 0, 200, [4600, 5000]),
        (-4800, 200, 0, 100, 0, 200, [-5000, -4600]),
        (6000, 20, 0, 5, 0, 0, [5000, 5000]),
        (-6000, 20, 0, -5, 0, 0, [-5000, -5000]),
    ]
    for balance, kp, kd, command, raw_gyro, expected_turn, expected_motors in cases:
        sample.update(balance=balance, gyro=raw_gyro)
        firmware.write("Turn_Kp", "<f", kp)
        firmware.write("Turn_Kd", "<f", kd)
        firmware.write("Target_Turn", "<i", command)
        firmware.call("control")
        assert motors == expected_motors, (balance, kp, kd, command, raw_gyro, motors)
        assert firmware.read("Turn_out", "<i") == expected_turn
        assert all(-5000 <= value <= 5000 for value in motors)
        assert sum(motors) / 2 == min(5000, max(-5000, balance))
    print(f"PASS: {len(cases)} steering/motor cases; UART application, legacy compatibility and integral retention")


if __name__ == "__main__":
    main()
