"""Check UART/DMA PING during normal startup and failed MPU initialization.

Runs actual ARM code with mocked sensor, display, timers, and peripheral registers.
Requires unicorn; no real hardware or motors are accessed.
"""
import struct
import sys

from unicorn import Uc, UC_ARCH_ARM, UC_HOOK_CODE, UC_HOOK_MEM_WRITE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0,
    UC_ARM_REG_R1, UC_ARM_REG_SP)
from test_pid_protocol import load_elf


def check_startup(firmware, mpu_result):
    segments, symbols = load_elf(firmware)
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for address, size in [(0x08000000, 0x20000), (0x20000000, 0x10000),
                          (0x40000000, 0x40000), (0xE000E000, 0x2000)]:
        cpu.mem_map(address, size)
    for address, data in segments:
        cpu.mem_write(address, data)
    cpu.reg_write(UC_ARM_REG_SP, 0x2000FFF0)
    cpu.emu_start(symbols["__main"] | 1, symbols["main"] & ~1,
                   timeout=2000000, count=500000)
    events, replies = [], []
    counts = {"delay": 0, "control": 0, "pwm": 0, "encoders": 0}

    def read32(address):
        return struct.unpack("<I", cpu.mem_read(address, 4))[0]

    def write32(address, value):
        cpu.mem_write(address, struct.pack("<I", value))

    def hook(name, handler=None, stub=True):
        def invoke(machine, address, size, unused):
            result = handler() if handler else 0
            if stub:
                machine.reg_write(UC_ARM_REG_R0, result or 0)
                machine.reg_write(UC_ARM_REG_PC, machine.reg_read(UC_ARM_REG_LR))
        address = symbols[name] & ~1
        cpu.hook_add(UC_HOOK_CODE, invoke, begin=address, end=address)

    def count(name):
        counts[name] += 1

    def sensor_init():
        events.append("mpu")
        ring = read32(0x4002003C)  # DMA1_Channel3 CMAR.
        assert 0x20000000 <= ring < 0x20010000, "UART DMA must start before MPU initialization"
        cpu.mem_write(ring, b"PING\n")
        write32(0x40020034, 507)  # DMA1_Channel3 CNDTR after receiving five bytes.
        return mpu_result

    def delay():
        write32(symbols["uwTick"], read32(symbols["uwTick"]) + 1)
        counts["delay"] += 1
        if counts["delay"] >= 5:
            cpu.emu_stop()

    def stopped_motor():
        assert cpu.reg_read(UC_ARM_REG_R0) == 0
        assert cpu.reg_read(UC_ARM_REG_R1) == 0

    for name in ["HAL_Init", "SystemClock_Config", "MX_GPIO_Init", "MX_TIM2_Init",
                 "MX_TIM1_Init", "MX_TIM4_Init", "OLED_Init", "OLED_Clear", "OLED_ShowString"]:
        hook(name)
    hook("Bluetooth_Init", lambda: events.append("bluetooth"), stub=False)
    hook("mpu_dmp_init", sensor_init)
    hook("HAL_Delay", delay)
    hook("Load", stopped_motor)
    hook("HAL_TIM_PWM_Start", lambda: count("pwm"))
    hook("HAL_TIM_Encoder_Start", lambda: count("encoders"))
    hook("control", lambda: count("control"))
    cpu.hook_add(UC_HOOK_MEM_WRITE,
        lambda machine, access, address, size, value, unused: replies.append(value & 255),
        begin=0x40004804, end=0x40004807)
    write32(0x40004800, 0xC0)  # USART3 TXE and TC; transmit immediately in this model.

    def call(name, arg=0):
        cpu.reg_write(UC_ARM_REG_SP, 0x2000FFF0)
        cpu.reg_write(UC_ARM_REG_R0, arg)
        cpu.reg_write(UC_ARM_REG_LR, 0x0801FF01)
        cpu.emu_start(symbols[name] | 1, 0x0801FF00, timeout=2000000, count=1000000)

    call("HAL_GPIO_EXTI_Callback", 1 << 14)
    assert counts["control"] == 0, "Balance IRQ must not run before initialization"
    call("main")
    assert events == ["bluetooth", "mpu"], events
    assert bytes(replies) == b"PONG\r\n", replies
    if mpu_result:
        assert counts["pwm"] == counts["encoders"] == 0
        assert cpu.mem_read(symbols["control_ready"], 1) == b"\0"
    else:
        assert counts["pwm"] == counts["encoders"] == 2
        assert cpu.mem_read(symbols["control_ready"], 1) == b"\1"
    call("HAL_GPIO_EXTI_Callback", 1 << 14)
    assert counts["control"] == (0 if mpu_result else 1)


if __name__ == "__main__":
    for status in (0, 1):
        check_startup(sys.argv[1], status)
    print("PASS: PING replies in normal/MPU-error startup; balance IRQ and motor-start guards")
