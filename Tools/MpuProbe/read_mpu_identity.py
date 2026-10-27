"""Read the MPU identity on PB8/PB9 via CMSIS-DAP, without flashing or resetting.

The CPU is briefly halted only when TIM1 motor PWM is inactive.
The script restores the GPIO configuration and the original CPU run state.
"""

import argparse
import json
import time
from pathlib import Path

GPIOB = 0x40010C00
RCC_APB2ENR = 0x40021018
TIM1 = 0x40012C00
SCL = 1 << 8
SDA = 1 << 9
BUS_MASK = SCL | SDA


def motor_pwm_active(target):
    control = target.read32(TIM1)
    channels = target.read32(TIM1 + 0x20)
    outputs = target.read32(TIM1 + 0x44)
    return bool(control & 1 and outputs & 0x8000 and (
        (channels & 1 and target.read32(TIM1 + 0x34)) or
        (channels & 0x1000 and target.read32(TIM1 + 0x40))))


class SensorBus:
    def __init__(self, target):
        self.target = target
        self.clock = None
        self.config = None
        self.output = None

    def pin(self, mask, high):
        self.target.write32(GPIOB + 0x10, mask if high else mask << 16)
        self.target.flush()
        time.sleep(0.00001)
        if mask == SCL and high:
            deadline = time.monotonic() + 0.1
            while not self.target.read32(GPIOB + 0x08) & SCL:
                if time.monotonic() > deadline:
                    raise RuntimeError('SCL remains low; check sensor power and wiring')

    def sample_sda(self):
        return bool(self.target.read32(GPIOB + 0x08) & SDA)

    def __enter__(self):
        self.clock = self.target.read32(RCC_APB2ENR)
        try:
            self.target.write32(RCC_APB2ENR, self.clock | 8)
            self.target.flush()
            self.config = self.target.read32(GPIOB + 0x04)
            self.output = self.target.read32(GPIOB + 0x0C) & BUS_MASK
            # Only PB8/PB9 change: 2 MHz open-drain outputs, initially released.
            self.target.write32(GPIOB + 0x10, BUS_MASK)
            self.target.write32(GPIOB + 0x04, (self.config & ~0xFF) | 0x55)
            self.target.flush()
            self.pin(SCL, True)
            if not self.sample_sda():
                raise RuntimeError('SDA is low; the I2C bus is not idle')
            return self
        except Exception:
            self.restore()
            raise

    def restore(self):
        if self.output is not None:
            set_bits = self.output
            reset_bits = BUS_MASK & ~self.output
            self.target.write32(GPIOB + 0x10, set_bits | (reset_bits << 16))
        if self.config is not None:
            self.target.write32(GPIOB + 0x04, self.config)
        if self.clock is not None:
            self.target.write32(RCC_APB2ENR, self.clock)
        self.target.flush()

    def __exit__(self, *_):
        try:
            self.stop()
        finally:
            self.restore()

    def start(self):
        self.pin(SDA, True)
        self.pin(SCL, True)
        self.pin(SDA, False)
        self.pin(SCL, False)

    def stop(self):
        self.pin(SCL, False)
        self.pin(SDA, False)
        self.pin(SCL, True)
        self.pin(SDA, True)

    def send(self, value):
        for shift in range(7, -1, -1):
            self.pin(SDA, bool(value & (1 << shift)))
            self.pin(SCL, True)
            self.pin(SCL, False)
        self.pin(SDA, True)
        self.pin(SCL, True)
        ack = not self.sample_sda()
        self.pin(SCL, False)
        return ack

    def receive(self, ack):
        self.pin(SDA, True)
        value = 0
        for _ in range(8):
            self.pin(SCL, True)
            value = (value << 1) | int(self.sample_sda())
            self.pin(SCL, False)
        self.pin(SDA, not ack)
        self.pin(SCL, True)
        self.pin(SCL, False)
        self.pin(SDA, True)
        return value

    def read(self, address, register, length=1):
        try:
            self.start()
            if not self.send(address << 1):
                return None
            if not self.send(register):
                raise RuntimeError(f'0x{address:02X}: register address NACK')
            self.start()
            if not self.send((address << 1) | 1):
                raise RuntimeError(f'0x{address:02X}: read address NACK')
            return [self.receive(i < length - 1) for i in range(length)]
        finally:
            self.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', help='CMSIS-DAP unique ID')
    parser.add_argument('--backend', choices=('native', 'pyocd'), default='native')
    parser.add_argument('--output', type=Path, help='Save actual readings as JSON')
    args = parser.parse_args()
    if args.backend == 'native':
        from cmsis_dap_native import NativeTarget
        session = NativeTarget()
        session.target = session
        Target = NativeTarget
    else:
        from pyocd.core.helpers import ConnectHelper
        from pyocd.core.target import Target
        session = ConnectHelper.session_with_chosen_probe(
            unique_id=args.probe, blocking=False, return_first=False,
            target_override='cortex_m', options={
                'connect_mode': 'attach',
                'halt_on_connect': False,
                'resume_on_disconnect': False,
                'frequency': 500000,
                'vector_catch': 'none',
                'no_config': True,
            })
    if session is None:
        raise RuntimeError('No connected debug probe found')

    result = {'method': 'SWD attach / PB8-PB9 I2C read', 'sensors': []}
    with session:
        target = session.target
        device_id = target.read32(0xE0042000)
        result['stm32_debug_id'] = f'0x{device_id:08X}'
        if device_id & 0xFFF != 0x410:
            raise RuntimeError(f'Unexpected MCU debug ID: 0x{device_id:08X}')
        if motor_pwm_active(target):
            raise RuntimeError('Motor PWM is active; stop the car before reading')
        was_running = target.get_state() == Target.State.RUNNING
        if was_running:
            target.halt()
        try:
            if motor_pwm_active(target):
                raise RuntimeError('Motor PWM became active; stop the car before reading')
            result['cpu_pc'] = f'0x{target.read_core_register("pc"):08X}'
            with SensorBus(target) as bus:
                for address in (0x68, 0x69):
                    samples = [bus.read(address, 0x75) for _ in range(3)]
                    if all(sample is None for sample in samples):
                        continue
                    if any(sample is None for sample in samples):
                        raise RuntimeError(f'0x{address:02X}: inconsistent I2C responses')
                    identity = samples[0][0]
                    if any(sample[0] != identity for sample in samples):
                        raise RuntimeError(f'0x{address:02X}: unstable WHO_AM_I readings')
                    sensor = {
                        'i2c_address': f'0x{address:02X}',
                        'who_am_i_samples': [f'0x{sample[0]:02X}' for sample in samples],
                        'reported_model': {0x68: 'MPU6050', 0x70: 'MPU6500',
                                           0x71: 'MPU9250', 0x73: 'MPU9255'}.get(identity, 'unknown'),
                    }
                    if identity == 0x68:
                        product = bus.read(address, 0x0C)
                        offsets = bus.read(address, 0x06, 6)
                        if product is not None and offsets is not None:
                            sensor['product_id'] = f'0x{product[0]:02X}'
                            sensor['software_revision'] = ((offsets[5] & 1) << 2) | (
                                (offsets[3] & 1) << 1) | (offsets[1] & 1)
                    result['sensors'].append(sensor)
        finally:
            if was_running:
                target.resume()
    if not result['sensors']:
        raise RuntimeError('Neither 0x68 nor 0x69 acknowledged on PB8/PB9')
    text = json.dumps(result, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
