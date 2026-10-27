"""Run the real Keil ARM firmware in Unicorn; no board/COM port is opened.

Usage: python test_camera_uart.py firmware.axf path/to/JDY31PidTuner/tests
Requires unicorn. The camera library itself is not part of this test.
"""
import struct
import sys

sys.path.insert(0, sys.argv[2])
from test_turn_control import Firmware
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2


def packet(error=0, valid=1, y=120, fields=None):
    if fields is None:
        fields = [1, error + 1000, error + 320 if valid else 0, y, valid]
    tail = ',' + ','.join(map(str, fields)) + '#'
    size = len('$0' + tail)
    while len('$' + str(size) + tail) != size:
        size = len('$' + str(size) + tail)
    return ('$' + str(size) + tail).encode('ascii')


class CameraFirmware(Firmware):
    def __init__(self, target_speed=30):
        super().__init__(sys.argv[1])
        self.write('Target_Speed', '<i', target_speed)
        self.camera_produced = 0
        self.cpu.mem_write(0x40004400, struct.pack('<I', 0xC0))
        assert self.call('Camera_Init') == 0
        self.ring = struct.unpack('<I', self.cpu.mem_read(0x40020078, 4))[0]
        assert 0x20000000 <= self.ring < 0x20005000
        assert self.read('Target_Speed', '<i') == target_speed
        assert self.read('Camera_Enabled', '<B') == 1

    def send(self, data, process=True):
        while data:
            offset = self.camera_produced % 512
            count = min(len(data), 512 - offset)
            self.cpu.mem_write(self.ring + offset, data[:count])
            data = data[count:]
            self.camera_produced += count
            remaining = 512 - self.camera_produced % 512
            self.cpu.mem_write(0x40020070, struct.pack('<I', remaining))
            if remaining == 512:
                # Let the actual DMA IRQ -> HAL callback count the wrap.
                self.cpu.mem_write(0x40020000, struct.pack('<I', 0x00200000))
                self.call('DMA1_Channel6_IRQHandler')
                self.cpu.mem_write(0x40020000, b'\0' * 4)
            if process:
                self.call('Camera_Process')

    def targets(self):
        raw = self.call('Camera_ControlTargets')
        speed = struct.unpack('<i', struct.pack('<I', raw))[0]
        return speed, self.read('Target_Turn', '<i')


def main():
    checks = 0
    for requested in [0, 12, 30, -17]:
        f = CameraFirmware(requested)
        assert f.targets() == (0, 0)
        assert f.read('Target_Speed', '<i') == requested
        f.send(packet(60))
        assert f.targets() == (requested, 60)
        assert f.read('Target_Speed', '<i') == requested
        f.send(packet(0, valid=0))
        assert f.targets() == (0, 0)
        assert f.read('Target_Speed', '<i') == requested
        f.send(packet(-60))
        assert f.targets() == (requested, -60)
        f.write('Target_Speed', '<i', requested + 3)
        assert f.targets() == (requested + 3, -60)
        checks += 5
    f = CameraFirmware()
    assert f.targets() == (0, 0), 'No initial frame must not drive forward'
    for error in [-320, -200, -60, 0, 60, 200, 319]:
        f.send(packet(error))
        assert f.targets() == (30, error)
        assert f.read('Camera_Error', '<i') == error
        checks += 1
    assert f.read('Camera_FrameCount', '<I') == 7
    f.send(packet(0, valid=0))
    assert f.targets() == (0, 0), 'Invalid recognition must clear motion'
    f.send(packet(-200))
    assert f.targets() == (30, -200)
    f.write('uwTick', '<I', 500)
    assert f.targets() == (30, -200)
    f.write('uwTick', '<I', 501)
    assert f.targets() == (0, 0), 'Receive timeout must keep only balance active'
    checks += 4

    f = CameraFirmware()
    data = packet(60)
    for split in range(1, len(data)):
        f.write('Camera_Valid', '<B', 0)
        f.send(data[:split])
        assert f.targets() == (0, 0)
        f.send(data[split:])
        assert f.targets() == (30, 60)
        checks += 1
    while f.camera_produced < 540:
        f.send(data)
        assert f.targets() == (30, 60)
    assert f.read('camera_wraps', '<I') >= 1, 'Exercise DMA wrap in a split frame'
    f.send(b'noise\r\n$999,1,' + packet(-60) + packet(200))
    assert f.targets() == (30, 200), 'Resync at $, accept consecutive frames'
    checks += 1

    f = CameraFirmware()
    malformed = [
        b'$1,1,1000,320,120,1#',
        packet(fields=[2, 1000, 320, 120, 1]),
        packet(fields=[1, 1320, 640, 120, 1]),
        packet(fields=[1, 679, 0, 120, 1]),
        packet(fields=[1, 1000, 320, 480, 1]),
        packet(fields=[1, 1000, 320, 120, 2]),
        packet(fields=[1, 1060, 320, 120, 1]),
        packet(fields=[1, 1000, 320, 120, 1, 0]),
        packet(fields=[1, 1000, 320, 120, '']),
        packet(fields=[1, '10\x000', 320, 120, 1]),
        packet(fields=[1, '-1000', 320, 120, 1]),
        packet(fields=[1, '9999999999999', 320, 120, 1]),
        b'$' + b'9' * 80 + b'#',
    ]
    for broken in malformed:
        f.send(packet(60))
        before = f.read('Camera_FrameCount', '<I')
        f.send(broken)
        assert f.targets() == (0, 0), broken
        assert f.read('Camera_FrameCount', '<I') == before, broken
        checks += 1
    f.send(packet(200))
    assert f.targets() == (30, 200), 'Recover after malformed input'
    f.send(b'x' * 600, process=False)
    f.call('Camera_Process')
    assert f.targets() == (0, 0)
    assert f.read('Camera_Overruns', '<I') == 1
    f.send(packet(-200))
    assert f.targets() == (30, -200)
    checks += 3

    f = CameraFirmware()
    f.write('uwTick', '<I', 0xFFFFFFF0)
    f.send(packet(60))
    f.write('uwTick', '<I', (0xFFFFFFF0 + 500) & 0xFFFFFFFF)
    assert f.targets() == (30, 60)
    f.write('uwTick', '<I', (0xFFFFFFF0 + 501) & 0xFFFFFFFF)
    assert f.targets() == (0, 0), 'Timeout must work across HAL tick rollover'
    checks += 2

    f = CameraFirmware()
    f.send(packet(60))
    f.cpu.mem_write(0x40004400, struct.pack('<I', 0xC8))  # UART overrun
    f.call('USART2_IRQHandler')
    f.cpu.mem_write(0x40004400, struct.pack('<I', 0xC0))
    assert f.read('camera_receive_error', '<B') == 1
    assert f.targets() == (0, 0), 'UART error must stop motion before foreground recovery'
    f.call('Camera_Process')
    assert f.read('camera_receive_error', '<B') == 0
    f.camera_produced = 0  # Real DMA was restarted at offset zero.
    f.send(packet(-60))
    assert f.targets() == (30, -60)
    checks += 2

    f = CameraFirmware()
    assert f.call('Bluetooth_Init') == 0
    f.send(packet(200))
    assert f.targets() == (30, 200)
    assert f.receive('PING') == b'PONG\r\n'
    assert f.read('Camera_Enabled', '<B') == 1, 'Heartbeat must not pause tracking'
    assert f.receive('PID,330,1.55,0.4,0.002,1.6,0,1,1.1,0') == b'OK\r\n'
    f.send(packet(-200))
    assert f.targets() == (0, 0), 'Camera must not undo a manual stop'
    assert f.read('Camera_Enabled', '<B') == 0
    assert f.receive('CAM,1') == b'OK\r\n'
    f.send(packet(-200))
    assert f.targets() == (0, -200), 'Resume must preserve a zero global speed'
    assert f.receive('PID,330,1.55,0.4,0.002,1.6,30,1,1.1,0') == b'OK\r\n'
    assert f.receive('CAM,1') == b'OK\r\n'
    assert f.targets() == (0, 0), 'Resume must wait for a fresh frame'
    f.send(packet(-200))
    assert f.targets() == (30, -200)
    partial = packet(60)
    f.send(partial[:10])
    assert f.receive('CAM,0') == b'OK\r\n'
    assert f.receive('CAM,1') == b'OK\r\n'
    f.send(partial[10:])
    assert f.targets() == (0, 0), 'Resume must discard a frame started before the mode switch'
    assert f.receive('CAM,0') == b'OK\r\n'
    f.send(packet(200))
    assert f.targets() == (0, 0)
    f.write('tilt_stop_latched', '<B', 1)
    assert f.receive('CAM,1') == b'ERR,TILT\r\n'
    assert f.targets() == (0, 0), 'Resume must not bypass tilt protection'
    checks += 9

    # Execute real steering and mixing; stub only sensors and motor hardware.
    f = CameraFirmware()
    motors = []
    f.stub('Read_Speed')
    f.stub('mpu_dmp_get_data')
    f.stub('MPU_Get_Gyroscope')
    f.stub('MPU_Get_Accelerometer')
    f.stub('Velocity')
    f.stub('Vertical', lambda: 4800)
    f.stub('Load', lambda: motors.extend([
        struct.unpack('<i', struct.pack('<I', f.cpu.reg_read(r)))[0]
        for r in (UC_ARM_REG_R0, UC_ARM_REG_R1)]))
    f.write('Turn_Kp', '<f', 1)
    f.write('Turn_Kp2', '<f', .01)
    f.write('Turn_Kd', '<f', 0)
    for error, expected in [(60, 96), (-60, -96), (200, 200), (-200, -200)]:
        motors.clear()
        f.send(packet(error))
        f.call('control')
        assert f.read('Turn_out', '<i') == expected
        assert motors == [4800 - expected, 4800 + expected]
        assert max(map(abs, motors)) <= 5000
        checks += 1
    print(f'PASS: {checks} camera UART/control cases against actual ARM firmware')
    print('Includes split/concatenated frames, DMA wrap/overflow, timeout, invalid data, manual stop and balance-priority mixing.')


if __name__ == '__main__':
    main()
