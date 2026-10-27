"""Exercise tilt shutdown in actual Keil-built ARM code without real motors.

Sensor reads are mocked; the real motor driver writes emulated TIM1 registers.
Requires unicorn. Usage: python test_tilt_shutdown.py path/to/firmware.axf
"""
import sys

from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2
from test_turn_control import Firmware


TIM1_BASE = 0x40012C00
TIM1_CCR1 = TIM1_BASE + 0x34
TIM1_CCR4 = TIM1_BASE + 0x40


class TiltFixture:
    def __init__(self, filename):
        self.firmware = Firmware(filename)
        self.pitch = 3.0
        self.encoder = 0
        self.gyro_z = 0
        fw = self.firmware

        def attitude():
            import struct
            fw.cpu.mem_write(fw.cpu.reg_read(UC_ARM_REG_R0),
                             struct.pack("<f", self.pitch))

        def gyro():
            import struct
            for register, value in [(UC_ARM_REG_R0, 0), (UC_ARM_REG_R1, 0),
                                    (UC_ARM_REG_R2, self.gyro_z)]:
                fw.cpu.mem_write(fw.cpu.reg_read(register), struct.pack("<h", value))

        fw.stub("Read_Speed", lambda: self.encoder)
        fw.stub("mpu_dmp_get_data", attitude)
        fw.stub("MPU_Get_Gyroscope", gyro)
        fw.stub("MPU_Get_Accelerometer")
        # TIM1 initialization is normally done by main; keep the real Load().
        fw.write("htim1", "<I", TIM1_BASE)
        for name, value in [("Vertical_Kp", 100), ("Vertical_Kd", 0),
                            ("Velocity_Kp", 0), ("Velocity_Ki", 0),
                            ("Med_Angle", 2), ("Turn_Kp", 20), ("Turn_Kd", 0)]:
            fw.write(name, "<f", value)
        fw.write("Target_Speed", "<i", 0)
        fw.write("Target_Turn", "<i", 0)

    def run(self):
        self.firmware.call("control")

    def duties(self):
        import struct
        return [struct.unpack("<I", self.firmware.cpu.mem_read(address, 4))[0]
                for address in (TIM1_CCR1, TIM1_CCR4)]

    def assert_stopped(self):
        fw = self.firmware
        assert fw.read("tilt_stop_latched", "<B") == 1
        assert self.duties() == [0, 0], self.duties()
        for name in ("Target_Speed", "Target_Turn", "Vertical_out", "Velocity_out",
                     "Turn_out", "MOTOR1", "MOTOR2"):
            assert fw.read(name, "<i") == 0, name
        for name in ("velocity_error_last", "velocity_error_sum",
                     "Vertical_Kp_Component", "Vertical_Kd_Component",
                     "Velocity_Kp_Component", "Velocity_Ki_Component"):
            assert fw.read(name, "<f") == 0, name


def main(filename):
    # Upright is 2 degrees: the strict boundaries are -28 and +32 degrees.
    for pitch in (2.0, 31.99, 32.0, -27.99, -28.0):
        fixture = TiltFixture(filename)
        fixture.pitch = pitch
        fixture.run()
        assert fixture.firmware.read("tilt_stop_latched", "<B") == 0, pitch
        if pitch != 2.0:
            assert all(value > 0 for value in fixture.duties()), pitch

    for pitch in (32.01, -28.01):
        fixture = TiltFixture(filename)
        fw = fixture.firmware
        fixture.encoder = -8
        fw.write("Velocity_Kp", "<f", 0.11)
        fw.write("Velocity_Ki", "<f", 0.0029)
        fw.write("Target_Turn", "<i", 3)
        for _ in range(8):
            fixture.run()
        assert fw.read("velocity_error_sum", "<f") > 0
        assert fw.read("velocity_error_last", "<f") > 0
        assert any(value > 0 for value in fixture.duties())

        fixture.pitch = pitch
        fixture.run()
        fixture.assert_stopped()

        # Returning upright and requesting motion cannot re-arm the motors.
        fixture.pitch = 2.0
        fixture.gyro_z = 300
        fw.write("Target_Speed", "<i", 12)
        fw.write("Target_Turn", "<i", 5)
        fw.write("Turn_Kd", "<f", 0.2)
        for _ in range(3):
            fixture.run()
            fixture.assert_stopped()

        assert fw.call("Bluetooth_Init") == 0
        assert fw.receive("PID,350,0.47,0.11,0.0029,2,12,20,-0.1,5") == b"OK\r\n"
        assert fw.read("Target_Speed", "<i") == 0
        assert fw.read("Target_Turn", "<i") == 0
        assert abs(fw.read("Turn_Kd", "<f") + 0.1) < 1e-6
        fixture.run()
        fixture.assert_stopped()

        # Changing the calibration must not clear a previously latched stop.
        assert fw.receive("PID,350,0.47,0.11,0.0029,20,0,20,0,0") == b"OK\r\n"
        fixture.run()
        fixture.assert_stopped()
        assert fw.receive("PING") == b"PONG\r\n"

    # The safety threshold must not follow a moving speed-loop angle target.
    fixture = TiltFixture(filename)
    fixture.pitch = 33.0
    fixture.firmware.write("Velocity_out", "<i", 31)
    fixture.run()
    fixture.assert_stopped()

    # A fresh MCU initialization clears the latch and restores normal control.
    fixture = TiltFixture(filename)
    fixture.run()
    assert fixture.firmware.read("tilt_stop_latched", "<B") == 0
    assert fixture.duties() == [100, 100]
    print("PASS: strict +/-30-degree boundaries, bidirectional shutdown, actual PWM "
          "zeroing, state cleanup, persistent interlock, UART motion blocking and reset")


if __name__ == "__main__":
    main(sys.argv[1])
