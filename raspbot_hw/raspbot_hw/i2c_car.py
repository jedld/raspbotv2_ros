import math
from typing import Optional


class I2CCar:
    """I2C wrapper for Yahboom Raspbot motor controller.

    Supported protocols:

    - legacy (common older examples): addr 0x16
        reg 0x01: [l_dir, l_speed, r_dir, r_speed]
        reg 0x02: stop (write 0x00)
        dir: 0 backward, 1 forward

    - pi5 (newer Raspbot_Lib): addr 0x2B
        reg 0x01: [motor_id, motor_dir, motor_speed]
        motor_id: 0=L1, 1=L2, 2=R1, 3=R2
        motor_dir: 0 forward, 1 backward
        motor_speed: 0..255

    Set protocol='auto' to pick based on i2c_addr.
    """

    def __init__(
        self,
        i2c_bus: int = 1,
        i2c_addr: int = 0x16,
        dry_run: bool = False,
        protocol: str = 'auto',
    ):
        self._addr = int(i2c_addr)
        self._bus_num = int(i2c_bus)
        self._dry_run = bool(dry_run)
        self._protocol = self._resolve_protocol(protocol)
        self._bus = None if self._dry_run else self._open_bus(self._bus_num)

    def _resolve_protocol(self, protocol: str) -> str:
        p = str(protocol).strip().lower()
        if p == 'auto':
            return 'pi5' if self._addr == 0x2B else 'legacy'
        if p in {'legacy', 'pi5'}:
            return p
        raise ValueError("protocol must be auto, legacy, or pi5")

    @staticmethod
    def _open_bus(i2c_bus: int):
        try:
            import smbus  # type: ignore

            return smbus.SMBus(i2c_bus)
        except Exception:
            # smbus2 is a common fallback on Ubuntu.
            import smbus2  # type: ignore

            return smbus2.SMBus(i2c_bus)

    def _write_u8(self, reg: int, data: int) -> None:
        if self._dry_run:
            return
        self._bus.write_byte_data(self._addr, int(reg), int(data) & 0xFF)

    def _write_block(self, reg: int, data) -> None:
        if self._dry_run:
            return
        self._bus.write_i2c_block_data(self._addr, int(reg), [int(x) & 0xFF for x in data])

    def _read_block(self, reg: int, length: int):
        if self._dry_run:
            return [0 for _ in range(int(length))]
        return self._bus.read_i2c_block_data(self._addr, int(reg), int(length))

    def _read_u8(self, reg: int) -> int:
        return int(self._read_block(reg, 1)[0]) & 0xFF

    @staticmethod
    def _dir_and_speed(speed: float, max_pwm: int) -> tuple[int, int]:
        direction = 1
        if speed < 0.0:
            direction = 0
        pwm = int(round(min(max(abs(speed), 0.0), float(max_pwm))))
        return direction, pwm

    @staticmethod
    def _pi5_dir_and_speed(speed: float, max_pwm: int) -> tuple[int, int]:
        # Vendor lib: dir=0 forward, dir=1 backward
        direction = 0
        if speed < 0.0:
            direction = 1
        pwm = int(round(min(max(abs(speed), 0.0), float(min(max_pwm, 255)))))
        return direction, pwm

    def set_wheel_pwms(self, left_pwm: float, right_pwm: float, max_pwm: int = 100) -> None:
        if self._protocol == 'legacy':
            l_dir, l_speed = self._dir_and_speed(left_pwm, max_pwm)
            r_dir, r_speed = self._dir_and_speed(right_pwm, max_pwm)
            self._write_block(0x01, [l_dir, l_speed, r_dir, r_speed])
            return

        # pi5 protocol: command all 4 motors as two sides.
        l_dir, l_speed = self._pi5_dir_and_speed(left_pwm, max_pwm)
        r_dir, r_speed = self._pi5_dir_and_speed(right_pwm, max_pwm)

        # motor_id: 0=L1, 1=L2, 2=R1, 3=R2
        for motor_id in (0, 1):
            self._write_block(0x01, [motor_id, l_dir, l_speed])
        for motor_id in (2, 3):
            self._write_block(0x01, [motor_id, r_dir, r_speed])

    @property
    def protocol(self) -> str:
        return self._protocol

    def set_motor_pwms(
        self,
        fl_pwm: float,
        fr_pwm: float,
        rl_pwm: float,
        rr_pwm: float,
        *,
        max_pwm: int = 100,
        motor_id_fl: int = 0,
        motor_id_rl: int = 1,
        motor_id_fr: int = 2,
        motor_id_rr: int = 3,
    ) -> None:
        """Set individual wheel PWMs (mecanum / holonomic).

        For legacy protocol (0x16), the controller only supports left/right. In that case we
        approximate by averaging the two left wheels and the two right wheels.

        For pi5 protocol (0x2B), each motor is commanded individually.
        """

        if self._protocol == 'legacy':
            left_pwm = 0.5 * (float(fl_pwm) + float(rl_pwm))
            right_pwm = 0.5 * (float(fr_pwm) + float(rr_pwm))
            self.set_wheel_pwms(left_pwm, right_pwm, max_pwm=max_pwm)
            return

        # pi5 protocol
        mapping = (
            (int(motor_id_fl), float(fl_pwm)),
            (int(motor_id_rl), float(rl_pwm)),
            (int(motor_id_fr), float(fr_pwm)),
            (int(motor_id_rr), float(rr_pwm)),
        )

        for motor_id, pwm in mapping:
            direction, speed = self._pi5_dir_and_speed(pwm, max_pwm)
            self._write_block(0x01, [motor_id, direction, speed])

    def stop(self) -> None:
        if self._protocol == 'legacy':
            self._write_u8(0x02, 0x00)
            return

        # pi5 protocol: set all motor speeds to 0.
        for motor_id in (0, 1, 2, 3):
            self._write_block(0x01, [motor_id, 0, 0])

    def set_servo(self, servo_id: int, angle_deg: float) -> None:
        angle = int(round(float(angle_deg)))
        if angle < 0:
            angle = 0
        if angle > 180:
            angle = 180

        sid = int(servo_id)

        # Some vendor firmwares limit servo 2 travel.
        if self._protocol == 'pi5' and sid == 2 and angle > 110:
            angle = 110

        # legacy notebooks: reg 0x03
        # pi5 vendor lib: reg 0x02
        reg = 0x03 if self._protocol == 'legacy' else 0x02
        self._write_block(reg, [sid, angle])

    def set_ultrasonic_enabled(self, enabled: bool) -> None:
        """Enable/disable ultrasonic measurement on the Pi5 controller.

        Vendor library uses reg 0x07 with [state] where state is 0/1.
        """
        if self._protocol != 'pi5':
            return
        state = 1 if bool(enabled) else 0
        self._write_block(0x07, [state])

    def read_ultrasonic_m(self) -> float:
        """Read ultrasonic distance (meters) from Pi5 controller registers.

        Vendor library example:
          diss_H = read(0x1b)
          diss_L = read(0x1a)
          dis_mm = diss_H<<8 | diss_L
        """
        if self._protocol != 'pi5':
            raise RuntimeError('ultrasonic read is only supported on pi5 protocol')
        diss_l = self._read_u8(0x1A)
        diss_h = self._read_u8(0x1B)
        mm = (diss_h << 8) | diss_l
        if mm <= 0:
            return float('nan')
        return float(mm) / 1000.0

    def close(self) -> None:
        if self._dry_run or self._bus is None:
            return
        close_method: Optional[object] = getattr(self._bus, "close", None)
        if callable(close_method):
            close_method()
