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

    @staticmethod
    def _dir_and_speed(speed: float, max_pwm: int) -> tuple[int, int]:
        direction = 1
        if speed < 0.0:
            direction = 0
        pwm = int(min(max(abs(speed), 0.0), float(max_pwm)))
        return direction, pwm

    @staticmethod
    def _pi5_dir_and_speed(speed: float, max_pwm: int) -> tuple[int, int]:
        # Vendor lib: dir=0 forward, dir=1 backward
        direction = 0
        if speed < 0.0:
            direction = 1
        pwm = int(min(max(abs(speed), 0.0), float(min(max_pwm, 255))))
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

    def stop(self) -> None:
        if self._protocol == 'legacy':
            self._write_u8(0x02, 0x00)
            return

        # pi5 protocol: set all motor speeds to 0.
        for motor_id in (0, 1, 2, 3):
            self._write_block(0x01, [motor_id, 0, 0])

    def close(self) -> None:
        if self._dry_run or self._bus is None:
            return
        close_method: Optional[object] = getattr(self._bus, "close", None)
        if callable(close_method):
            close_method()
