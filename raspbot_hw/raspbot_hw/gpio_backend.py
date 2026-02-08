from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Dict, Optional


_BOARD_TO_BCM: Dict[int, int] = {
    3: 2,
    5: 3,
    7: 4,
    8: 14,
    10: 15,
    11: 17,
    12: 18,
    13: 27,
    15: 22,
    16: 23,
    18: 24,
    19: 10,
    21: 9,
    22: 25,
    23: 11,
    24: 8,
    26: 7,
    29: 5,
    31: 6,
    32: 12,
    33: 13,
    35: 19,
    36: 16,
    37: 26,
    38: 20,
    40: 21,
}


def _to_bcm(pin: int, gpio_mode: str) -> int:
    mode = gpio_mode.upper()
    if mode == "BCM":
        return int(pin)
    if mode == "BOARD":
        try:
            return int(_BOARD_TO_BCM[int(pin)])
        except KeyError as e:
            raise ValueError(f"Unsupported BOARD pin {pin} (add mapping if needed)") from e
    raise ValueError(f"gpio_mode must be BOARD or BCM (got {gpio_mode!r})")


@dataclass
class GpioConfig:
    backend: str = "auto"  # auto|rpi|lgpio
    mode: str = "BOARD"  # BOARD|BCM
    lgpio_chip: int = 0


class BaseGpio:
    def setup_in(self, pin: int) -> None:
        raise NotImplementedError

    def setup_out(self, pin: int, initial: bool = False) -> None:
        raise NotImplementedError

    def read(self, pin: int) -> bool:
        raise NotImplementedError

    def write(self, pin: int, value: bool) -> None:
        raise NotImplementedError

    def cleanup(self) -> None:
        raise NotImplementedError


class RPiGpioBackend(BaseGpio):
    def __init__(self, gpio_mode: str):
        import RPi.GPIO as GPIO  # type: ignore

        self._GPIO = GPIO
        self._GPIO.setwarnings(False)
        if gpio_mode.upper() == "BCM":
            self._GPIO.setmode(self._GPIO.BCM)
        else:
            self._GPIO.setmode(self._GPIO.BOARD)

    def setup_in(self, pin: int) -> None:
        self._GPIO.setup(int(pin), self._GPIO.IN)

    def setup_out(self, pin: int, initial: bool = False) -> None:
        self._GPIO.setup(int(pin), self._GPIO.OUT)
        self._GPIO.output(int(pin), self._GPIO.HIGH if initial else self._GPIO.LOW)

    def read(self, pin: int) -> bool:
        return bool(self._GPIO.input(int(pin)))

    def write(self, pin: int, value: bool) -> None:
        self._GPIO.output(int(pin), self._GPIO.HIGH if value else self._GPIO.LOW)

    def cleanup(self) -> None:
        self._GPIO.cleanup()


class LgpioBackend(BaseGpio):
    def __init__(self, gpio_mode: str, chip: int = 0):
        import lgpio  # type: ignore

        self._lgpio = lgpio
        self._mode = gpio_mode
        self._chip = self._resolve_chip(int(chip))
        self._h = self._lgpio.gpiochip_open(self._chip)

        self._claimed: Dict[int, str] = {}

    @staticmethod
    def _resolve_chip(chip: int) -> int:
        if chip != -1:
            return chip

        # Auto-select: prefer the RP1 GPIO controller on Raspberry Pi 5.
        rp1 = LgpioBackend._detect_gpiochip_by_compatible("raspberrypi,rp1-gpio")
        if rp1 is not None:
            return rp1

        return 0

    @staticmethod
    def _detect_gpiochip_by_compatible(needle: str) -> Optional[int]:
        base = "/sys/bus/gpio/devices"
        try:
            entries = sorted(e for e in os.listdir(base) if e.startswith("gpiochip"))
        except Exception:
            return None

        for entry in entries:
            suffix = entry.replace("gpiochip", "")
            if not suffix.isdigit():
                continue
            n = int(suffix)
            of_node = os.path.join(base, entry, "of_node")
            try:
                real_of = os.path.realpath(of_node)
                compat_path = os.path.join(real_of, "compatible")
                with open(compat_path, "rb") as f:
                    compat = f.read().replace(b"\x00", b" ").decode("utf-8", errors="ignore")
                if needle in compat:
                    return n
            except Exception:
                continue
        return None

    def _line(self, pin: int) -> int:
        return _to_bcm(int(pin), self._mode)

    def setup_in(self, pin: int) -> None:
        line = self._line(pin)
        if self._claimed.get(line) == "in":
            return
        self._lgpio.gpio_claim_input(self._h, line)
        self._claimed[line] = "in"

    def setup_out(self, pin: int, initial: bool = False) -> None:
        line = self._line(pin)
        if self._claimed.get(line) == "out":
            self.write(pin, initial)
            return
        self._lgpio.gpio_claim_output(self._h, line, 1 if initial else 0)
        self._claimed[line] = "out"

    def read(self, pin: int) -> bool:
        return bool(self._lgpio.gpio_read(self._h, self._line(pin)))

    def write(self, pin: int, value: bool) -> None:
        self._lgpio.gpio_write(self._h, self._line(pin), 1 if value else 0)

    def cleanup(self) -> None:
        try:
            self._lgpio.gpiochip_close(self._h)
        except Exception:
            pass


def create_gpio(cfg: GpioConfig, logger: Optional[object] = None) -> BaseGpio:
    backend = cfg.backend.lower().strip()

    if backend not in {"auto", "rpi", "lgpio"}:
        raise ValueError("gpio_backend must be auto, rpi, or lgpio")

    if backend in {"auto", "rpi"}:
        try:
            gpio = RPiGpioBackend(cfg.mode)
            return gpio
        except Exception as e:
            if backend == "rpi":
                raise
            if logger is not None and hasattr(logger, "warn"):
                logger.warn(f"RPi.GPIO unusable ({e!r}); falling back to lgpio")

    return LgpioBackend(cfg.mode, chip=cfg.lgpio_chip)
