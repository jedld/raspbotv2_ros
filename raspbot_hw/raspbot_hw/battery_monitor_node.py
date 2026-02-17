"""ROS 2 node — Battery monitor via Pi 5 PMIC.

Reads the regulated 5 V rail voltage (`EXT5V_V`) from the Raspberry Pi 5's
Renesas DA9091 PMIC via ``vcgencmd pmic_read_adc``.

The Yahboom Raspbot V2 uses 2× 18650 Li-ion cells in series (7.4 V nominal,
8.4 V full charge, ~6.0 V cutoff).  A buck converter on the expansion board
regulates this to 5 V for the Pi.  The regulated output stays near 5.1 V
until the battery drops low, then sags — so the 5 V rail is only useful for
rough state-of-charge estimation and low-battery detection, **not** precise
percentage.

Published topics
~~~~~~~~~~~~~~~~
  battery/state     (sensor_msgs/BatteryState)  periodic battery report
  battery/low       (std_msgs/Bool)             True when estimated SoC < low_threshold

The node also provides a ``battery/percentage`` service (std_srvs/Trigger)
that returns the current estimated percentage instantly.

Hardware note
~~~~~~~~~~~~~
For accurate battery monitoring you would need an ADC connected to the
raw battery voltage via a resistor divider (e.g. Arduino analog pin or
an INA219/ADS1115 on the I2C bus).  This node uses the best available
proxy **without** any hardware modification.
"""

import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# ── Empirical voltage→SoC mapping ────────────────────────────────────
# These thresholds were measured on a Raspbot V2 with fresh 18650 cells.
# The 5 V buck converter maintains regulation until ~6.2 V battery input,
# then the 5 V rail begins to droop.
#
#   EXT5V_V ≥ 5.10 V  →  battery is healthy  (roughly 40–100 %)
#   EXT5V_V    5.00 V  →  battery is getting low  (~20 %)
#   EXT5V_V    4.80 V  →  battery is low           (~10 %)
#   EXT5V_V ≤  4.50 V  →  battery is critical      (~0 %)
#
# Because the buck converter clamps the output, only the tail end of the
# discharge curve is visible.  We use a piecewise-linear mapping.

_SOC_TABLE = [
    # (ext5v_voltage, estimated_percentage)
    (5.15, 100.0),
    (5.10,  80.0),
    (5.05,  60.0),
    (5.00,  40.0),
    (4.90,  20.0),
    (4.80,  10.0),
    (4.50,   0.0),
]


def _voltage_to_soc(v: float) -> float:
    """Piecewise-linear interpolation of EXT5V_V → estimated SoC %."""
    if v >= _SOC_TABLE[0][0]:
        return _SOC_TABLE[0][1]
    if v <= _SOC_TABLE[-1][0]:
        return _SOC_TABLE[-1][1]
    for i in range(len(_SOC_TABLE) - 1):
        v_hi, soc_hi = _SOC_TABLE[i]
        v_lo, soc_lo = _SOC_TABLE[i + 1]
        if v_lo <= v <= v_hi:
            t = (v - v_lo) / (v_hi - v_lo) if (v_hi - v_lo) > 0 else 0.0
            return soc_lo + t * (soc_hi - soc_lo)
    return 50.0  # fallback


_EXT5V_RE = re.compile(r'EXT5V_V\s+volt\(\d+\)\s*=\s*([\d.]+)\s*V', re.IGNORECASE)


def _read_ext5v() -> float | None:
    """Read EXT5V_V from the Pi 5 PMIC.  Returns voltage or None on failure."""
    try:
        out = subprocess.check_output(
            ['vcgencmd', 'pmic_read_adc'],
            timeout=3.0,
            stderr=subprocess.DEVNULL,
        ).decode('utf-8', errors='replace')
        m = _EXT5V_RE.search(out)
        if m:
            return float(m.group(1))
    except Exception:
        pass
    return None


class BatteryMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('battery_monitor')

        self.declare_parameter('poll_interval_sec', 5.0)
        self.declare_parameter('low_battery_threshold', 20.0)  # percent

        self._poll_interval = float(self.get_parameter('poll_interval_sec').value)
        self._low_threshold = float(self.get_parameter('low_battery_threshold').value)

        self._battery_pub = self.create_publisher(BatteryState, 'battery/state', 10)
        self._low_pub = self.create_publisher(Bool, 'battery/low', 10)
        self._srv = self.create_service(Trigger, 'battery/percentage', self._on_percentage_srv)

        # Latest readings (thread-safe via the GIL for simple float assignment)
        self._ext5v_voltage = 0.0
        self._soc_percent = 0.0
        self._is_low = False
        self._last_read_ok = False

        # Background reader thread — subprocess calls shouldn't block spin
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        # Periodic publisher (runs in the rclpy executor)
        self._timer = self.create_timer(self._poll_interval, self._publish)

        self.get_logger().info(
            f'Battery monitor started (poll every {self._poll_interval:.1f} s, '
            f'low threshold {self._low_threshold:.0f} %)'
        )

    # ── Background reader ─────────────────────────────────────────────

    def _reader_loop(self) -> None:
        while self._running:
            v = _read_ext5v()
            if v is not None:
                self._ext5v_voltage = v
                self._soc_percent = _voltage_to_soc(v)
                self._is_low = self._soc_percent < self._low_threshold
                self._last_read_ok = True
            else:
                self._last_read_ok = False
            time.sleep(max(1.0, self._poll_interval))

    # ── Publisher (timer callback) ────────────────────────────────────

    def _publish(self) -> None:
        if not self._last_read_ok:
            return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(self._ext5v_voltage)
        msg.percentage = float(self._soc_percent / 100.0)  # BatteryState uses 0.0–1.0
        msg.present = True
        # POWER_SUPPLY_STATUS_DISCHARGING = 3 (there's no charger on the Raspbot)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        # Fields we can't measure: current, charge, capacity, design_capacity
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN

        self._battery_pub.publish(msg)

        low_msg = Bool()
        low_msg.data = self._is_low
        self._low_pub.publish(low_msg)

    # ── Service ───────────────────────────────────────────────────────

    def _on_percentage_srv(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._last_read_ok:
            resp.success = True
            resp.message = (
                f'{self._soc_percent:.1f}% '
                f'(EXT5V={self._ext5v_voltage:.3f} V)'
            )
        else:
            resp.success = False
            resp.message = 'PMIC read failed — battery status unavailable'
        return resp

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = BatteryMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
