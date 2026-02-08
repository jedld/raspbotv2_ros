import os
import time
from typing import Iterable, List

import rclpy
from rclpy.node import Node


def _parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    s = str(value).strip().lower()
    return s in {"1", "true", "t", "yes", "y", "on"}


class StartupSoundNode(Node):
    def __init__(self) -> None:
        super().__init__("startup_sound")

        # Optional path to the bringup params YAML. If present, we try to reuse the I2C
        # configuration from motor_driver/camera_gimbal/ultrasonic sections.
        self.declare_parameter("bringup_params_file", "")

        self.declare_parameter("startup_delay_sec", 1.5)
        self.declare_parameter("wait_timeout_sec", 10.0)
        self.declare_parameter("require_all_enabled_nodes", True)

        # Bringup flags (passed from launch). Defaults match bringup.launch.py.
        self.declare_parameter("enable_motors", True)
        self.declare_parameter("enable_ultrasonic", True)
        self.declare_parameter("enable_gpio_sensors", True)
        self.declare_parameter("enable_camera", True)
        self.declare_parameter("enable_gimbal", True)

        # Buzzer / I2C settings (Pi5 controller).
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_addr", 43)  # 0x2B
        self.declare_parameter("i2c_protocol", "auto")
        self.declare_parameter("dry_run", False)

        # Success beep pattern.
        self.declare_parameter("beep_count", 3)
        self.declare_parameter("beep_on_sec", 0.07)
        self.declare_parameter("beep_off_sec", 0.08)

    def _try_read_i2c_settings_from_bringup_yaml(self):
        path = str(self.get_parameter("bringup_params_file").value).strip()
        if not path:
            return None

        path = os.path.expanduser(path)
        if not os.path.exists(path):
            return None

        try:
            import yaml  # type: ignore

            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            return None

        if not isinstance(data, dict):
            return None

        for node_key in ("motor_driver", "camera_gimbal", "ultrasonic"):
            block = data.get(node_key)
            if not isinstance(block, dict):
                continue
            ros_params = block.get("ros__parameters")
            if not isinstance(ros_params, dict):
                continue

            try:
                i2c_bus = int(ros_params.get("i2c_bus", 1))
                i2c_addr = int(ros_params.get("i2c_addr", 43))
                i2c_protocol = str(ros_params.get("i2c_protocol", "auto"))
            except Exception:
                continue

            return {"i2c_bus": i2c_bus, "i2c_addr": i2c_addr, "i2c_protocol": i2c_protocol}

        return None

    def _enabled_hw_nodes(self) -> List[str]:
        enabled = {
            "motor_driver": _parse_bool(self.get_parameter("enable_motors").value),
            "ultrasonic": _parse_bool(self.get_parameter("enable_ultrasonic").value),
            "gpio_sensors": _parse_bool(self.get_parameter("enable_gpio_sensors").value),
            "opencv_camera": _parse_bool(self.get_parameter("enable_camera").value),
            "camera_gimbal": _parse_bool(self.get_parameter("enable_gimbal").value),
        }
        return [name for name, is_on in enabled.items() if is_on]

    def _nodes_present(self, names: Iterable[str]) -> List[str]:
        present = set(self.get_node_names())
        return [n for n in names if n in present]

    def wait_for_hw_ready(self) -> bool:
        delay = float(self.get_parameter("startup_delay_sec").value)
        timeout = float(self.get_parameter("wait_timeout_sec").value)
        require_all = _parse_bool(self.get_parameter("require_all_enabled_nodes").value)

        required_nodes = self._enabled_hw_nodes()
        if not required_nodes:
            self.get_logger().warn("No hardware nodes enabled; skipping startup sound")
            return False

        if delay > 0:
            self.get_logger().info(f"Startup sound: waiting {delay:.1f}s before checking bringup status")
            end_delay = time.monotonic() + delay
            while rclpy.ok() and time.monotonic() < end_delay:
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            f"Startup sound: waiting for hardware nodes ({'all' if require_all else 'any'}): {required_nodes}"
        )

        deadline = time.monotonic() + max(timeout, 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            present = self._nodes_present(required_nodes)
            if require_all:
                if len(present) == len(required_nodes):
                    return True
            else:
                if present:
                    return True

        missing = [n for n in required_nodes if n not in set(self.get_node_names())]
        self.get_logger().warn(f"Startup sound: bringup not ready before timeout; missing nodes: {missing}")
        return False

    def play_success_beeps(self) -> None:
        from raspbot_hw.i2c_car import I2CCar

        yaml_i2c = self._try_read_i2c_settings_from_bringup_yaml()
        if yaml_i2c is not None:
            i2c_bus = int(yaml_i2c["i2c_bus"])
            i2c_addr = int(yaml_i2c["i2c_addr"])
            i2c_protocol = str(yaml_i2c["i2c_protocol"])
        else:
            i2c_bus = int(self.get_parameter("i2c_bus").value)
            i2c_addr = int(self.get_parameter("i2c_addr").value)
            i2c_protocol = str(self.get_parameter("i2c_protocol").value)
        dry_run = _parse_bool(self.get_parameter("dry_run").value)

        beep_count = int(self.get_parameter("beep_count").value)
        beep_on = float(self.get_parameter("beep_on_sec").value)
        beep_off = float(self.get_parameter("beep_off_sec").value)

        car = I2CCar(i2c_bus=i2c_bus, i2c_addr=i2c_addr, protocol=i2c_protocol, dry_run=dry_run)
        try:
            for i in range(max(beep_count, 0)):
                car.set_beep_enabled(True)
                time.sleep(max(beep_on, 0.0))
                car.set_beep_enabled(False)
                if i != beep_count - 1:
                    time.sleep(max(beep_off, 0.0))
        finally:
            try:
                car.set_beep_enabled(False)
            except Exception:
                pass
            car.close()


def main() -> None:
    rclpy.init()
    node = StartupSoundNode()
    try:
        if node.wait_for_hw_ready():
            node.get_logger().info("Startup sound: bringup ready; playing success sound")
            node.play_success_beeps()
        else:
            node.get_logger().info("Startup sound: not playing (bringup not ready)")
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().warn(f"Startup sound: failed: {exc!r}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
