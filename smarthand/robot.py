"""Serial controller for the Delta robot."""

import threading
import time
from typing import List, Optional

import numpy as np
import serial
import serial.tools.list_ports


class RobotController:
    """Thin wrapper around the robot's serial protocol."""

    def __init__(
        self,
        handshake_command: str = "IsDelta",
        handshake_response: str = "YesDelta",
        default_feedrate: int = 2000,
        home_z: float = -291.28,
    ):
        self.handshake_command = handshake_command
        self.handshake_response = handshake_response
        self.default_feedrate = default_feedrate
        self.home_z = float(home_z)
        self.conn: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.current_position = np.array([0.0, 0.0, self.home_z], dtype=float)

    # ------------------------------------------------------------------
    # Connection helpers
    # ------------------------------------------------------------------
    def list_ports(self) -> List[str]:
        return [port.device for port in serial.tools.list_ports.comports()]

    def is_connected(self) -> bool:
        return self.conn is not None and self.conn.is_open

    def connect(self, port: str, baudrate: int):
        if self.is_connected():
            self.disconnect()

        try:
            conn = serial.Serial(port, baudrate, timeout=2)
            time.sleep(0.5)

            if self.handshake_command:
                conn.write((self.handshake_command + "\n").encode())
                conn.flush()
                response = conn.readline().decode(errors="ignore").strip()
                if response != self.handshake_response:
                    conn.close()
                    if response:
                        return False, f"Unexpected handshake response: {response}"
                    return False, "No handshake response from device"

            self.conn = conn
            self.current_position = np.array([0.0, 0.0, self.home_z], dtype=float)
            self.set_absolute_mode()
            return True, ""
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)

    def disconnect(self):
        if self.conn:
            try:
                if self.conn.is_open:
                    self.conn.close()
            except Exception:  # noqa: BLE001
                pass
        self.conn = None

    # ------------------------------------------------------------------
    # Low-level IO
    # ------------------------------------------------------------------
    def _drain_input(self):
        if not self.is_connected():
            return
        try:
            waiting = self.conn.in_waiting
        except AttributeError:
            waiting = 0
        if waiting:
            try:
                self.conn.read(waiting)
            except Exception:  # noqa: BLE001
                pass

    def send_command(self, command: str, wait_for_ok: bool = True, timeout: float = 5.0) -> List[str]:
        if not self.is_connected():
            raise RuntimeError("Robot not connected")

        command = command.strip()
        responses: List[str] = []

        with self.lock:
            self._drain_input()
            self.conn.write((command + "\n").encode())
            self.conn.flush()

            if wait_for_ok:
                end_time = time.time() + timeout
                while time.time() < end_time:
                    line = self.conn.readline().decode(errors="ignore").strip()
                    if not line:
                        continue
                    responses.append(line)
                    lower = line.lower()
                    if lower.startswith("ok") or lower.startswith("error"):
                        break
            return responses

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------
    def set_absolute_mode(self):
        if self.is_connected():
            self.send_command("G90", wait_for_ok=False)

    def set_relative_mode(self):
        if self.is_connected():
            self.send_command("G91", wait_for_ok=False)

    def home(self):
        responses = self.send_command("G28")
        self.current_position = np.array([0.0, 0.0, self.home_z], dtype=float)
        return responses

    def move_linear_absolute(self, x=None, y=None, z=None, feedrate=None):
        if not self.is_connected():
            raise RuntimeError("Robot not connected")
        if feedrate is None:
            feedrate = self.default_feedrate

        cmd_parts = ["G01"]
        if x is not None:
            cmd_parts.append(f"X{x:.3f}")
        if y is not None:
            cmd_parts.append(f"Y{y:.3f}")
        if z is not None:
            cmd_parts.append(f"Z{z:.3f}")
        cmd_parts.append(f"F{feedrate}")

        responses = self.send_command(" ".join(cmd_parts))

        if x is not None:
            self.current_position[0] = x
        if y is not None:
            self.current_position[1] = y
        if z is not None:
            self.current_position[2] = z
        return responses

    def move_linear_relative(self, dx=None, dy=None, dz=None, feedrate=None):
        if not self.is_connected():
            raise RuntimeError("Robot not connected")
        if feedrate is None:
            feedrate = self.default_feedrate

        self.set_relative_mode()
        cmd_parts = ["G01"]
        if dx is not None:
            cmd_parts.append(f"X{dx:.3f}")
        if dy is not None:
            cmd_parts.append(f"Y{dy:.3f}")
        if dz is not None:
            cmd_parts.append(f"Z{dz:.3f}")
        cmd_parts.append(f"F{feedrate}")
        responses = self.send_command(" ".join(cmd_parts))
        self.set_absolute_mode()

        if dx is not None:
            self.current_position[0] += dx
        if dy is not None:
            self.current_position[1] += dy
        if dz is not None:
            self.current_position[2] += dz
        return responses

    def dwell(self, duration: float):
        if duration <= 0:
            return []
        milliseconds = max(0.0, duration) * 1000.0
        return self.send_command(f"G04 P{milliseconds:.0f}")

    # ------------------------------------------------------------------
    # Status helpers
    # ------------------------------------------------------------------
    def get_position(self):
        if not self.is_connected():
            return self.current_position.copy()

        responses = self.send_command("M114")
        for line in responses:
            try:
                parts = line.split(" ")
                x_part = next((p for p in parts if p.startswith("X:")), None)
                y_part = next((p for p in parts if p.startswith("Y:")), None)
                z_part = next((p for p in parts if p.startswith("Z:")), None)
                if x_part and y_part and z_part:
                    x = float(x_part.split(":")[1])
                    y = float(y_part.split(":")[1])
                    z = float(z_part.split(":")[1])
                    self.current_position = np.array([x, y, z], dtype=float)
                    break
            except Exception:  # noqa: BLE001
                continue

        return self.current_position.copy()


__all__ = ["RobotController"]
