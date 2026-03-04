#!/usr/bin/env python3
import sys
import time
import termios
import tty
import select

import rclpy
from rclpy.node import Node

import serial


def pelco_d(address, cmd1, cmd2, data1, data2):
    """Build a Pelco-D frame: FF addr cmd1 cmd2 data1 data2 checksum"""
    address &= 0xFF
    cmd1 &= 0xFF
    cmd2 &= 0xFF
    data1 &= 0xFF
    data2 &= 0xFF
    checksum = (address + cmd1 + cmd2 + data1 + data2) & 0xFF
    return bytes([0xFF, address, cmd1, cmd2, data1, data2, checksum])


class PTZ:
    def __init__(self, port="/dev/ttyUSB0", baud=9600, address=1):
        self.address = int(address)
        self.ser = serial.Serial(
            port=port,
            baudrate=int(baud),
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.05
        )

    def send(self, cmd1, cmd2, data1=0x00, data2=0x00):
        self.ser.write(pelco_d(self.address, cmd1, cmd2, data1, data2))
        self.ser.flush()

    def stop(self):
        self.send(0x00, 0x00, 0x00, 0x00)

    # Pan/Tilt (speed 0x00~0x3F)
    def left(self, speed):   self.send(0x00, 0x04, speed & 0xFF, 0x00)
    def right(self, speed):  self.send(0x00, 0x02, speed & 0xFF, 0x00)
    def up(self, speed):     self.send(0x00, 0x08, 0x00, speed & 0xFF)
    def down(self, speed):   self.send(0x00, 0x10, 0x00, speed & 0xFF)

    # Zoom
    def zoom_in(self):       self.send(0x00, 0x20, 0x00, 0x00)
    def zoom_out(self):      self.send(0x00, 0x40, 0x00, 0x00)

    def close(self):
        try:
            self.stop()
        finally:
            self.ser.close()


def read_key_nonblocking(timeout_s=0.02):
    """Return one key if available, else None."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if rlist:
        return sys.stdin.read(1)
    return None


USAGE = r"""
Keyboard PTZ Control (Pelco-D over RS485)
--------------------------------------------------
i/k/j/l : tilt up / tilt down / pan left / pan right
o/p     : zoom in / zoom out
n/m     : speed down / speed up
h       : stop
ESC     : quit
--------------------------------------------------
"""


class PTZKeyboardNode(Node):
    def __init__(self):
        super().__init__("ptz_camera_keyboard")

        # --- Params ---
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("address", 1)

        self.declare_parameter("speed", 0x20)
        self.declare_parameter("min_speed", 0x05)
        self.declare_parameter("max_speed", 0x3F)

        self.declare_parameter("idle_stop_s", 0.25)
        self.declare_parameter("poll_hz", 50.0)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.address = int(self.get_parameter("address").value)

        self.speed = int(self.get_parameter("speed").value)
        self.min_speed = int(self.get_parameter("min_speed").value)
        self.max_speed = int(self.get_parameter("max_speed").value)

        self.idle_stop_s = float(self.get_parameter("idle_stop_s").value)
        self.poll_hz = float(self.get_parameter("poll_hz").value)
        if self.poll_hz <= 0:
            self.poll_hz = 50.0

        # --- Key map (no overlap with teleop) ---
        self.KEY_TILT_UP = "i"
        self.KEY_TILT_DOWN = "k"
        self.KEY_PAN_LEFT = "j"
        self.KEY_PAN_RIGHT = "l"
        self.KEY_ZOOM_IN = "o"
        self.KEY_ZOOM_OUT = "p"
        self.KEY_STOP = "h"
        self.KEY_SPEED_DOWN = "n"
        self.KEY_SPEED_UP = "m"
        self.KEY_QUIT_ESC = "\x1b"  # ESC

        # --- PTZ ---
        self.ptz = PTZ(self.port, self.baud, self.address)

        # --- Terminal raw mode ---
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # --- Idle stop state ---
        self.last_cmd_time = 0.0
        self.sent_stop_after_idle = True

        print(USAGE)
        print(f"Port={self.port}, Baud={self.baud}, Addr={self.address}, Speed=0x{self.speed:02X}\n")

        # Timer loop
        self.timer = self.create_timer(1.0 / self.poll_hz, self._tick)

        self.get_logger().info(
            f"PTZ keyboard node started. port={self.port}, baud={self.baud}, address={self.address}"
        )

    def _tick(self):
        now = time.time()

        # auto stop after idle
        if (now - self.last_cmd_time) > self.idle_stop_s and not self.sent_stop_after_idle:
            self.ptz.stop()
            self.sent_stop_after_idle = True

        key = read_key_nonblocking()
        if key is None:
            return

        # quit
        if key == self.KEY_QUIT_ESC:
            self.get_logger().info("ESC pressed, shutting down.")
            rclpy.shutdown()
            return

        key = key.lower()

        # stop
        if key == self.KEY_STOP:
            self.ptz.stop()
            self.last_cmd_time = now
            self.sent_stop_after_idle = True
            print("\rSTOP                 ", end="", flush=True)
            return

        # speed
        if key == self.KEY_SPEED_UP:
            self.speed = min(self.max_speed, self.speed + 0x02)
            print(f"\rSpeed=0x{self.speed:02X}   ", end="", flush=True)
            return

        if key == self.KEY_SPEED_DOWN:
            self.speed = max(self.min_speed, self.speed - 0x02)
            print(f"\rSpeed=0x{self.speed:02X}   ", end="", flush=True)
            return

        # movement / zoom
        moved = True
        if key == self.KEY_TILT_UP:
            self.ptz.up(self.speed)
        elif key == self.KEY_TILT_DOWN:
            self.ptz.down(self.speed)
        elif key == self.KEY_PAN_LEFT:
            self.ptz.left(self.speed)
        elif key == self.KEY_PAN_RIGHT:
            self.ptz.right(self.speed)
        elif key == self.KEY_ZOOM_IN:
            self.ptz.zoom_in()
        elif key == self.KEY_ZOOM_OUT:
            self.ptz.zoom_out()
        else:
            moved = False

        if moved:
            self.last_cmd_time = now
            self.sent_stop_after_idle = False

    def destroy_node(self):
        # restore terminal + close serial safely
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except Exception:
            pass
        try:
            self.ptz.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PTZKeyboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()