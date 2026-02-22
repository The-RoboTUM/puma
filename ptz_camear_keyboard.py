#!/usr/bin/env python3
import sys
import time
import serial
import termios
import tty
import select


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
        self.address = address
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
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

    # Pan/Tilt (speed usually 0x00~0x3F)
    def left(self, speed):   self.send(0x00, 0x04, speed, 0x00)
    def right(self, speed):  self.send(0x00, 0x02, speed, 0x00)
    def up(self, speed):     self.send(0x00, 0x08, 0x00, speed)
    def down(self, speed):   self.send(0x00, 0x10, 0x00, speed)

    # Zoom (common Pelco-D bits)
    def zoom_in(self):       self.send(0x00, 0x20, 0x00, 0x00)
    def zoom_out(self):      self.send(0x00, 0x40, 0x00, 0x00)

    def close(self):
        try:
            self.stop()
        finally:
            self.ser.close()


def read_key_nonblocking():
    """Return one key (string) if available, else None."""
    rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
    if rlist:
        ch = sys.stdin.read(1)
        return ch
    return None


def main():
    # ======= Modify here if needed =======
    PORT = "/dev/ttyUSB0"
    BAUD = 9600
    ADDR = 1
    # ====================================

    speed = 0x20  # default speed
    min_speed = 0x05
    max_speed = 0x3F

    ptz = PTZ(PORT, BAUD, ADDR)

    print("\nKeyboard PTZ Control (Pelco-D over RS485)")
    print("--------------------------------------------------")
    print("W/A/S/D : tilt up / pan left / tilt down / pan right")
    print("+ / -   : speed up / speed down")
    print("I / O   : zoom in / zoom out")
    print("SPACE   : stop")
    print("Q       : quit")
    print("--------------------------------------------------")
    print(f"Port={PORT}, Baud={BAUD}, Addr={ADDR}, Speed=0x{speed:02X}\n")

    # Put terminal into raw mode to read keys instantly
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    # Simple "key hold" behavior:
    # if no key pressed for a short time, send STOP once.
    last_cmd_time = 0.0
    sent_stop_after_idle = True
    idle_stop_s = 0.25

    try:
        while True:
            key = read_key_nonblocking()
            now = time.time()

            if key is None:
                # if idle, stop once
                if (now - last_cmd_time) > idle_stop_s and not sent_stop_after_idle:
                    ptz.stop()
                    sent_stop_after_idle = True
                continue

            key = key.lower()

            if key == 'q':
                ptz.stop()
                print("\nQuit.")
                break

            if key == ' ':
                ptz.stop()
                last_cmd_time = now
                sent_stop_after_idle = True
                continue

            if key == '+':
                speed = min(max_speed, speed + 0x02)
                print(f"\rSpeed=0x{speed:02X}   ", end="", flush=True)
                continue

            if key == '-':
                speed = max(min_speed, speed - 0x02)
                print(f"\rSpeed=0x{speed:02X}   ", end="", flush=True)
                continue

            # Movement / zoom
            if key == 'w':
                ptz.up(speed)
            elif key == 's':
                ptz.down(speed)
            elif key == 'a':
                ptz.left(speed)
            elif key == 'd':
                ptz.right(speed)
            elif key == 'i':
                ptz.zoom_in()
            elif key == 'o':
                ptz.zoom_out()
            else:
                # ignore unknown key
                continue

            last_cmd_time = now
            sent_stop_after_idle = False

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        ptz.close()


if __name__ == "__main__":
    main()