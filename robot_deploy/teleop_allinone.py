#!/usr/bin/env python3
"""
PUMA All-in-One Teleop (Keyboard + UDP, no separate bridge needed)
===================================================================
Run this ON THE ROBOT via SSH. No other scripts required!

This script:
  1. Reads keyboard input
  2. Sends UDP commands directly to vendor controller
  3. Optionally publishes to ROS2 for logging/visualization

Usage:
  source /opt/robot/scripts/setup_ros2.sh
  python3 teleop_allinone.py
"""
import sys
import time
import termios
import tty
import select
import socket
import json
import struct

# Robot network configuration
ROBOT_IP = "127.0.0.1"
ROBOT_PORT = 30000

# Protocol constants
SYNC_BYTES = bytes([0xEB, 0x91, 0xEB, 0x90])

# Motion state mapping
MOTION_STATES = {
    "stand": 1,
    "sit": 4,
    "damping": 3,
    "standard": 6,
}

# Gait mapping
GAITS = {
    "basic": 1,
    "stair": 14,
}

HELP_TEXT = """
════════════════════════════════════════════════════════
  PUMA All-in-One Teleop (Direct UDP)
════════════════════════════════════════════════════════

Movement:        Commands:
  q   w   e      u - Stand
  a   s   d      z - Sit
                 m - Damping
w/s: forward     6 - Standard (enables walking)
a/d: strafe      
q/e: rotate      Gaits:
                 b - Basic
+/-: speed       t - Stair

                 Space: Stop
                 Ctrl+C: Quit

Workflow: u (stand) → 6 (standard) → wasd (move)
════════════════════════════════════════════════════════
"""

SPEED = 0.3
TURN = 0.5

MOVE_BINDINGS = {
    'w': (SPEED, 0, 0),
    's': (-SPEED, 0, 0),
    'a': (0, SPEED, 0),
    'd': (0, -SPEED, 0),
    'q': (0, 0, TURN),
    'e': (0, 0, -TURN),
}

CONTROL_BINDINGS = {
    'u': 'stand',
    'z': 'sit',
    'm': 'damping',
    '6': 'standard',
}

GAIT_BINDINGS = {
    'b': 'basic',
    't': 'stair',
}


class DirectTeleop:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.msg_id = 0
        self.speed_mult = 1.0
        self.last_heartbeat = 0
        
    def _build_header(self, payload_len: int) -> bytes:
        header = (
            SYNC_BYTES +
            struct.pack('<H', payload_len) +
            struct.pack('<H', self.msg_id) +
            bytes([0x01]) +
            bytes(7)
        )
        self.msg_id = (self.msg_id + 1) % 65536
        return header

    def _send(self, type_code: int, cmd_code: int, items: dict = None):
        payload = {
            "PatrolDevice": {
                "Type": type_code,
                "Command": cmd_code,
                "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "Items": items or {}
            }
        }
        payload_bytes = json.dumps(payload).encode('utf-8')
        header = self._build_header(len(payload_bytes))
        try:
            self.sock.sendto(header + payload_bytes, (ROBOT_IP, ROBOT_PORT))
        except Exception as e:
            print(f"\rUDP Error: {e}", end="")

    def send_heartbeat(self):
        now = time.time()
        if now - self.last_heartbeat > 0.5:
            self._send(100, 100)
            self.last_heartbeat = now

    def send_velocity(self, x, y, yaw):
        self._send(2, 21, {
            "X": float(x),
            "Y": float(y),
            "Z": 0.0,
            "Roll": 0.0,
            "Pitch": 0.0,
            "Yaw": float(yaw)
        })

    def send_motion_state(self, cmd):
        if cmd in MOTION_STATES:
            self._send(2, 22, {"MotionParam": MOTION_STATES[cmd]})
            return True
        return False

    def send_gait(self, cmd):
        if cmd in GAITS:
            self._send(2, 23, {"GaitParam": GAITS[cmd]})
            return True
        return False

    def close(self):
        self.sock.close()


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    teleop = DirectTeleop()
    
    print(HELP_TEXT)
    print("Ready! Sending heartbeats...\n")
    
    try:
        while True:
            # Always send heartbeat
            teleop.send_heartbeat()
            
            key = get_key(settings)
            
            if key in MOVE_BINDINGS:
                x, y, yaw = MOVE_BINDINGS[key]
                x *= teleop.speed_mult
                y *= teleop.speed_mult
                yaw *= teleop.speed_mult
                teleop.send_velocity(x, y, yaw)
                print(f"\rVel: x={x:+.2f} y={y:+.2f} yaw={yaw:+.2f}  ", end="", flush=True)
            
            elif key in CONTROL_BINDINGS:
                cmd = CONTROL_BINDINGS[key]
                teleop.send_motion_state(cmd)
                print(f"\rCommand: {cmd:<12}", end="", flush=True)
            
            elif key in GAIT_BINDINGS:
                cmd = GAIT_BINDINGS[key]
                teleop.send_gait(cmd)
                print(f"\rGait: {cmd:<12}", end="", flush=True)
            
            elif key == '+' or key == '=':
                teleop.speed_mult = min(teleop.speed_mult + 0.1, 2.0)
                print(f"\rSpeed: {teleop.speed_mult:.1f}x        ", end="", flush=True)
            
            elif key == '-':
                teleop.speed_mult = max(teleop.speed_mult - 0.1, 0.1)
                print(f"\rSpeed: {teleop.speed_mult:.1f}x        ", end="", flush=True)
            
            elif key == ' ':
                teleop.send_velocity(0, 0, 0)
                print("\rSTOPPED              ", end="", flush=True)
            
            elif key == '\x03':
                break

    except Exception as e:
        print(f"\nError: {e}")
    
    finally:
        teleop.send_velocity(0, 0, 0)
        teleop.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n\nTeleop stopped.")


if __name__ == '__main__':
    main()
