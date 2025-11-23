import asyncio
import time

from .protocol_utils import (
    ROBOT_IP, ROBOT_PORT, HEARTBEAT_FREQ,
    build_header, parse_header, create_json_payload
)

CONTROL_FREQ = 20.0   # Hz for sending control commands


class RobotClient:
    def __init__(self, host=ROBOT_IP, port=ROBOT_PORT):
        self.host = host
        self.port = port
        self.reader = None
        self.writer = None
        self.message_id = 0
        self.running = False
        self.loop = None

        # Motion State
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vw = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vw = 0.0

        # Constants
        self.MAX_SPEED = 0.5   # m/s
        self.MAX_OMEGA = 1.0   # rad/s
        self.ACCEL = 0.5       # m/s^2
        self.OMEGA_ACCEL = 0.5 # rad/s^2

        self.last_update_time = time.time()
        self.mode = None  # 0: Regular, 1: Navigation
        self.on_message_received = None

    async def connect(self):
        print(f"Connecting to {self.host}:{self.port}...")
        self.reader, self.writer = await asyncio.open_connection(self.host, self.port)
        print("Connected!")
        self.running = True

    async def send_message(self, type_code, command_code, items=None):
        payload = create_json_payload(type_code, command_code, items)
        header = build_header(len(payload), self.message_id, is_json=True)
        if self.writer:
            self.writer.write(header + payload)
            await self.writer.drain()
            self.message_id = (self.message_id + 1) % 65536

    async def heartbeat_loop(self):
        while self.running:
            try:
                # Heartbeat: Type 100, Command 100
                await self.send_message(100, 100)
                await asyncio.sleep(1.0 / HEARTBEAT_FREQ)
            except Exception as e:
                print(f"Heartbeat error: {e}")
                break

    async def control_loop(self):
        while self.running:
            try:
                now = time.time()
                dt = now - self.last_update_time
                self.last_update_time = now

                # Smooth acceleration logic
                self.current_vx = self._update_val(self.current_vx, self.target_vx, self.ACCEL, dt)
                self.current_vy = self._update_val(self.current_vy, self.target_vy, self.ACCEL, dt)
                self.current_vw = self._update_val(self.current_vw, self.target_vw, self.OMEGA_ACCEL, dt)

                # Motion Control: Type 2, Command 21
                items = {
                    "X": self.current_vx,
                    "Y": self.current_vy,
                    "Z": 0.0,
                    "Roll": 0.0,
                    "Pitch": 0.0,
                    "Yaw": self.current_vw
                }
                await self.send_message(2, 21, items)
                await asyncio.sleep(1.0 / CONTROL_FREQ)
            except Exception as e:
                print(f"Control error: {e}")
                break

    def _update_val(self, current, target, accel, dt):
        step = accel * dt
        if abs(target - current) <= step:
            return target
        return current + step if target > current else current - step

    # --- Robot Commands ---

    async def set_mode(self, mode):
        """Switch Usage Mode (0: Regular, 1: Navigation)"""
        print(f"Switching to mode {mode}...")
        await self.send_message(1101, 5, {"Mode": mode})
        self.mode = mode

    async def set_motion_state(self, state):
        """
        Set Motion State:
        0: Idle, 1: Stand, 2: Soft Estop, 3: Damping, 4: Sit, 6: Standard
        """
        print(f"Switching motion state to {state}...")
        await self.send_message(2, 22, {"MotionParam": state})

    async def set_gait(self, gait):
        """
        Set Gait:
        0x1001: Basic, 0x1002: High Obstacles, 0x3002: Flat, 0x3003: Stair
        """
        print(f"Setting gait to {gait}")
        await self.send_message(2, 23, {"GaitParam": gait})

    async def soft_stop(self):
        print("Triggering Soft Emergency Stop")
        await self.set_motion_state(2)

    async def set_sleep_mode(self, sleep, auto=False, time_val=30):
        """Set Sleep Mode (Command 1)"""
        print(f"Setting sleep mode to {sleep}...")
        items = {
            "Sleep": sleep,
            "Auto": auto,
            "Time": time_val
        }
        await self.send_message(1101, 1, items)

    async def set_flashlight(self, on: bool):
        """Control Flashlight (Command 2)"""
        val = 1 if on else 0
        print(f"Setting flashlight to {val}...")
        items = {"Front": val, "Back": val}
        await self.send_message(1101, 2, items)

    async def listen_loop(self):
        while self.running:
            try:
                header_data = await self.reader.readexactly(16)
                payload_len, msg_id = parse_header(header_data)

                if payload_len and payload_len > 0:
                    payload_data = await self.reader.readexactly(payload_len)
                    if self.on_message_received:
                        try:
                            self.on_message_received(payload_data)
                        except Exception as e:
                            print(f"Callback error: {e}")
            except asyncio.IncompleteReadError:
                print("Disconnected")
                self.running = False
                break
            except Exception:
                break

    async def close(self):
        self.running = False
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()

