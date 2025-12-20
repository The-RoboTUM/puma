import sys
import asyncio
import threading
import json
import time
import os
import pygame

# Import PyQt6 first to ensure correct Qt libraries are loaded before OpenCV
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QGroupBox, QGridLayout, QPushButton)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap, QFont

import cv2

# Ensure the parent directory is in sys.path if running from mission_control
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from teleop.teleop_robot import RobotClient
from monitoring.stream_video import VideoStreamer
from protocol import ROBOT_IP, ROBOT_PORT

class AsyncWorker(QObject):
    """Worker to run asyncio loop in a separate thread"""
    def __init__(self, client):
        super().__init__()
        self.client = client
        self.loop = asyncio.new_event_loop()

    def run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._run_client())

    async def _run_client(self):
        await self.client.connect()
        tasks = [
            asyncio.create_task(self.client.heartbeat_loop()),
            asyncio.create_task(self.client.listen_loop()),
            asyncio.create_task(self.client.control_loop())
        ]
        await asyncio.gather(*tasks)

class Dashboard(QMainWindow):
    telemetry_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PUMA Mission Control")
        self.resize(1280, 720)
        self.last_msg_time = time.time()
        self.control_mode = "KEYBOARD"

        # --- Robot Client Setup ---
        # Switched to UDP as requested
        self.client = RobotClient(ROBOT_IP, ROBOT_PORT, use_udp=True)
        self.client.on_message_received = self.on_robot_message
        
        # Start Asyncio Thread
        self.worker_thread = threading.Thread(target=self.run_async_loop, daemon=True)
        self.worker_thread.start()

        # --- Video Setup ---
        self.video_urls = [
            f"rtsp://{ROBOT_IP}:8554/video1",
            f"rtsp://{ROBOT_IP}:8554/video2",
        ]
        self.streamer = VideoStreamer(self.video_urls, target_height=480)
        self.streamer.start()

        # --- UI Setup ---
        self.init_ui()

        # --- Timers ---
        self.video_timer = QTimer()
        self.video_timer.timeout.connect(self.update_video)
        self.video_timer.start(30) # 30 FPS

        self.conn_timer = QTimer()
        self.conn_timer.timeout.connect(self.check_connection)
        self.conn_timer.start(1000) # Check every 1s

        # --- Signals ---
        self.telemetry_signal.connect(self.update_telemetry_ui)

        # --- Joystick Setup ---
        self.init_joystick()
        self.joystick_timer = QTimer()
        self.joystick_timer.timeout.connect(self.poll_joystick)
        self.joystick_timer.start(20) # Poll every 20ms

    def run_async_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self.client.loop = loop  # Store loop reference for threadsafe calls
        
        async def run_tasks():
            try:
                await self.client.connect()
                
                await asyncio.gather(
                    self.client.heartbeat_loop(),
                    self.client.listen_loop(),
                    self.client.control_loop()
                )
            except Exception as e:
                print(f"Async loop error: {e}")

        loop.run_until_complete(run_tasks())

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Top: Video Feed
        video_group = QGroupBox("Live Feed")
        video_layout = QVBoxLayout(video_group)
        
        # Connection Status
        self.lbl_conn = QLabel("Connecting...")
        self.lbl_conn.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_conn.setStyleSheet("background-color: orange; color: white; font-weight: bold; padding: 5px;")
        video_layout.addWidget(self.lbl_conn)

        self.video_label = QLabel("Waiting for video...")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white;")
        self.video_label.setSizePolicy(self.video_label.sizePolicy().Policy.Expanding, self.video_label.sizePolicy().Policy.Expanding)
        self.video_label.setMinimumSize(640, 360) # 16:9 aspect ratio min
        video_layout.addWidget(self.video_label)
        
        main_layout.addWidget(video_group, stretch=3)

        # Bottom: Controls & Telemetry
        bottom_layout = QHBoxLayout()
        
        # Left Bottom: Telemetry
        telemetry_group = QGroupBox("Telemetry")
        telemetry_layout = QGridLayout(telemetry_group)
        
        self.lbl_mode = QLabel("Mode: Unknown")
        self.lbl_motion = QLabel("Motion: Unknown")
        self.lbl_batt_l = QLabel("Batt L: --%")
        self.lbl_batt_r = QLabel("Batt R: --%")
        self.lbl_gps = QLabel("GPS: --, --")
        self.lbl_temp = QLabel("Max Temp: --°C")
        self.lbl_vel = QLabel("Vel: X=0.0 Y=0.0 W=0.0")
        
        font = QFont()
        font.setPointSize(10)
        for lbl in [self.lbl_mode, self.lbl_motion, self.lbl_batt_l, self.lbl_batt_r, self.lbl_gps, self.lbl_temp, self.lbl_vel]:
            lbl.setFont(font)

        telemetry_layout.addWidget(self.lbl_mode, 0, 0)
        telemetry_layout.addWidget(self.lbl_motion, 0, 1)
        telemetry_layout.addWidget(self.lbl_batt_l, 1, 0)
        telemetry_layout.addWidget(self.lbl_batt_r, 1, 1)
        telemetry_layout.addWidget(self.lbl_gps, 2, 0, 1, 2)
        telemetry_layout.addWidget(self.lbl_temp, 3, 0, 1, 2)
        telemetry_layout.addWidget(self.lbl_vel, 4, 0, 1, 2)
        
        bottom_layout.addWidget(telemetry_group, stretch=1)

        # Right Bottom: Control Panel
        control_group = QGroupBox("Control Panel")
        control_layout = QGridLayout(control_group)

        # Wake Up / Sleep
        btn_wake = QPushButton("WAKE UP")
        btn_wake.setStyleSheet("background-color: green; color: white; font-weight: bold;")
        btn_wake.clicked.connect(self.wake_up_robot)
        
        btn_sleep = QPushButton("SLEEP (SIT)")
        btn_sleep.setStyleSheet("background-color: gray; color: white;")
        # "Sleep" now triggers Sit (Motion State 4)
        btn_sleep.clicked.connect(lambda: self.send_command(self.client.set_motion_state, 4))
        
        control_layout.addWidget(btn_wake, 0, 0)
        control_layout.addWidget(btn_sleep, 0, 1)

        # Light Control
        self.light_state = False
        btn_light = QPushButton("Light Toggle")
        btn_light.clicked.connect(self.toggle_light)
        control_layout.addWidget(btn_light, 0, 2)

        # Control Mode Toggle
        self.btn_control_mode = QPushButton("Control: KEYBOARD")
        self.btn_control_mode.clicked.connect(self.toggle_control_mode)
        self.btn_control_mode.setStyleSheet("background-color: #444; color: white;")
        control_layout.addWidget(self.btn_control_mode, 0, 3)

        # Motion State Buttons
        btn_stand = QPushButton("Stand")
        btn_stand.clicked.connect(lambda: self.send_command(self.client.set_motion_state, 1))
        btn_sit = QPushButton("Sit")
        btn_sit.clicked.connect(lambda: self.send_command(self.client.set_motion_state, 4))
        btn_damp = QPushButton("Damping")
        btn_damp.clicked.connect(lambda: self.send_command(self.client.set_motion_state, 3))
        btn_std = QPushButton("Standard")
        btn_std.clicked.connect(lambda: self.send_command(self.client.set_motion_state, 6))

        control_layout.addWidget(QLabel("Motion:"), 1, 0)
        control_layout.addWidget(btn_stand, 1, 1)
        control_layout.addWidget(btn_sit, 1, 2)
        control_layout.addWidget(btn_damp, 1, 3)
        control_layout.addWidget(btn_std, 1, 4)

        # Gait Buttons
        btn_basic = QPushButton("Basic")
        btn_basic.clicked.connect(lambda: self.send_command(self.client.set_gait, 0x1001))
        btn_obs = QPushButton("Obstacles")
        btn_obs.clicked.connect(lambda: self.send_command(self.client.set_gait, 0x1002))
        btn_flat = QPushButton("Flat")
        btn_flat.clicked.connect(lambda: self.send_command(self.client.set_gait, 0x3002))
        btn_stair = QPushButton("Stair")
        btn_stair.clicked.connect(lambda: self.send_command(self.client.set_gait, 0x3003))

        control_layout.addWidget(QLabel("Gait:"), 2, 0)
        control_layout.addWidget(btn_basic, 2, 1)
        control_layout.addWidget(btn_obs, 2, 2)
        control_layout.addWidget(btn_flat, 2, 3)
        control_layout.addWidget(btn_stair, 2, 4)

        bottom_layout.addWidget(control_group, stretch=2)
        
        # Help Group (Far Right Bottom)
        help_group = QGroupBox("Keys")
        help_layout = QVBoxLayout(help_group)
        help_text = """
        <b>WASD</b>: Move<br>
        <b>Q/E</b>: Rotate<br>
        <b>Space</b>: Stop<br>
        <b>Z/U</b>: Sleep/Wake
        """
        lbl_help = QLabel(help_text)
        lbl_help.setFont(QFont("Arial", 9))
        help_layout.addWidget(lbl_help)
        bottom_layout.addWidget(help_group, stretch=0)

        main_layout.addLayout(bottom_layout, stretch=1)

    def send_command(self, func, *args):
        if self.client.loop:
            asyncio.run_coroutine_threadsafe(func(*args), self.client.loop)

    def check_connection(self):
        if time.time() - self.last_msg_time > 2.0:
            self.lbl_conn.setText("Disconnected")
            self.lbl_conn.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 5px;")
        else:
            self.lbl_conn.setText("Connected")
            self.lbl_conn.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 5px;")

    def update_video(self):
        frame = self.streamer.get_tiled_frame()
        if frame is not None:
            # Convert BGR to RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            
            # Scale to fit label
            pixmap = QPixmap.fromImage(qt_img)
            scaled_pixmap = pixmap.scaled(self.video_label.size(), Qt.AspectRatioMode.KeepAspectRatio)
            self.video_label.setPixmap(scaled_pixmap)

    def on_robot_message(self, payload):
        try:
            self.last_msg_time = time.time()
            txt = payload.decode('utf-8', errors='replace')
            data = json.loads(txt)
            
            # Debug: Print Basic Status & DevEnable
            device = data.get("PatrolDevice", {})
            if device.get("Type") == 1002:
                items = device.get("Items", {})
                
                # Status 6: Basic Status
                if device.get("Command") == 6:
                    status = items.get("BasicStatus", {})
                    print(f"Status: Sleep={status.get('Sleep')}, Motion={status.get('MotionState')}, Mode={status.get('ControlUsageMode')}")
                
                # Status 5: Device State (includes DevEnable)
                if device.get("Command") == 5:
                    dev_enable = items.get("DevEnable", {})
                    video_power = dev_enable.get("Video", {})
                    print(f"DevEnable: Video Front={video_power.get('Front')}, Back={video_power.get('Back')}")

            # Emit signal to update UI in main thread
            self.telemetry_signal.emit(data)
        except:
            pass

    def update_telemetry_ui(self, data):
        device = data.get("PatrolDevice", {})
        items = device.get("Items", {})
        
        # Check message type (1002/5 is Device State, 1002/6 is Basic Status)
        msg_type = device.get("Type")
        msg_cmd = device.get("Command")

        if msg_type == 1002 and msg_cmd == 6: # Basic Status
            status = items.get("BasicStatus", {})
            self.lbl_mode.setText(f"Mode: {status.get('ControlUsageMode')}")
            motion_map = {0:'Idle', 1:'Stand', 2:'Estop', 3:'Damping', 4:'Sit', 6:'Standard', 17:'RL Control'}
            self.lbl_motion.setText(f"Motion: {motion_map.get(status.get('MotionState'), 'Unknown')}")

        if msg_type == 1002 and msg_cmd == 4: # Motion Control Status
            motion = items.get("MotionStatus", {})
            if motion:
                vx = motion.get("LinearX", 0.0)
                vy = motion.get("LinearY", 0.0)
                wz = motion.get("OmegaZ", 0.0)
                self.lbl_vel.setText(f"Vel: X={vx:.2f} Y={vy:.2f} W={wz:.2f}")

        if msg_type == 1002 and msg_cmd == 5: # Device State
            # Battery
            batt = items.get("BatteryStatus", {})
            if batt:
                self.lbl_batt_l.setText(f"Batt L: {batt.get('BatteryLevelLeft', 0):.1f}%")
                self.lbl_batt_r.setText(f"Batt R: {batt.get('BatteryLevelRight', 0):.1f}%")
            
            # GPS
            gps = items.get("GPS", {})
            if gps:
                self.lbl_gps.setText(f"GPS: {gps.get('Latitude', 0):.4f}, {gps.get('Longitude', 0):.4f}")

            # Temp
            temps = items.get("DeviceTemperature", {})
            if temps:
                max_t = 0
                for k, v in temps.items():
                    if "Motor" in k and v > max_t:
                        max_t = v
                self.lbl_temp.setText(f"Max Motor Temp: {max_t:.1f}°C")

    def init_joystick(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Joystick initialized: {self.joystick.get_name()}")
        else:
            self.joystick = None
            print("No joystick found.")

    def poll_joystick(self):
        if not self.joystick or self.control_mode != "CONTROLLER":
            return

        pygame.event.pump()
        
        # Deadzone
        DEADZONE = 0.1

        # Read Axes
        axis_0 = self.joystick.get_axis(0) # Left Stick X (Left/Right)
        axis_1 = self.joystick.get_axis(1) # Left Stick Y (Up/Down)
        axis_2 = self.joystick.get_axis(2) # Right Stick X (Rotate)

        # Apply Deadzone
        if abs(axis_0) < DEADZONE: axis_0 = 0
        if abs(axis_1) < DEADZONE: axis_1 = 0
        if abs(axis_2) < DEADZONE: axis_2 = 0

        # Map to Robot Velocities
        # Axis 1: Up (-1) -> +Vx
        self.client.target_vx = -axis_1 * self.client.MAX_SPEED
        
        # Axis 0: Left (-1) -> +Vy
        self.client.target_vy = -axis_0 * self.client.MAX_SPEED
        
        # Axis 2: Left (-1) -> +Vw
        self.client.target_vw = -axis_2 * self.client.MAX_OMEGA

        # Read Buttons
        # Button 0 (A): Stand
        if self.joystick.get_button(0):
            self.send_command(self.client.set_motion_state, 1)
        
        # Button 1 (B): Sit
        if self.joystick.get_button(1):
            self.send_command(self.client.set_motion_state, 4)

        # Button 2 (X): Obstacles Mode
        if self.joystick.get_button(2):
            self.send_command(self.client.set_gait, 0x1002)

        # Button 3 (Y): Standard Mode
        if self.joystick.get_button(3):
            self.send_command(self.client.set_motion_state, 6)
        
        # Button 4 (LB): Damping
        if self.joystick.get_button(4):
            self.send_command(self.client.set_motion_state, 3)

    # --- Key Events for Teleop ---
    def keyPressEvent(self, event):
        key = event.key()
        if event.isAutoRepeat():
            return

        # Mode Switching
        if key == Qt.Key.Key_M: asyncio.run_coroutine_threadsafe(self.client.set_mode(0), self.client.loop)
        elif key == Qt.Key.Key_N: asyncio.run_coroutine_threadsafe(self.client.set_mode(1), self.client.loop)
        elif key == Qt.Key.Key_R: asyncio.run_coroutine_threadsafe(self.client.set_motion_state(1), self.client.loop)
        elif key == Qt.Key.Key_F: asyncio.run_coroutine_threadsafe(self.client.set_motion_state(4), self.client.loop)
        elif key == Qt.Key.Key_Z: asyncio.run_coroutine_threadsafe(self.client.set_sleep_mode(True), self.client.loop)
        elif key == Qt.Key.Key_U: asyncio.run_coroutine_threadsafe(self.client.set_sleep_mode(False), self.client.loop)
        elif key == Qt.Key.Key_Space: asyncio.run_coroutine_threadsafe(self.client.soft_stop(), self.client.loop)

        # Movement (Set Target) - Only if in KEYBOARD mode
        if self.control_mode == "KEYBOARD":
            if key == Qt.Key.Key_W: self.client.target_vx = self.client.MAX_SPEED
            elif key == Qt.Key.Key_S: self.client.target_vx = -self.client.MAX_SPEED
            elif key == Qt.Key.Key_A: self.client.target_vy = self.client.MAX_SPEED
            elif key == Qt.Key.Key_D: self.client.target_vy = -self.client.MAX_SPEED
            elif key == Qt.Key.Key_Q: self.client.target_vw = self.client.MAX_OMEGA
            elif key == Qt.Key.Key_E: self.client.target_vw = -self.client.MAX_OMEGA

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat() or self.control_mode != "KEYBOARD":
            return
        
        key = event.key()
        # Stop if key released
        if key in [Qt.Key.Key_W, Qt.Key.Key_S]: self.client.target_vx = 0.0
        if key in [Qt.Key.Key_A, Qt.Key.Key_D]: self.client.target_vy = 0.0
        if key in [Qt.Key.Key_Q, Qt.Key.Key_E]: self.client.target_vw = 0.0

    def closeEvent(self, event):
        self.client.running = False
        self.streamer.stop()
        event.accept()

    def wake_up_robot(self):
        if self.client.loop:
            # "Wake Up" now means: Damping -> Stand -> Regular Mode
            asyncio.run_coroutine_threadsafe(self._wake_up_sequence(), self.client.loop)

    async def _wake_up_sequence(self):
        print("Initiating wake-up sequence (Motion State Transition)...")
        
        # 1. Send Motion State = Damping (3)
        print("Sending Motion State = Damping (3)...")
        await self.client.set_motion_state(3)
        await asyncio.sleep(1)

        # 2. Send Motion State = Stand (1)
        print("Sending Motion State = Stand (1)...")
        await self.client.set_motion_state(1)
        await asyncio.sleep(1)

        # 3. Set Control Mode = Regular (0)
        print("Sending Control Mode = Regular (0)...")
        await self.client.set_mode(0)
        
        print("Wake-up sequence completed.")

    def toggle_control_mode(self):
        if self.control_mode == "KEYBOARD":
            self.control_mode = "CONTROLLER"
            self.btn_control_mode.setText("Control: CONTROLLER")
            self.btn_control_mode.setStyleSheet("background-color: blue; color: white; font-weight: bold;")
        else:
            self.control_mode = "KEYBOARD"
            self.btn_control_mode.setText("Control: KEYBOARD")
            self.btn_control_mode.setStyleSheet("background-color: #444; color: white;")
            # Stop robot when switching to keyboard to prevent stuck controller values
            self.client.target_vx = 0.0
            self.client.target_vy = 0.0
            self.client.target_vw = 0.0

    def toggle_light(self):
        self.light_state = not self.light_state
        self.send_command(self.client.set_flashlight, self.light_state)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Dashboard()
    window.show()
    sys.exit(app.exec())