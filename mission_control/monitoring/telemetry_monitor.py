import asyncio
import sys
import os
import json

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol import (
    ROBOT_IP, ROBOT_PORT, HEARTBEAT_FREQ, get_timestamp,
    build_header, parse_header, create_json_payload
)

class TelemetryMonitor:
    def __init__(self, host=ROBOT_IP, port=ROBOT_PORT):
        self.host = host
        self.port = port
        self.reader = None
        self.writer = None
        self.message_id = 0
        self.running = False
        self.telemetry = {}

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

    async def listen_loop(self):
        while self.running:
            try:
                header_data = await self.reader.readexactly(16)
                payload_len, msg_id = parse_header(header_data)
                
                if payload_len and payload_len > 0:
                    payload_data = await self.reader.readexactly(payload_len)
                    try:
                        txt = payload_data.decode('utf-8', errors='replace')
                        data = json.loads(txt)
                        
                        device_data = data.get("PatrolDevice", {})
                        msg_type = device_data.get("Type")
                        msg_cmd = device_data.get("Command")
                        items = device_data.get("Items", {})

                        # Type 1002, Command 5: Device State Report
                        if msg_type == 1002 and msg_cmd == 5:
                            self.telemetry = items
                            self.display_telemetry()
                            
                    except json.JSONDecodeError:
                        pass
                        
            except asyncio.IncompleteReadError:
                print("Disconnected")
                self.running = False
                break
            except Exception as e:
                print(f"Error reading: {e}")
                break

    def display_telemetry(self):
        # Clear screen (ANSI escape code)
        print("\033[H\033[J", end="")
        
        print("=== ROBOT TELEMETRY ===")
        print(f"Last Update: {get_timestamp()}")
        print("-" * 30)
        
        # Battery
        batt = self.telemetry.get("BatteryStatus", {})
        if batt:
            print("BATTERY:")
            print(f"  Left:  {batt.get('BatteryLevelLeft', 0):.1f}% ({batt.get('VoltageLeft', 0):.1f}V) {batt.get('battery_temperatureLeft', 0):.1f}°C")
            print(f"  Right: {batt.get('BatteryLevelRight', 0):.1f}% ({batt.get('VoltageRight', 0):.1f}V) {batt.get('battery_temperatureRight', 0):.1f}°C")
            print(f"  Charging: L={batt.get('chargeLeft')} R={batt.get('chargeRight')}")
        
        # GPS
        gps = self.telemetry.get("GPS", {})
        if gps:
            print("\nGPS:")
            print(f"  Lat: {gps.get('Latitude', 0):.6f}  Lon: {gps.get('Longitude', 0):.6f}")
            print(f"  Satellites: {gps.get('NumSatellites', 0)} (Visible: {gps.get('VisibleSatellites', 0)})")
            print(f"  Fix Quality: {gps.get('FixQuality', 0)}")

        # CPU Temps
        cpu = self.telemetry.get("CPU", {})
        if cpu:
            print("\nCPU TEMPERATURES:")
            for cpu_name, data in cpu.items():
                print(f"  {cpu_name}: {data.get('Temperature', 0):.1f}°C")

        # Device Temps (Summary)
        temps = self.telemetry.get("DeviceTemperature", {})
        if temps:
            print("\nMOTOR TEMPERATURES (Max):")
            # Find max motor temp
            max_temp = 0
            max_name = ""
            for name, val in temps.items():
                if "Motor" in name and val > max_temp:
                    max_temp = val
                    max_name = name
            print(f"  Hottest Motor: {max_name} ({max_temp:.1f}°C)")

    async def close(self):
        self.running = False
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()

async def main():
    monitor = TelemetryMonitor()
    try:
        await monitor.connect()
        
        tasks = [
            asyncio.create_task(monitor.heartbeat_loop()),
            asyncio.create_task(monitor.listen_loop())
        ]
        
        await asyncio.gather(*tasks)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        await monitor.close()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
