import struct
import json
from datetime import datetime

# Network Configuration
ROBOT_IP = "10.21.31.103"
ROBOT_PORT = 30001
ROBOT_UDP_PORT = 30000
HEARTBEAT_FREQ = 2.0

# Protocol Constants
SYNC_BYTES = bytes([0xEB, 0x91, 0xEB, 0x90])

def get_timestamp():
    """Returns current time formatted for the robot protocol."""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def build_header(payload_len, msg_id, is_json=True):
    """Constructs the 16-byte protocol header."""
    return (
        SYNC_BYTES +
        struct.pack('<H', payload_len) +
        struct.pack('<H', msg_id) +
        bytes([0x01 if is_json else 0x00]) +
        bytes(7)  # Reserved
    )

def parse_header(header):
    """Parses the 16-byte header. Returns (payload_len, msg_id) or None."""
    if len(header) != 16 or header[:4] != SYNC_BYTES:
        return None
    payload_len = struct.unpack('<H', header[4:6])[0]
    msg_id = struct.unpack('<H', header[6:8])[0]
    return payload_len, msg_id

def create_json_payload(type_code, cmd_code, items=None):
    """Creates a JSON payload encoded in bytes."""
    payload = {
        "PatrolDevice": {
            "Type": type_code,
            "Command": cmd_code,
            "Time": get_timestamp(),
            "Items": items or {}
        }
    }
    return json.dumps(payload).encode('utf-8')
