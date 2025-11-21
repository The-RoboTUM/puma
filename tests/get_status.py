import asyncio
import json
import struct
from datetime import datetime
import xmltodict

def get_current_time():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
# build header
def build_header(payload_length, message_id, is_json=True):
    sync_chars = bytes([0xEB, 0x91, 0xEB, 0x90])
    length_bytes = struct.pack('<H', payload_length)
    message_id_bytes = struct.pack('<H', message_id)
    asdu_structure = bytes([0x01 if is_json else 0x00])
    reserved = bytes(7)
    header = sync_chars + length_bytes + message_id_bytes + asdu_structure + reserved
    assert len(header) == 16
    return header

def parse_header(header_bytes):
    # header_bytes must be exactly 16 bytes
    assert len(header_bytes) == 16
    assert header_bytes[0:4] == bytes([0xEB, 0x91, 0xEB, 0x90])
    payload_length = struct.unpack('<H', header_bytes[4:6])[0]
    message_id = struct.unpack('<H', header_bytes[6:8])[0]
    asdu_structure = header_bytes[8]
    # reserved bytes 9-15 can be checked if needed
    return payload_length, message_id, asdu_structure

# JSON heartbeat: must match XML structure with PatrolDevice root
def create_heartbeat_payload_json(time_str):
    payload = {
        "PatrolDevice": {
            "Type": 100,
            "Command": 100,
            "Time": time_str,
            "Items": {}
        }
    }
    return json.dumps(payload).encode('utf-8')

# XML heartbeat
def create_heartbeat_payload_xml(time_str):
    xml_payload = f"""<?xml version="1.0" encoding="UTF-8"?>
    <PatrolDevice>
        <Type>100</Type>
        <Command>100</Command>
        <Time>{time_str}</Time>
        <Items/>
    </PatrolDevice>
    """
    return xml_payload.encode('utf-8')

async def send_heartbeat(writer):
    message_id = 0  # start at 0
    while True:
        time_str = get_current_time()
        
        # Create payload based on mode
        if HEARTBEAT_MODE == "JSON":
            payload_bytes = create_heartbeat_payload_json(time_str)
            is_json = True
        elif HEARTBEAT_MODE == "XML":
            payload_bytes = create_heartbeat_payload_xml(time_str)
            is_json = False
        else:
            raise ValueError(f"Invalid HEARTBEAT_MODE: {HEARTBEAT_MODE}. Must be 'JSON' or 'XML'")
        
        # Build and send message
        header = build_header(len(payload_bytes), message_id, is_json=is_json)
        message = header + payload_bytes

        writer.write(message)
        await writer.drain()
        print(f"Heartbeat #{message_id} sent at {time_str} with message_id {message_id}")
        
        message_id = (message_id + 1) % 65536  # wrap-around at 65535

        await asyncio.sleep(1 / HEARTBEAT_FREQ)

async def listen_responses(reader):
    while True:
        try:
            header_data = await reader.readexactly(16)
        except asyncio.IncompleteReadError:
            print("Connection closed by server")
            break

        payload_len, message_id, asdu_structure = parse_header(header_data)
        fmt = "JSON" if asdu_structure == 0x01 else "XML"
        print(f"Received message of size {payload_len} with id {message_id} and format: {fmt}")

        try:
            payload_data = await reader.readexactly(payload_len)

            # Handle empty payload
            if payload_len == 0:
                print("Received empty payload")
                continue

            # Decode payload
            txt = payload_data.decode('utf-8', errors='replace')

            # Parse based on declared format
            if fmt == "JSON":
                try:
                    obj = json.loads(txt)
                    print(json.dumps(obj, indent=2))
                except json.JSONDecodeError as e:
                    print(f"Failed to parse JSON payload: {e}")
                    # Try XML as fallback (server might send XML even when claiming JSON)
                    if txt.strip().startswith('<') or txt.strip().startswith('<?xml'):
                        try:
                            obj = xmltodict.parse(txt)
                            print("(Server sent XML despite JSON header)")
                            print(json.dumps(obj, indent=2))
                        except Exception as e_xml:
                            print(f"Also failed to parse as XML: {e_xml}")
                            print("Raw payload (as text):")
                            print(txt)
                    else:
                        print("Raw payload (as text):")
                        print(txt)
            elif fmt == "XML":
                try:
                    obj = xmltodict.parse(txt)
                    print(json.dumps(obj, indent=2))
                except Exception as e:
                    print(f"Failed to parse XML payload: {e}")
                    print("Raw payload (as text):")
                    print(txt)
            else:
                # Should never happen, but handle gracefully
                print(f"Unknown format: {fmt}")
                print("Raw payload (as text):")
                print(txt)

        except asyncio.IncompleteReadError:
            print("Connection closed by server during payload read")
            break



async def main(host, port):
    try:
        print(f"Trying to establish connection to {host}:{port} ...")
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(host, port),
            timeout=3.0  # seconds
        )
        print(f"Opened connection to {host}:{port}, sending heartbeats at {HEARTBEAT_FREQ}Hz")

        await asyncio.gather(
            send_heartbeat(writer),
            listen_responses(reader)
        )
    except asyncio.TimeoutError:
        print(f"Connection to {host}:{port} timed out after 3 seconds")
    except (ConnectionRefusedError, OSError) as e:
        print(f"Connection failed: {e}")
    

ADDR = "10.21.31.103"
PORT = 30001
HEARTBEAT_FREQ = 0.1  # hz
HEARTBEAT_MODE = "XML"  # Must be "JSON" or "XML"


# Validate configuration
if HEARTBEAT_MODE not in ("JSON", "XML"):
    raise ValueError(f"Invalid HEARTBEAT_MODE: '{HEARTBEAT_MODE}'. Must be 'JSON' or 'XML'")

print(get_current_time())
if __name__ == "__main__":
    asyncio.run(main(ADDR, PORT))
