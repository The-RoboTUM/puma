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

# create heartbeat payload
def create_heartbeat_payload(time_str):
    payload = {
        "Type": 100,
        "Command": 100,
        "Time": time_str,
        "Items": {
		}
    }
    return json.dumps(payload).encode('utf-8')

async def send_heartbeat(writer):
    message_id = 0  # start at 0
    while True:
        time_str = get_current_time()
        payload_bytes = create_heartbeat_payload(time_str)
        header = build_header(len(payload_bytes), message_id)
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
            print(payload_data)
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
HEARTBEAT_FREQ = 1 # hz
print(get_current_time())
if __name__ == "__main__":
    asyncio.run(main(ADDR, PORT))
