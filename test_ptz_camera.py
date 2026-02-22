import serial
import time

def pelco_d(address, cmd1, cmd2, data1, data2):
    checksum = (address + cmd1 + cmd2 + data1 + data2) % 256
    return bytes([0xFF, address, cmd1, cmd2, data1, data2, checksum])

try:
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=9600,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=1
    )

    print("Turning LEFT...")
    cmd = pelco_d(1, 0x00, 0x04, 0x20, 0x00)
    ser.write(cmd)
    time.sleep(2)

    print("STOP")
    cmd_stop = pelco_d(1, 0x00, 0x00, 0x00, 0x00)
    ser.write(cmd_stop)

    ser.close()
    print("Done.")

except Exception as e:
    print("Error:", e)