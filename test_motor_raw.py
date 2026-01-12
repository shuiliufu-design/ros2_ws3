#!/usr/bin/env python3
import serial
import time

PORT = '/dev/ttyUSB0'
BAUDS = [57600, 115200, 9600]

def test_baud(baud):
    print(f"----------------------------------------")
    print(f"Testing {PORT} at {baud}...")
    try:
        ser = serial.Serial(PORT, baud, timeout=2)
        time.sleep(2) # Wait for Arduino Reset
        
        # Flush garbage
        ser.reset_input_buffer()
        
        # 1. Ping / Read Encoders
        print(f"[{baud}] Sending 'e'...")
        ser.write(b'e\r')
        resp = ser.read_until().decode(errors='ignore').strip()
        print(f"[{baud}] Response: '{resp}'")
        
        if resp:
            print(f"SUCCESS! Found device at {baud}")
            ser.close()
            return True
            
        ser.close()
    except Exception as e:
        print(f"[{baud}] Error: {e}")
    return False

if __name__ == '__main__':
    found = False
    for b in BAUDS:
        if test_baud(b):
            found = True
            break
    
    if not found:
        print("----------------------------------------")
        print("FAILED to communicate at any baud rate.")
        print("Possible causes:")
        print("1. Wrong Port (Are you sure it's USB0?)")
        print("2. Firmware needs 'rosserial' (ROS_MODE=1) which is binary, not text.")
        print("3. Bad USB Cable or Board.")
