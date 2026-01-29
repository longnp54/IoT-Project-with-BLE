import serial
import time

# Cấu hình COM port
COM_PORT = 'COM7'  # Sửa số COM ở đây
BAUD_RATE = 115200

print(f"Opening {COM_PORT} at {BAUD_RATE} baud...")
print("Ctrl+C to stop\n")

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    
    while True:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            hex_str = ' '.join(f'{b:02X}' for b in data)
            print(f"RX: {hex_str}")
        time.sleep(0.1)
        
except KeyboardInterrupt:
    print("\n\nStopped")
finally:
    if 'ser' in locals():
        ser.close()