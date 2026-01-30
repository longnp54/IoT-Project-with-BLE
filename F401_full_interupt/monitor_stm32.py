#!/usr/bin/env python3
"""
Monitor STM32 raw output to see what it's sending
"""
import serial
import time

def monitor_stm32():
    try:
        # Connect to STM32
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        print("Connected to STM32 at /dev/ttyACM0")
        print("Monitoring all data from STM32...")
        print("Send commands from ESP32 and watch output here")
        print("Press Ctrl+C to exit")
        print("-" * 50)
        
        while True:
            if ser.in_waiting > 0:
                # Read any available data
                data = ser.read(ser.in_waiting)
                
                # Print as hex
                hex_str = ' '.join(f'{b:02X}' for b in data)
                print(f"HEX: {hex_str}")
                
                # Print as text (if printable)
                try:
                    text = data.decode('utf-8', errors='ignore')
                    if text.strip():
                        print(f"TXT: {text.strip()}")
                except:
                    pass
                print("-" * 30)
            
            time.sleep(0.01)  # Small delay
            
    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nMonitoring stopped")
        ser.close()

if __name__ == "__main__":
    monitor_stm32()