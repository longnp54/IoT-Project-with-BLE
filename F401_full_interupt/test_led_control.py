"""
STM32F401 Pure Interrupt-Driven Test Script
Test LED control và sensor reading via UART commands

NEW Protocol:
- Sensor Request: [0xAA, 0x01, 0x00, 0x55] (4 bytes)
- LED ON:  [0xAA, 0x02, 0x01, 0x01, 0x55] (5 bytes)  
- LED OFF: [0xAA, 0x02, 0x01, 0x00, 0x55] (5 bytes)
- Response: [0xAA, 0x01, 0x03, TEMP, HUM, LED, 0x55] (7 bytes)

Usage:
    python test_led_control.py COM3 115200
"""

import serial
import time
import sys

def send_sensor_request(ser):
    """
    Send sensor data request
    Frame: [AA][01][00][55] (4 bytes)
    """
    frame = bytes([0xAA, 0x01, 0x00, 0x55])
    print(f"Request: {''.join(f'{b:02X}' for b in frame)}")
    ser.write(frame)
    
def send_led_command(ser, led_state):
    """
    Send LED control command
    Frame: [AA][02][01][LED_STATE][55] (5 bytes)
    """
    frame = bytes([0xAA, 0x02, 0x01, led_state, 0x55])
    print(f"LED {['OFF','ON'][led_state]}: {''.join(f'{b:02X}' for b in frame)}")
    ser.write(frame)
def read_response(ser):
    """
    Read and parse response
    Expected: [AA][01][03][TEMP][HUM][LED][55] (7 bytes)
    """
    time.sleep(0.1)  # Wait for response
    if ser.in_waiting >= 7:
        response = ser.read(7)
        print(f"Response: {''.join(f'{b:02X}' for b in response)}")
        
        if len(response) == 7 and response[0] == 0xAA and response[6] == 0x55:
            if response[1] == 0x01 and response[2] == 0x03:  # Type=01, Len=03
                temp = response[3]
                hum = response[4]
                led = response[5]
                led_str = "ON" if led == 1 else "OFF"
                print(f"✓ Temp: {temp}°C, Hum: {hum}%, LED: {led_str}")
                return temp, hum, led
        
        print("✗ Invalid response format")
        return None
    else:
        print("✗ No response or incomplete")
        return None

def test_sensor_and_led():
    """Test both sensor reading and LED control"""
    import serial.tools.list_ports
    
    # Find available ports
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        print("No serial ports found!")
        return
    
    # Try each port
    for port in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1'] + ports:
        try:
            print(f"\nTrying port: {port}")
            ser = serial.Serial(port, 115200, timeout=0.5)
            print(f"✓ Port {port} is available")
            print(f"Connected to {port}")
            time.sleep(1)  # Wait for STM32 startup
            
            print("\nTesting STM32 với DHT11 trên PA1 và LED trên PA5")
            print("=" * 60)
            
            # Test sequence
            for i in range(3):
                print(f"\nTest #{i+1}:")
                
                # Send sensor request
                send_sensor_request(ser)
                result = read_response(ser)
                
                if result:
                    temp, hum, led = result
                    if temp > 0 and hum > 0:
                        print("✅ DHT11 đang hoạt động tốt!")
                    else:
                        print("⚠️ DHT11 có thể chưa kết nối hoặc đang dùng default values")
                else:
                    print("❌ Không nhận được response hợp lệ")
                
                # Toggle LED
                led_state = i % 2  # Alternate 0, 1, 0
                send_led_command(ser, led_state)
                read_response(ser)
                
                time.sleep(1)
            
            print("\n" + "=" * 60)
            print("Test completed!")
            ser.close()
            return
            
        except Exception as e:
            print(f"✗ Failed to connect to {port}: {e}")
            continue
    
    print("Could not find working serial port!")

def test_continuous():
    """Continuously send requests to test response"""
    import serial.tools.list_ports
    
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        print("No serial ports found!")
        return
    
    # Try each port
    for port in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1'] + ports:
        try:
            print(f"\nConnecting to {port}...")
            ser = serial.Serial(port, 115200, timeout=0.5)
            print(f"✓ Connected to {port}")
            time.sleep(1)
            
            print("\n" + "=" * 60)
            print("Continuous Test - Sending requests automatically")
            print("(Ctrl+C to stop)")
            print("=" * 60)
            
            led_state = 0
            test_num = 0
            
            try:
                while True:
                    test_num += 1
                    print(f"\n[Test {test_num}] Sensor Request:")
                    send_sensor_request(ser)
                    result = read_response(ser)
                    
                    if result:
                        temp, hum, led = result
                        print(f"  ✓ Data received: {temp}°C, {hum}%, LED={'ON' if led else 'OFF'}")
                    else:
                        print(f"  ✗ No response")
                    
                    # Send LED command every 2 requests
                    if test_num % 2 == 0:
                        print(f"\n[Test {test_num}] LED Control:")
                        led_state = 1 - led_state
                        send_led_command(ser, led_state)
                        result = read_response(ser)
                        if result:
                            print(f"  ✓ LED command executed: LED={'ON' if led_state else 'OFF'}")
                        else:
                            print(f"  ✗ No response")
                    
                    time.sleep(1)
                    
            except KeyboardInterrupt:
                print(f"\n\n{'=' * 60}")
                print(f"Test stopped! ({test_num} requests sent)")
                print("=" * 60)
            
            ser.close()
            return
            
        except Exception as e:
            print(f"✗ Failed to connect to {port}: {e}")
            continue
    
    print("Could not find working serial port!")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "continuous":
        test_continuous()
    else:
        test_sensor_and_led()
