"""
STM32 Response Test Script
Gửi request tới STM32 và chi tiết log tất cả response bytes

Protocol:
- Sensor Request: [0xAA, 0x01, 0x00, 0x55] (4 bytes)
- LED ON:  [0xAA, 0x02, 0x01, 0x01, 0x55] (5 bytes)  
- LED OFF: [0xAA, 0x02, 0x01, 0x00, 0x55] (5 bytes)
- Response: [0xAA, 0x01, 0x03, TEMP, HUM, LED, 0x55] (7 bytes)

Usage:
    python test_stm32_response.py /dev/ttyUSB0 115200
"""

import serial
import time
import sys
import serial.tools.list_ports

class STM32ResponseTester:
    def __init__(self, port, baudrate=115200, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.test_count = 0
        
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"✓ Kết nối tới {self.port} @ {self.baudrate} baud")
            time.sleep(1)  # Đợi STM32 khởi động
            return True
        except Exception as e:
            print(f"✗ Lỗi kết nối {self.port}: {e}")
            return False
    
    def close(self):
        if self.ser:
            self.ser.close()
    
    def send_request(self, name, data):
        """Gửi request và log"""
        print(f"\n{'='*70}")
        self.test_count += 1
        print(f"[Test {self.test_count}] {name}")
        print(f"{'='*70}")
        
        frame = bytes(data)
        hex_str = ' '.join(f'{b:02X}' for b in frame)
        print(f"📤 TX ({len(frame)} bytes): {hex_str}")
        
        self.ser.write(frame)
        self.ser.flush()
        
        # Đợi response
        time.sleep(0.2)
        
        # Đọc tất cả bytes có sẵn
        response_bytes = []
        start_time = time.time()
        
        while time.time() - start_time < self.timeout:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if byte:
                    response_bytes.append(byte[0])
            else:
                # Nếu đã có bytes và không còn bytes nữa, dừng
                if response_bytes:
                    time.sleep(0.05)
                    if self.ser.in_waiting == 0:
                        break
        
        # Log response
        if response_bytes:
            print(f"\n📥 RX ({len(response_bytes)} bytes):")
            hex_str = ' '.join(f'{b:02X}' for b in response_bytes)
            print(f"   HEX: {hex_str}")
            
            # Parse từng byte
            print(f"\n   Byte-by-byte breakdown:")
            for i, b in enumerate(response_bytes):
                print(f"   [{i}] = 0x{b:02X} ({b:3d})", end="")
                
                # Annotation
                if i == 0:
                    print(f"  ← START byte (0xAA)" if b == 0xAA else f"  ✗ WRONG START!")
                elif i == 1:
                    print(f"  ← Response Type (0x01)" if b == 0x01 else f"  ? Type: 0x{b:02X}")
                elif i == 2:
                    print(f"  ← Data Length (3 bytes)" if b == 0x03 else f"  ? Length: {b}")
                elif i == 3:
                    print(f"  ← Temperature: {b}°C")
                elif i == 4:
                    print(f"  ← Humidity: {b}%")
                elif i == 5:
                    led_state = "ON" if b == 0x01 else "OFF"
                    print(f"  ← LED State: {led_state}")
                elif i == 6:
                    print(f"  ← END byte (0x55)" if b == 0x55 else f"  ✗ WRONG END!")
                else:
                    print(f"  ? Extra byte")
            
            # Validate
            print(f"\n   Validation:")
            if len(response_bytes) == 7:
                print(f"   ✓ Length = 7 bytes (correct)")
            else:
                print(f"   ✗ Length = {len(response_bytes)} bytes (expected 7)")
            
            if response_bytes[0] == 0xAA:
                print(f"   ✓ START byte = 0xAA (correct)")
            else:
                print(f"   ✗ START byte = 0x{response_bytes[0]:02X} (expected 0xAA)")
            
            if len(response_bytes) >= 7 and response_bytes[6] == 0x55:
                print(f"   ✓ END byte = 0x55 (correct)")
            elif len(response_bytes) >= 7:
                print(f"   ✗ END byte = 0x{response_bytes[6]:02X} (expected 0x55)")
            
            # Extract & display values
            if len(response_bytes) >= 7:
                temp = response_bytes[3]
                hum = response_bytes[4]
                led = response_bytes[5]
                led_str = "ON" if led == 0x01 else "OFF"
                print(f"\n   📊 Data extracted:")
                print(f"      Temperature: {temp}°C")
                print(f"      Humidity: {hum}%")
                print(f"      LED State: {led_str} (0x{led:02X})")
        else:
            print(f"\n📥 RX: ✗ No response received (timeout after {self.timeout}s)")
    
    def run_tests(self):
        """Chạy các test"""
        if not self.connect():
            return
        
        try:
            # Test 1: Sensor Request
            self.send_request(
                "Sensor Data Request",
                [0xAA, 0x01, 0x00, 0x55]
            )
            time.sleep(0.5)
            
            # Test 2: LED ON
            self.send_request(
                "LED Control - Turn ON",
                [0xAA, 0x02, 0x01, 0x01, 0x55]
            )
            time.sleep(0.5)
            
            # Test 3: Sensor Request (after LED ON)
            self.send_request(
                "Sensor Data Request (LED should be ON)",
                [0xAA, 0x01, 0x00, 0x55]
            )
            time.sleep(0.5)
            
            # Test 4: LED OFF
            self.send_request(
                "LED Control - Turn OFF",
                [0xAA, 0x02, 0x01, 0x00, 0x55]
            )
            time.sleep(0.5)
            
            # Test 5: Sensor Request (after LED OFF)
            self.send_request(
                "Sensor Data Request (LED should be OFF)",
                [0xAA, 0x01, 0x00, 0x55]
            )
            
        except KeyboardInterrupt:
            print(f"\n\n⏹ Test dừng lại bởi user")
        except Exception as e:
            print(f"✗ Error: {e}")
        finally:
            self.close()
            print(f"\n{'='*70}")
            print(f"Test completed ({self.test_count} requests sent)")
            print(f"{'='*70}")

def find_ports():
    """Tìm tất cả serial ports"""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return ports

def main():
    # Nếu có argument, dùng port đó
    if len(sys.argv) > 1:
        port = sys.argv[1]
        baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    else:
        # Auto-detect
        ports = find_ports()
        if not ports:
            print("✗ Không tìm thấy serial port nào!")
            print("\nDanh sách ports khả dụng:")
            print("  Linux/Mac: /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyACM0")
            print("  Windows:   COM3, COM4, COM5")
            print("\nSử dụng: python test_stm32_response.py /dev/ttyUSB0 115200")
            return
        
        print(f"Các ports được tìm thấy: {ports}")
        port = ports[0]
        baudrate = 115200
        print(f"Sử dụng port: {port}")
    
    # Chạy test
    tester = STM32ResponseTester(port, baudrate)
    tester.run_tests()

if __name__ == "__main__":
    main()
