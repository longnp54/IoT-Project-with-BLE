"""
ESP32 Request Reader
Đọc các request từ ESP32 qua UART

Protocol:
- Sensor Request: [0xAA, 0x01, 0x00, 0x55] (4 bytes)
- LED ON:  [0xAA, 0x02, 0x01, 0x01, 0x55] (5 bytes)  
- LED OFF: [0xAA, 0x02, 0x01, 0x00, 0x55] (5 bytes)
- Response: [0xAA, 0x03, TEMP, HUM, LED, 0x55] (6 bytes)

Usage:
    python test_esp32_request.py COM4 115200
    or
    python test_esp32_request.py /dev/ttyUSB0 115200
"""

import serial
import sys
import time
from typing import Optional, Tuple

class ESP32Reader:
    """Đọc và parse request từ ESP32"""
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.5):
        """
        Khởi tạo kết nối serial
        
        Args:
            port: COM port (e.g., 'COM4', '/dev/ttyUSB0')
            baudrate: Tốc độ baud (default 115200)
            timeout: Timeout cho read (seconds)
        """
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
    def connect(self) -> bool:
        """Kết nối tới ESP32"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"✓ Đã kết nối tới {self.port} @ {self.baudrate} baud")
            time.sleep(1)  # Đợi ESP32 khởi động
            return True
        except Exception as e:
            print(f"✗ Lỗi kết nối {self.port}: {e}")
            return False
    
    def read_frame(self) -> Optional[bytes]:
        """
        Đọc một frame hoàn chỉnh từ ESP32
        
        Returns:
            bytes: Frame nhận được hoặc None nếu timeout
        """
        if not self.ser or not self.ser.is_open:
            return None
        
        # Đợi START byte (0xAA)
        while True:
            byte = self.ser.read(1)
            if not byte:
                return None  # Timeout
            if byte[0] == 0xAA:
                frame = bytearray([byte[0]])
                break
        
        # Đọc TYPE và LEN
        frame.extend(self.ser.read(2))
        if len(frame) < 3:
            return None
        
        # Xác định độ dài expected
        data_len = frame[2]
        expected_len = 1 + 1 + 1 + data_len + 1  # START + TYPE + LEN + DATA + END
        
        # Đọc phần còn lại của frame
        remaining = expected_len - len(frame)
        frame.extend(self.ser.read(remaining))
        
        # Kiểm tra END byte (0x55)
        if frame[-1] != 0x55:
            print(f"⚠ END byte không đúng: {frame[-1]:02X}")
            return None
        
        return bytes(frame)
    
    def parse_frame(self, frame: bytes) -> Optional[dict]:
        """
        Parse frame và trích thông tin
        
        Args:
            frame: bytes của frame
            
        Returns:
            dict với thông tin frame hoặc None nếu invalid
        """
        if len(frame) < 4:
            return None
        
        if frame[0] != 0xAA or frame[-1] != 0x55:
            return None
        
        frame_type = frame[1]
        data_len = frame[2]
        data = frame[3:3+data_len] if data_len > 0 else b''
        
        result = {
            'type': frame_type,
            'len': data_len,
            'data': data,
            'hex': ' '.join(f'{b:02X}' for b in frame),
        }
        
        # Parse theo loại command
        if frame_type == 0x01 and data_len == 0:
            result['name'] = 'SENSOR_REQUEST'
            
        elif frame_type == 0x02 and data_len == 1:
            result['name'] = 'LED_CONTROL'
            result['led_state'] = 'ON' if data[0] == 0x01 else 'OFF'
            
        else:
            result['name'] = 'UNKNOWN'
        
        return result
    
    def print_frame(self, parsed: dict):
        """In thông tin frame"""
        print(f"\n📥 Request từ ESP32:")
        print(f"   Loại: {parsed['name']} (0x{parsed['type']:02X})")
        print(f"   Dữ liệu: {parsed['hex']}")
        
        if parsed['name'] == 'SENSOR_REQUEST':
            print(f"   → ESP32 yêu cầu đọc cảm biến")
        elif parsed['name'] == 'LED_CONTROL':
            print(f"   → Điều khiển LED: {parsed['led_state']}")
        else:
            print(f"   → Dữ liệu: {parsed['data'].hex()}")
    
    def run(self, count: int = 0):
        """
        Đọc liên tục request từ ESP32
        
        Args:
            count: Số request cần đọc (0 = vô hạn)
        """
        if not self.connect():
            return
        
        print(f"\n{'='*60}")
        print("Đang đợi request từ ESP32... (Ctrl+C để dừng)")
        print(f"{'='*60}")
        
        frame_count = 0
        try:
            while True:
                if count > 0 and frame_count >= count:
                    break
                
                frame = self.read_frame()
                if frame:
                    parsed = self.parse_frame(frame)
                    if parsed:
                        frame_count += 1
                        self.print_frame(parsed)
                    else:
                        print("⚠ Frame không hợp lệ")
                        
        except KeyboardInterrupt:
            print(f"\n\n⏹ Dừng lại (đã nhận {frame_count} request)")
        except Exception as e:
            print(f"✗ Lỗi: {e}")
        finally:
            self.close()
    
    def close(self):
        """Đóng kết nối"""
        if self.ser:
            self.ser.close()
            print("✓ Đã đóng kết nối")


def main():
    """Main entry point"""
    import serial.tools.list_ports
    
    # Lấy port từ command line hoặc auto-detect
    if len(sys.argv) > 1:
        port = sys.argv[1]
        baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    else:
        # Auto-detect ESP32 ports
        ports = [p.device for p in serial.tools.list_ports.comports()]
        esp32_ports = [p for p in ports if 'USB' in p or 'COM' in p]
        
        if not esp32_ports:
            print("Không tìm thấy port serial nào!")
            print("\nSử dụng: python test_esp32_request.py COM4 115200")
            return
        
        port = esp32_ports[0]
        baudrate = 115200
        print(f"🔍 Tự động phát hiện port: {port}")
    
    # Tạo reader và chạy
    reader = ESP32Reader(port, baudrate)
    reader.run()


if __name__ == "__main__":
    main()
