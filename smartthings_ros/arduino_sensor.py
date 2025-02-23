import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import serial
from serial.tools import list_ports

class SerialDataPublisher(Node):
    def __init__(self, comport, baudrate):
        super().__init__('arduino_sensor_node')
        self.ser = serial.Serial(comport, baudrate, timeout=0.1)
        self.bump_pub = self.create_publisher(Int64, 'bump', 10)
        self.timer = self.create_timer(0.001, self.read_serial_and_publish)

    def read_serial_and_publish(self):
        if self.ser.in_waiting > 0:
            data = self.ser.readline().strip()
            try:
                if data.startswith(b'b='):
                    # Parse bump sensor data
                    bump_value = int(data.split(b'=')[1])

                    # Publish bump sensor data
                    bump_msg = Int64()
                    bump_msg.data = bump_value
                    self.bump_pub.publish(bump_msg)
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Error parsing serial data: {str(e)}")


def find_arduino_nano_port(baudrate, vid, pid, serial_number=None):
    # List all available serial ports
    ports = list_ports.comports()
    for port in ports:
        if (port.vid == vid) and (port.pid == pid):
            # Check for the specific serial number if provided
            if serial_number and port.serial_number != serial_number:
                continue
            try:
                # Try opening the port to confirm it's available
                ser = serial.Serial(port.device, baudrate, timeout=0.1)
                ser.close()
                return port.device
            except (serial.SerialException, OSError):
                continue
    raise RuntimeError("No Arduino Nano found with the specified VID, PID, and serial number.")

def main(args=None):
    rclpy.init(args=args)
    baudrate = 9600
    
    # Replace these with the specific VID, PID, and serial number of your Arduino Nano
    arduino_nano_vid = 0x0403  # Example VID for FT232R USB UART
    arduino_nano_pid = 0x6001  # Example PID for FT232R USB UART
    arduino_nano_serial = "AQ02T6IH"  # Your Arduino's serial number

    try:
        # Find the Arduino Nano port
        available_port = find_arduino_nano_port(baudrate, arduino_nano_vid, arduino_nano_pid, arduino_nano_serial)
        serial_publisher = SerialDataPublisher(available_port, baudrate)
        rclpy.spin(serial_publisher)
    except RuntimeError as e:
        print(f"Error: {e}")
    finally:
        if 'serial_publisher' in locals():
            serial_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
