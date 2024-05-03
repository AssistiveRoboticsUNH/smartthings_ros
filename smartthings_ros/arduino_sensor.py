import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import serial

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
                    #print(bump_value)

                    # Publish bump sensor data
                    bump_msg = Int64()
                    bump_msg.data = bump_value
                    self.bump_pub.publish(bump_msg)
            except (ValueError, IndexError) as e:
                self.get_logger().error("Error parsing serial data: %s", str(e))

def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialDataPublisher('/dev/ttyUSB3', 9600)
    rclpy.spin(serial_publisher)
    serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
