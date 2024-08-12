import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
import struct

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        self.serial_port = serial.Serial('/dev/ttyACM1', 2000000, timeout=1)
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            start_byte = self.serial_port.read(1)
            if start_byte == b'\x7E':  # Check for start byte
                data = self.serial_port.read(128)  # Read 64 distances (2 bytes each)
                end_byte = self.serial_port.read(1)
                if end_byte == b'\x7F':  # Check for end byte
                    distances = struct.unpack('H' * 64, data)
                    message = String()
                    message.data = ' '.join(map(str, distances))
                    self.publisher_.publish(message)
                    self.get_logger().info('Published sensor data')

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
