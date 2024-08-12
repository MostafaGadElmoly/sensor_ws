import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_top_ = self.create_publisher(String, '/sensor_data_top', 100)
        self.publisher_bottom_ = self.create_publisher(String, '/sensor_data_bottom', 100)
        self.ser = serial.Serial('/dev/ttyACM1', 2000000, timeout=1)
        self.timer = self.create_timer(0.0027, self.timer_callback)

    def read_data_packet(self):
        start_bit = self.ser.read(1)
        if len(start_bit) == 0 or start_bit[0] != 0x7E:
            self.get_logger().warn('Invalid start byte')
            return None

        sensor_id = self.ser.read(1)
        if len(sensor_id) == 0:
            self.get_logger().warn('Failed to read sensor ID')
            return None

        zone_id = self.ser.read(1)
        if len(zone_id) == 0:
            self.get_logger().warn('Failed to read zone ID')
            return None

        distance_bytes = self.ser.read(2)
        if len(distance_bytes) < 2:
            self.get_logger().warn('Failed to read distance bytes')
            return None

        end_bit = self.ser.read(1)
        if len(end_bit) == 0 or end_bit[0] != 0x7F:
            self.get_logger().warn('Invalid end byte')
            return None

        distance = (distance_bytes[1] << 8) | distance_bytes[0]
        return sensor_id[0], zone_id[0], distance

    def timer_callback(self):
        packet = self.read_data_packet()
        if packet:
            sensor_id, zone_id, distance = packet
            data_str = f"{{{sensor_id};{zone_id};{distance}}}"
            msg = String()
            msg.data = data_str

            if sensor_id == 1:
                self.publisher_top_.publish(msg)
            elif sensor_id == 2:
                self.publisher_bottom_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()