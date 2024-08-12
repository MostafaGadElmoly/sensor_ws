import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np

class PointCloudPublisher(Node):

    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.subscription_top = self.create_subscription(
            String,
            '/sensor_data_top',
            self.listener_callback_top,
            10)
        self.subscription_bottom = self.create_subscription(
            String,
            '/sensor_data_bottom',
            self.listener_callback_bottom,
            10)
        self.publisher_top = self.create_publisher(PointCloud2, '/point_cloud_top', 100)
        self.publisher_bottom = self.create_publisher(PointCloud2, '/point_cloud_bottom', 100)
        self.distances_top = np.zeros((8, 8), dtype=np.float32)
        self.distances_bottom = np.zeros((8, 8), dtype=np.float32)

    def listener_callback_top(self, msg):
        self.process_data(msg.data, 'top')

    def listener_callback_bottom(self, msg):
        self.process_data(msg.data, 'bottom')

    def process_data(self, data_str, sensor):
        data = data_str.strip('{}').split(';')
        sensor_id = int(data[0])
        zone_id = int(data[1])
        distance = float(data[2])
        
        row = zone_id // 8
        col = zone_id % 8
        
        if sensor == 'top':
            self.distances_top[row, col] = distance
            self.publish_point_cloud('top')
        elif sensor == 'bottom':
            self.distances_bottom[row, col] = distance
            self.publish_point_cloud('bottom')

    def publish_point_cloud(self, sensor):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'sensor_frame'

        points = []
        fov = 60.0  
        scaling_factor = 1000.0  #diviaded 

        if sensor == 'top':
            distances = self.distances_top
            publisher = self.publisher_top
        else:
            distances = self.distances_bottom
            publisher = self.publisher_bottom

        width, height = distances.shape
        focal_length_x = width / (2 * np.tan(np.radians(fov / 2)))
        focal_length_y = height / (2 * np.tan(np.radians(fov / 2)))

        for v in range(height):
            for u in range(width):
                z = distances[v, u] / scaling_factor
                if z <= 0:
                    continue
                x = (u - width / 2) * z / focal_length_x
                y = (v - height / 2) * z / focal_length_y
                points.append([x, y, z])

        point_cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        publisher.publish(point_cloud_msg)
        self.get_logger().info(f'Published point cloud for {sensor} sensor')

def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PointCloudPublisher()
    rclpy.spin(point_cloud_publisher)
    point_cloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
