import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np

class PointCloudNode(Node):
    def __init__(self):
        super().__init__('point_cloud_node')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)

    def listener_callback(self, msg):
        data = msg.data.split()
        distances = np.array([float(distance) for distance in data])
        distances = distances.reshape((8, 8))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'sensor_frame'

        points = []
        fov = 60.0  # Field of view is 60 degrees
        width, height = distances.shape
        scaling_factor = 100.0  # Example scaling factor, adjust as needed
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
        self.publisher.publish(point_cloud_msg)
        self.get_logger().info('Published point cloud')

def main(args=None):
    rclpy.init(args=args)
    point_cloud_node = PointCloudNode()
    rclpy.spin(point_cloud_node)
    point_cloud_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
