import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import open3d as o3d
import numpy as np

class PointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('point_cloud_visualizer')
        self.subscription = self.create_subscription(
            PointCloud2,
            'point_cloud',
            self.listener_callback,
            10
        )
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window('Point Cloud Viewer')
        self.point_cloud = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.point_cloud)
        self.is_initialized = False

    def listener_callback(self, msg):
        points = []
        for point in point_cloud2.read_points(msg, skip_nans=True):
            points.append([point[0], point[1], point[2]])

        if not points:
            return

        points = np.array(points, dtype=np.float32)

        self.point_cloud.points = o3d.utility.Vector3dVector(points)

        if not self.is_initialized:
            self.vis.update_geometry(self.point_cloud)
            self.vis.poll_events()
            self.vis.update_renderer()
            self.is_initialized = True
        else:
            self.vis.update_geometry(self.point_cloud)
            self.vis.poll_events()
            self.vis.update_renderer()

def main(args=None):
    rclpy.init(args=args)
    point_cloud_visualizer = PointCloudVisualizer()
    
    def timer_callback():
        if point_cloud_visualizer.is_initialized:
            point_cloud_visualizer.vis.poll_events()
            point_cloud_visualizer.vis.update_renderer()
    
    timer = point_cloud_visualizer.create_timer(0.1, timer_callback)
    
    rclpy.spin(point_cloud_visualizer)
    point_cloud_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
