import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class EMAFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.ema = None

    def update(self, measurement):
        if self.ema is None:
            self.ema = measurement
        else:
            self.ema = self.alpha * measurement + (1 - self.alpha) * self.ema
        return self.ema

class FilterNode(Node):

    def __init__(self):
        super().__init__('filter_node')
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
        self.publisher_top_filtered = self.create_publisher(String, '/sensor_data_top_filtered', 10)
        self.publisher_bottom_filtered = self.create_publisher(String, '/sensor_data_bottom_filtered', 10)

        # EMA Filter instances for top and bottom sensors
        self.ema_filter_top = EMAFilter(alpha=0.1)
        self.ema_filter_bottom = EMAFilter(alpha=0.1)

    def listener_callback_top(self, msg):
        filtered_data = self.apply_ema_filter(msg.data, self.ema_filter_top)
        filtered_msg = String()
        filtered_msg.data = filtered_data
        self.publisher_top_filtered.publish(filtered_msg)

    def listener_callback_bottom(self, msg):
        filtered_data = self.apply_ema_filter(msg.data, self.ema_filter_bottom)
        filtered_msg = String()
        filtered_msg.data = filtered_data
        self.publisher_bottom_filtered.publish(filtered_msg)

    def apply_ema_filter(self, data, ema_filter):
        parts = data.strip('{}').split(';')
        sensor_id = int(parts[0])
        zone_id = int(parts[1])
        distance = int(parts[2])

        # Apply the EMA filter to the distance measurement
        filtered_distance = ema_filter.update(distance)

        return f"{{{sensor_id};{zone_id};{int(filtered_distance)}}}"

def main(args=None):
    rclpy.init(args=args)
    filter_node = FilterNode()
    rclpy.spin(filter_node)
    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
