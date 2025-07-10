import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        # self.timer = self.create_timer(1.0, self.publish_marker)  # every 1 second
        self.points = [
            (6.442733285277182, -6.459006281775705),
            (6.244635627674585, -0.470162914459487)
        ]
        self.frame_id = "map"

        # Publish marker once
        self.publish_marker()

    def publish_marker(self):
        """Publish a marker with the given points."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Appearance
        marker.scale.x = 0.2  # point width
        marker.scale.y = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # alpha (transparency)

        # Add points
        for x, y in self.points:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = 0.0
            marker.points.append(pt)

        self.publisher_.publish(marker)
        self.get_logger().info('Published Marker with %d points' % len(self.points))


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
