import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)

        # จุดและสีแต่ละจุด
        self.points = [
            (6.491631984710693, -6.295205116271973),
            (8.899580955505371, -0.8954309821128845)
        ]
        self.colors = [
            (0.0, 0.0, 1.0),  # Blue
            (0.0, 0.5, 0.5)   # Green
        ]

        self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "colored_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2

        for i, (x, y) in enumerate(self.points):
            pt = Point(x=x, y=y, z=0.0)
            marker.points.append(pt)

            color = ColorRGBA()
            color.r = self.colors[i][0]
            color.g = self.colors[i][1]
            color.b = self.colors[i][2]
            color.a = 1.0
            marker.colors.append(color)

            self.get_logger().info(f'Added point ({x}, {y}) with color {self.colors[i]}')

        self.publisher_.publish(marker)
        self.get_logger().info('Published colored markers.')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
