import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import random


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        # self.timer = self.create_timer(1.0, self.publish_markers)
        self.points = [
            (6.442733285277182, -6.459006281775705, 0.5),  # HOMEC
            (6.244635627674585, -0.470162914459487, 0.5),  # SA1
            (-1.199011605480747, 1.092156605996984, 0.5),  # MB2
            (-1.2184909630182754, 9.985049021812763, 0.5),  # SIP
            (3.278832722073614, -6.7252372642518665, 0.5),  # KITT
            (-0.8565749446912965, -7.8072362559664885, 0.5)  # BL1
        ]

        self.publish_markers()

    def random_color(self):
        return ColorRGBA(
            r=random.uniform(0.0, 1.0),
            g=random.uniform(0.0, 1.0),
            b=random.uniform(0.0, 1.0),
            a=1.0
        )

    def publish_markers(self):
        # Create POINTS marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.lifetime.sec = 0

        for (x, y, z) in self.points:
            pt = Point(x=x, y=y, z=z)
            marker.points.append(pt)
            marker.colors.append(self.random_color())

        self.publisher_.publish(marker)
        self.get_logger().info("Published POINTS marker with %d points." % len(self.points))

        # Also publish text markers
        for i, (x, y, z) in enumerate(self.points):
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "text"
            text_marker.id = i - 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + 0.4
            text_marker.scale.z = 0.3
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"({x:.2f}, {y:.2f}, {z:.2f})"
            text_marker.lifetime.sec = 0
            self.publisher_.publish(text_marker)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin_once(node)  # publish once only
    # rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
