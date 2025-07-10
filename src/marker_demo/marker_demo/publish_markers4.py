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

        # # Cal_comp
        # self.points = [
        #     ("HOMEC", 6.342733285277182, -6.459006281775705, 0.0),
        #     ("SA1", 6.244635627674585, -0.470162914459487, 0.0),
        #     ("MB2", -1.2184909630182754, 1.092156605996984, 0.0),
        #     ("SIP", -1.2184909630182754, 9.985049021812763, 0.0),
        #     ("BL1", -1.2184909630182754, -7.8072362559664885, 0.0),
        #     ("KITT", 3.278832722073614, -6.459006281775705, 0.0),
        # ]

        # AiSin
        self.points = [
            ("HOME", -0.16532379437427291, 0.0511149246529164, 0.0),
            ("FG", 38.279809624020245, -5.179790868034932, 0.0),
            ("EMPTY", 38.279809624020245, -3.5919566725656797, 0.0),
            ("AS272", 4.678047900828085, 7.5384415030074425, 0.0),
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
        # POINTS marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.lifetime.sec = 0  # 0 for no lifetime , > 0 for lifetime

        for (name, x, y, z) in self.points:
            pt = Point(x=x, y=y, z=z)
            marker.points.append(pt)
            marker.colors.append(self.random_color())

        self.publisher_.publish(marker)
        self.get_logger().info("Published POINTS marker with %d points." % len(self.points))

        # TEXT markers
        for i, (name, x, y, z) in enumerate(self.points):
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "text"
            text_marker.id = i + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + 1.0
            text_marker.scale.z = 0.6
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            # text_marker.text = f"{name}, {x:.2f}, {y:.2f}"
            text_marker.text = f"{name}"
            text_marker.lifetime.sec = 0  # 0 for no lifetime , > 0 for lifetime
            self.publisher_.publish(text_marker)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin_once(node)  # publish once only
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
