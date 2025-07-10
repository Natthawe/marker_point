import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import random
import time
import signal
import threading


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.points = [
            ("HOME", 38.111368218364824, -6.459006281775705, 0.0),
            ("FGOUT", 36.72657869014043, -12.446353218156073, 0.0),
            ("BOXIN", 36.87938224733446, -11.054809307319665, 0.0),
            ("AS251", 5.358836202085763, -0.6503615376793641, 0.0),
            ("AS131", 29.804792224173816, 1.5814516670059395, 0.0),
            ("AS271", 23.256697464203565, 2.8354399854738364, 0.0),
            ("AS270", 17.489443736435778, 3.8388743783300785, 0.0),
            ("AS273", 11.87705367592784, 4.848301229685419, 0.0),
            ("AS254", 12.219800378765989, 4.63341934730527, 0.0),
            ("AS253", 6.3538274007337945, 5.648351148750713, 0.0),
            ("AS272", 5.699434726954529, 5.933248414235129, 0.0),
            ("AS252", 11.0358211669264, -1.684823307306622, 0.0),
            ("TEST", 0.0, 0.0, 0.0),
        ]
        self.running = True
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
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.lifetime.sec = 0             # 0 for no lifetime , > 0 for lifetime

        for (_, x, y, z) in self.points:
            pt = Point(x=x, y=y, z=z)
            marker.points.append(pt)
            marker.colors.append(self.random_color())

        self.publisher_.publish(marker)
        self.get_logger().info("Published POINTS marker with %d points." % len(self.points))

        # TEXT markers
        for i, (label, x, y, z) in enumerate(self.points):
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = f"text_{label}"
            text_marker.id = 100 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + 1.5
            text_marker.scale.z = 0.5
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"{label},{x:.2f},{y:.2f}"
            # text_marker.text = label
            text_marker.lifetime.sec = 0
            self.publisher_.publish(text_marker)
            time.sleep(0.05)

    def clear_markers(self):
        # CLEAR POINTS AND TEXT MARKERS
        self.get_logger().info("Clearing all markers...")

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "points"
        marker.id = 0
        marker.action = Marker.DELETE
        self.publisher_.publish(marker)

        for i, (label, _, _, _) in enumerate(self.points):
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = f"text_{label}"
            text_marker.id = 100 + i
            text_marker.action = Marker.DELETE
            self.publisher_.publish(text_marker)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()

    # Ctrl+C
    def shutdown_handler(signum, frame):
        node.get_logger().info("SIGINT received. Clearing markers...")
        node.clear_markers()
        time.sleep(0.5)  # remove markers
        node.get_logger().info("Shutting down marker publisher...")
        node.running = False

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
