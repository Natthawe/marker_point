import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import random
import time
import signal
import ast


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.get_logger().info("🔄 Initializing MarkerPublisher...")

        self.declare_parameter('points', rclpy.Parameter.Type.STRING_ARRAY)
        raw_points = self.get_parameter('points').get_parameter_value().string_array_value

        self.points = []
        for item in raw_points:
            try:
                d = ast.literal_eval(item)
                if all(k in d for k in ('label', 'x', 'y', 'z')):
                    self.points.append((d['label'], d['x'], d['y'], d['z']))
                else:
                    self.get_logger().warn(f"⚠️ Missing keys in: {d}")
            except Exception as e:
                self.get_logger().warn(f"❌ Failed to parse point: {item}, error: {e}")

        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
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
        marker.lifetime.sec = 0

        for (_, x, y, z) in self.points:
            pt = Point(x=x, y=y, z=z)
            marker.points.append(pt)
            marker.colors.append(self.random_color())

        self.publisher_.publish(marker)
        self.get_logger().info(f"✅ Published POINTS marker with {len(self.points)} points.")

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
            text_marker.lifetime.sec = 0
            self.publisher_.publish(text_marker)
            time.sleep(0.05)

    def clear_markers(self):
        self.get_logger().info("🧹 Clearing all markers...")
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

    def shutdown_handler(signum, frame):
        node.get_logger().info("🛑 SIGINT received. Cleaning up...")
        node.clear_markers()
        time.sleep(0.5)
        node.get_logger().info("✅ Shutdown complete.")
        node.running = False

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
