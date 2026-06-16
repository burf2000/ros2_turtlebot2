#!/usr/bin/env python3
"""odom_tf_bridge — publish odom->base_footprint TF from the /odom topic.

The Kobuki node publishes the /odom topic but does NOT broadcast the
odom->base_footprint transform (even with publish_tf:true), so slam_toolbox
drops every scan ("Message Filter dropping ... queue is full"). This node
bridges /odom into TF. The transform is stamped with the current node clock
(now()) — NOT the odom message time — otherwise slam still drops scans on a
timing mismatch between the depth-scan stamps and the odom stamps.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBridge(Node):
    def __init__(self):
        super().__init__('odom_tf_bridge')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.cb, 30)
        self.get_logger().info('odom_tf_bridge up: /odom -> TF odom->base_footprint (now-stamped)')

    def cb(self, m):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = m.pose.pose.position.x
        t.transform.translation.y = m.pose.pose.position.y
        t.transform.translation.z = 0.0
        q = m.pose.pose.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            q.w = 1.0
        t.transform.rotation = q
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
