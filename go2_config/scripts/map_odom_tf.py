#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np
from tf_transformations import quaternion_from_matrix

class MapOdomTfNode(Node):
    def __init__(self):
        super().__init__('map_odom_tf_node')

        # static broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # 원하는 map->base 초기 pose
        self.map_base_translation = np.array([0.0, 0.0, 0.0])
        self.map_base_rotation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion

        # odom 메시지를 단 1번만 받아서 map->odom을 계산
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback_once,
            10
        )
        self.got_initial_odom = False

    def odom_callback_once(self, msg: Odometry):
        if self.got_initial_odom:
            return

        self.get_logger().info("Received initial odom. Publishing static map->odom transform.")
        self.got_initial_odom = True

        # odom -> base_footprint transform
        t = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # convert odom->base to matrix
        T_odom_base = np.eye(4)
        T_odom_base[:3, :3] = self.quat_to_rotmat([q.x, q.y, q.z, q.w])
        T_odom_base[0, 3] = t.x
        T_odom_base[1, 3] = t.y
        T_odom_base[2, 3] = t.z

        # map -> base
        T_map_base = np.eye(4)
        T_map_base[:3, :3] = self.quat_to_rotmat(self.map_base_rotation)
        T_map_base[:3, 3] = self.map_base_translation

        # map -> odom = map->base * inv(odom->base)
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        # publish static transform
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = T_map_odom[0, 3]
        tf_msg.transform.translation.y = T_map_odom[1, 3]
        tf_msg.transform.translation.z = T_map_odom[2, 3]
        q = quaternion_from_matrix(T_map_odom)
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(tf_msg)

    def quat_to_rotmat(self, q):
        # converts quaternion to rotation matrix
        from tf_transformations import quaternion_matrix
        return quaternion_matrix(q)[:3, :3]


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
