"""ROS 2 adapter for the pure EKF-SLAM implementation."""

import math
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .ekf import EkfSlam, wrap_angle
from .landmark_detector import detect_circular_landmarks


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def set_yaw(quaternion, yaw: float) -> None:
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(0.5 * yaw)
    quaternion.w = math.cos(0.5 * yaw)


class EkfSlamNode(Node):
    def __init__(self) -> None:
        super().__init__('ekf_slam')
        self.declare_parameter('sensor_x_offset', 0.20)
        self.declare_parameter('landmark_radius_min', 0.07)
        self.declare_parameter('landmark_radius_max', 0.18)
        self.declare_parameter('circle_residual_max', 0.018)
        self.declare_parameter('range_std', 0.045)
        self.declare_parameter('bearing_std', 0.025)
        self.declare_parameter('association_gate', 5.991)
        self.declare_parameter('maximum_landmarks', 20)
        self.declare_parameter('odometry_distance_std', 0.015)
        self.declare_parameter('odometry_yaw_std', 0.012)
        self.declare_parameter('encoder_distance_scale', 1.015)
        self.declare_parameter('encoder_yaw_scale', 1.025)

        self.filter = EkfSlam(
            range_std=self._p('range_std'),
            bearing_std=self._p('bearing_std'),
            gate=self._p('association_gate'),
            max_landmarks=int(self._p('maximum_landmarks')),
        )
        self.previous_odom = None
        self.latest_raw_pose = None
        self.scan_counter = 0
        self.path = Path()
        self.path.header.frame_id = 'map'

        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/slam/pose', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/slam/odom', 10)
        self.path_publisher = self.create_publisher(Path, '/slam/path', 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/slam/landmarks', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, '/odom', self._odom_callback, 30)
        self.create_subscription(
            LaserScan, '/scan', self._scan_callback, qos_profile_sensor_data)
        self.get_logger().info(
            'Classical EKF-SLAM ready: odometry prediction + lidar landmarks')

    def _p(self, name):
        return self.get_parameter(name).value

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        raw_pose = np.array([
            position.x,
            position.y,
            yaw_from_quaternion(message.pose.pose.orientation),
        ])
        if self.previous_odom is not None:
            delta_world = raw_pose[:2] - self.previous_odom[:2]
            previous_yaw = self.previous_odom[2]
            forward = (
                math.cos(previous_yaw) * delta_world[0]
                + math.sin(previous_yaw) * delta_world[1]
            )
            lateral = (
                -math.sin(previous_yaw) * delta_world[0]
                + math.cos(previous_yaw) * delta_world[1]
            )
            delta_yaw = wrap_angle(raw_pose[2] - previous_yaw)
            if abs(forward) < 0.5 and abs(delta_yaw) < 0.5:
                self.filter.predict(
                    forward * self._p('encoder_distance_scale'),
                    delta_yaw * self._p('encoder_yaw_scale'),
                    self._p('odometry_distance_std'),
                    self._p('odometry_yaw_std'),
                    lateral_distance=(
                        lateral * self._p('encoder_distance_scale')),
                )
        self.previous_odom = raw_pose
        self.latest_raw_pose = raw_pose
        self._publish_estimate(message.header.stamp)

    def _scan_callback(self, message: LaserScan) -> None:
        observations = detect_circular_landmarks(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
            sensor_x_offset=self._p('sensor_x_offset'),
            radius_min=self._p('landmark_radius_min'),
            radius_max=self._p('landmark_radius_max'),
            residual_max=self._p('circle_residual_max'),
        )
        matched, added = self.filter.observe(observations)
        self.scan_counter += 1
        if added or self.scan_counter % 75 == 0:
            self.get_logger().info(
                f'landmarks={self.filter.landmark_count}, '
                f'detections={len(observations)}, matched={matched}, new={added}')
        self._publish_markers(message.header.stamp)

    def _publish_estimate(self, stamp) -> None:
        pose_message = PoseWithCovarianceStamped()
        pose_message.header.stamp = stamp
        pose_message.header.frame_id = 'map'
        pose_message.pose.pose.position.x = float(self.filter.state[0])
        pose_message.pose.pose.position.y = float(self.filter.state[1])
        set_yaw(pose_message.pose.pose.orientation, float(self.filter.state[2]))
        covariance = np.zeros((6, 6))
        covariance[0, 0] = self.filter.covariance[0, 0]
        covariance[0, 1] = self.filter.covariance[0, 1]
        covariance[1, 0] = self.filter.covariance[1, 0]
        covariance[1, 1] = self.filter.covariance[1, 1]
        covariance[0, 5] = self.filter.covariance[0, 2]
        covariance[1, 5] = self.filter.covariance[1, 2]
        covariance[5, 0] = self.filter.covariance[2, 0]
        covariance[5, 1] = self.filter.covariance[2, 1]
        covariance[5, 5] = self.filter.covariance[2, 2]
        covariance[2, 2] = covariance[3, 3] = covariance[4, 4] = 1.0e3
        pose_message.pose.covariance = covariance.reshape(-1).tolist()
        self.pose_publisher.publish(pose_message)

        slam_odom = Odometry()
        slam_odom.header = pose_message.header
        slam_odom.child_frame_id = 'slam_base_footprint'
        slam_odom.pose = pose_message.pose
        self.odom_publisher.publish(slam_odom)

        path_pose = PoseStamped()
        path_pose.header = pose_message.header
        path_pose.pose = pose_message.pose.pose
        self.path.header.stamp = stamp
        self.path.poses.append(path_pose)
        if len(self.path.poses) > 4000:
            del self.path.poses[:1000]
        self.path_publisher.publish(self.path)
        self._publish_transforms(stamp)

    def _publish_transforms(self, stamp) -> None:
        if self.latest_raw_pose is None:
            return
        raw_x, raw_y, raw_yaw = self.latest_raw_pose
        estimate_x, estimate_y, estimate_yaw = self.filter.state[:3]
        correction_yaw = wrap_angle(estimate_yaw - raw_yaw)
        correction_x = estimate_x - (
            math.cos(correction_yaw) * raw_x
            - math.sin(correction_yaw) * raw_y)
        correction_y = estimate_y - (
            math.sin(correction_yaw) * raw_x
            + math.cos(correction_yaw) * raw_y)

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = float(correction_x)
        map_to_odom.transform.translation.y = float(correction_y)
        set_yaw(map_to_odom.transform.rotation, float(correction_yaw))

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_footprint'
        odom_to_base.transform.translation.x = float(raw_x)
        odom_to_base.transform.translation.y = float(raw_y)
        set_yaw(odom_to_base.transform.rotation, float(raw_yaw))
        self.tf_broadcaster.sendTransform([map_to_odom, odom_to_base])

    def _publish_markers(self, stamp) -> None:
        markers: List[Marker] = []
        for index in range(self.filter.landmark_count):
            offset = 3 + 2 * index
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'ekf_landmarks'
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(self.filter.state[offset])
            marker.pose.position.y = float(self.filter.state[offset + 1])
            marker.pose.position.z = 0.28
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.24
            marker.scale.z = 0.56
            marker.color.r = 0.95
            marker.color.g = 0.25
            marker.color.b = 0.10
            marker.color.a = 0.85
            markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = 'ekf_landmark_labels'
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = marker.pose.position.x
            label.pose.position.y = marker.pose.position.y
            label.pose.position.z = 0.72
            label.pose.orientation.w = 1.0
            label.scale.z = 0.20
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'L{index + 1}'
            markers.append(label)
        self.marker_publisher.publish(MarkerArray(markers=markers))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
