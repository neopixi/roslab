"""Waypoint controller that demonstrates steering by braking one front wheel."""

import math
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker

from .ekf import wrap_angle
from .ekf_slam_node import yaw_from_quaternion


class Phase(Enum):
    STRAIGHT = auto()
    DETOUR = auto()
    FINISHED = auto()


class MotionController(Node):
    def __init__(self) -> None:
        super().__init__('motion_controller')
        self.declare_parameter('cruise_speed', 0.62)
        self.declare_parameter('wheel_separation', 0.50)
        self.declare_parameter('wheel_radius', 0.11)
        self.declare_parameter('obstacle_trigger_distance', 1.25)
        self.declare_parameter('detour_offset', 1.25)
        self.declare_parameter('goal_x', 10.5)
        self.declare_parameter('max_angular_speed', 1.10)
        self.declare_parameter('heading_gain', 1.8)

        self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wheel_publisher = self.create_publisher(
            Float64MultiArray, '/controller/front_wheel_speed_reference', 10)
        self.state_publisher = self.create_publisher(String, '/controller/state', 10)
        self.path_publisher = self.create_publisher(Path, '/controller/odom_path', 10)
        self.waypoint_publisher = self.create_publisher(
            Marker, '/controller/waypoints', 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 20)
        self.create_subscription(
            LaserScan, '/scan', self._scan_callback, qos_profile_sensor_data)

        self.phase = Phase.STRAIGHT
        self.obstacle_handled = False
        self.pose = None
        self.front_distance = math.inf
        self.waypoints = []
        self.waypoint_index = 0
        self.raw_path = Path()
        self.raw_path.header.frame_id = 'odom'
        self.timer = self.create_timer(0.05, self._control)
        self.get_logger().info('Obstacle-avoidance controller ready')

    def _p(self, name):
        return self.get_parameter(name).value

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.pose = (position.x, position.y,
                     yaw_from_quaternion(message.pose.pose.orientation))
        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = 'odom'
        pose.pose = message.pose.pose
        self.raw_path.header.stamp = message.header.stamp
        self.raw_path.poses.append(pose)
        if len(self.raw_path.poses) > 4000:
            del self.raw_path.poses[:1000]
        self.path_publisher.publish(self.raw_path)

    def _scan_callback(self, scan: LaserScan) -> None:
        minimum = math.inf
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) < math.radians(24.0) and math.isfinite(distance):
                minimum = min(minimum, distance)
        self.front_distance = minimum

    def _begin_detour(self) -> None:
        x, y, _ = self.pose
        offset = self._p('detour_offset')
        # The first point is before the obstacle, the second is beside/after it,
        # and the third smoothly returns the cart to the original y=0 line.
        self.waypoints = [
            (x + 0.70, y + offset),
            (x + 2.15, y + offset),
            (x + 3.05, 0.0),
        ]
        self.waypoint_index = 0
        self.phase = Phase.DETOUR
        self.obstacle_handled = True
        self.get_logger().info('Obstacle detected: executing left-side detour')
        self._publish_waypoints()

    def _control(self) -> None:
        if self.pose is None:
            return
        x, y, yaw = self.pose
        if x >= self._p('goal_x'):
            self.phase = Phase.FINISHED

        if (self.phase == Phase.STRAIGHT
                and not self.obstacle_handled
                and self.front_distance < self._p('obstacle_trigger_distance')):
            self._begin_detour()

        if self.phase == Phase.FINISHED:
            self._publish_command(0.0, 0.0)
            self.state_publisher.publish(String(data='FINISHED'))
            return

        if self.phase == Phase.DETOUR:
            target = self.waypoints[self.waypoint_index]
            distance_to_target = math.hypot(target[0] - x, target[1] - y)
            if distance_to_target < 0.24:
                self.waypoint_index += 1
                if self.waypoint_index >= len(self.waypoints):
                    self.phase = Phase.STRAIGHT
                    self.get_logger().info('Detour complete: returning to straight line')
                    target = (x + 2.0, 0.0)
                else:
                    target = self.waypoints[self.waypoint_index]
            state_text = f'DETOUR_{min(self.waypoint_index + 1, 3)}'
        else:
            target = (x + 2.0, 0.0)
            state_text = 'STRAIGHT'

        desired_heading = math.atan2(target[1] - y, target[0] - x)
        heading_error = wrap_angle(desired_heading - yaw)
        speed = self._p('cruise_speed')
        speed *= max(0.40, 1.0 - 0.45 * abs(heading_error))
        angular = self._p('heading_gain') * heading_error
        angular = max(-self._p('max_angular_speed'),
                      min(self._p('max_angular_speed'), angular))

        # Never reverse a wheel: the inside front wheel is slowed/braked while
        # the outside wheel keeps pulling, exactly as specified for the cart.
        max_without_reverse = 1.80 * speed / self._p('wheel_separation')
        angular = max(-max_without_reverse, min(max_without_reverse, angular))

        # Last-resort clearance if the physical skid makes the turn wider.
        if self.front_distance < 0.38:
            speed = 0.12
            angular = min(self._p('max_angular_speed'),
                          1.80 * speed / self._p('wheel_separation'))
            state_text = 'EMERGENCY_BRAKE_LEFT'

        self._publish_command(speed, angular)
        self.state_publisher.publish(String(data=state_text))

    def _publish_command(self, linear: float, angular: float) -> None:
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.command_publisher.publish(command)

        half_track = 0.5 * self._p('wheel_separation')
        radius = self._p('wheel_radius')
        left = (linear - angular * half_track) / radius
        right = (linear + angular * half_track) / radius
        self.wheel_publisher.publish(Float64MultiArray(data=[left, right]))

    def _publish_waypoints(self) -> None:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'detour_waypoints'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.055
        marker.color.r = 0.15
        marker.color.g = 0.85
        marker.color.b = 0.20
        marker.color.a = 1.0
        if self.pose is not None:
            marker.points.append(Point(x=self.pose[0], y=self.pose[1], z=0.03))
        for x, y in self.waypoints:
            marker.points.append(Point(x=x, y=y, z=0.03))
        self.waypoint_publisher.publish(marker)

    def destroy_node(self):
        self._publish_command(0.0, 0.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
