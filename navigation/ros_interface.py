"""
ROS interface stub for the Navigator.

This module provides a ROS2 node wrapper around the Navigator class.
It gracefully degrades if ROS2 is not installed.
"""

from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class NavigatorNode:
    """
    ROS2 node that integrates the Navigator with ROS topics.

    If ROS2 (rclpy) is available, subscribes to /robot_pose and
    /delivery_goal, and publishes velocity commands on /cmd_vel.
    If ROS2 is not available, operates in stub mode with warnings.
    """

    def __init__(self, navigator):
        """
        Initialize the NavigatorNode.

        Args:
            navigator: Navigator instance
        """
        self.navigator = navigator
        self.ros_available = False
        self.current_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw
        self.current_goal: Optional[Tuple[float, float]] = None

        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped, Twist
            self.rclpy = rclpy
            self.PoseStamped = PoseStamped
            self.Twist = Twist
            self.ros_available = True
        except ImportError:
            print("[NavigatorNode] ROS2 not available — running in stub mode")
            return

        # Initialize ROS node
        rclpy.init()
        self.node = rclpy.create_node('navigator_node')
        self.node.get_logger().info('NavigatorNode initialized')

        # Subscribers
        self.pose_sub = self.node.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )
        self.goal_sub = self.node.create_subscription(
            PoseStamped,
            '/delivery_goal',
            self.goal_callback,
            10
        )

        # Publisher
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        print("[NavigatorNode] ROS subscribers/publisher initialized")

    def pose_callback(self, msg: 'PoseStamped') -> None:
        """
        Handle incoming robot pose messages.

        Args:
            msg: PoseStamped message
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        # Extract yaw from quaternion (w, x, y, z)
        q = msg.pose.orientation
        yaw = self._quaternion_to_yaw(q.w, q.x, q.y, q.z)
        self.current_pose = (x, y, yaw)
        logger.debug(f"Pose received: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        self._step_and_publish()

    def goal_callback(self, msg: 'PoseStamped') -> None:
        """
        Handle incoming delivery goal messages.

        Args:
            msg: PoseStamped message
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.current_goal = (x, y)
        logger.info(f"New goal received: ({x:.2f}, {y:.2f})")

        if self.current_pose is not None:
            # Reset navigator with new goal
            self.navigator.reset(
                self.current_pose[:2],
                self.current_goal
            )
            print(f"[NavigatorNode] Replanned for new goal")

    def publish_cmd_vel(self, vx: float, vy: float, yaw_rate: float) -> None:
        """
        Publish velocity command to /cmd_vel.

        Args:
            vx: Linear velocity x (m/s)
            vy: Linear velocity y (m/s)
            yaw_rate: Angular velocity (rad/s)
        """
        if not self.ros_available:
            return

        twist = self.Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = yaw_rate
        self.cmd_vel_pub.publish(twist)

    def _step_and_publish(self) -> None:
        """Execute one navigation step and publish velocity."""
        if self.current_pose is None or self.current_goal is None:
            return

        x, y, yaw = self.current_pose
        vx, vy, yaw_rate, done = self.navigator.step(x, y, yaw)
        self.publish_cmd_vel(vx, vy, yaw_rate)

        if done:
            print("[NavigatorNode] Goal reached!")

    def spin(self) -> None:
        """
        Spin the ROS node.

        If ROS2 is available, blocks and processes callbacks.
        Otherwise, prints warning.
        """
        if not self.ros_available:
            print("[NavigatorNode] ROS not available — use Navigator directly in sim")
            return

        try:
            self.rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            self.rclpy.shutdown()

    @staticmethod
    def _quaternion_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
        """
        Convert quaternion to yaw angle (psi) around Z axis.

        Args:
            qw, qx, qy, qz: Quaternion components
        Returns:
            Yaw angle in radians, normalized to [-pi, pi]
        """
        import math
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # Normalize to [-pi, pi]
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw
