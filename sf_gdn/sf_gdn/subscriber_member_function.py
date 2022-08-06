import rclpy
from rclpy.node import Node

from symforce import symbolic as sf
from symforce.notebook_util import display
from symforce import typing as T


def bearing_residual(
    pose: sf.Pose2, landmark: sf.V2, angle: sf.Scalar, epsilon: sf.Scalar
) -> sf.V1:
    """Residual from relative bearing measurement of 2D pose to a landmark."""
    t_body = pose.inverse() * landmark
    predicted_angle = sf.atan2(t_body[1], t_body[0], epsilon=epsilon)
    return sf.V1(sf.wrap_angle(predicted_angle - angle))


def odometry_residual(
    pose_a: sf.Pose2, pose_b: sf.Pose2, dist: sf.Scalar, epsilon: sf.Scalar
) -> sf.V1:
    """Residual from the scalar distance between two poses."""
    return sf.V1((pose_b.t - pose_a.t).norm(epsilon=epsilon) - dist)


class FactorSubscriber(Node):
    """Class for subscribing to factors."""

    def __init__(self):
        """Initialize the Node."""
        super().__init__("factor_subscriber")
        self.subscription = self.create_subscription(
            odometry_factor_msg,
            'odometry_factor',
            self.odometry_factor_callback)
        self.subscription  # just to prevent unused variable warning

    def odometry_factor_callback(self, msg):
        """Process incoming odometry factors.

        The odometry msgs have 2 poses and a distance between them.
        """
        self.get_logger().info("I heard something")


def main(args=None):
    """Start the program."""
    rclpy.init(args=args)

    factor_subscriber = FactorSubscriber()

    rclpy.spin(factor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    factor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
