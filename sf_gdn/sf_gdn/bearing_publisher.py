import rclpy
from rclpy.node import Node
import numpy as np
from sf_gdn_interfaces.msg import LandmarkBearings
from geometry_msgs.msg import Point


class BearingPublisher(Node):
    def __init__(self):
        """Publish bearing measures to each landmark at current pose"""
        super().__init__("bearing_publisher")
        self.publisher_ = self.create_publisher(LandmarkBearings,
                                                "bearing_factor",
                                                10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.landmark_seq = [0, 1, 2]
        self.landmarks = [
            Point(x=-2.0, y=2.0, z=0.0),
            Point(x=1.0, y=-3.0, z=0.0),
            Point(x=5.0, y=2.0, z=0.0),
        ]
        self.angles = np.deg2rad(
            [[55, 245, -35], [95, 220, -20], [125, 220, -20]]
        ).tolist()

    def timer_callback(self):
        msg = LandmarkBearings()
        msg.landmark_seq = self.landmark_seq
        msg.landmarks = self.landmarks
        msg.angles = self.angles[self.i]
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing bearing")
        self.i = self.i + 1


def main(args=None):
    rclpy.init(args=args)

    bearing_publisher = BearingPublisher()

    rclpy.spin(bearing_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bearing_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
