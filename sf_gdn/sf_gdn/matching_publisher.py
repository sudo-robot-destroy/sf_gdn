import rclpy
from rclpy.node import Node
import numpy as np
from sf_gdn_interfaces.msg import LandmarkMatch3D
from geometry_msgs.msg import Point


class MatchPublisher(Node):
    def __init__(self):
        """relative translation mesurement of a 3D pose to a landmark."""
        super().__init__("matching_publisher")
        self.publisher_ = self.create_publisher(LandmarkMatch3D,
                                                "matching_factor",
                                                10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        # TODO: construct the matches here  

    def timer_callback(self):
        # TODO: publish the matches in the list
        pass

def main(args=None):
    rclpy.init(args=args)

    match_publisher = MatchPublisher()

    rclpy.spin(match_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    match_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()