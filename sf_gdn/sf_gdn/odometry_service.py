import rclpy
from rclpy.node import Node
from sf_gdn_interfaces.srv import OdometryFactor


class OdometryService(Node):

    def __init__(self):
        super().__init__('odometry_service')
        self.get_logger().info("Odometry service is ready")
        self.srv = self.create_service(
            OdometryFactor, 'odometry_factor', self.odometry_callback)
        self.i = 0
        self.distances = [1.7, 1.4]

    def odometry_callback(self, request, response):
        response.distance = self.distances[self.i]
        self.get_logger().info(
            f'Incoming odometry request, responding with {response.distance}')
        self.i = self.i + 1
        return response


def main():
    rclpy.init()
    odometry_service = OdometryService()
    rclpy.spin(odometry_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
