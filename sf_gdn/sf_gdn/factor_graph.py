import rclpy
from rclpy.node import Node
from symforce import symbolic as sf
from sf_gdn_interfaces.msg import LandmarkBearings
from sf_gdn_interfaces.srv import OdometryFactor
from symforce.opt.factor import Factor
from symforce.values import Values
from symforce.opt.optimizer import Optimizer


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
    """Class for subscribing to factors and running pose optimizations.

    The goal is for this to be able to subscribe to all kinds of factor
    publishers and collect all that information into a factor graph, then,
    when triggered perform a optimization on the graph to produce pose
    estimations.

    Triggers could be several different types, time based (generate poses every
    5 seconds), sensor based (a visual odometry publisher could trigger every
    key frame), or distanced based.

    Some producers of factors aren't publishers, but rather need to be told
    when to create a factor, such as an odometry factor. These are services
    that need to be called to provide info, usually between two poses.

    """

    def __init__(self):
        """Initialize the Node."""
        super().__init__("factor_subscriber")
        # Create bearing factor subscriber.
        # TODO: the node name should be a ROS parameter.
        self.subscription = self.create_subscription(
            LandmarkBearings,
            "bearing_factor",
            self.bearing_factor_callback,
            10
        )
        self.subscription  # just to prevent unused variable warning

        # Set up client for the odometry factors which are in-between factors.
        self.odometry_client = self.create_client(
            OdometryFactor, 'odometry_factor')
        self.odometry_client_futures = []  # Used for some fancy threading
        while not self.odometry_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # How to trigger optimizations
        self.optimization_trigger = 3
        self.optimize_now = False

        # Some items to keep track of program flow and collect information
        self.factors = []
        self.landmark_seqs = []
        self.cur_pose = 0

        # The intial_values (Values) is going to contain poses, landmarks,
        # angles and distances. It is constructed before optimization.
        # The values are collected during normal operaiton and add to a
        # Values object right before optimization
        self.poses = []
        self.landmarks = []  # From bearing factors
        self.angles = []  # From bearing factors
        self.distances = []  # From odometry factors

    def bearing_factor_callback(self, msg: LandmarkBearings):
        """Process incoming bearing factors.

        The bearing msgs has landmarks and the bearing angle to them.
        Receiving a bearing factor creates a new pose in the factor graph.
        """
        self.get_logger().info(
            "Received bearing factor containing"
            f" {len(msg.landmark_seq)} landmarks"
        )

        # if this isn't the first pose we need an odometry factor btw poses
        if self.cur_pose != 0:
            self.request_odometry()

        # Each new bearing factor represents a new pose for the vehicle.
        self.poses.append(sf.Pose2.identity())

        # Process all the landmarks in the message
        for i in range(len(msg.landmark_seq)):
            # Don't add a landmark to the list if it's already in there
            if msg.landmark_seq[i] not in self.landmark_seqs:
                self.landmark_seqs.append(msg.landmark_seq[i])
                lm = sf.V2(msg.landmarks[i].x, msg.landmarks[i].y)
                self.landmarks.append(lm)

            # Add the bearing measurements for each landmark to this pose.
            self.angles.append(msg.angles[i])
            self.get_logger().info(
                f"Adding bearing factor for pose {self.cur_pose}, "
                f"landmark {self.landmark_seqs[i]}, "
                f"bearing angle = {self.angles[len(self.angles)-1]}"
            )
            # Just collecting the data (i.e. the appends above) is not enough
            # We have to tell the optimizer how all the data is connected in
            # the graph, so we create these factors: The keys needs to match
            # what is in the Values container
            self.factors.append(
                Factor(
                    residual=bearing_residual,
                    keys=[
                        f"poses[{self.cur_pose}]",
                        f"landmarks[{self.landmark_seqs[i]}]",
                        f"angles[{len(self.angles)-1}]",
                        "epsilon",
                    ],
                )
            )
        self.cur_pose = self.cur_pose + 1
        if self.cur_pose >= self.optimization_trigger:
            self.optimize_now = True

    def request_odometry(self):
        """Call the odometry service
        # we have to handle this weirdly since this gets called from within a
        # callback. Calling a service usually blocks a thread and it relys on
        # an external process so make sure we don't have to wait on it. See
        # link on nested services.
        # https://github.com/codebot/ros2_patterns/blob/master/nested_services/sub_calling_service.py
        """
        self.add_odometry_factor()
        self.odometry_client_futures.append(
            self.odometry_client.call_async(OdometryFactor.Request()))

    def add_odometry_factor(self):
        # The result from the odometry request ends up in spin and we don't
        # want to wait on that, so just make the factor here and let the
        # spin add the actual numbers to the self.distances list. Since we're
        # using cur_pose this needs to be quick, otherwise that value could
        # change by the time we use it.

        # TODO: The odometry service request might actually need the poses
        # (treating them as timestamps) to keep track of where they belong in
        # the graph.
        self.get_logger().info(
            f"adding odometry factor between poses"
            f" [{self.cur_pose - 1} and {self.cur_pose}]")
        self.factors.append(
            Factor(
                residual=odometry_residual,
                keys=[f"poses[{self.cur_pose - 1}]",
                      f"poses[{self.cur_pose}]",
                      f"distances[{self.cur_pose - 1}]",
                      "epsilon"],
            )
        )

    def optimize(self):
        """Construct the optimization problem using the factors"""
        initial_values = Values(
            poses=self.poses,
            landmarks=self.landmarks,
            distances=self.distances,
            angles=self.angles,
            epsilon=sf.numeric_epsilon,
        )
        # Select the keys to optimize - the rest will be held constant
        optimized_keys = [f"poses[{i}]" for i in range(self.cur_pose)]

        # Create the optimizer
        optimizer = Optimizer(
            factors=self.factors,
            optimized_keys=optimized_keys,
            debug_stats=True,  # Return problem stats for every iteration
            # Customize optimizer behavior
            params=Optimizer.Params(verbose=True),
        )

        # Solve and return the result
        result = optimizer.optimize(initial_values)
        for i, pose in enumerate(result.optimized_values["poses"]):
            self.get_logger().info(
                f"Pose {i}: t = {pose.position()}, "
                f"heading = {pose.rotation().to_tangent()[0]}")

    def spin(self):
        """Since we do some nest threading we need to have a custom spin
        function. For each spin we check if any nested threads (service calls)
        have finished, and if they have, handle the results

        Also we check and see if a flag has been set to run the optimization"""
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.odometry_client_futures:
                if f.done():
                    # An odometry factor request finished
                    self.distances.append(f.result().distance)
                else:
                    incomplete_futures.append(f)
            self.odometry_client_futures = incomplete_futures
            # Make sure service threads are done before optimizing!
            if self.optimize_now and len(incomplete_futures) == 0:
                self.optimize()
                self.optimize_now = False


def main(args=None):
    """Start the program."""
    rclpy.init(args=args)
    factor_subscriber = FactorSubscriber()
    factor_subscriber.spin()
    # Destroy the node explicitly
    factor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
