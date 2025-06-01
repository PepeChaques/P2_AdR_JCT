import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from irobot_create_msgs.msg import WheelVels

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, Odom2DDriftSimulator
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # TODO: Initialize filter with initial state and covariance
        initial_state = np.zeros(3)
        initial_covariance = np.eye(3) * 1.0

        self.kf = KalmanFilter(initial_state, initial_covariance)
        self.visualizer = Visualizer()
        self.odom_simulator = Odom2DDriftSimulator()
        self.initial_pose = None
        self.first_prediction_done = False
        self.u = np.zeros(2)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf_estimate',
            10
        )

    def odom_callback(self, msg):
        # TODO: Extract velocities and timestep
        # TODO: Run predict() and update() of KalmanFilter
        # TODO: Publish estimated state
        if not self.initial_pose:
            self.initial_pose = odom_to_pose2D(msg)
            self.prev_time = self.get_clock().now().nanoseconds
            return

        current_pose = odom_to_pose2D(msg)
        self.normalized_pose = np.array(get_normalized_pose2D(self.initial_pose, current_pose))

        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.u = np.array([linear.x, angular.z])

        curr_time = self.get_clock().now().nanoseconds
        if self.prev_time is not None:
            dt = (curr_time - self.prev_time) / 1e9
        else:
            dt = 0.0
        
        mu, sigma = self.kf.predict(self.u, dt)
        self.first_prediction_done = True

        if self.first_prediction_done:

            curr_time_secs = self.get_clock().now().nanoseconds
            #z = self.odom_simulator.add_drift(self.normalized_pose,curr_time_secs)
            z = self.normalized_pose

            mu, Sigma = self.kf.update(z)
            self.visualizer.update(self.normalized_pose, mu, Sigma, step ='update')
            self.publish_estimated_pose(mu, Sigma)
            self.publish_real_pose(self.normalized_pose)
            self.get_logger().info(f"Updated State")

    def publish_estimated_pose(self,mu,sigma):
            
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = mu[0]
        msg.pose.pose.position.y = mu[1]
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = np.sin(mu[2]/2.0)
        msg.pose.pose.orientation.w = np.sin(mu[2]/2.0)

        for i in range(3):
            for j in range(3):
                msg.pose.covariance[i*6 + j] = sigma[i,j]

        self.publisher.publish(msg)

    def publish_real_pose(self,pose):

        msg2 = PoseWithCovarianceStamped()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.header.frame_id = "map"
        msg2.pose.pose.position.x = pose[0]
        msg2.pose.pose.position.y = pose[1]
        msg2.pose.pose.position.z = 0.0

        msg2.pose.pose.orientation.z = np.sin(pose[2]/2.0)
        msg2.pose.pose.orientation.w = np.sin(pose[2]/2.0)

        self.publisher.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()