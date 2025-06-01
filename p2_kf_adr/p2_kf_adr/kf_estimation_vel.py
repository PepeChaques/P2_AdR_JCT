import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped

import numpy as np

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer


class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')
        self.initial_pose = None
        initial_state = np.zeros(6)
        initial_covariance = np.eye(6) * 0.1

        self.kf = KalmanFilter_2(initial_state, initial_covariance)
        self.visualizer = Visualizer()
        self.first_prediction_done = False
        self.prev_time = None
        self.u = np.zeros(2)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',
            10
        )
        self.real_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_real',
            10
        )

    def odom_callback(self, msg):
        pose = odom_to_pose2D(msg)
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        vx = linear.x
        omega = angular.z
        if self.initial_pose is None:
            self.initial_pose = pose
            self.kf.mu = np.zeros(6)
            self.kf.mu[:3] = pose
            self.prev_time = self.get_clock().now().nanoseconds
        
        current_pose = pose
        self.normalized_pose = np.array(get_normalized_pose2D(list(self.initial_pose[:3]),list(current_pose[:3])))

        self.u = np.array([vx, omega])

        curr_time = self.get_clock().now().nanoseconds
        if self.prev_time is not None:
            dt = (curr_time - self.prev_time)/1e9
        else:
            dt = 0.0
        self.prev_time = curr_time

        mu, sigma = self.kf.predict(self.u, dt)
        self.first_prediction_done = True
        print("Visualizer update called")

        

        # TODO: Run predict() and update() of KalmanFilter_2

        if self.first_prediction_done:
            vx, vy, omega = self.kf.mu[3:6]
            # z debe ser [x, y, theta, vx, vy, omega]
            z = np.array(list(current_pose) + [vx, vy, omega])
            mu, sigma = self.kf.update(z)
            self.visualizer.update(self.normalized_pose[:3], mu[:3], sigma[:3, :3], step="update")
            self.publish_estimated_pose(mu, sigma)
            self.publish_real_pose(self.normalized_pose)

        # TODO: Publish estimated full state

    def publish_estimated_pose(self,mu,sigma):
        print("Visualizer update called")
            
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

        #self_publish_real_pose(self.normalized_pose)
        print("Visualizer update called")
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
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    rclpy.shutdown()