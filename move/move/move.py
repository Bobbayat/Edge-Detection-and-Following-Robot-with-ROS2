import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class Move(Node):
    def __init__(self):
        super().__init__('move')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            Image,
            'edges',
            self.follow_edge,
            10)
        self.bridge = CvBridge()

    def follow_edge(self, msg):
        # Convert the ROS message to an OpenCV image
        edges = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Convert the edges image to a binary mask
        thresh = cv2.threshold(edges, 128, 255, cv2.THRESH_BINARY)[1]

        # Find the contours of the edges
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If no contours are found, stop moving
        if len(contours) == 0:
            twist = Twist()
            self.publisher_.publish(twist)
            return

        # Choose the largest contour as the edge to follow
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the centroid of the largest contour
        moments = cv2.moments(largest_contour)
        # If the area of the contour is zero, stop moving
        if moments['m00'] == 0:
            twist = Twist()
            self.publisher_.publish(twist)
            return
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Calculate the error between the centroid and the center of the image
        image_center_x = int(edges.shape[1] / 2)
        error = image_center_x - cx

        # Calculate the linear and angular velocity commands based on the error
        linear_vel = 0.2
        angular_vel = -0.002 * error

        # Publish the velocity commands
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = Move()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

