import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class EdgeDetection(Node):
    def __init__(self):
        super().__init__('edge_detection')
        self.publisher_ = self.create_publisher(Image, 'edges', 10)
        self.timer_ = self.create_timer(0.1, self.detect_edges)
        self.bridge = CvBridge()

    def detect_edges(self):
        cap = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()

            # converting BGR to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # define range of red color in HSV
            lower_red = np.array([5,50,50])
            upper_red = np.array([15,255,255])

            # create a red HSV colour boundary and
            # threshold HSV image
            mask = cv2.inRange(hsv, lower_red, upper_red)

            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(frame,frame, mask= mask)

            # Display an original image
            cv2.imshow('Original',frame)

            # finds edges in the input image image and
            # marks them in the output map edges
            edges = cv2.Canny(frame,100,200)

            # Display edges in a frame
            cv2.imshow('Edges',edges)

            # Convert the edges image to a ROS message
            edges_msg = self.bridge.cv2_to_imgmsg(edges, encoding="passthrough")
            edges_msg.header.stamp = self.get_clock().now().to_msg()

            # Publish the edges image
            self.publisher_.publish(edges_msg)

            # Wait for Esc key to stop
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break

        # Close the window
        cap.release()

        # De-allocate any associated memory usage
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    node = EdgeDetection()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

