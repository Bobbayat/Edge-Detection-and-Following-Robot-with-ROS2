import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
from interfaces.msg import Edge


class EdgeDetector(Node):

    def __init__(self):
        super().__init__('edge_detector')
        self.edge_publisher = self.create_publisher(Edge, 'cmd_edge', 10)
        self.subscription = self.create_subscription(
                Image,
                'video_frames',
                self.listener_callback,
                10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

    def listener_callback(self, frame):
        image = self.br.imgmsg_to_cv2(frame)
        edges = cv2.Canny(image, 100, 200)
        if self.edge_publisher.get_subscription_count() > 0:
            msg = Edge()
            # Fill in edge data to msg here
            self.edge_publisher.publish(msg)
        else:
            self.get_logger().info('Waiting for subscriber')


def main(args=None):
    rclpy.init(args=args)

    edge_detector = EdgeDetector()

    rclpy.spin(edge_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    edge_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
