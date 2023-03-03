import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2


class Move(Node):

    def __init__(self):
        super().__init__('move')
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.edge_callback,
            10)

    def edge_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Convert the image to grayscale and apply Canny edge detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 100, 200)

            # Determine the center of the image
            height, width = edges.shape
            center = int(width / 2)

            # Find the edge pixels
            edge_pixels = []
            for i in range(height):
                for j in range(width):
                    if edges[i, j] == 255:
                        edge_pixels.append((j, i))

            # If no edge pixels are found, stop moving
            if len(edge_pixels) == 0:
                msg = Twist()
                msg.direction = "stop"
                self.vel_publisher.publish(msg)
        

            # Find the x-coordinate of the closest edge pixel to the center of the image
            closest_pixel = min(edge_pixels, key=lambda p: abs(p[0] - center))
            closest_x = closest_pixel[0]

            # Determine the direction to move based on the x-coordinate of the closest edge pixel
            if closest_x < center - 50:
                msg = Twist()
                msg.direction = "left"
                self.vel_publisher.publish(msg)
            elif closest_x > center + 50:
                msg = Twist()
                msg.direction = "right"
                self.vel_publisher.publish(msg)
            else:
                msg = Twist()
                msg.direction = "forward"
                self.vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    move = Move()

    rclpy.spin(move)

    move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
