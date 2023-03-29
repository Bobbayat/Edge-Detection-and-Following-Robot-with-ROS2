# Create3-iRobot
Developed two ROS2 nodes using Python that work together to detect edges from a camera
feed and navigate a robot along the detected edges. The Edge Detection node captures
an input image from the camera, applies color thresholding, and performs Canny
Edge detection to extract edges. The Move node subscribes to the edges topic, extracts
the largest contour from the edge image, and generates velocity commands to guide the
robot along the detected edges.
