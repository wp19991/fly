import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# Define axis for visualization (length of 0.1 meters)
axis = np.float32([[0.1, 0, 0], [0, 0.1, 0], [0, 0, -0.1]]).reshape(-1, 3)

# Camera intrinsic matrix
K = np.array([[391.95377974, 0.0, 335.17043033],
              [0.0, 377.71297362, 245.03757622],
              [0.0, 0.0, 1]])
dist_coeffs = np.array([0.04714445, -0.07145486, 0.00588382, 0.00876541, 0])

# Initialize the detector parameters using default values
parameters = cv2.aruco.DetectorParameters_create()

# Create the dictionary that will be used to identify the aruco markers.
# The marker IDs will correspond to the ones in this dictionary.
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg)

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect the markers in the image
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if len(corners) > 0:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, K, dist_coeffs)
            cv_image = cv2.aruco.drawAxis(cv_image, K, dist_coeffs, rvec, tvec, 0.1)  # 绘制轴
            cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners)  # 在标记周围画一个正方形
            # Draw detected markers on the image
            cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        # Display the image; Note: This will not work if your computer does not have a GUI.
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def draw_cube(self, img, corners, imgpts):
        imgpts = np.int32(imgpts).reshape(-1, 2)

        # draw cube edges
        for i, j in zip(range(4), range(4, 8)):
            img = cv2.line(img, tuple(corners[i]), tuple(imgpts[j]), (255), 3)

        return img


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
