import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# 模拟环境中相机参数在/root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/depth_camera/depth_camera.sdf
# 计算公式
# K[0][0]=fx=K[1][1]=fy=width/(2*tan(FOV/2)=848/(2*tan(1.5009831567/2))=454.6843330573585
# K[0][2]=cx=width/2=848/2=424.0
# K[1][2]=cy=height/2=480/2=240.0
# Camera intrinsic matrix
K = np.array([[454.6843330573585, 0.0, 424.0],
              [0.0, 454.6843330573585, 240.0],
              [0.0, 0.0, 1]])
dist_coeffs = np.array([0.0001, 0.0001, 0.0001, 0.0001, 0])

# Initialize the detector parameters using default values
parameters = cv2.aruco.DetectorParameters()
# cv2.aruco.De
# Create the dictionary that will be used to identify the aruco markers.
# The marker IDs will correspond to the ones in this dictionary.
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

font = cv2.FONT_HERSHEY_SIMPLEX


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        start = time.time()
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 获取图像的框，id，rejectedImgPoints
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if len(corners) > 0:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, K, dist_coeffs)
            cv2.drawFrameAxes(cv_image, K, dist_coeffs, rvec[0, :, :], tvec[0, :, :], 0.03)
            # 在二维码的附近画框
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            cv2.rectangle(cv_image, (10, 10), (500, 150), (255, 255, 255), -1)

            # 显示ID，rvec,tvec, 旋转向量和平移向量
            cv2.putText(cv_image, "Id: " + str(ids), (10, 40), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(cv_image, "rvec: " + str(rvec[0, :, :]), (10, 70), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(cv_image, "tvec: " + str(tvec[0, :, :]), (10, 100), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        # Display the image; Note: This will not work if your computer does not have a GUI.
        end = time.time()
        # 计算帧率并显示
        cv2.putText(cv_image, "rate: " + str(1 / (end - start)), (10, 130), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


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
