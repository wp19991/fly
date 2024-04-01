import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # 防止未使用警告
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # 在窗口中显示图像; 注意：如果你的电脑没有GUI界面，则该步骤无法正常运行。
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    # 销毁节点并清理RCL资源
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
