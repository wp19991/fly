import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # 这里可以对接收到的消息进行处理，此处仅打印
        # self.get_logger().info('Received: "%s"' % msg)
        # 无人机在模拟环境中的位置，二维码中心点为0，0，0
        print("position   :x:{: <4}_,y:{: <4}_,z:{: <4}_".format(msg.pose.pose.position.x.__round__(3),
                                                                 msg.pose.pose.position.y.__round__(3),
                                                                 msg.pose.pose.position.z.__round__(3)))
        # 无人机的偏转情况,xyz轴的翻转情况
        print("orientation:x:{: <4}_,y:{: <4}_,z:{: <4}_,w:{: <4}_".format(msg.pose.pose.orientation.x.__round__(3),
                                                                           msg.pose.pose.orientation.y.__round__(3),
                                                                           msg.pose.pose.orientation.z.__round__(3),
                                                                           msg.pose.pose.orientation.w.__round__(3)))
        print('=' * 30)


def main(args=None):
    rclpy.init(args=args)

    odom_subscriber = OdomSubscriber()

    rclpy.spin(odom_subscriber)

    # Destroy the node explicitly
    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
