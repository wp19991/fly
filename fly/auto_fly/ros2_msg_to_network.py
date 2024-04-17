import threading

import uvicorn
from fastapi import FastAPI
from fastapi.encoders import jsonable_encoder
from fastapi.responses import StreamingResponse, JSONResponse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

app = FastAPI()

frame = None
drone_real_position = []
drone_real_orientation = []


class Ros2TopicSubscription(Node):
    def __init__(self):
        super().__init__('ros2_topic_subscriber')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.subscription_pos = self.create_subscription(
            Odometry, '/odom', self.position_callback, 10)

    def position_callback(self, msg):
        # 这里可以对接收到的消息进行处理，此处仅打印
        # self.get_logger().info('Received: "%s"' % msg)
        # 无人机在模拟环境中的位置，二维码中心点为0，0，0
        global drone_real_position, drone_real_orientation
        drone_real_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        drone_real_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    def image_callback(self, msg):
        global frame
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        ret, jpeg = cv2.imencode('.jpg', cv_image)
        frame = jpeg.tobytes()


def main(args=None):
    rclpy.init(args=args)
    ros2_topic_subscriber = Ros2TopicSubscription()
    rclpy.spin(ros2_topic_subscriber)
    ros2_topic_subscriber.destroy_node()
    rclpy.shutdown()


def get_frame():
    global frame
    yield (b'--frame\r\n'
           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # 生成帧数据


def get_position():
    global drone_real_position, drone_real_orientation
    return drone_real_position, drone_real_orientation


@app.get('/video_feed')
def video_feed():
    return StreamingResponse(get_frame(), media_type='multipart/x-mixed-replace; boundary=frame')


@app.get('/position')
def drone_real_position():
    data = get_position()
    res_data = {"real_position": data[0], "real_orientation": data[1]}
    json_compatible_item_data = jsonable_encoder(res_data)
    return JSONResponse(content=json_compatible_item_data)


if __name__ == '__main__':
    # 启动无人机监听线程
    threading.Thread(target=main).start()
    uvicorn.run(app, host='0.0.0.0', port=8000)
