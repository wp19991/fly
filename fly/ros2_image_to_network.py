import threading

import uvicorn
from fastapi import FastAPI
from fastapi.responses import StreamingResponse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

app = FastAPI()

frame = None


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        global frame
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        ret, jpeg = cv2.imencode('.jpg', cv_image)
        frame = jpeg.tobytes()


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


def get_frame():
    global frame
    yield (b'--frame\r\n'
           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # 生成帧数据


@app.get('/video_feed')
def video_feed():
    global image_subscriber
    return StreamingResponse(get_frame(), media_type='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    # 启动无人机监听线程
    threading.Thread(target=main).start()
    uvicorn.run(app, host='0.0.0.0', port=8000)
