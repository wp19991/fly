import time
import asyncio
import threading
import keyboard
import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

# 先进行初始化

# 相机的初始化
# 模拟环境中相机参数在/root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/depth_camera/depth_camera.sdf
# 计算公式
# K[0][0]=fx=K[1][1]=fy=width/(2*tan(FOV/2)=848/(2*tan(1.5009831567/2))=454.6843330573585
# K[0][2]=cx=width/2=848/2=424.0
# K[1][2]=cy=height/2=480/2=240.0
# Camera intrinsic matrix
K = np.array([[454.6843330573585,0.0,424.0],[0.0,454.6843330573585,240.0],[0.0,0.0,1]])
dist_coeffs = np.array([0.0001,0.0001,0.0001,0.0001,0])

# 识别二维码的初始化
parameters = cv2.aruco.DetectorParameters()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
font = cv2.FONT_HERSHEY_SIMPLEX

# 无人机控制步长的初始化
VelocityBodyYawSpeed = [0.0, 0.0, 0.0, 0.0]
step_size_m_s = 0.5  # 1 m/s step size for velocity control
response_time_s = 0.2

# 无人机的初始化
is_running = False
is_move_to_tag_above = False

# aruco坐标系下无人机的xyz位置
aruco_pos = [0, 0, 0]

# 无人机在模拟环境中真实位置
drone_real_position = [0, 0, 0]
drone_real_orientation = [0, 0, 0, 0]

# 二维码在相机画面中的位置，中心点为0，0，右上为正
aruco_in_camera = [0, 0]


# 编写任务飞行到坐标点处
# 地面铺设多个二维码，防止摄像头获取不到二维码，多个二维码融合获取定位
# 当前单个二维码计算误差在0-10cm

def fly_start(e):
    global is_running
    is_running = True


def move_to_tag_above(e):
    global is_move_to_tag_above
    is_move_to_tag_above = True


def landing(e):
    global is_running
    is_running = False


def handle_keyboard_input():
    keyboard.on_press_key('s', fly_start)
    keyboard.on_press_key('t', move_to_tag_above)
    keyboard.on_press_key('l', landing)


# 对图像和无人机在模拟环境中的位置进行订阅
class Ros2TopicSubscription(Node):
    def __init__(self):
        super().__init__('ros2_topic_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.subscription_pos = self.create_subscription(
            Odometry,
            '/odom',
            self.position_callback,
            10)

    def position_callback(self, msg):
        # 这里可以对接收到的消息进行处理，此处仅打印
        # self.get_logger().info('Received: "%s"' % msg)
        # 无人机在模拟环境中的位置，二维码中心点为0，0，0
        global drone_real_position, drone_real_orientation
        drone_real_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        drone_real_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    def image_callback(self, msg):
        global aruco_pos, aruco_in_camera
        start = time.time()
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 获取图像的框，id，rejectedImgPoints
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if len(corners) > 0:
            # 二维码在相机画面中的位置，中心点为0，0，右上为正
            # 假设corners是检测到的第一个aruco marker的角点位置
            corner = corners[0][0]
            # 计算二维码的中心点位置
            center_marker = np.mean(corner, axis=0)
            # 获取图片的大小
            height, width = gray.shape
            # 计算图片的中心点位置
            center_image = np.array([width / 2, height / 2])
            # 计算二维码中心相对于图片中心的位置，并进行归一化，使得在边框处为-1或1，中心为0
            aruco_in_camera = 2 * (center_marker - center_image) / np.array([width, height])
            aruco_in_camera = aruco_in_camera.tolist()
            aruco_in_camera[1] = -aruco_in_camera[1]
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.78833, K, dist_coeffs)
            cv2.drawFrameAxes(cv_image, K, dist_coeffs, rvec[0, :, :], tvec[0, :, :], 0.03)
            # 在二维码的附近画框
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            cv2.rectangle(cv_image, (10, 10), (500, 150), (255, 255, 255), -1)

            # 显示ID，rvec,tvec, 旋转向量和平移向量
            cv2.putText(cv_image, "Id: " + str(ids), (10, 40), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(cv_image, "rvec: " + str(rvec[0, :, :]), (10, 70), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(cv_image, "tvec: " + str(tvec[0, :, :]), (10, 100), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            # 更新位置
            aruco_pos = tvec[0, :, :][0]

        # Display the image; Note: This will not work if your computer does not have a GUI.
        end = time.time()
        # 计算帧率并显示
        cv2.putText(cv_image, "rate: " + str(1 / (end - start)), (10, 130), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    ro2_topic_subscriber = Ros2TopicSubscription()
    rclpy.spin(ro2_topic_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ro2_topic_subscriber.destroy_node()
    rclpy.shutdown()


async def auto_control_drone():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("等待无人机连接...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("无人机连接成功")
            break

    async for is_ready in drone.telemetry.health():
        if is_ready.is_armable:
            print("无人机可以进行启动")
            break

    global is_running, VelocityBodyYawSpeed, is_move_to_tag_above, aruco_in_camera

    # 准备完成，等待命令进行起飞
    while True:
        if is_running:
            break

    print("启动中...")
    await drone.action.arm()

    print("无人机起飞...")
    await drone.action.set_takeoff_altitude(2)
    await drone.action.takeoff()
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(*VelocityBodyYawSpeed))
    # Now use set_velocity_ned instead of set_position_ned
    try:
        await drone.offboard.start()
        # 起飞需要油门给到-1以上
        print('油门给到-1，进行起飞')
        VelocityBodyYawSpeed[2] = -1.5
        # Keep the script running and listen to keyboard input
        while True:
            await asyncio.sleep(response_time_s)
            # 先控制高度在5米
            hight_m = 0
            async for position in drone.telemetry.position():
                print(f"当前气压计高度: {position.relative_altitude_m.__round__(2): <4}m")
                hight_m = position.relative_altitude_m
                break
            setup_speed_hight = abs(hight_m - 5) * 3
            if hight_m > 5:
                VelocityBodyYawSpeed[2] = setup_speed_hight
            elif 4 < hight_m < 5:
                VelocityBodyYawSpeed[2] = -setup_speed_hight

            # 根据aruco_pos的位置进行控制VelocityBodyYawSpeed
            print('计算位置x:{: <4}_,y:{: <4}_,z:{: <4}_'.format(*map(lambda x: round(x, 5), aruco_pos)))
            print('真实位置x:{: <4}_,y:{: <4}_,z:{: <4}_'.format(*map(lambda x: round(x, 5), drone_real_position)))
            # 误差计算目前使用三者差值取平均
            abs_wc = ((abs(drone_real_position[0] - aruco_pos[0]) +
                       abs(drone_real_position[1] - aruco_pos[1]) +
                       abs(drone_real_position[2] - aruco_pos[2])) / 3).__round__(5)
            print('计算位置与真实位置误差：{: <6}_'.format(abs_wc))
            # 根据气压算比例尺
            if hight_m != 0:
                bili = aruco_pos[2] / hight_m
                if bili != 0:
                    print('比例位置x:{: <4}m,y:{: <4}m,z:{: <4}m'.format(*map(lambda x: round(x / bili, 4), aruco_pos)))
            # 让无人机将pose的0，1到0，及无人机的上方
            if is_move_to_tag_above:
                # 需要知道无人机的坐标系下，二维码的坐标，需要进行转换
                # TODO 需要将通过二维码的坐标和相机中二维码的平面位置两个进行融合，来控制无人机
                #  因为无人机飞行的时候相机一直在震动，无法准确的确定无人机是否在给定的位置

                # 因为无人机起飞后的朝向和二维码的地面坐标系xyz轴不一样
                # 需要根据旋转变量来进行计算
                print('aruco图像位置x:{: <4}%,y:{: <4}%'.format(*map(lambda x: round(x, 4), aruco_in_camera)))
                setup_speed_0 = abs(aruco_in_camera[0] - 0) * 1
                # 差距越大越要向那个地方飞，因为二维码是相对于无人机的
                if aruco_in_camera[0] > 0:  # x -> left,right
                    VelocityBodyYawSpeed[1] = setup_speed_0
                else:
                    VelocityBodyYawSpeed[1] = -setup_speed_0
                setup_speed_1 = abs(aruco_in_camera[1] - 0) * 1
                if aruco_in_camera[1] > 0:  # y -> forward,back
                    VelocityBodyYawSpeed[0] = setup_speed_1
                else:
                    VelocityBodyYawSpeed[0] = -setup_speed_1

            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(*VelocityBodyYawSpeed))
            # 上下为负数，展示变成正数
            VelocityBodyYawSpeed[2] = -VelocityBodyYawSpeed[2]
            print("无人机自主飞行控制:前后:{: <4}m/s,左右:{: <4}m/s,上下:{: <4}m/s".format(
                *map(lambda x: round(x, 4), VelocityBodyYawSpeed)))
            VelocityBodyYawSpeed[2] = -VelocityBodyYawSpeed[2]
            print('=' * 30)
            # 进行降落，则跳出循环
            if not is_running:
                break

    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
    except KeyboardInterrupt:
        print("User interrupted, stopping...")
    finally:
        await drone.action.land()
        await asyncio.sleep(15)
        # Stop offboard mode.
        print("Stopping offboard mode...")
        await drone.offboard.stop()
        # Disarm the drone.
        print("Disarming...")
        await drone.action.disarm()


def fly():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(auto_control_drone())


if __name__ == '__main__':
    # 启动一个线程对键盘进行订阅，控制无人机的起飞降落和关机，还有其他任务，例如飞向tag降落等
    threading.Thread(target=handle_keyboard_input).start()

    # 启动无人机监听线程
    threading.Thread(target=main).start()

    fly()
