import asyncio
import datetime
import json
import sys
import os
import time

import cv2
import numpy as np
import requests
from PyQt5 import QtCore
from PyQt5.QtGui import QImage, QPixmap
from qasync import QEventLoop, QApplication, asyncSlot
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QTimer, QThread, pyqtSignal

from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

from fly_gui import Ui_fly_app as fly_window

os.chdir(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 运行无人机的时候更新这些参数
app_data = {
    "mavsdk_server_address": "",
    "mavsdk_server_port": "",
    "system_address": "",
    "drone_forward_m_s": 0.,
    "drone_right_m_s": 0.,
    "drone_down_m_s": 1,
    "drone_yawspeed_deg_s": 0.,
    "drone_step_size_m_s": 0.2,
    "drone_response_time_s": 0.2,
    "camera_k": [[], [], []],  # 相机的内参
    "camera_dis_coeffs": [],  # 相机的内参
    "drone_xyz_of_aruco": [[0, 0, 0]],  # m 在二维码下无人机的位置
    "aruco_in_camera": [[0, 0]],  # % 在相机的画面中，二维码的位置，中心点为0，0，右上为正
    "drone_real_position": [0, 0, 0],  # 无人机的真实位置
    "drone_real_orientation": [0, 0, 0, 0],  # 无人机的真实旋转向量
    "drone_altitude": 0.,  # 无人机的气压计高度
    "limit_height_m": 1.,
    "test_connect_status": "status",  # 连接成功，连接失败，可以启动，不能启动
    "drone_is_running": False,  # 无人机是否起飞，当为不起飞的是否，跳出运行的while循环
    "image_and_data_get_url": "http://192.168.1.216:8000",
}

drone1: System = System()
# 识别二维码的初始化
parameters = cv2.aruco.DetectorParameters()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
font = cv2.FONT_HERSHEY_SIMPLEX


class VideoThread(QThread):
    changePixmap = pyqtSignal(QImage)

    def run(self):
        while True:
            self.fresh()
            time.sleep(0.01)

    def fresh(self):
        global app_data, parameters, aruco_dict, font
        # 获取位置的网址
        url1 = f'{app_data["image_and_data_get_url"]}/position'
        res = requests.get(url1)
        app_data["drone_real_position"] = res.json()["real_position"]
        app_data["drone_real_orientation"] = res.json()["real_orientation"]
        # 获取无人机底部画面的网址
        url2 = f'{app_data["image_and_data_get_url"]}/video_feed'
        stream = requests.get(url2, stream=True)
        bytes = b''
        img = ''
        for chunk in stream.iter_content(chunk_size=1024):
            bytes += chunk
            a = bytes.find(b'\xff\xd8')
            b = bytes.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes[a:b + 2]
                bytes = bytes[b + 2:]
                img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    # 识别二维码
                    # Convert to grayscale
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
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
                        # 更新参数
                        app_data["aruco_in_camera"] = [aruco_in_camera]
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.78833,
                                                                            np.array(app_data["camera_k"]),
                                                                            np.array(app_data["camera_dis_coeffs"]))
                        cv2.drawFrameAxes(img, np.array(app_data["camera_k"]),
                                          np.array(app_data["camera_dis_coeffs"]),
                                          rvec[0, :, :], tvec[0, :, :], 0.03)
                        # 在二维码的附近画框
                        cv2.aruco.drawDetectedMarkers(img, corners, ids)
                        cv2.rectangle(img, (10, 10), (500, 150), (255, 255, 255), -1)
                        # 更新位置
                        app_data['drone_xyz_of_aruco'] = [list(tvec[0, :, :][0])]

                    h, w, ch = img.shape
                    qt_img = QImage(img.data, w, h, ch * w, QImage.Format_RGB888).rgbSwapped()
                    self.changePixmap.emit(qt_img)


class main_win(QMainWindow, fly_window):
    def __init__(self):
        super(main_win, self).__init__()
        self.setupUi(self)

        # 绑定事件
        self.test_connect_pushButton.clicked.connect(self.test_connect_pushButton_event)
        self.drone_takeoff_pushButton.clicked.connect(self.drone_takeoff_pushButton_event)
        self.drone_landing_pushButton.clicked.connect(self.drone_landing_pushButton_event)
        self.drone_forward_add_pushButton.clicked.connect(self.drone_forward_add_pushButton_event)
        self.drone_forward_sub_pushButton.clicked.connect(self.drone_forward_sub_pushButton_event)
        self.drone_right_sub_pushButton.clicked.connect(self.drone_right_sub_pushButton_event)
        self.drone_right_add_pushButton.clicked.connect(self.drone_right_add_pushButton_event)
        # doubleSpinBox修改事件
        self.drone_forward_m_s_doubleSpinBox.valueChanged.connect(self.drone_forward_m_s_doubleSpinBox_event)
        self.drone_right_m_s_doubleSpinBox.valueChanged.connect(self.drone_right_m_s_doubleSpinBox_event)
        self.drone_down_m_s_doubleSpinBox.valueChanged.connect(self.drone_down_m_s_doubleSpinBox_event)
        self.drone_yawspeed_deg_s_doubleSpinBox.valueChanged.connect(self.drone_yawspeed_deg_s_doubleSpinBox_event)
        self.drone_step_size_m_s_doubleSpinBox.valueChanged.connect(self.drone_step_size_m_s_doubleSpinBox_event)
        self.drone_response_time_s_doubleSpinBox.valueChanged.connect(self.drone_response_time_s_doubleSpinBox_event)
        self.limit_height_m_doubleSpinBox.valueChanged.connect(self.limit_height_m_doubleSpinBox_event)

        # 创建一个QTimer对象
        self.fresh_data_timer = QTimer(self)
        # 设置定时器每100ms触发一次
        self.fresh_data_timer.setInterval(100)
        # 将定时器的timeout信号连接到自定义的函数，即每100ms会调用一次self.my_func函数
        self.fresh_data_timer.timeout.connect(self.fresh_data)
        # 启动定时器
        self.fresh_data_timer.start()

        self.video_th = VideoThread(self)
        self.video_th.changePixmap.connect(self.set_image)
        self.video_th.start()

    def set_image(self, image):
        self.label.clear()
        self.pix = QPixmap(image).scaled(self.label.width(), self.label.height())
        self.label.setPixmap(self.pix)
        self.label.setScaledContents(True)

    def drone_forward_add_pushButton_event(self):
        global app_data
        self.drone_forward_m_s_doubleSpinBox.setValue(
            app_data['drone_forward_m_s'] + float(app_data['drone_step_size_m_s']))

    def drone_forward_sub_pushButton_event(self):
        global app_data
        self.drone_forward_m_s_doubleSpinBox.setValue(
            app_data['drone_forward_m_s'] - float(app_data['drone_step_size_m_s']))

    def drone_right_sub_pushButton_event(self):
        global app_data
        self.drone_right_m_s_doubleSpinBox.setValue(
            app_data['drone_right_m_s'] - float(app_data['drone_step_size_m_s']))

    def drone_right_add_pushButton_event(self):
        global app_data
        self.drone_right_m_s_doubleSpinBox.setValue(
            app_data['drone_right_m_s'] + float(app_data['drone_step_size_m_s']))

    def drone_landing_pushButton_event(self):
        global app_data
        app_data['drone_is_running'] = False

    @asyncSlot()
    async def drone_takeoff_pushButton_event(self):
        global drone1, app_data
        if app_data['test_connect_status'] != "可以启动":
            self.print_log("无人机启动失败，当前不能起飞，请检查无人机状态")
            return
        if float(app_data["drone_down_m_s"]) > -1:
            self.print_log("油门太小，不足以启动无人机，数字越小油门越大")
            return
        # 准备完成，进行起飞
        self.print_log("等待启动...")
        await drone1.action.arm()
        self.print_log("无人机起飞...")
        await drone1.action.set_takeoff_altitude(2)
        await drone1.action.takeoff()
        v_list = [float(app_data["drone_forward_m_s"]), float(app_data["drone_right_m_s"]),
                  float(app_data["drone_down_m_s"]), float(app_data["drone_yawspeed_deg_s"])]
        await drone1.offboard.set_velocity_body(VelocityBodyYawspeed(*v_list))
        await drone1.offboard.start()
        app_data["drone_is_running"] = True
        # 起飞需要油门给到-1以上
        self.print_log(f'油门给到{app_data["drone_down_m_s"]}，进行起飞')
        try:
            while True:
                if not app_data["drone_is_running"]:
                    break
                await asyncio.sleep(float(app_data["drone_response_time_s"]))
                # 可以更新气压计高度
                now_height_m = 0
                async for position in drone1.telemetry.position():
                    # self.print_log(f"当前气压计高度: {position.relative_altitude_m.__round__(2): <4}m")
                    app_data["drone_altitude"] = position.relative_altitude_m
                    now_height_m = position.relative_altitude_m
                    break
                # 进行无人机的额飞行控制
                limit_height_m = float(app_data["limit_height_m"])  # 无人机的限制高度

                setup_speed_hight = abs(now_height_m - limit_height_m) * 3
                if now_height_m > limit_height_m:
                    app_data["drone_down_m_s"] = setup_speed_hight
                else:
                    app_data["drone_down_m_s"] = -setup_speed_hight

                v_list = [float(app_data["drone_forward_m_s"]), float(app_data["drone_right_m_s"]),
                          float(app_data["drone_down_m_s"]), float(app_data["drone_yawspeed_deg_s"])]
                await drone1.offboard.set_velocity_body(VelocityBodyYawspeed(*v_list))
                self.print_log("无人机自主飞行控制:前后:{: <4}m/s,左右:{: <4}m/s,上下:{: <4}m/s".format(
                    *map(lambda x: round(x, 4), v_list)))
        except OffboardError as e:
            self.print_log(f"启动非车载模式失败，错误代码为: {e._result.result}")
        finally:
            self.print_log("无人机降落，等待15s...")
            await drone1.action.land()
            await asyncio.sleep(15)
            self.print_log("停止板载模式...")
            await drone1.offboard.stop()
            self.print_log("解除武装...")
            await drone1.action.disarm()

    @asyncSlot()
    async def test_connect_pushButton_event(self):
        global drone1, app_data
        drone1 = System(mavsdk_server_address=app_data["mavsdk_server_address"],
                        port=int(app_data["mavsdk_server_port"]))
        await drone1.connect()
        app_data['test_connect_status'] = "等待连接..."
        self.print_log("等待无人机进行连接...")
        async for state in drone1.core.connection_state():
            if state.is_connected:
                app_data['test_connect_status'] = "连接成功"
                break
            else:
                app_data['test_connect_status'] = "连接失败"
                self.print_log("无人机连接失败")
        self.print_log("无人机连接成功")
        async for is_ready in drone1.telemetry.health():
            if is_ready.is_armable:
                app_data['test_connect_status'] = "可以启动"
                break
            else:
                app_data['test_connect_status'] = "不能启动"
                self.print_log("无人机启动失败，当前不能启动，请检查无人机状态")
                self.print_log(str(is_ready))

    def drone_forward_m_s_doubleSpinBox_event(self):
        global app_data
        app_data['drone_forward_m_s'] = self.drone_forward_m_s_doubleSpinBox.text()
        self.print_log(f"修改飞行控制参数drone_forward_m_s:{app_data['drone_forward_m_s']}")

    def drone_right_m_s_doubleSpinBox_event(self):
        global app_data
        app_data['drone_right_m_s'] = self.drone_right_m_s_doubleSpinBox.text()
        self.print_log(f"修改飞行控制参数drone_right_m_s:{app_data['drone_right_m_s']}")

    def drone_down_m_s_doubleSpinBox_event(self):
        global app_data
        app_data['drone_down_m_s'] = self.drone_down_m_s_doubleSpinBox.text()
        self.print_log(f"修改飞行控制参数drone_down_m_s:{app_data['drone_down_m_s']}")

    def drone_yawspeed_deg_s_doubleSpinBox_event(self):
        global app_data
        app_data['drone_yawspeed_deg_s'] = self.drone_yawspeed_deg_s_doubleSpinBox.text()
        self.print_log(f"修改飞行控制参数drone_yawspeed_deg_s:{app_data['drone_yawspeed_deg_s']}")

    def drone_step_size_m_s_doubleSpinBox_event(self):
        global app_data
        app_data['drone_step_size_m_s'] = self.drone_step_size_m_s_doubleSpinBox.text()
        self.print_log(f"修改控制参数drone_step_size_m_s:{app_data['drone_step_size_m_s']}")

    def drone_response_time_s_doubleSpinBox_event(self):
        global app_data
        app_data['drone_response_time_s'] = self.drone_response_time_s_doubleSpinBox.text()
        self.print_log(f"修改响应参数drone_response_time_s:{app_data['drone_response_time_s']}")

    def limit_height_m_doubleSpinBox_event(self):
        global app_data
        app_data['limit_height_m'] = self.limit_height_m_doubleSpinBox.text()
        self.print_log(f"修改响应参数limit_height_m:{app_data['limit_height_m']}")

    def print_log(self, *text):
        self.textBrowser.append(f"{datetime.datetime.now().strftime('[%Y-%m-%d %H:%M:%S]')}"
                                f" -- {text}")

    def fresh_data(self):
        # 将gui的数据写道全局变量中
        global app_data
        app_data["mavsdk_server_address"] = self.mavsdk_server_address_lineEdit.text()
        app_data["mavsdk_server_port"] = self.mavsdk_server_port_lineEdit.text()
        app_data["system_address"] = self.system_address_lineEdit.text()
        app_data["drone_forward_m_s"] = float(self.drone_forward_m_s_doubleSpinBox.text())
        app_data["drone_right_m_s"] = float(self.drone_right_m_s_doubleSpinBox.text())
        app_data["drone_down_m_s"] = float(self.drone_down_m_s_doubleSpinBox.text())
        app_data["drone_yawspeed_deg_s"] = float(self.drone_yawspeed_deg_s_doubleSpinBox.text())
        app_data["drone_step_size_m_s"] = float(self.drone_step_size_m_s_doubleSpinBox.text())
        app_data["drone_response_time_s"] = float(self.drone_response_time_s_doubleSpinBox.text())
        app_data["camera_k"] = json.loads(self.camera_k_lineEdit.text())
        app_data["camera_dis_coeffs"] = json.loads(self.camera_dis_coeffs_lineEdit.text())
        app_data["limit_height_m"] = float(self.limit_height_m_doubleSpinBox.text())
        # 将全局变量中的值写到gui中
        # 目前使用一个二维码
        # TODO 如果有多个，可以进行计算融合
        self.drone_xyz_of_aruco_label.setText('x:{: <4}m,y:{: <4}m,z:{: <4}m'.format(
            *map(lambda x: round(float(x), 4), app_data["drone_xyz_of_aruco"][0])))
        # 在相机中，二维码的位置，目前使用第一个二维码
        self.aruco_in_camera_label.setText('x:{: <4}%,y:{: <4}%'.format(
            *map(lambda x: round(float(x), 4), app_data["aruco_in_camera"][0])))
        self.drone_real_position_label.setText('x:{: <4}m,y:{: <4}m,z:{: <4}m'.format(
            *map(lambda x: round(float(x), 4), app_data["drone_real_position"])))
        self.drone_altitude_label.setText('{: <4}m'.format(float(app_data["drone_altitude"]).__round__(3)))
        self.test_connect_status_label.setText(app_data["test_connect_status"])
        self.drone_control_xy_label.setText('x:{: <4}m/s\ny:{: <4}m/s'.format(float(app_data["drone_forward_m_s"]),
                                                                              float(app_data["drone_right_m_s"])))


if __name__ == "__main__":
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)

    app = QApplication(sys.argv)

    event_loop = QEventLoop(app)
    asyncio.set_event_loop(event_loop)

    app_close_event = asyncio.Event()
    app.aboutToQuit.connect(app_close_event.set)

    main_window = main_win()
    main_window.show()

    with event_loop:
        event_loop.run_until_complete(app_close_event.wait())
