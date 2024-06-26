import asyncio
import base64
import datetime
import json
import sys
import os
import time

import cv2
import numpy as np
import requests
from PyQt5 import QtCore, QtGui
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
    "camera_k": [[], [], []],  # 相机的内参k
    "camera_dis_coeffs": [],  # 相机的内参dis_coeffs
    "drone_xyz_of_aruco": [[0, 0, 0]],  # m 在二维码下无人机的位置
    "drone_xyz_rvec_of_aruco": [[0, 0, 0]],  # 在二维码下相机的旋转位置，和无人机去向指定坐标有关
    "aruco_in_camera": [[0, 0]],  # % 在相机的画面中，二维码的位置，中心点为0，0，右上为正
    "drone_real_position": [0, 0, 0],  # 无人机的真实位置
    "drone_real_orientation": [0, 0, 0, 0],  # 无人机的真实旋转向量
    "drone_altitude": 0.,  # 无人机的气压计高度
    "limit_height_m": 1.,
    "test_connect_status": "status",  # 连接成功，连接失败，可以启动，不能启动
    "drone_is_running": False,  # 无人机是否起飞，当为不起飞的是否，跳出运行的while循环
    "image_and_data_get_url": "http://192.168.1.112:8000",
    "drone_is_auto_search_aruco": False,  # 无人机是否自动搜寻二维码，并且停在上空
    "drone_is_kill_fly": False,  # 是否直接关闭飞行的电机
}

drone1: System = System()
# 识别二维码的初始化
parameters = cv2.aruco.DetectorParameters()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


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
        try:
            url2 = f'{app_data["image_and_data_get_url"]}/video_feed'
            video_feed = requests.get(url2).json()
            video_feed = video_feed['frame_encoded_string'][0]
            img_bytes = base64.b64decode(video_feed)
            # print(len(img_bytes))
            # You need to know the original shape of the image. Suppose it's (480, 640, 3)
            img_shape = (480, 640, 3)
            # You also need to know the data type of the image. Let's assume it's uint8
            img_dtype = np.uint8
            img = np.frombuffer(img_bytes, dtype=img_dtype).reshape(img_shape)
            # print(img.shape)
            # cv2.imwrite("temp.jpg", img)
            # img = cv2.imread("temp.jpg")

            ret, buffer = cv2.imencode('.jpg', img)
            img = cv2.imdecode(buffer, cv2.IMREAD_COLOR)

            # print(img.shape)
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
                # 获取识别后的坐标
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.78833,
                                                                    np.array(app_data["camera_k"]),
                                                                    np.array(app_data["camera_dis_coeffs"]))
                # 画轴
                cv2.drawFrameAxes(img, np.array(app_data["camera_k"]),
                                  np.array(app_data["camera_dis_coeffs"]),
                                  rvec[0, :, :], tvec[0, :, :], 0.03)
                # 画框
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
                # 更新位置
                app_data['drone_xyz_of_aruco'] = [list(tvec[0, :, :][0])]
                # 更新转转变量
                app_data['drone_xyz_rvec_of_aruco'] = [list(rvec[0, :, :][0])]
            h, w, ch = img.shape
            qt_img = QImage(img.data, w, h, ch * w, QImage.Format_RGB888).rgbSwapped()
            self.changePixmap.emit(qt_img)
        except Exception as e:
            print(e)


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
        self.drone_search_aruco_pushButton.clicked.connect(self.drone_search_aruco_pushButton_event)
        self.drone_kill_pushButton.clicked.connect(self.drone_kill_pushButton_event)
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
        # 将定时器的timeout信号连接到自定义的函数，即每100ms会调用一次
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

    def drone_kill_pushButton_event(self):
        global app_data
        app_data['drone_is_kill_fly'] = True

    def drone_search_aruco_pushButton_event(self):
        global app_data
        if app_data["drone_is_auto_search_aruco"]:
            app_data["drone_is_auto_search_aruco"] = False
            self.drone_search_status_label.setText("悬停关闭")
            self.drone_search_aruco_pushButton.setText("启动悬停")
            return
        # 首先确保二维码在画面中，不在画面中，提示用户不能进行
        # 获取QLabel的Pixmap
        pixmap = self.label.pixmap()
        # 将QPixmap转换为QImage
        qimage = pixmap.toImage()
        # 获取QImage宽高
        width = qimage.width()
        height = qimage.height()
        # 获取QImage的bytearray，并reshape为(height, width, 4)
        ptr = qimage.bits()
        ptr.setsize(height * width * 4)
        arr = np.frombuffer(ptr, np.uint8).reshape((height, width, 4))
        # 将图像转换为OpenCV格式
        gray_img = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
        # 获取图像的框，id，rejectedImgPoints
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
        if len(corners) > 0:
            app_data["drone_is_auto_search_aruco"] = True
            self.drone_search_status_label.setText("启动自动悬停")
            self.drone_search_aruco_pushButton.setText("关闭悬停")
        else:
            app_data["drone_is_auto_search_aruco"] = False
            self.drone_search_status_label.setText("未检测到二维码")

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
        await drone1.action.set_takeoff_altitude(float(app_data["limit_height_m"]))
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
                if app_data['drone_is_kill_fly']:
                    app_data["drone_is_auto_search_aruco"] = False
                    self.drone_search_status_label.setText("悬停关闭")
                    self.drone_search_aruco_pushButton.setText("启动悬停")
                    app_data['drone_is_kill_fly'] = False
                    await drone1.action.kill()
                    return
                if not app_data["drone_is_running"]:
                    app_data["drone_is_auto_search_aruco"] = False
                    self.drone_search_status_label.setText("悬停关闭")
                    self.drone_search_aruco_pushButton.setText("启动悬停")
                    break
                await asyncio.sleep(float(app_data["drone_response_time_s"]))
                # 可以更新气压计高度
                now_height_m = 0
                async for position in drone1.telemetry.position():
                    app_data["drone_altitude"] = position.relative_altitude_m
                    now_height_m = position.relative_altitude_m
                    break
                # 进行无人机的飞行控制
                limit_height_m = float(app_data["limit_height_m"])  # 无人机的限制高度
                setup_speed_hight = abs(now_height_m - limit_height_m) * 3
                if now_height_m > limit_height_m:
                    app_data["drone_down_m_s"] = setup_speed_hight
                else:
                    app_data["drone_down_m_s"] = -setup_speed_hight

                # 自动在二维码上空悬停，二维码在相机正中间
                if app_data["drone_is_auto_search_aruco"]:
                    setup_speed_0 = abs(app_data["aruco_in_camera"][0][0] - 0) * 1
                    # 差距越大越要向那个地方飞，因为二维码是相对于无人机的
                    if app_data["aruco_in_camera"][0][0] > 0:  # x -> left,right
                        app_data["drone_right_m_s"] = setup_speed_0
                    else:
                        app_data["drone_right_m_s"] = -setup_speed_0
                    setup_speed_1 = abs(app_data["aruco_in_camera"][0][1] - 0) * 1
                    if app_data["aruco_in_camera"][0][1] > 0:  # y -> forward,back
                        app_data["drone_forward_m_s"] = setup_speed_1
                    else:
                        app_data["drone_forward_m_s"] = -setup_speed_1

                v_list = [float(app_data["drone_forward_m_s"]), float(app_data["drone_right_m_s"]),
                          float(app_data["drone_down_m_s"]), float(app_data["drone_yawspeed_deg_s"])]
                await drone1.offboard.set_velocity_body(VelocityBodyYawspeed(*v_list))
                if app_data["drone_is_auto_search_aruco"]:
                    self.print_log_one_line(
                        "无人机自主悬停飞行控制:前后:{: <7}m/s,左右:{: <7}m/s,上下:{: <7}m/s".format(
                            *map(lambda x: round(x, 4), v_list)))
                else:
                    self.print_log_one_line("当前飞行控制:前后:{: <7}m/s,左右:{: <7}m/s,上下:{: <7}m/s".format(
                        *map(lambda x: round(x, 4), v_list)))
        except OffboardError as e:
            self.print_log(f"启动非车载模式失败，错误代码为: {e._result.result}")
        # 0.25m/s下降
        # v_list = [0.001, 0.001, 0.25, 0.001]
        # await drone1.offboard.set_velocity_body(VelocityBodyYawspeed(*v_list))
        await drone1.action.land()
        self.print_log("无人机降落，等待15s...")
        await asyncio.sleep(15)
        self.print_log("停止板载模式...")
        await drone1.offboard.stop()
        await asyncio.sleep(1)
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
        if len(text) == 1:
            self.textBrowser.append(f"{datetime.datetime.now().strftime('[%Y-%m-%d %H:%M:%S]')}"
                                    f" -- {text[0]}")
        else:
            self.textBrowser.append(f"{datetime.datetime.now().strftime('[%Y-%m-%d %H:%M:%S]')}"
                                    f" -- {list(text)}")

    def print_log_one_line(self, new_text):
        new_text = f"{datetime.datetime.now().strftime('[%Y-%m-%d %H:%M:%S]')}  -- {new_text}"
        text = self.textBrowser.toPlainText()
        lines = text.split('\n')

        if len(lines) > 0:
            # Remove the last line
            if '飞行控制:前后:' in lines[-1]:
                lines = lines[:-1]
            # Append the new text
            lines.append(new_text)
            # Update the text browser content
            self.textBrowser.setPlainText('\n'.join(lines))
        # Move cursor to end and scroll to cursor
        cursor = self.textBrowser.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
        self.textBrowser.setTextCursor(cursor)

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
        self.drone_xyz_of_aruco_label.setText('x:{: <7}m,y:{: <7}m,z:{: <7}m'.format(
            *map(lambda x: round(float(x), 4), app_data["drone_xyz_of_aruco"][0])))
        # 在相机中，二维码的位置，目前使用第一个二维码
        self.aruco_in_camera_label.setText('x:{: <7}%,y:{: <7}%'.format(
            *map(lambda x: round(float(x), 4), app_data["aruco_in_camera"][0])))
        self.drone_real_position_label.setText('x:{: <7}m,y:{: <7}m,z:{: <7}m'.format(
            *map(lambda x: round(float(x), 4), app_data["drone_real_position"])))
        self.drone_altitude_label.setText('{: <7}m'.format(float(app_data["drone_altitude"]).__round__(3)))
        self.test_connect_status_label.setText(app_data["test_connect_status"])
        self.drone_control_xy_label.setText('x:{: <7}m/s\ny:{: <7}m/s'.format(float(app_data["drone_forward_m_s"]),
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
