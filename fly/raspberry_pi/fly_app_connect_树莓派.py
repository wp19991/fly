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
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt

from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

from fly_gui import Ui_fly_app as fly_window

os.chdir(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 运行无人机的时候更新这些参数
app_data = {
    # 可以修改下面5个默认的参数，在程序启动后会变成下面的参数
    "mavsdk_server_address": "192.168.1.112",  # 真实环境，101的wifi
    # "mavsdk_server_address": "192.168.1.216",  # 模拟环境
    # "mavsdk_server_address": "192.168.77.23",  # 真实环境，移动热点
    "mavsdk_server_port": "50051",
    "image_and_data_get_url": "http://192.168.1.112:8000",  # 101的wifi
    # "image_and_data_get_url": "http://192.168.77.23:8000",  # 移动热点
    "system_address": "udp://:14540",
    "limit_height_m": 0.6,  # 真实环境中需要光流模块获取高度信息
    "is_simulation": False,  # 模拟环境中需要修改这个为True
    "drone_down_m_s": -1,  # 起飞的速度
    "drone_max_up_down_m_s": 1,  # 无人机飞行上升下降速度的最大值
    "drone_forward_m_s": 0.,
    "drone_right_m_s": 0.,
    "drone_yawspeed_deg_s": 0.,
    "drone_step_size_m_s": 0.2,
    "drone_response_time_s": 0.2,
    "camera_k": [[391.95377974, 0.0, 335.17043033], [0.0, 377.71297362, 245.03757622], [0.0, 0.0, 1.0]],  # 相机的内参k
    "camera_dis_coeffs": [0.04714445, -0.07145486, 0.00588382, 0.00876541, 0.07204812],  # 相机的内参dis_coeffs
    "aruco_id_list": [-1 for o in range(20)],  # 当前识别到的aruco的id：0-20
    "drone_xyz_of_aruco": [[0, 0, 0] for i in range(20)],  # m 在二维码下无人机的位置
    "drone_xyz_rvec_of_aruco": [[0, 0, 0] for j in range(20)],  # 在二维码下相机的旋转位置，和无人机去向指定坐标有关
    "aruco_in_camera": [[0, 0] for k in range(20)],  # % 在相机的画面中，二维码的位置，中心点为0，0，右上为正
    "time_sub_microseconds": 0.,  # 从得到的时候到完成识别的时间差值
    "drone_real_position": [0, 0, 0],  # 无人机的真实位置
    "drone_real_orientation": [0, 0, 0, 0],  # 无人机的真实旋转向量
    "drone_altitude": 0.,  # 无人机的实时高度
    "test_connect_status": "status",  # 连接成功，连接失败，可以启动，不能启动
    "drone_is_running": False,  # 无人机是否起飞，当为不起飞的是否，跳出运行的while循环
    "drone_is_auto_search_aruco": False,  # 无人机是否自动搜寻二维码，并且停在上空
    "drone_is_kill_fly": False,  # 是否直接关闭飞行的电机
}

drone1: System = System()
# 识别二维码的初始化
parameters = cv2.aruco.DetectorParameters()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


class GetDataThread(QThread):
    # 构造函数
    def __init__(self):
        super(GetDataThread, self).__init__()
        self.isCancel = False

    def cancel(self):
        # 线程取消
        self.isCancel = True

    def run(self):
        while True:
            if self.isCancel:
                break
            try:
                self.fresh()
            except Exception as e:
                print(e)
                break
            # time.sleep(0.1)

    def fresh(self):
        global app_data
        # 获取位置的网址
        url1 = f'{app_data["image_and_data_get_url"]}/app_data'
        res = requests.get(url1).json()
        app_data["aruco_id_list"] = res["aruco_id_list"]
        app_data["aruco_in_camera"] = res["aruco_in_camera"]
        app_data["drone_xyz_of_aruco"] = res["drone_xyz_of_aruco"]
        app_data["drone_xyz_rvec_of_aruco"] = res["drone_xyz_rvec_of_aruco"]
        app_data["time_sub_microseconds"] = res["time_sub_microseconds"]
        app_data["camera_k"] = res["camera_k"]
        app_data["camera_dis_coeffs"] = res["camera_dis_coeffs"]
        app_data["drone_real_position"] = res["drone_real_position"]
        app_data["drone_real_orientation"] = res["drone_real_orientation"]


class GetVideoThread(QThread):
    changePixmap = pyqtSignal(QImage)

    def __init__(self):
        super(GetVideoThread, self).__init__()
        self.isCancel = False

    def cancel(self):
        # 线程取消
        self.isCancel = True

    def run(self):
        while True:
            if self.isCancel:
                break
            try:
                self.fresh()
            except Exception as e:
                print(e)
                break
            # time.sleep(0.01)

    def fresh(self):
        global app_data
        url1 = f'{app_data["image_and_data_get_url"]}/video_frame'
        res = requests.get(url1).text
        frame_encoded_bytes = base64.b64decode(res)
        img_shape = (480, 640, 3)
        img_dtype = np.uint8
        frame_decoded = np.frombuffer(frame_encoded_bytes, dtype=img_dtype).reshape(img_shape)
        ret, buffer = cv2.imencode('.jpg', frame_decoded)
        img = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        h, w, ch = img.shape
        qt_img = QImage(img.data, w, h, ch * w, QImage.Format_RGB888).rgbSwapped()
        self.changePixmap.emit(qt_img)


class main_win(QMainWindow, fly_window):
    def __init__(self):
        super(main_win, self).__init__()
        self.setupUi(self)
        # 写一些默认参数到ui中，在程序启动后会变成app_data中的参数
        self.mavsdk_server_address_lineEdit.setText(app_data["mavsdk_server_address"])
        self.mavsdk_server_port_lineEdit.setText(app_data["mavsdk_server_port"])
        self.image_and_data_get_url_lineEdit.setText(app_data["image_and_data_get_url"])
        self.system_address_lineEdit.setText(app_data["system_address"])
        self.drone_down_m_s_doubleSpinBox.setValue(app_data["drone_down_m_s"])
        self.limit_height_m_doubleSpinBox.setValue(app_data["limit_height_m"])

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
        self.get_video_frame_pushButton.clicked.connect(self.get_video_frame_pushButton_event)
        # doubleSpinBox修改事件
        self.drone_forward_m_s_doubleSpinBox.valueChanged.connect(self.drone_forward_m_s_doubleSpinBox_event)
        self.drone_right_m_s_doubleSpinBox.valueChanged.connect(self.drone_right_m_s_doubleSpinBox_event)
        self.drone_down_m_s_doubleSpinBox.valueChanged.connect(self.drone_down_m_s_doubleSpinBox_event)
        self.drone_yawspeed_deg_s_doubleSpinBox.valueChanged.connect(self.drone_yawspeed_deg_s_doubleSpinBox_event)
        self.drone_step_size_m_s_doubleSpinBox.valueChanged.connect(self.drone_step_size_m_s_doubleSpinBox_event)
        self.drone_response_time_s_doubleSpinBox.valueChanged.connect(self.drone_response_time_s_doubleSpinBox_event)
        self.limit_height_m_doubleSpinBox.valueChanged.connect(self.limit_height_m_doubleSpinBox_event)

        # 创建一个定时器对象刷新参数显示与收集
        self.fresh_data_timer = QTimer(self)
        self.fresh_data_timer.setInterval(100)
        self.fresh_data_timer.timeout.connect(self.fresh_data)
        self.fresh_data_timer.start()

        # 从网络获取无人机识别二维码参数的线程
        self.get_data_th = GetDataThread()
        # 获取画面进行展示的线程
        self.get_video_th = GetVideoThread()
        self.get_video_th.changePixmap.connect(self.set_image)
        # 启动线程的按钮
        self.test_connect_data_url_pushButton.clicked.connect(self.test_connect_data_url_pushButton_event)

    def set_image(self, image):
        self.label.clear()
        pix = QPixmap(image).scaled(self.label.width(), self.label.height())
        self.label.setPixmap(pix)
        self.label.setScaledContents(True)

    def test_connect_data_url_pushButton_event(self):
        if self.test_connect_data_url_pushButton.text() == "关闭连接":
            self.get_data_th.cancel()
            del self.get_data_th
            self.get_video_th.cancel()
            del self.get_video_th
            self.get_data_th = GetDataThread()
            self.get_video_th = GetVideoThread()
            self.get_video_th.changePixmap.connect(self.set_image)
            self.test_connect_data_url_pushButton.setText("测试连接")
            return
        self.get_data_th.start()
        self.get_video_th.start()
        self.test_connect_data_url_pushButton.setText("关闭连接")

    def get_video_frame_pushButton_event(self):
        try:
            url1 = f'{app_data["image_and_data_get_url"]}/video_frame'
            res = requests.get(url1, timeout=3).text
            frame_encoded_bytes = base64.b64decode(res)
            img_shape = (480, 640, 3)
            img_dtype = np.uint8
            frame_decoded = np.frombuffer(frame_encoded_bytes, dtype=img_dtype).reshape(img_shape)
            ret, buffer = cv2.imencode('.jpg', frame_decoded)
            img = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            h, w, ch = img.shape
            qt_img = QImage(img.data, w, h, ch * w, QImage.Format_RGB888).rgbSwapped()
            self.label.clear()
            pix = QPixmap(qt_img).scaled(self.label.width(), self.label.height())
            self.label.setPixmap(pix)
            self.label.setScaledContents(True)
        except Exception as e:
            self.print_log(e)

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
        if app_data["drone_xyz_of_aruco"][0][0] != 0 \
                or app_data["drone_xyz_rvec_of_aruco"][0][1] != 0 \
                or app_data["aruco_in_camera"][0][0] != 0:
            app_data["drone_is_auto_search_aruco"] = True
            self.drone_search_status_label.setText("启动自动悬停")
            self.drone_search_aruco_pushButton.setText("关闭悬停")
        else:
            app_data["drone_is_auto_search_aruco"] = False
            self.drone_search_status_label.setText("未检测到二维码")

    def keyPressEvent(self, event):
        global app_data
        if event.key() == Qt.Key_W:
            self.print_log("键盘控制:W")
            self.drone_forward_m_s_doubleSpinBox.setValue(
                app_data['drone_forward_m_s'] + float(app_data['drone_step_size_m_s']))
        elif event.key() == Qt.Key_S:
            self.print_log("键盘控制:S")
            self.drone_forward_m_s_doubleSpinBox.setValue(
                app_data['drone_forward_m_s'] - float(app_data['drone_step_size_m_s']))
        elif event.key() == Qt.Key_A:
            self.print_log("键盘控制:A")
            self.drone_right_m_s_doubleSpinBox.setValue(
                app_data['drone_right_m_s'] - float(app_data['drone_step_size_m_s']))
        elif event.key() == Qt.Key_D:
            self.print_log("键盘控制:D")
            self.drone_right_m_s_doubleSpinBox.setValue(
                app_data['drone_right_m_s'] + float(app_data['drone_step_size_m_s']))
        elif event.key() == Qt.Key_K:
            self.print_log("键盘控制:K")

        # 添加更多键盘事件...

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
        if float(app_data["drone_down_m_s"]) < -1:
            self.print_log("油门太大，目前上升速度大于1m/s，不足以启动无人机，数字越小油门越大")
            return
        # 准备完成，进行起飞
        self.print_log("等待启动...")
        await drone1.action.arm()
        self.print_log(f"无人机设置起飞限制高度{app_data['limit_height_m']}")
        await drone1.action.set_takeoff_altitude(float(app_data["limit_height_m"]))
        self.print_log("无人机设置初始点")
        await drone1.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        self.print_log("无人机开始板载模式")
        try:
            await drone1.offboard.start()
        except OffboardError as error:
            self.print_log(f"启动非车载模式失败，错误为: {error._result.result}")
            self.print_log("解除武装")
            await drone1.action.disarm()
            return

        app_data["drone_is_running"] = True
        # 起飞需要油门给到-1以上
        self.print_log(f'油门给到{app_data["drone_down_m_s"]}，进行起飞')
        await drone1.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0,
                                                                     app_data["drone_down_m_s"], 0))
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
                if app_data["is_simulation"]:
                    async for position in drone1.telemetry.position():
                        app_data["drone_altitude"] = position.relative_altitude_m
                        now_height_m = position.relative_altitude_m
                        break
                else:
                    # 更新高度，需要有光流模块
                    async for distance_sensor in drone1.telemetry.distance_sensor():
                        app_data["drone_altitude"] = distance_sensor.current_distance_m
                        now_height_m = distance_sensor.current_distance_m
                        break
                # 进行无人机的飞行控制
                limit_height_m = float(app_data["limit_height_m"])  # 无人机的限制高度
                setup_speed_hight = abs(now_height_m - limit_height_m) * 3
                # 如果飞行控制的最大速度超过了限制，则改为这个速度
                if setup_speed_hight > abs(app_data["drone_max_up_down_m_s"]):
                    setup_speed_hight = abs(app_data["drone_max_up_down_m_s"])
                if now_height_m > limit_height_m:
                    app_data["drone_down_m_s"] = setup_speed_hight
                else:
                    app_data["drone_down_m_s"] = -setup_speed_hight

                # 自动在二维码上空悬停，二维码在相机正中间
                if app_data["drone_is_auto_search_aruco"]:
                    setup_speed_0 = abs(float(app_data["aruco_in_camera"][0][0]) - 0) * 1
                    # 差距越大越要向那个地方飞，因为二维码是相对于无人机的
                    if float(app_data["aruco_in_camera"][0][0]) > 0:  # x -> left,right
                        app_data["drone_right_m_s"] = setup_speed_0
                    else:
                        app_data["drone_right_m_s"] = -setup_speed_0
                    setup_speed_1 = abs(float(app_data["aruco_in_camera"][0][1]) - 0) * 1
                    if float(app_data["aruco_in_camera"][0][1]) > 0:  # y -> forward,back
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
        except Exception as e:
            self.print_log(str(e))

        await drone1.action.land()
        self.print_log("无人机降落，等待15s...")
        for i in range(30):
            await asyncio.sleep(0.5)
            if app_data['drone_is_kill_fly']:
                app_data["drone_is_auto_search_aruco"] = False
                self.drone_search_status_label.setText("悬停关闭")
                self.drone_search_aruco_pushButton.setText("启动悬停")
                app_data['drone_is_kill_fly'] = False
                await drone1.action.kill()
                return
        self.print_log("停止板载模式...")
        await drone1.offboard.stop()
        # entries = await drone1.log_files.get_entries()
        # # 下载日志信息
        # self.print_log(f"下载日志{entries[-1]}")
        # async for progress in drone1.log_files.download_log_file(entries[-1], "log"):
        #     self.print_log_one_line(progress)
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
        # 保存无人机参数
        self.print_log("保存无人机参数...")
        all_params = await drone1.param.get_all_params()
        data_dict_list = []
        for param in all_params.int_params:
            data_dict_list.append({"name": param.name, "value": param.value})
        for param in all_params.float_params:
            data_dict_list.append({"name": param.name, "value": param.value})
        with open("params.json", "w", encoding='utf-8') as file:
            file.write(json.dumps(data_dict_list, ensure_ascii=False, indent=4))
        self.print_log("保存无人机参数成功")
        self.print_log("获取无人机当前高度信息...")
        if app_data["is_simulation"]:
            async for position in drone1.telemetry.position():
                app_data["drone_altitude"] = position.relative_altitude_m
                break
        else:
            # 更新高度，需要有光流模块
            async for distance_sensor in drone1.telemetry.distance_sensor():
                app_data["drone_altitude"] = distance_sensor.current_distance_m
                break
        self.print_log("无人机当前高度{}".format(app_data["drone_altitude"]))
        self.print_log("无人机设置初始点...")
        await drone1.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        self.print_log("启动无人机开始板载模式")
        try:
            await drone1.offboard.start()
        except OffboardError as error:
            self.print_log(f"启动非车载模式失败，错误为: {error._result.result}")
            self.print_log("解除武装")
            await drone1.action.disarm()
            return

        app_data['test_connect_status'] = "可以启动"

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
            if '飞行控制:前后:' in lines[-1] \
                    or 'ProgressData:' in lines[-1]:
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
        try:
            app_data["mavsdk_server_address"] = self.mavsdk_server_address_lineEdit.text()
            app_data["mavsdk_server_port"] = self.mavsdk_server_port_lineEdit.text()
            app_data["system_address"] = self.system_address_lineEdit.text()
            app_data["drone_forward_m_s"] = float(self.drone_forward_m_s_doubleSpinBox.text())
            app_data["drone_right_m_s"] = float(self.drone_right_m_s_doubleSpinBox.text())
            app_data["drone_down_m_s"] = float(self.drone_down_m_s_doubleSpinBox.text())
            app_data["drone_yawspeed_deg_s"] = float(self.drone_yawspeed_deg_s_doubleSpinBox.text())
            app_data["drone_step_size_m_s"] = float(self.drone_step_size_m_s_doubleSpinBox.text())
            app_data["drone_response_time_s"] = float(self.drone_response_time_s_doubleSpinBox.text())
            # app_data["camera_k"] = json.loads(self.camera_k_lineEdit.text())
            # app_data["camera_dis_coeffs"] = json.loads(self.camera_dis_coeffs_lineEdit.text())
            # 数据从远端读取写到这里面
            self.camera_k_lineEdit.setText(str(app_data["camera_k"]))
            self.camera_dis_coeffs_lineEdit.setText(str(app_data["camera_dis_coeffs"]))
            app_data["limit_height_m"] = float(self.limit_height_m_doubleSpinBox.text())
            app_data["image_and_data_get_url"] = self.image_and_data_get_url_lineEdit.text()
            # 将全局变量中的值写到gui中
            # 目前使用一个二维码
            # TODO 如果有多个，可以进行计算融合
            # "aruco_id_list": [-1 for o in range(20)],  # 当前识别到的aruco的id：0-20
            # "drone_xyz_of_aruco": [[0, 0, 0] for i in range(20)],  # m 在二维码下无人机的位置
            # "drone_xyz_rvec_of_aruco": [[0, 0, 0] for j in range(20)],  # 在二维码下相机的旋转位置，和无人机去向指定坐标有关
            # "aruco_in_camera": [[0, 0] for k in range(20)],  # % 在相机的画面中，二维码的位置，中心点为0，0，右上为正
            self.drone_xyz_of_aruco_label.setText('x:{: <7}m,y:{: <7}m,z:{: <7}m({: <4}ms)'.format(
                *map(lambda x: round(float(x), 4), app_data["drone_xyz_of_aruco"][0]),
                (app_data["time_sub_microseconds"] / 1000).__round__(2)))
            # 在相机中，二维码的位置，目前使用第一个二维码
            self.aruco_in_camera_label.setText('x:{: <7}%,y:{: <7}%'.format(
                *map(lambda x: round(float(x), 4), app_data["aruco_in_camera"][0])))
            self.drone_real_position_label.setText('x:{: <7}m,y:{: <7}m,z:{: <7}m'.format(
                *map(lambda x: round(float(x), 4), app_data["drone_real_position"])))
            self.drone_altitude_label.setText('{: <7}m'.format(float(app_data["drone_altitude"]).__round__(3)))
            self.test_connect_status_label.setText(app_data["test_connect_status"])
            self.drone_control_xy_label.setText('x:{: <7}m/s\ny:{: <7}m/s'.format(float(app_data["drone_forward_m_s"]),
                                                                                  float(app_data["drone_right_m_s"])))
        except:
            self.print_log("数值填入不合法")


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
