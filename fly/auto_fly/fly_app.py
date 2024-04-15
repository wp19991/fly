import asyncio
import datetime
import sys
import os

from PyQt5 import QtCore
from qasync import QEventLoop, QApplication, asyncSlot
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QTimer

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
    "drone_altitude": 0.,  # 无人机的气压计高度
    "limit_height_m": 1.,
    "test_connect_status": "status",  # 连接成功，连接失败，可以启动，不能启动
    "drone_is_running": False,  # 无人机是否起飞，当为不起飞的是否，跳出运行的while循环
}

drone1: System = System()


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
                    self.print_log(f"当前气压计高度: {position.relative_altitude_m.__round__(2): <4}m")
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

    def print_log(self, text):
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
        app_data["camera_k"] = self.camera_k_lineEdit.text()
        app_data["camera_dis_coeffs"] = self.camera_dis_coeffs_lineEdit.text()
        app_data["limit_height_m"] = float(self.limit_height_m_doubleSpinBox.text())
        # 将全局变量中的值写到gui中
        # 目前使用一个二维码
        # TODO 如果有多个，可以进行计算融合
        self.drone_xyz_of_aruco_label.setText('x:{: <4}m,y:{: <4}m,z:{: <4}m'.format(
            *map(lambda x: round(float(x), 4), app_data["drone_xyz_of_aruco"][0])))
        # 在相机中，二维码的位置，目前使用第一个二维码
        self.aruco_in_camera_label.setText('x:{: <4}%,y:{: <4}%'.format(
            *map(lambda x: round(float(x), 4), app_data["aruco_in_camera"][0])))
        self.drone_real_position_label.setText('x:{: <4}%,y:{: <4}%,z:{: <4}m'.format(
            *map(lambda x: round(float(x), 4), app_data["drone_real_position"])))
        self.drone_altitude_label.setText('{: <4}m'.format(float(app_data["drone_altitude"])))
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