# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'fly_gui.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_fly_app(object):
    def setupUi(self, fly_app):
        fly_app.setObjectName("fly_app")
        fly_app.resize(801, 603)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/ico/无人机.ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        fly_app.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(fly_app)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QtCore.QRect(10, 320, 451, 161))
        self.groupBox_2.setObjectName("groupBox_2")
        self.formLayout_2 = QtWidgets.QFormLayout(self.groupBox_2)
        self.formLayout_2.setObjectName("formLayout_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem)
        self.label_4 = QtWidgets.QLabel(self.groupBox_2)
        self.label_4.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_4.setObjectName("label_4")
        self.verticalLayout_3.addWidget(self.label_4)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem1)
        self.formLayout_2.setLayout(0, QtWidgets.QFormLayout.LabelRole, self.verticalLayout_3)
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_8 = QtWidgets.QLabel(self.groupBox_2)
        self.label_8.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_8.setObjectName("label_8")
        self.gridLayout_2.addWidget(self.label_8, 0, 0, 1, 1)
        self.drone_forward_m_s_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox_2)
        self.drone_forward_m_s_doubleSpinBox.setMinimum(-10.0)
        self.drone_forward_m_s_doubleSpinBox.setMaximum(10.0)
        self.drone_forward_m_s_doubleSpinBox.setSingleStep(0.1)
        self.drone_forward_m_s_doubleSpinBox.setObjectName("drone_forward_m_s_doubleSpinBox")
        self.gridLayout_2.addWidget(self.drone_forward_m_s_doubleSpinBox, 0, 1, 1, 1)
        self.label_16 = QtWidgets.QLabel(self.groupBox_2)
        self.label_16.setObjectName("label_16")
        self.gridLayout_2.addWidget(self.label_16, 0, 2, 1, 1)
        self.label_15 = QtWidgets.QLabel(self.groupBox_2)
        self.label_15.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_15.setObjectName("label_15")
        self.gridLayout_2.addWidget(self.label_15, 0, 3, 1, 1)
        self.drone_right_m_s_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox_2)
        self.drone_right_m_s_doubleSpinBox.setMinimum(-10.0)
        self.drone_right_m_s_doubleSpinBox.setMaximum(10.0)
        self.drone_right_m_s_doubleSpinBox.setSingleStep(0.1)
        self.drone_right_m_s_doubleSpinBox.setObjectName("drone_right_m_s_doubleSpinBox")
        self.gridLayout_2.addWidget(self.drone_right_m_s_doubleSpinBox, 0, 4, 1, 1)
        self.label_17 = QtWidgets.QLabel(self.groupBox_2)
        self.label_17.setObjectName("label_17")
        self.gridLayout_2.addWidget(self.label_17, 0, 5, 1, 1)
        self.label_22 = QtWidgets.QLabel(self.groupBox_2)
        self.label_22.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_22.setObjectName("label_22")
        self.gridLayout_2.addWidget(self.label_22, 1, 0, 1, 1)
        self.drone_down_m_s_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox_2)
        self.drone_down_m_s_doubleSpinBox.setMinimum(-10.0)
        self.drone_down_m_s_doubleSpinBox.setMaximum(10.0)
        self.drone_down_m_s_doubleSpinBox.setSingleStep(0.1)
        self.drone_down_m_s_doubleSpinBox.setProperty("value", -0.9)
        self.drone_down_m_s_doubleSpinBox.setObjectName("drone_down_m_s_doubleSpinBox")
        self.gridLayout_2.addWidget(self.drone_down_m_s_doubleSpinBox, 1, 1, 1, 1)
        self.label_18 = QtWidgets.QLabel(self.groupBox_2)
        self.label_18.setObjectName("label_18")
        self.gridLayout_2.addWidget(self.label_18, 1, 2, 1, 1)
        self.label_23 = QtWidgets.QLabel(self.groupBox_2)
        self.label_23.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_23.setObjectName("label_23")
        self.gridLayout_2.addWidget(self.label_23, 1, 3, 1, 1)
        self.drone_yawspeed_deg_s_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox_2)
        self.drone_yawspeed_deg_s_doubleSpinBox.setMinimum(-10.0)
        self.drone_yawspeed_deg_s_doubleSpinBox.setMaximum(10.0)
        self.drone_yawspeed_deg_s_doubleSpinBox.setSingleStep(0.1)
        self.drone_yawspeed_deg_s_doubleSpinBox.setProperty("value", 0.0)
        self.drone_yawspeed_deg_s_doubleSpinBox.setObjectName("drone_yawspeed_deg_s_doubleSpinBox")
        self.gridLayout_2.addWidget(self.drone_yawspeed_deg_s_doubleSpinBox, 1, 4, 1, 1)
        self.label_19 = QtWidgets.QLabel(self.groupBox_2)
        self.label_19.setObjectName("label_19")
        self.gridLayout_2.addWidget(self.label_19, 1, 5, 1, 1)
        self.formLayout_2.setLayout(0, QtWidgets.QFormLayout.FieldRole, self.gridLayout_2)
        self.label_5 = QtWidgets.QLabel(self.groupBox_2)
        self.label_5.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_5.setObjectName("label_5")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_5)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.drone_step_size_m_s_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox_2)
        self.drone_step_size_m_s_doubleSpinBox.setMinimum(-10.0)
        self.drone_step_size_m_s_doubleSpinBox.setMaximum(10.0)
        self.drone_step_size_m_s_doubleSpinBox.setSingleStep(0.1)
        self.drone_step_size_m_s_doubleSpinBox.setProperty("value", 0.2)
        self.drone_step_size_m_s_doubleSpinBox.setObjectName("drone_step_size_m_s_doubleSpinBox")
        self.horizontalLayout_2.addWidget(self.drone_step_size_m_s_doubleSpinBox)
        self.label_20 = QtWidgets.QLabel(self.groupBox_2)
        self.label_20.setObjectName("label_20")
        self.horizontalLayout_2.addWidget(self.label_20)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem2)
        self.formLayout_2.setLayout(1, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_2)
        self.label_6 = QtWidgets.QLabel(self.groupBox_2)
        self.label_6.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_6.setObjectName("label_6")
        self.formLayout_2.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_6)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.drone_response_time_s_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox_2)
        self.drone_response_time_s_doubleSpinBox.setMinimum(0.0)
        self.drone_response_time_s_doubleSpinBox.setMaximum(10.0)
        self.drone_response_time_s_doubleSpinBox.setSingleStep(0.1)
        self.drone_response_time_s_doubleSpinBox.setProperty("value", 0.2)
        self.drone_response_time_s_doubleSpinBox.setObjectName("drone_response_time_s_doubleSpinBox")
        self.horizontalLayout_3.addWidget(self.drone_response_time_s_doubleSpinBox)
        self.label_21 = QtWidgets.QLabel(self.groupBox_2)
        self.label_21.setObjectName("label_21")
        self.horizontalLayout_3.addWidget(self.label_21)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem3)
        self.formLayout_2.setLayout(2, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_3)
        self.label_24 = QtWidgets.QLabel(self.groupBox_2)
        self.label_24.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_24.setObjectName("label_24")
        self.formLayout_2.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.label_24)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.limit_height_m_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox_2)
        self.limit_height_m_doubleSpinBox.setMinimum(-10.0)
        self.limit_height_m_doubleSpinBox.setMaximum(10.0)
        self.limit_height_m_doubleSpinBox.setSingleStep(0.1)
        self.limit_height_m_doubleSpinBox.setProperty("value", 0.0)
        self.limit_height_m_doubleSpinBox.setObjectName("limit_height_m_doubleSpinBox")
        self.horizontalLayout.addWidget(self.limit_height_m_doubleSpinBox)
        self.label_25 = QtWidgets.QLabel(self.groupBox_2)
        self.label_25.setObjectName("label_25")
        self.horizontalLayout.addWidget(self.label_25)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem4)
        self.formLayout_2.setLayout(3, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout)
        self.groupBox_3 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QtCore.QRect(10, 170, 451, 151))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_3.sizePolicy().hasHeightForWidth())
        self.groupBox_3.setSizePolicy(sizePolicy)
        self.groupBox_3.setObjectName("groupBox_3")
        self.formLayout = QtWidgets.QFormLayout(self.groupBox_3)
        self.formLayout.setObjectName("formLayout")
        self.label_28 = QtWidgets.QLabel(self.groupBox_3)
        self.label_28.setObjectName("label_28")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_28)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.image_and_data_get_url_lineEdit = QtWidgets.QLineEdit(self.groupBox_3)
        self.image_and_data_get_url_lineEdit.setObjectName("image_and_data_get_url_lineEdit")
        self.horizontalLayout_9.addWidget(self.image_and_data_get_url_lineEdit)
        self.test_connect_data_url_pushButton = QtWidgets.QPushButton(self.groupBox_3)
        self.test_connect_data_url_pushButton.setObjectName("test_connect_data_url_pushButton")
        self.horizontalLayout_9.addWidget(self.test_connect_data_url_pushButton)
        self.formLayout.setLayout(0, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_9)
        self.label_7 = QtWidgets.QLabel(self.groupBox_3)
        self.label_7.setObjectName("label_7")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_7)
        self.drone_xyz_of_aruco_label = QtWidgets.QLabel(self.groupBox_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.drone_xyz_of_aruco_label.sizePolicy().hasHeightForWidth())
        self.drone_xyz_of_aruco_label.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("Consolas")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.drone_xyz_of_aruco_label.setFont(font)
        self.drone_xyz_of_aruco_label.setObjectName("drone_xyz_of_aruco_label")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.drone_xyz_of_aruco_label)
        self.label_11 = QtWidgets.QLabel(self.groupBox_3)
        self.label_11.setObjectName("label_11")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_11)
        self.aruco_in_camera_label = QtWidgets.QLabel(self.groupBox_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.aruco_in_camera_label.sizePolicy().hasHeightForWidth())
        self.aruco_in_camera_label.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("Consolas")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.aruco_in_camera_label.setFont(font)
        self.aruco_in_camera_label.setObjectName("aruco_in_camera_label")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.aruco_in_camera_label)
        self.label_9 = QtWidgets.QLabel(self.groupBox_3)
        self.label_9.setObjectName("label_9")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.label_9)
        self.drone_real_position_label = QtWidgets.QLabel(self.groupBox_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.drone_real_position_label.sizePolicy().hasHeightForWidth())
        self.drone_real_position_label.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("Consolas")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.drone_real_position_label.setFont(font)
        self.drone_real_position_label.setObjectName("drone_real_position_label")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.drone_real_position_label)
        self.label_13 = QtWidgets.QLabel(self.groupBox_3)
        self.label_13.setObjectName("label_13")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.label_13)
        self.drone_altitude_label = QtWidgets.QLabel(self.groupBox_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.drone_altitude_label.sizePolicy().hasHeightForWidth())
        self.drone_altitude_label.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("Consolas")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.drone_altitude_label.setFont(font)
        self.drone_altitude_label.setObjectName("drone_altitude_label")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.drone_altitude_label)
        self.groupBox_5 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_5.setGeometry(QtCore.QRect(470, 280, 321, 201))
        self.groupBox_5.setObjectName("groupBox_5")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.groupBox_5)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.drone_takeoff_pushButton = QtWidgets.QPushButton(self.groupBox_5)
        self.drone_takeoff_pushButton.setObjectName("drone_takeoff_pushButton")
        self.verticalLayout_4.addWidget(self.drone_takeoff_pushButton)
        self.takeoff_and_hold_and_land_pushButton = QtWidgets.QPushButton(self.groupBox_5)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.takeoff_and_hold_and_land_pushButton.setFont(font)
        self.takeoff_and_hold_and_land_pushButton.setObjectName("takeoff_and_hold_and_land_pushButton")
        self.verticalLayout_4.addWidget(self.takeoff_and_hold_and_land_pushButton)
        self.horizontalLayout_4.addLayout(self.verticalLayout_4)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        spacerItem5 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem5)
        self.drone_search_aruco_pushButton = QtWidgets.QPushButton(self.groupBox_5)
        self.drone_search_aruco_pushButton.setObjectName("drone_search_aruco_pushButton")
        self.verticalLayout.addWidget(self.drone_search_aruco_pushButton)
        self.drone_search_status_label = QtWidgets.QLabel(self.groupBox_5)
        self.drone_search_status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.drone_search_status_label.setObjectName("drone_search_status_label")
        self.verticalLayout.addWidget(self.drone_search_status_label)
        spacerItem6 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem6)
        self.horizontalLayout_4.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        spacerItem7 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem7)
        self.drone_landing_pushButton = QtWidgets.QPushButton(self.groupBox_5)
        self.drone_landing_pushButton.setObjectName("drone_landing_pushButton")
        self.verticalLayout_2.addWidget(self.drone_landing_pushButton)
        spacerItem8 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem8)
        self.drone_kill_pushButton = QtWidgets.QPushButton(self.groupBox_5)
        self.drone_kill_pushButton.setObjectName("drone_kill_pushButton")
        self.verticalLayout_2.addWidget(self.drone_kill_pushButton)
        spacerItem9 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem9)
        self.horizontalLayout_4.addLayout(self.verticalLayout_2)
        self.verticalLayout_5.addLayout(self.horizontalLayout_4)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.drone_forward_add_pushButton = QtWidgets.QPushButton(self.groupBox_5)
        self.drone_forward_add_pushButton.setObjectName("drone_forward_add_pushButton")
        self.gridLayout.addWidget(self.drone_forward_add_pushButton, 0, 1, 1, 1)
        self.drone_right_sub_pushButton = QtWidgets.QPushButton(self.groupBox_5)
        self.drone_right_sub_pushButton.setObjectName("drone_right_sub_pushButton")
        self.gridLayout.addWidget(self.drone_right_sub_pushButton, 1, 0, 1, 1)
        self.drone_right_add_pushButton = QtWidgets.QPushButton(self.groupBox_5)
        self.drone_right_add_pushButton.setObjectName("drone_right_add_pushButton")
        self.gridLayout.addWidget(self.drone_right_add_pushButton, 1, 2, 1, 1)
        self.drone_forward_sub_pushButton = QtWidgets.QPushButton(self.groupBox_5)
        self.drone_forward_sub_pushButton.setObjectName("drone_forward_sub_pushButton")
        self.gridLayout.addWidget(self.drone_forward_sub_pushButton, 2, 1, 1, 1)
        self.drone_control_xy_label = QtWidgets.QLabel(self.groupBox_5)
        self.drone_control_xy_label.setAlignment(QtCore.Qt.AlignCenter)
        self.drone_control_xy_label.setObjectName("drone_control_xy_label")
        self.gridLayout.addWidget(self.drone_control_xy_label, 1, 1, 1, 1)
        self.label_26 = QtWidgets.QLabel(self.groupBox_5)
        self.label_26.setAlignment(QtCore.Qt.AlignCenter)
        self.label_26.setObjectName("label_26")
        self.gridLayout.addWidget(self.label_26, 0, 2, 1, 1)
        self.label_27 = QtWidgets.QLabel(self.groupBox_5)
        self.label_27.setAlignment(QtCore.Qt.AlignCenter)
        self.label_27.setObjectName("label_27")
        self.gridLayout.addWidget(self.label_27, 2, 0, 1, 1)
        self.verticalLayout_5.addLayout(self.gridLayout)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(470, 20, 320, 240))
        self.label.setObjectName("label")
        self.textBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(10, 490, 781, 101))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.textBrowser.sizePolicy().hasHeightForWidth())
        self.textBrowser.setSizePolicy(sizePolicy)
        self.textBrowser.setObjectName("textBrowser")
        self.get_video_frame_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.get_video_frame_pushButton.setGeometry(QtCore.QRect(370, 240, 81, 23))
        self.get_video_frame_pushButton.setObjectName("get_video_frame_pushButton")
        self.groupBox_4 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QtCore.QRect(10, 10, 451, 81))
        self.groupBox_4.setObjectName("groupBox_4")
        self.formLayout_4 = QtWidgets.QFormLayout(self.groupBox_4)
        self.formLayout_4.setObjectName("formLayout_4")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_10 = QtWidgets.QLabel(self.groupBox_4)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_6.addWidget(self.label_10)
        self.mavsdk_server_address_lineEdit = QtWidgets.QLineEdit(self.groupBox_4)
        self.mavsdk_server_address_lineEdit.setObjectName("mavsdk_server_address_lineEdit")
        self.horizontalLayout_6.addWidget(self.mavsdk_server_address_lineEdit)
        self.formLayout_4.setLayout(0, QtWidgets.QFormLayout.LabelRole, self.horizontalLayout_6)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_12 = QtWidgets.QLabel(self.groupBox_4)
        self.label_12.setObjectName("label_12")
        self.horizontalLayout_5.addWidget(self.label_12)
        self.mavsdk_server_port_lineEdit = QtWidgets.QLineEdit(self.groupBox_4)
        self.mavsdk_server_port_lineEdit.setObjectName("mavsdk_server_port_lineEdit")
        self.horizontalLayout_5.addWidget(self.mavsdk_server_port_lineEdit)
        self.formLayout_4.setLayout(0, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_5)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_14 = QtWidgets.QLabel(self.groupBox_4)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_7.addWidget(self.label_14)
        self.system_address_lineEdit = QtWidgets.QLineEdit(self.groupBox_4)
        self.system_address_lineEdit.setEnabled(False)
        self.system_address_lineEdit.setDragEnabled(False)
        self.system_address_lineEdit.setReadOnly(False)
        self.system_address_lineEdit.setObjectName("system_address_lineEdit")
        self.horizontalLayout_7.addWidget(self.system_address_lineEdit)
        self.formLayout_4.setLayout(1, QtWidgets.QFormLayout.LabelRole, self.horizontalLayout_7)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.test_connect_pushButton = QtWidgets.QPushButton(self.groupBox_4)
        self.test_connect_pushButton.setObjectName("test_connect_pushButton")
        self.horizontalLayout_8.addWidget(self.test_connect_pushButton)
        self.test_connect_status_label = QtWidgets.QLabel(self.groupBox_4)
        self.test_connect_status_label.setObjectName("test_connect_status_label")
        self.horizontalLayout_8.addWidget(self.test_connect_status_label)
        self.formLayout_4.setLayout(1, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_8)
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(10, 90, 451, 76))
        self.groupBox.setObjectName("groupBox")
        self.formLayout_3 = QtWidgets.QFormLayout(self.groupBox)
        self.formLayout_3.setObjectName("formLayout_3")
        self.label_2 = QtWidgets.QLabel(self.groupBox)
        self.label_2.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_2.setIndent(-1)
        self.label_2.setObjectName("label_2")
        self.formLayout_3.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_2)
        self.camera_k_lineEdit = QtWidgets.QLineEdit(self.groupBox)
        self.camera_k_lineEdit.setObjectName("camera_k_lineEdit")
        self.formLayout_3.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.camera_k_lineEdit)
        self.label_3 = QtWidgets.QLabel(self.groupBox)
        self.label_3.setObjectName("label_3")
        self.formLayout_3.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_3)
        self.camera_dis_coeffs_lineEdit = QtWidgets.QLineEdit(self.groupBox)
        self.camera_dis_coeffs_lineEdit.setObjectName("camera_dis_coeffs_lineEdit")
        self.formLayout_3.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.camera_dis_coeffs_lineEdit)
        fly_app.setCentralWidget(self.centralwidget)

        self.retranslateUi(fly_app)
        QtCore.QMetaObject.connectSlotsByName(fly_app)

    def retranslateUi(self, fly_app):
        _translate = QtCore.QCoreApplication.translate
        fly_app.setWindowTitle(_translate("fly_app", "fly_app"))
        self.groupBox_2.setTitle(_translate("fly_app", "无人机飞行控制"))
        self.label_4.setText(_translate("fly_app", "各方向速度"))
        self.label_8.setText(_translate("fly_app", "前"))
        self.label_16.setText(_translate("fly_app", "m/s"))
        self.label_15.setText(_translate("fly_app", "右"))
        self.label_17.setText(_translate("fly_app", "m/s"))
        self.label_22.setText(_translate("fly_app", "下"))
        self.label_18.setText(_translate("fly_app", "m/s"))
        self.label_23.setText(_translate("fly_app", "yaw"))
        self.label_19.setText(_translate("fly_app", "deg/s"))
        self.label_5.setText(_translate("fly_app", "控制移动步长"))
        self.label_20.setText(_translate("fly_app", "m/s"))
        self.label_6.setText(_translate("fly_app", "控制响应时间"))
        self.label_21.setText(_translate("fly_app", "s"))
        self.label_24.setText(_translate("fly_app", "限制高度"))
        self.label_25.setText(_translate("fly_app", "m"))
        self.groupBox_3.setTitle(_translate("fly_app", "响应参数"))
        self.label_28.setToolTip(_translate("fly_app", "树莓派的远程传递数据的端口地址"))
        self.label_28.setText(_translate("fly_app", "data_url"))
        self.image_and_data_get_url_lineEdit.setToolTip(_translate("fly_app", "树莓派的远程传递数据的端口地址"))
        self.image_and_data_get_url_lineEdit.setText(_translate("fly_app", "http://192.168.1.112:8000"))
        self.test_connect_data_url_pushButton.setText(_translate("fly_app", "测试连接"))
        self.label_7.setText(_translate("fly_app", "aruco坐标系下"))
        self.drone_xyz_of_aruco_label.setText(_translate("fly_app", "x:{}m,y:{}m,z:{}m"))
        self.label_11.setText(_translate("fly_app", "aruco相机画面中位置"))
        self.aruco_in_camera_label.setText(_translate("fly_app", "x:{}%,y:{}%"))
        self.label_9.setText(_translate("fly_app", "无人机真实的位置"))
        self.drone_real_position_label.setText(_translate("fly_app", "x:{}m,y:{}m,z:{}m"))
        self.label_13.setText(_translate("fly_app", "无人机气压计高度"))
        self.drone_altitude_label.setText(_translate("fly_app", "z:{}m"))
        self.groupBox_5.setTitle(_translate("fly_app", "飞行控制"))
        self.drone_takeoff_pushButton.setText(_translate("fly_app", "准备起飞"))
        self.takeoff_and_hold_and_land_pushButton.setText(_translate("fly_app", "起飞悬停降落"))
        self.drone_search_aruco_pushButton.setText(_translate("fly_app", "自动悬停"))
        self.drone_search_status_label.setText(_translate("fly_app", "status"))
        self.drone_landing_pushButton.setText(_translate("fly_app", "降落"))
        self.drone_kill_pushButton.setText(_translate("fly_app", "急停关闭"))
        self.drone_forward_add_pushButton.setText(_translate("fly_app", "前进"))
        self.drone_right_sub_pushButton.setText(_translate("fly_app", "向左"))
        self.drone_right_add_pushButton.setText(_translate("fly_app", "向右"))
        self.drone_forward_sub_pushButton.setText(_translate("fly_app", "后退"))
        self.drone_control_xy_label.setText(_translate("fly_app", "x:{}m/s\n"
"y:{}m/s"))
        self.label_26.setText(_translate("fly_app", "前右为正+"))
        self.label_27.setText(_translate("fly_app", "左后为负-"))
        self.label.setText(_translate("fly_app", "opencv-image"))
        self.get_video_frame_pushButton.setText(_translate("fly_app", "获取相机画面"))
        self.groupBox_4.setTitle(_translate("fly_app", "无人机连接参数"))
        self.label_10.setText(_translate("fly_app", "mavsdk_server_address"))
        self.mavsdk_server_address_lineEdit.setText(_translate("fly_app", "192.168.1.112"))
        self.label_12.setText(_translate("fly_app", "port"))
        self.mavsdk_server_port_lineEdit.setText(_translate("fly_app", "50051"))
        self.label_14.setText(_translate("fly_app", "system_address(drop)"))
        self.system_address_lineEdit.setText(_translate("fly_app", "udp://:14540"))
        self.test_connect_pushButton.setText(_translate("fly_app", "测试连接"))
        self.test_connect_status_label.setText(_translate("fly_app", "status"))
        self.groupBox.setTitle(_translate("fly_app", "相机参数设置"))
        self.label_2.setToolTip(_translate("fly_app", "相机的内参k"))
        self.label_2.setText(_translate("fly_app", "k"))
        self.camera_k_lineEdit.setToolTip(_translate("fly_app", "相机的内参k"))
        self.camera_k_lineEdit.setText(_translate("fly_app", "[[391.95377974, 0.0, 335.17043033], [0.0, 377.71297362, 245.03757622], [0.0, 0.0, 1.0]]"))
        self.camera_k_lineEdit.setPlaceholderText(_translate("fly_app", "[[,,],[,,],[,,]]"))
        self.label_3.setToolTip(_translate("fly_app", "相机的内参dis_coeffs"))
        self.label_3.setText(_translate("fly_app", "dist_coeffs"))
        self.camera_dis_coeffs_lineEdit.setToolTip(_translate("fly_app", "相机的内参dis_coeffs"))
        self.camera_dis_coeffs_lineEdit.setText(_translate("fly_app", "[0.04714445, -0.07145486, 0.00588382, 0.00876541, 0.07204812]"))
        self.camera_dis_coeffs_lineEdit.setPlaceholderText(_translate("fly_app", "[,,,,]"))
from res import res_rc
