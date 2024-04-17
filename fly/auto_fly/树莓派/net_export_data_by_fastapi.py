import threading
import time

import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.encoders import jsonable_encoder
from fastapi.responses import JSONResponse

import cv2

app = FastAPI()

app_data = {
    "camera_k": [[391.95377974, 0.0, 335.17043033],
                 [0.0, 377.71297362, 245.03757622],
                 [0.0, 0.0, 1.0]],  # 相机的内参k
    "camera_dis_coeffs": [0.04714445, -0.07145486, 0.00588382, 0.00876541, 0.07204812],  # 相机的内参dis_coeffs
    "drone_xyz_of_aruco": [[0, 0, 0]],  # m 在二维码下无人机的位置
    "drone_xyz_rvec_of_aruco": [[0, 0, 0]],  # 在二维码下相机的旋转位置，和无人机去向指定坐标有关
    "aruco_in_camera": [[0, 0]],  # % 在相机的画面中，二维码的位置，中心点为0，0，右上为正
    # 下面的参数如果获取到也可以进行更新
    "drone_real_position": [0, 0, 0],  # 无人机的真实位置
    "drone_real_orientation": [0, 0, 0, 0],  # 无人机的真实旋转向量
}

# 识别二维码的初始化
parameters = cv2.aruco.DetectorParameters()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def main_for_video():
    global app_data
    cap = cv2.VideoCapture(0)
    while True:
        time.sleep(0.1)
        ret, img = cap.read()
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
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.052,
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
        else:
            # 如果没有及那个参数设置为0
            app_data["aruco_in_camera"] = [[0, 0]]
            app_data['drone_xyz_of_aruco'] = [[0, 0, 0]]
            app_data['drone_xyz_rvec_of_aruco'] = [[0, 0, 0]]


@app.get('/app_data')
async def drone_data():
    global app_data
    json_compatible_item_data = jsonable_encoder(app_data)
    return JSONResponse(content=json_compatible_item_data)


if __name__ == '__main__':
    # 启动摄像头进程
    threading.Thread(target=main_for_video).start()
    uvicorn.run(app, host='0.0.0.0', port=8000)
