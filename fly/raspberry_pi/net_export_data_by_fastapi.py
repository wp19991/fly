import base64
import datetime
import threading

import cv2
import numpy as np

import uvicorn
from fastapi import FastAPI
from fastapi.encoders import jsonable_encoder
from fastapi.responses import PlainTextResponse
from starlette.responses import JSONResponse

app = FastAPI()

app_data = {
    "aruco_length_m": 0.052,  # aruco实际的的大小边长
    "aruco_id_in_real_map": [[0, 0] for _ in range(22)],  # 对应二维码在现实中对应的位置，为了多个二维码进行计算融合，指定一个原点
    "aruco_id_list": [-1 for _ in range(22)],  # 当前识别到的aruco的id：0-20
    "drone_xyz_of_aruco": [[0, 0, 0] for _ in range(22)],  # m 在二维码下无人机的位置
    "drone_xyz_rvec_of_aruco": [[0, 0, 0] for _ in range(22)],  # 在二维码下相机的旋转位置，和无人机去向指定坐标有关
    "aruco_in_camera": [[0, 0] for _ in range(22)],  # % 在相机的画面中，二维码的位置，中心点为0，0，右上为正
    "time_sub_microseconds": 0.,  # 从得到的是否到完成识别的时间差值
    "get_img_time": f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
    "get_img_aruco_time_stamp": f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
    # 下面的参数如果获取到也可以进行更新
    "drone_real_position": [0, 0, 0],  # 无人机的真实位置
    "drone_real_orientation": [0, 0, 0, 0],  # 无人机的真实旋转向量
    # 相机的参数
    "camera_k": [[391.95377974, 0.0, 335.17043033], [0.0, 377.71297362, 245.03757622], [0.0, 0.0, 1.0]],  # 相机的内参k
    "camera_dis_coeffs": [0.04714445, -0.07145486, 0.00588382, 0.00876541, 0.07204812],  # 相机的内参dis_coeffs
}

# 识别二维码的初始化
parameters = cv2.aruco.DetectorParameters()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
video_frame = b""


def main_for_video():
    global app_data, video_frame
    cap = cv2.VideoCapture(0)
    while True:
        # time.sleep(0.1)
        start_time = datetime.datetime.now()
        app_data["get_img_time"] = f"{start_time.strftime('%Y-%m-%d %H:%M:%S')}"
        ret, img = cap.read()
        # 识别二维码
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 获取图像的框，id，rejectedImgPoints
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # 画框
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        if len(corners) > 0:
            for i in range(len(corners)):
                # 二维码在相机画面中的位置，中心点为0，0，右上为正
                # 假设corners是检测到的第一个aruco marker的角点位置
                corner = corners[i][0]
                # 计算二维码的中心点位置
                center_marker = np.mean(corner, axis=0)
                # 获取图片的大小
                height, width = gray.shape
                # 计算图片的中心点位置
                center_image = np.array([width / 2, height / 2])
                # 计算二维码中心相对于图片中心的位置，并进行归一化，使得在边框处为-1或1，中心为0
                aruco_in_camera: np.array = 2 * (center_marker - center_image) / np.array([width, height])
                aruco_in_camera = aruco_in_camera.tolist()
                aruco_in_camera[1] = -aruco_in_camera[1]
                # 获取识别后的坐标
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i],
                                                                    app_data["aruco_length_m"],  # 二维码的实际大小m
                                                                    np.array(app_data["camera_k"]),
                                                                    np.array(app_data["camera_dis_coeffs"]))
                # 画轴
                cv2.drawFrameAxes(img, np.array(app_data["camera_k"]), np.array(app_data["camera_dis_coeffs"]),
                                  rvec[0, :, :], tvec[0, :, :], 0.03)
                # 更新参数
                app_data["aruco_in_camera"].insert(i, aruco_in_camera)
                app_data["drone_xyz_of_aruco"].insert(i, tvec[0, :, :][0].tolist())
                app_data["drone_xyz_rvec_of_aruco"].insert(i, rvec[0, :, :][0].tolist())
                app_data["aruco_id_list"].insert(i, int(ids[i]))
        else:
            # 如果没有识别到二维码，参数设置为默认值
            app_data["aruco_in_camera"] = [[0, 0] for _ in range(22)]
            app_data['drone_xyz_of_aruco'] = [[0, 0, 0] for _ in range(22)]
            app_data['drone_xyz_rvec_of_aruco'] = [[0, 0, 0] for _ in range(22)]
            app_data["aruco_id_list"] = [-1 for _ in range(22)]
        end_time = datetime.datetime.now()
        app_data["get_img_aruco_time_stamp"] = f"{end_time.strftime('%Y-%m-%d %H:%M:%S')}"
        app_data["time_sub_microseconds"] = (end_time - start_time).microseconds
        video_frame = img.tobytes()
        # if app_data["aruco_in_camera"][0][0] != 0:
        #     print(f"[{datetime.datetime.now().strftime('[%Y-%m-%d %H:%M:%S]')}]"
        #           f" -- get video frame size: {len(video_frame)} , find aruco : {app_data['aruco_in_camera']}")
        # else:
        #     print(f"[{datetime.datetime.now().strftime('[%Y-%m-%d %H:%M:%S]')}]"
        #           f" -- get video frame size: {len(video_frame)}")


@app.get('/app_data')
def drone_data():
    global app_data
    json_compatible_item_data = jsonable_encoder(app_data)
    return JSONResponse(content=json_compatible_item_data)


@app.get('/video_frame')
def drone_video_frame_data():
    global video_frame
    frame_encoded_string = base64.b64encode(video_frame).decode()
    return PlainTextResponse(content=frame_encoded_string)


if __name__ == '__main__':
    # 启动摄像头进程
    threading.Thread(target=main_for_video).start()
    uvicorn.run(app, host='0.0.0.0', port=8000)
