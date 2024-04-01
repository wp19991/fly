import random
import time

from pymavlink import mavutil

"""
PX4配置(1.14.x):
MAV_1_CONFIG   TELEM 2
重启飞控
MAV_1_MODE    Normal
SER_TEL2_BAUD   115200 8N1
SENS_FLOW_ROT  No rotation
EKF2_OF_CTRL   Enabled
EKF2_RNG_CTRL   Enabled
EKF2_HGT_REF   Range sensor
"""

# 创建一个UDP连接
master = mavutil.mavlink_connection('udpin:localhost:14550')
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

print('connect success')
while True:
    # OPTICAL_FLOW_RAD信息
    time_usec = 0  # 时间戳（微秒）
    sensor_id = 0  # 传感器ID
    integration_time_us = 1000000  # 积分时间（微秒）
    integrated_x = random.uniform(-5, 0)  # 光流x向量（rad）
    integrated_y = random.uniform(-5, 0)  # 光流y向量（rad）
    integrated_xgyro = 0  # 积分陀螺仪x细分
    integrated_ygyro = 0  # 积分陀螺仪y细分
    integrated_zgyro = 0  # 积分陀螺仪z细分
    temperature = int(20.0 * 100)  # 温度（摄氏度*100）
    quality = 255  # 图像质量度量, [0,255]
    time_delta_distance_us = 10000000  # 时间戳（微秒） - 最后一次距离传感器更新
    distance = 1  # 距离传感器的距离，以米为单位
    print(integrated_x, integrated_y, distance)

    # 发送OPTICAL_FLOW_RAD消息
    master.mav.optical_flow_rad_send(
        time_usec,
        sensor_id,
        integration_time_us,
        integrated_x,
        integrated_y,
        integrated_xgyro,
        integrated_ygyro,
        integrated_zgyro,
        temperature,
        quality,
        time_delta_distance_us,
        distance)

    # 每次发送消息后，程序休眠一秒
    time.sleep(1)
