# fly

px4相关

## tpis

- 下载`https://www.gyan.dev/ffmpeg/builds/ffmpeg-release-essentials.zip`

```bash
# 视频处理
ffmpeg -i w.mp4 -c:v libx264 -c:a aac w_safe.mp4
```

## 演示

- 模拟
- ![模拟.png](doc/1.png)

- 键盘控制飞行_带摄像头画面
- ![键盘控制飞行_带摄像头画面.png](doc/2.png)

- 键盘控制飞行_带摄像头画面_地面有二维码
- ![键盘控制飞行_带摄像头画面_地面有二维码.png](doc/3.png)

- 识别地面aruco
- ![识别地面aruco.png](doc/4.png)

- 室内环境搭建，识别aruco
- ![室内_无风_地面有二维码_识别aruco](doc/5.png)

- 室内无人机自主飞行到二维码上空5m处，悬停
- ![自主飞行到二维码上空5m悬停_有风](doc/6.png)

- 通过aruco计算的位置与真实的位置误差
- ![通过aruco计算的位置与真实的位置误差](doc/7.png)