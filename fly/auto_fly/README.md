## 编写控制程序

```bash
conda create --name pyqt5_env python=3.10
conda activate pyqt5_env
pip install pyqt5 pyqt5-tools

C:\DevelopmentTools\Miniconda3\miniconda3\envs\pyqt5_env\Scripts\pyrcc5.exe res/res.qrc -o res/res_rc.py
C:\DevelopmentTools\Miniconda3\miniconda3\envs\pyqt5_env\Scripts\pyuic5.exe fly_gui.ui -o fly_gui.py  --import-from=res
```

## 实现功能

- [x] 在gui中控制无人机前后左右飞行
- [x] 设置无人机起飞高度
- [x] 设置无人机飞行的启动参数
- [x] 设置打印消息，或者标签显示消息
- [x] 显示ros2传回的画面展示到gui中
- [x] 识别二维码
- [x] 实现自动悬停

允许设置使用ros连接查看摄像头或者直接通过opencv连接远程摄像头
ros连接摄像头可能需要系统安装ros，可以在运行摄像头的电脑上使用vlc来将摄像头发布为网络摄像头，那样在windows下就不需要安装ros2以及其他依赖
需要测试延迟

### 使用mavsdk远程连接控制，设置连接参数

https://github.com/mavlink/MAVSDK/releases

- 控制模拟环境中的无人机

```bash
# windows下下载：mavsdk-windows-x64-release.zip
# 模拟环境使用下面的命令启动
# MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)
MAV_1_BROADCAST=1 make px4_sitl gazebo-classic_iris_downward_depth_camera
# 使用下面的命令连接到远程的模拟环境中的无人机
./mavsdk_server_musl_x86_64 -p 50051 udp://:14580
# 启动网络转发ros2图像和真实位置


# cd C:\Users\Admin\Desktop\fly\file\mavsdk-windows-x64-release\bin
# mavsdk_server_bin.exe -p 50051 udp://192.168.1.216:14580
```

- 控制现实中的无人机

```bash
# 在ubuntu下执行下面的
./mavsdk_server_musl_x86_64 -p 50051 serial:///dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00 

# 然后在外部，使用python代码就可以连接上这个无人机
drone = System(mavsdk_server_address='192.168.1.216', port=50051)
await drone.connect()

# 可以在一个代码里面控制多个无人机
drone1 = System(mavsdk_server_address='192.168.1.216', port=50052)
await drone1.connect()
```

### 通过opencv连接远程摄像头

问题0:
opencv连接远程的摄像头
gpt-4:
OpenCV可以通过rtsp或http协议连接远程摄像头。此处以rtsp为例，示例代码如下：

```python
import cv2

# 这里的URL应该替换为你的远程摄像头的rtsp地址
url = 'rtsp://username:password@your_camera_ip:port/path'
cap = cv2.VideoCapture(url)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    # 显示图像
    cv2.imshow('Remote Video', frame)
    # 按Q键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
```

请将上述代码中的url变量替换为你的远程摄像头RTSP流地址，username和password是你的摄像头登录用户名和密码，your_camera_ip是你的摄像头IP地址，port是你的摄像头端口号，path是视频流路径。
如果使用HTTP协议获取视频流，同样可以使用`cv2.VideoCapture()`函数来创建一个视频捕获对象。只需将上述代码中的url替换为HTTP流地址即可。例如：

```python
url = 'http://your_camera_ip:port/video'
```

注意：以上两个方法都需要你确定摄像头支持RTSP或HTTP协议，并且知道正确的流地址。如果不确定，请参考摄像头制造商的说明书或者联系他们获得支持。
---------------2024-04-15 10:24:39----------------
问题1:
如何将电脑摄像头转换为rtsp协议，在ubuntu下
gpt-4:
首先，Ubuntu系统中需要安装VLC播放器，如果没有的话可以通过以下命令安装：

```shell
sudo apt-get install vlc
```

然后可以通过以下命令将电脑的摄像头转为RTSP流：

```shell
cvlc v4l2:///dev/video0:width=640:height=480 :input-slave=alsa://hw:0,0 --sout "#transcode{vcodec=h264,vb=800,scale=1,acodec=mpga,ab=128,channels=2,samplerate=44100}:rtp{sdp=rtsp://:8554/live.ts}"
```

其中，"v4l2:///dev/video0"是电脑默认的摄像头设备，"width=640:height=480"是设置分辨率，"--sout"
后面的参数是设置视频编解码方式和音频编解码方式以及RTSP流地址。
这样运行之后，在其他设备上就可以通过访问 "rtsp://your_ip_address:8554/live.ts" 来获取到这个rtsp流了。
注意：你可能需要根据你的实际情况修改上述命令中的参数。如果你不确定，你可以查看VLC的文档或者在线寻找帮助。


