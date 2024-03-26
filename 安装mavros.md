需要先安装ros2，然后安装mavros和mavlink，之后连接usb，可以运行下面的代码

## 前置安装
```bash
sudo apt update
sudo apt install vim git proxychains openssh-server curl -y
# 使用代理
sudo vim /etc/proxychains.conf
http	192.168.1.81	7890

# 安装conda环境
wget https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-py310_24.1.2-0-Linux-x86_64.sh
chmod +x Miniconda3-py310_24.1.2-0-Linux-x86_64.sh
./Miniconda3-py310_24.1.2-0-Linux-x86_64.sh

python -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple --upgrade pip
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

# 下载qgc
# https://github.com/mavlink/QGroundControl/releases
proxychains wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.3.0/QGroundControl.AppImage
```


## 安装ros2和mavros mavlink
- https://docs.px4.io/main/zh/ros/ros2_comm.html#%E5%AE%89%E8%A3%85-ros-2
- https://cloudkerneltech.gitbook.io/kerlouduav/userguide-zh/tutorial-zh/offboard_python_zh


```bash
# 添加ros源
sudo apt install software-properties-common
sudo add-apt-repository universe

# 使用代理下载文件：sudo proxychains curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# 如果有安装依赖冲突，安装下面的可以解决
# sudo apt install libpulse0=1:15.99.1+dfsg1-1ubuntu1 libpulse-mainloop-glib0=1:15.99.1+dfsg1-1ubuntu1 libusb-1.0-0=2:1.0.25-1ubuntu1
# 进行安装
sudo apt install ros-humble-desktop ros-dev-tools ros-humble-mavros ros-humble-mavlink -y
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```


## 测试usb连接代码，需要安装mavsdk的库
- https://mavsdk.mavlink.io/main/en/python/quickstart.html

```bash
# 必须要在root模式下才可以启动，因此conda可以在root模式下安装

sudo python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple --upgrade pip
sudo pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
sudo pip3 install mavsdk aioconsole


sudo python3 t_arm.py
```

```python
import asyncio
from mavsdk import System

async def run():
    # Connect to the drone
    drone = System()
    # 端口使用`ls /dev/serial/by-id`进行查看
    await drone.connect(system_address="serial:///dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00:57600")
    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone is connected")
            break

    async for is_ready in drone.telemetry.health():
        if is_ready.is_armable:
            print("Drone is ready to arm")
            break
        else:
            print("Drone not ready to arm, please check!")

    print("Arming...")
    await drone.action.arm()
    print("Drone is armed!")
	
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())

```
