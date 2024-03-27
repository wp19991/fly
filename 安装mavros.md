需要先安装ros2，然后安装mavros和mavlink，之后连接usb，可以运行下面的代码

## 前置安装

- 使用ubuntu-22.04.4-desktop-amd64.iso
- https://ubuntu.com/download/desktop

```bash
# 使用root模式
sudo -i
# 更新镜像源
cat <<'EOF' > /etc/apt/sources.list
# 默认注释了源码镜像以提高 apt update 速度，如有需要可自行取消注释
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse

deb http://security.ubuntu.com/ubuntu/ jammy-security main restricted universe multiverse
# deb-src http://security.ubuntu.com/ubuntu/ jammy-security main restricted universe multiverse

# 预发布软件源，不建议启用
# deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-proposed main restricted universe multiverse
# # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-proposed main restricted universe multiverse
EOF

apt update
apt install vim git proxychains openssh-server curl -y
# 使用代理
vim /etc/proxychains.conf
http	192.168.1.81	7890

# 安装conda环境在root模式下
wget https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-py310_24.1.2-0-Linux-x86_64.sh
chmod +x Miniconda3-py310_24.1.2-0-Linux-x86_64.sh
./Miniconda3-py310_24.1.2-0-Linux-x86_64.sh

# 配置conda镜像源
cat <<'EOF' > ~/.condarc
channels:
  - defaults
show_channel_urls: true
default_channels:
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/msys2
custom_channels:
  conda-forge: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  msys2: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  bioconda: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  menpo: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  pytorch: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  pytorch-lts: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  simpleitk: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
  deepmodeling: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/
EOF

# 在root模式下使用conda环境
python -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple --upgrade pip
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
# 退出root模式
exit

# 下载qgc
# https://github.com/mavlink/QGroundControl/releases
proxychains wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.3.0/QGroundControl.AppImage
mv QGroundControl.AppImage qgc.AppImage
chmod +x qgc..AppImage
```

## 安装ros2和mavros mavlink

- https://docs.px4.io/main/zh/ros/ros2_comm.html#%E5%AE%89%E8%A3%85-ros-2
- https://cloudkerneltech.gitbook.io/kerlouduav/userguide-zh/tutorial-zh/offboard_python_zh

```bash
sudo -i
# 添加ros源
apt install software-properties-common
add-apt-repository universe

# 使用代理下载文件：sudo proxychains curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 使用国内镜像源 https://mirrors.tuna.tsinghua.edu.cn/help/ros2/
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update

# 进行安装
apt install ros-humble-desktop ros-dev-tools ros-humble-mavros ros-humble-mavlink -y
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

exit
```

## 测试usb连接代码，需要安装mavsdk的库

- https://mavsdk.mavlink.io/main/en/python/quickstart.html

```bash
# 必须要在root模式下才可以启动
sudo -i
# 当前是conda,pyhton3.10环境
pip install mavsdk aioconsole
vim t_arm.py
python t_arm.py
```

```python
import asyncio

from mavsdk import System


async def run():
    # Connect to the drone
    drone = System()
    # 端口使用`ls /dev/serial/by-id`进行查看
    await drone.connect(system_address="serial:///dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00:57600")
    # 模拟仿真使用下面的
    # await drone.connect(system_address="udp://:14540")

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
