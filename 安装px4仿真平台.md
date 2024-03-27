## 安装模拟软件

- 安装环境ubuntu2204
- 先进行安装mavros

```bash
# 使用root模式
sudo -i
# 安装px4
proxychains git clone https://github.com/PX4/PX4-Autopilot.git --recursive
proxychains bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# 安装gz-garden模拟软件，ubuntu2204只支持gz-garden，不能安装Gazebo-Classic
# https://docs.px4.io/main/zh/sim_gazebo_gz/
proxychains wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
apt update
apt install gz-garden
exit

# 启动模拟
cd ./PX4-Autopilot/
# https://docs.px4.io/main/zh/sim_gazebo_gz/
make px4_sitl gz_x500_vision

commander takeoff
commander land
```

## 仿真流程

```bash
# 运行模拟仿真
cd ./PX4-Autopilot/
make px4_sitl gz_x500_vision
# 此时会暴露udp://:14540和显示gz-garden模拟软件
# 现在可以运行t_arm.py的命令，模拟软件中的无人机可以正常启动
# 编写python指令，可以控制无人机的运行状态
```

- 可能可以使用pymavlink库来连接无人机？

## 官方推荐的控制流程

> uXRCE-DDS的中间件由运行在PX4上的客户端(Client)和运行在机载计算机上的代理端(Agent)组成， 通过串口、UDP、TCP或其他链路实现双向数据互联。
> 代理端(Agent)充当客户端(Client)的代理在DDS全局数据空间中发布和订阅话题。
> - https://docs.px4.io/main/zh/ros/ros2_comm.html#setup-micro-xrce-dds-agent-client

- 用户编写c/c++语言的uORB的topic === Micro-XRCE-DDS-Agent === ros2 === 电脑 === 无人机
- 使用Micro-XRCE-DDS-Agent来控制无人机
- 运行后会暴露udp:8888端口
- 可以使用编写uORB的topic来进行对无人机的控制

```bash
# 安装Micro-XRCE-DDS-Agent，不能在root模式下，在用户模式下
proxychains git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
proxychains make -j12
sudo make install
sudo ldconfig /usr/local/lib/

# 启动代理并设置以连接运行在模拟器上的 uXRCE-DDS客户端(Client)：
MicroXRCEAgent udp4 -p 8888
```