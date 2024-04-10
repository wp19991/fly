- 读取ros2的topic不能使用conda环境

```bash
# 不能在conda或者虚拟环境中运行下面的代码
# conda deactivate
# 需要使用python3.8，ubuntu2004默认是python38，不需要改动
# 安装库
pip install opencv-python opencv-contrib-python cv_bridge
```

## 运行px4模拟

```bash
cd ./PX4-Autopilot/
make px4_sitl gazebo-classic_iris_downward_depth_camera__empty
pkill -f px4

commander takeoff
commander land

# 查看ros2的topic
ros2 topic list
# 打开摄像头信息
# ros2 run image_view image_view image:=/camera/image_raw

# 使用python读取tag的topic
conda deactivate
python3 search_tag.py
```

- 下面的命令是用ros2启动模拟的，还在修改
```bash
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
/opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
```


## 进行控制起飞

```bash
cd fly
python keyboard_fly.py
```

windows下运行`keybpard_fly.py`，需要先去`https://github.com/mavlink/mavsdk/releases` 下载mavsdk的服务器
运行`./mavsdk_server -p 50051`
然后在代码中修改`drone = System(mavsdk_server_address='localhost', port=50051)`


