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

## 获取环境位置

修改模型文件，发送在模拟环境中位置的ros消息
修改`/root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_downward_depth_camera/iris_downward_depth_camera.sdf`文件为：

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
    <model name='iris_downward_depth_camera'>
        <include>
            <uri>model://iris</uri>
        </include>

        <include>
            <uri>model://depth_camera</uri>
            <!-- x y z roll pitch yaw-->
            <pose>0.108 0 -0.01 0 1.5708 0</pose>
        </include>
        <joint name="depth_camera_joint" type="revolute">
            <child>depth_camera::link</child>
            <parent>iris::base_link</parent>
            <axis>
                <xyz>0 0 1</xyz>
                <limit>
                    <upper>0</upper>
                    <lower>0</lower>
                </limit>
            </axis>
        </joint>

        <plugin name="ground_truth_odometry" filename="/opt/ros/foxy/lib/libgazebo_ros_p3d.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>10</updateRate>
            <body_name>iris::base_link</body_name>
            <topic_name>odom_comb</topic_name>
            <frameName>map</frameName>
        </plugin>

    </model>
</sdf>
```

