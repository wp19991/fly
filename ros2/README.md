参考教程：https://fishros.com/d2lros2/#/


创建功能包
ros2 pkg create my_pkg  --build-type  ament_python --dependencies gazebo_ros rclpy
ament_python表示创建一个使用python变成的ros2包
dependencies表示需要使用到的ros依赖

打开话题订阅情况图
rqt
选择Introspection->Node Graph即可看到当前话题的订阅情况

可以实现ros2的服务节点
把数据传给服务节点，服务节点处理后会返回处理后的数据给当前节点
- https://fishros.com/d2lros2/#/humble/chapt3/get_started/6.服务之RCLPY实现

launch启动文件
- launch文件允许我们同时启动和配置多个包含 ROS 2 节点的可执行文件
- https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.启动管理工具-Launch
- https://www.robotsfan.com/posts/7a5950c4.html

rosbag录制数据
- 将话题数据存储为文件 ，后续我们无需启动节点，直接可以将bag文件里的话题数据发布出来。
- 可以录制视频数据，后面无需启动模拟，直接播放之前录制好的视频，在订阅节点里面可以看到。用于验证算法等。
- https://fishros.com/d2lros2/#/humble/chapt5/get_started/5.数据录播工具-rosbag

gazebo仿真
- 使用`gazebo /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world`运行仿真后，可以看到场景中有无人车，然后使用`ros2 topic list -t`可以看到发布的命令，使用对应的话题，可以控制小车移动
- Gazebo是独立于ROS/ROS2的软件（还有很多仿真软件可以用ROS/ROS2）
- ROS2和Gazebo之间的桥梁是：gazebo_ros_pkgs
- https://fishros.com/d2lros2/#/humble/chapt5/get_started/6.兼容仿真工具-Gazebo

加载多个模型
- https://fishros.com/d2lros2/#/humble/chapt9/get_started/3.在Gazebo加载机器人模型



