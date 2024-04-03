一个node可以订阅或者发布多个ros中的topic话题
订阅用于展示或者收集数据
发布的topic，其他的节点可以对这个topic进行订阅，用于

mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src/
ros2 pkg create read_topic_example --build-type ament_python --dependencies gazebo_ros rclpy

cd read_topic_example/read_topic_example
写文件

cd ~/ros2_ws/
colcon build
source install/setup.bash

发布节点
ros2 run example_topic_rclpy topic_publisher_02

订阅节点
ros2 run example_topic_rclpy topic_subscribe_02

rqt

