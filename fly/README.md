- 读取ros2的topic不能使用conda环境

```bash
# 不能在conda或者虚拟环境中运行下面的代码
conda deactivate
# 需要使用python3.8，ubuntu2004默认是python38，不需要改动
# 安装库
pip install opencv-python opencv-contrib-python cv_bridge
```

## 运行px4模拟

```bash
cd ./PX4-Autopilot/
pkill -f px4
make px4_sitl gazebo-classic_iris_downward_depth_camera__baylands


# 查看ros2的topic
ros2 topic list
# 打开摄像头信息
# ros2 run image_view image_view image:=/camera/image_raw

# 使用python读取tag的topic
conda deactivate
python3 search_tag.py
```

## 进行控制起飞

```bash
cd fly
python keyboard_fly.py
```

## 飞行稳定性

- 如果无人机起飞在位置模式下悬停左右前后小范围飘,对位置误差调整能力弱 ，调整加大速度控制器增益
- MPC_XY_VEL_P_ACC
- MPC_XY_VEL_I_ACC
- https://blog.csdn.net/z1872385/article/details/131700007
- https://docs.px4.io/main/zh/flight_stack/controller_diagrams.html#多旋翼速度控制器

- 修改位置控制器增益，增加这两个值将使无人机对位置偏差有更快的响应，控制无人机在位置控制模式下的响应。
- MPC_XY_P
- MPC_Z_P

- 调整这些参数可能会对飞行性能产生很大影响，在每次修改参数后都进行飞行测试。

