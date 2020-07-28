# 配置小车环境

### 开启高性能模式
将小车的工作功耗切换到 10W（使用四个 CPU 核），此命令只需运行一次：
```shell
sudo nvpmodel -m 0
```
使用命令 `nvpmodel -q` 来确认小车是否已切换到 MAXN 模式。切换完成后应该不会再变回去了，可以重启一次确认一下。

### 设置时间以及时区

因为运行在小车上的 ROS 结点会附带时间戳信息，配置好小车的时间有利于测试延迟，以及按照时间来正确匹配图像帧与神经网络的输出。
```shell
sudo apt install ntp
sudo vim /etc/ntp.conf  # 此处建议按照 https://tuna.moe/help/ntp/ 修改，以方便国内使用
sudo systemctl restart ntp
sudo timedatectl set-timezone Asia/Shanghai
date  # 确认输出的时间日期是否正确
```

------

# 安装依赖

### 在小车上
```shell
sudo apt install ros-melodic-image-transport ros-melodic-compressed-image-transport
```
然后克隆 https://github.com/ZeppLu/jetbot_ros 及 https://github.com/ZeppLu/ros_deep_learning ，并用 `catkin_make` 编译。

### 在电脑上
```shell
sudo apt install ros-melodic-image-transport ros-melodic-compressed-image-transport ros-melodic-cv-bridge ros-melodic-message-filters ros-melodic-rqt-image-viewer
```
然后克隆本项目，并用 `catkin_make` 编译。

------

# 运行

### 在小车上
打开三个 ssh 窗口，分别运行（以 ssd-mobilenet-v2 为例，其他模型未测试）：
```shell
roscore
```
```shell
rosrun jetbot_ros jetbot_camera _width:=1280 _height:=720 _framerate:=30.0
```
```shell
rosrun ros_deep_learning detectnet /detectnet/image_in:=/jetbot_camera/raw _model_name:=ssd-mobilenet-v2
```

### 在电脑上
```shell
roslaunch vision_viz detections_viz.launch
```

图片源为 720p 30fps 时，网络大约能以 14fps 的速率输出检测结果（用命令 `rostopic hz /detectnet/detections` 可查看输出速率）。尝试修改 `jetbot_camera` 启动参数也许能提高速率。
