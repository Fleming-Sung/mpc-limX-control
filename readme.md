# 基于MPC的双足机器人步态控制基线算法实现
# MPC based biped robot locomotion baseline

该项目基于limX（逐际动力）TRON1机器人实现了一种基于MPC的控制算法。该项目以代码简洁、结构清晰为目标，可以作为相关研究和实验的基础，或作为各类改进算法的对比基线。

项目使用limX提供的SDk进行仿真或真机部署。该项目理论上同时支持ROS1和ROS2,但并未在ROS1上经过测试。
本项目默认在ubuntu22+ROS2 humble环境下进行测试。如使用其他Ubuntu发行版或ROS版本，可自行调整环境配置相关shell命令和编译文件中的版本指令。

## 1. 使用说明

Step1：创建工作空间
'''
mkdir -p ~/workspace/mpc_limX_ws/src
'''

Step2: clone本项目代码和limX提供的sdk和仿真平台
'''
cd ~/workspace/mpc_limX_ws/src

git clone https://github.com/Fleming-Sung/mpc-limX-control.git
git clone https://github.com/limxdynamics/robot-description.git
git clone https://github.com/limxdynamics/pointfoot-sdk-lowlevel.git
git clone https://github.com/limxdynamics/robot-visualization.git
git clone -b feature/humble https://github.com/limxdynamics/pointfoot-gazebo-ros2.git
'''
上述clone的5个项目中，第一个是本项目,后4个均为limX提供的接口和仿真平台相关代码。详情可参考：[limX：  pointfoot-gazebo-ros2
](https://github.com/limxdynamics/pointfoot-gazebo-ros2/tree/feature/humble)

请确保clone得到的5个文件夹并列放置于‘<你的工作空间>/src/’目录下。

Step3：安装必备的库和工具包
'''
sudo apt update
sudo apt install ros-humble-urdf \
                 ros-humble-urdfdom \
                 ros-humble-urdfdom-headers \
                 ros-humble-kdl-parser \
                 ros-humble-hardware-interface \
                 ros-humble-controller-manager \
                 ros-humble-controller-interface \
                 ros-humble-controller-manager-msgs \
                 ros-humble-control-msgs \
                 ros-humble-controller-interface \
                 ros-humble-gazebo-* \
                 ros-humble-rqt-gui \
                 ros-humble-rqt-robot-steering \
                 ros-humble-plotjuggler* \
                 ros-humble-rviz* \
                 ros-humble-control-toolbox \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-robot-controllers \
                 ros-humble-robot-controllers-interface \
                 ros-humble-robot-controllers-msgs \
                 ros-dev-tools \
                 cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                 python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
'''

Step4 设置机器人型号环境变量
'''
echo 'export ROBOT_TYPE=PF_P441C' >> ~/.bashrc && source ~/.bashrc
'''
注意：设置为你自己的实际机器人型号

Step5 编译
'''
cd ~/workspace/mpc_limX_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
'''

Step6 运行
TODO：运行自己的包和节点
'''
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
source install/setup.bash
ros2 launch pointfoot_gazebo empty_world.launch.py
'''
