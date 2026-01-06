#!/bin/bash

# ==========================================
# XTDrone 视觉 SLAM 仿真一键启动脚本
# ==========================================

# 设置工作空间路径 
WORKSPACE=~/catkin_ws
PX4_PATH=~/PX4_Firmware
XTDRONE_COMM_PATH=~/XTDrone/communication # 假设 XTDrone 通信脚本在此目录
ORB_SLAM3_ROS_PATH=$WORKSPACE/src/ORB_SLAM3/Examples/ROS/ORB_SLAM3

if [ ! -d "$ORB_SLAM3_ROS_PATH" ]; then
    echo "错误: 找不到 ORB_SLAM3 ROS 包路径: $ORB_SLAM3_ROS_PATH"
    echo "请检查你的文件管理器，确认 package.xml 在哪个文件夹里。"
    exit 1
fi

source $PX4_PATH/Tools/setup_gazebo.bash $PX4_PATH $PX4_PATH/build/px4_sitl_default > /dev/null
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH/Tools/sitl_gazebo

# 1. 启动仿真环境 (Gazebo + PX4)
gnome-terminal --window --title="Gazebo Simulation" -- bash -c "
source $WORKSPACE/devel/setup.bash;
source $PX4_PATH/Tools/setup_gazebo.bash $PX4_PATH $PX4_PATH/build/px4_sitl_default;
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_PATH;
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_PATH/Tools/sitl_gazebo;
roslaunch px4 outdoor3.launch;
exec bash"

echo "正在等待仿真环境加载 (10秒)..."
sleep 10

# 2. 启动 MAVROS 通信节点
gnome-terminal --window --title="MAVROS Communication" -- bash -c "
cd $XTDRONE_COMM_PATH;
python3 multirotor_communication.py solo 0;
exec bash"

# 等待通信链路建立
sleep 2

# 3. 启动 ORB-SLAM3 视觉节点 (带重映射)
gnome-terminal --window --title="ORB-SLAM3" -- bash -c "
source $WORKSPACE/devel/setup.bash;
# 核心修正：手动添加 ORB_SLAM3 的 ROS 路径
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$ORB_SLAM3_ROS_PATH;
rosrun ORB_SLAM3 Stereo \
$WORKSPACE/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
$WORKSPACE/src/ORB_SLAM3/Examples/Stereo-Inertial/px4_sitl.yaml \
true \
/camera/left/image_raw:=/solo_0/stereo_camera/left/image_raw \
/camera/right/image_raw:=/solo_0/stereo_camera/right/image_raw;
exec bash"

# 4. 启动键盘控制节点
gnome-terminal --window --title="Keyboard Control" -- bash -c "
cd ~/XTDrone/control/keyboard;
python3 multirotor_keyboard_control.py solo 1 vel;
exec bash"

echo "所有节点已尝试启动。"
