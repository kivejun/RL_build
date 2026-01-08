#!/bin/bash

# 1. 设置路径变量 (请确保这是你电脑上的真实路径)
FIRMWARE_DIR="/home/kivejun/PX4_Firmware"

# 2. 构建完整的 ROS_PACKAGE_PATH
# 我们需要包含 Firmware 本身，以及它的 Tools/sitl_gazebo 目录
PX4_PACKAGES="$FIRMWARE_DIR:$FIRMWARE_DIR/Tools/sitl_gazebo"

# 3. 启动 PX4 仿真环境
gnome-terminal --window --title="PX4 SITL" -- bash -c "\
source ~/.bashrc; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_PACKAGES; \
roslaunch px4 indoor5.launch; \
exec bash"

# 等待仿真环境完全加载
sleep 10

# 4. 运行 VIO 脚本
gnome-terminal --tab --title="VIO Script" -- bash -c "source ~/.bashrc; cd ~/catkin_ws && bash scripts/xtdrone_run_vio.sh; exec bash"

sleep 3

# 5. 运行 VINS 坐标转换
gnome-terminal --tab --title="VINS Transfer" -- bash -c "source ~/.bashrc; cd ~/XTDrone/sensing/slam/vio && python vins_transfer.py iris 0; exec bash"

sleep 3

# 6. 运行多旋翼通信脚本
gnome-terminal --tab --title="Communication" -- bash -c "source ~/.bashrc; cd ~/XTDrone/communication && python multirotor_communication.py iris 0; exec bash"

sleep 3

# 7. 运行采集脚本
gnome-terminal --tab --title="DATA_COLLECTOR" -- bash -c "source ~/.bashrc; cd ~/catkin_ws/src/rl_data_collector/scripts && python3 expert_collector.py; exec bash"

echo "VINS-Fusion 环境及采集脚本已全面启动。"