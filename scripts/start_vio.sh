#!/bin/bash

# 设置窗口标题
TITLE="XTDrone VINS-Fusion One-Click Start"

# 1. 启动 PX4 仿真环境 (在第一个标签页)
gnome-terminal --window --title="$TITLE" -- bash -c "roslaunch px4 indoor5.launch; exec bash"

# 等待几秒确保 PX4 启动完毕（根据电脑性能调整）
sleep 8

# 2. 运行 VIO 脚本
gnome-terminal --tab --title="VIO Script" -- bash -c "cd ~/catkin_ws && bash scripts/xtdrone_run_vio.sh; exec bash"

sleep 3

# 3. 运行 VINS 坐标转换
gnome-terminal --tab --title="VINS Transfer" -- bash -c "cd ~/XTDrone/sensing/slam/vio && python vins_transfer.py iris 0; exec bash"

sleep 3
# 4. 运行多旋翼通信脚本
gnome-terminal --tab --title="Communication" -- bash -c "cd ~/XTDrone/communication && python multirotor_communication.py iris 0; exec bash"

sleep 3
# 5. 运行键盘控制 (通常放在最后，方便直接操作)
gnome-terminal --tab --title="Keyboard Control" -- bash -c "cd ~/XTDrone/control/keyboard && python multirotor_keyboard_control.py iris 1 vel; exec bash"

echo "VINS-Fusion 仿真环境已尝试全面启动。"