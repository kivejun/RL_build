#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import numpy as np  # 导入 numpy

def visualize_data():
    # 1. 路径设置
    data_dir = os.path.expanduser("~/flight_dataset")
    
    if not os.path.exists(data_dir):
        print(f"错误: 找不到文件夹 {data_dir}")
        return

    # 2. 列出文件
    files = sorted([f for f in os.listdir(data_dir) if f.endswith('.csv')], reverse=True)
    if not files:
        print("文件夹内没有 CSV 数据文件。")
        return

    print("\n" + "="*30)
    for i, f in enumerate(files):
        print(f"{i}: {f}")
    print("="*30)
    
    try:
        idx = int(input("请选择要可视化的文件编号: "))
        selected_file = os.path.join(data_dir, files[idx])
    except:
        print("输入无效。")
        return

    # 3. 读取数据
    df = pd.read_csv(selected_file)
    
    # 将关键列提前转换为 numpy 数组，彻底避免索引问题
    pos_x = df['pos_x'].values
    pos_y = df['pos_y'].values
    pos_z = df['pos_z'].values
    t_rel = (df['timestamp'] - df['timestamp'][0]).values
    
    roll = df['roll'].values
    pitch = df['pitch'].values
    yaw = df['yaw'].values
    
    vx = df['vel_x'].values
    vy = df['vel_y'].values
    vz = df['vel_z'].values

    # 4. 创建画布
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f"UAV Trajectory Analysis: {files[idx]}", fontsize=16)

    # --- 子图 1: 3D 轨迹图 (使用 .values 修复报错) ---
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(pos_x, pos_y, pos_z, label='Trajectory', color='blue', lw=2)
    ax1.scatter(pos_x[0], pos_y[0], pos_z[0], color='green', s=100, label='Start')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Flight Path')
    ax1.legend()

    # --- 子图 2: 动作空间 (速度指令) ---
    ax2 = fig.add_subplot(222)
    ax2.plot(t_rel, vx, label='Cmd_Vel_X')
    ax2.plot(t_rel, vy, label='Cmd_Vel_Y')
    ax2.plot(t_rel, vz, label='Cmd_Vel_Z')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Action Space (Velocity)')
    ax2.legend()
    ax2.grid(True)

    # --- 子图 3: 状态空间 (姿态角) ---
    ax3 = fig.add_subplot(223)
    ax3.plot(t_rel, roll, label='Roll')
    ax3.plot(t_rel, pitch, label='Pitch')
    ax3.plot(t_rel, yaw, label='Yaw')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (rad)')
    ax3.set_title('State Space (Attitude)')
    ax3.legend()
    ax3.grid(True)

    # --- 子图 4: XY 平面投影 ---
    ax4 = fig.add_subplot(224)
    ax4.plot(pos_x, pos_y, color='red')
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_title('Top View (XY Projection)')
    ax4.axis('equal')
    ax4.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    visualize_data()