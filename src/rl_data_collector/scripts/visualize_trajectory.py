#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import numpy as np

def visualize_data():
    data_dir = os.path.expanduser("~/flight_dataset")
    
    if not os.path.exists(data_dir):
        print(f"错误: 找不到文件夹 {data_dir}")
        return

    files = sorted([f for f in os.listdir(data_dir) if f.endswith('.csv')])
    if not files:
        print("文件夹内没有 CSV 数据文件。")
        return

    print("="*30)
    for i, f in enumerate(files):
        print(f"{i}: {f}")
    print("="*30)
    
    try:
        idx = int(input("请选择要可视化的文件编号: "))
        selected_file = os.path.join(data_dir, files[idx])
    except:
        print("输入无效。")
        return

    # 读取数据
    df = pd.read_csv(selected_file)

    # 【核心修复】将数据显式转换为 numpy 数组，避免 Pandas 索引报错
    x, y, z = df['x'].values, df['y'].values, df['z'].values
    vx, vy, vz = df['vx'].values, df['vy'].values, df['vz'].values
    r, p, yaw = df['roll'].values, df['pitch'].values, df['yaw'].values
    t = df['timestamp'].values - df['timestamp'].values[0]

    # 创建画布
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(f"Flight Data Analysis: {files[idx]}", fontsize=16)

    # --- 子图 1: 3D 轨迹图 ---
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(x, y, z, label='Trajectory', color='b', lw=2)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Flight Path')
    
    # --- 子图 2: 速度指令 (Action) ---
    ax2 = fig.add_subplot(222)
    ax2.plot(t, vx, label='vx')
    ax2.plot(t, vy, label='vy')
    ax2.plot(t, vz, label='vz')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('m/s')
    ax2.set_title('Expert Actions (Velocity)')
    ax2.legend()
    ax2.grid(True)

    # --- 子图 3: 姿态状态 (State) ---
    ax3 = fig.add_subplot(223)
    ax3.plot(t, r, label='roll')
    ax3.plot(t, p, label='pitch')
    ax3.plot(t, yaw, label='yaw')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('rad')
    ax3.set_title('Drone Attitude')
    ax3.legend()
    ax3.grid(True)

    # --- 子图 4: XY 平面投影 ---
    ax4 = fig.add_subplot(224)
    ax4.plot(x, y, color='r')
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_title('Top View (XY)')
    ax4.axis('equal')
    ax4.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    visualize_data()