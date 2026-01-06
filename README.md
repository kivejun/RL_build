# XTDrone Simulation Workspace (catkin_ws)

本项目是一个基于 **XTDrone** 的无人机仿真开发工作空间，旨在实现无人机在 Gazebo 环境下的多机仿真、视觉 SLAM 定位、物体识别及闭环控制。

---

## 1. 项目概览 (Project Overview)

本工作空间主要集成了以下功能模块：
* **仿真引擎**: Gazebo + PX4 (SITL)
* **通信中间件**: MAVROS (连接 PX4 与 ROS)
* **感知模块**: ORB-SLAM3 (视觉定位) & YOLOv5 (物体识别) and so on

---

## 2. 环境要求 (Prerequisites)

| 组件 | 要求版本 |
| :--- | :--- |
| **操作系统** | Ubuntu 20.04 LTS |
| **ROS** | Noetic |
| **仿真器** | Gazebo 11 |
| **飞控固件** | PX4 v1.13.0+ |
| **Python** | 3.8+ (需安装 numpy, pyyaml, ros-noetic-mavros) |

---

## 3. 工作空间结构 (Workspace Structure)

```text
catkin_ws/
├── src/
│   ├── XTDrone/               # XTDrone 源码 (通信、控制、场景)
│   ├── ORB_SLAM3/             # 修改后的 ORB-SLAM3 ROS 插件
│   ├── mavros/                # MAVROS 功能包
│   └── (其他传感器插件)
├── scripts/                   # 自动化运行脚本 (sh)
└── README.md                  # 本文档


