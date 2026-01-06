# XTDrone 视觉 SLAM 仿真部署手册 (ORB-SLAM3)

本仓库包含了在 **XTDrone** 仿真环境下部署 **ORB-SLAM3** 的核心配置、脚本以及避坑指南。适用于双目（Stereo）及双目惯性（Stereo-Inertial）定位方案。
（*不建议在ubuntu 20.04 部署ORB-SLAM2，过于老了，依赖冲突*）
---

## 1. 环境构建 (Compilation)

在编译 ROS 插件前，必须确保基础依赖库完整。

### 1.1 依赖安装
* **Sophus**: 核心李代数库。若编译报错 `sophus/se3.hpp` 缺失，请执行：
    ```bash
    sudo apt-get install libsophus-dev
    ```
* **OpenCV**: 推荐使用 **4.2.0** (Ubuntu 20.04 默认版本)。
```bash
find_package(OpenCV 4.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.2 not found.")
   endif()
   ```
   *要将默认的OpenCV 4.4 改为 4.2*

*依赖安装见（[链接](https://gitee.com/robin_shaun/XTDrone/tree/master/sensing/slam/vslam/ORB_SLAM3)）*

### 1.2 编译流程
1.  **编译基础库**: 运行 `ORB_SLAM3` 根目录下的 `./build.sh`。
2.  **编译 ROS 插件**: 运行 `./build_ros.sh`。
    * **注意**: 编译前需确认工作空间已初始化 `rosdep`。

---

## 2. 配置文件指南 (YAML Config)

**注意：** 不要直接使用 EuRoC 数据集配置文件跑仿真，必须使用针对 Gazebo 环境适配的 YAML。
*EuRoC YAML文件无法匹配iris_0 无人机模型，这里我选择带双目相机的solo_stereo_camera，并编写px4_sitl.yaml文件*

### 2.1 核心矩阵参数
* **双目校正矩阵 ($P$ / $R$)**: 运行双目节点时，必须在 YAML 中包含 `LEFT.P`, `RIGHT.P` 等矩阵，否则会报 `Calibration missing` 错误。
* **IMU 外参 ($T_{bc}$)**: 运行 `Stereo_Inertial` 模式必须提供相机到 IMU 的变换矩阵。若 `Tbc` 为空，程序会直接触发 **Segmentation fault (段错误)**。

### 2.2 话题订阅 (Topic Matching)
确保 YAML 末尾的话题名称与你的模型命名空间一致。
> **Solo 无人机示例配置：**
> * `Camera.Topic: "/solo_0/stereo_camera/left/image_raw"`
> * `Camera2.Topic: "/solo_0/stereo_camera/right/image_raw"`
> * `IMU.Topic: "/solo_0/mavros/imu/data"`

*这里可以运行`rostopic list`来查看自己的话题*

---

## 3. 仿真运行 (Execution)

### 3.1 启动顺序
1.  启动 Gazebo 并加载带有 `stereo_camera` 的无人机模型（如 `solo` 或 `iris_stereo_camera`）。
```bash
roslaunch px4 outdoor3.launch
```
*如何设置加载不同的无人机模型，附上XTDrone的[官方使用文档](https://www.yuque.com/xtdrone/manual_cn/vehicle_config)*
2.  启动 MAVROS 通信节点。
```bash
python multirotor_communication.py solo 0
```
3.  启动 SLAM 脚本：`sh xtdrone_run_stereo.sh`
4.  可以使用键盘控制无人机运动（可选择）
```bash
python multirotor_keyboard_control.py solo 1 vel
```



### 3.2 强行重映射运行
若 YAML 话题配置不生效，推荐使用命令行重映射参数：
```bash
rosrun ORB_SLAM3 Stereo \
Vocabulary/ORBvoc.txt \
Examples/Stereo-Inertial/px4_sitl.yaml \
true \
/camera/left/image_raw:=/solo_0/stereo_camera/left/image_raw \
/camera/right/image_raw:=/solo_0/stereo_camera/right/image_raw
```