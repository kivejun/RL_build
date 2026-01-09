#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import csv
import os
import time
import threading
from geometry_msgs.msg import Twist, PoseStamped, Pose
from mavros_msgs.msg import State
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

class GPSAerobaticCollectorV7:
    def __init__(self):
        rospy.init_node('expert_collector_node')
        
        # 命名空间配置
        self.uav_id = "iris_0"
        self.xtdrone_ns = "/xtdrone/" + self.uav_id
        self.mavros_ns = "/" + self.uav_id
        
        self.target_alt = 2.3
        self.is_collecting = False 

        # --- 发布者 ---
        self.pos_pub = rospy.Publisher(self.xtdrone_ns + "/cmd_pose_enu", Pose, queue_size=10)
        self.vel_pub = rospy.Publisher(self.xtdrone_ns + "/cmd_vel_flu", Twist, queue_size=10)
        self.cmd_pub = rospy.Publisher(self.xtdrone_ns + "/cmd", String, queue_size=10)
        
        # --- 状态订阅 ---
        self.current_state = State()
        self.current_pose = PoseStamped()
        rospy.Subscriber(self.mavros_ns + "/mavros/state", State, self.state_cb)
        rospy.Subscriber(self.mavros_ns + "/mavros/local_position/pose", PoseStamped, self.pose_cb)
        
        self.rate = rospy.Rate(30)
        self.prev_z_err = 0

        # 数据保存路径
        self.save_path = os.path.expanduser("~/flight_dataset")
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            rospy.loginfo(f"创建数据集目录: {self.save_path}")

        # 后台心跳线程
        self.heartbeat_thread = threading.Thread(target=self.maintain_hover_audit)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg

    def maintain_hover_audit(self):
        hover_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if not self.is_collecting and self.current_state.armed:
                target = Pose()
                target.position.x = 0
                target.position.y = 0
                target.position.z = self.target_alt
                self.pos_pub.publish(target)
            hover_rate.sleep()

    def run(self):
        # 1. 等待连接
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("等待 MAVROS 与 FCU 连接...")
            self.rate.sleep()

        # 2. 解锁起飞
        rospy.loginfo("请求解锁并进入 OFFBOARD 模式...")
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD":
                self.cmd_pub.publish("OFFBOARD")
            if not self.current_state.armed:
                self.cmd_pub.publish("ARM")
            if self.current_pose.pose.position.z > (self.target_alt - 0.3):
                rospy.loginfo(">>> 起飞完成，已平稳悬停")
                break
            takeoff_p = Pose()
            takeoff_p.position.z = self.target_alt
            self.pos_pub.publish(takeoff_p)
            self.rate.sleep()

        # 3. 主循环
        while not rospy.is_shutdown():
            print("\n" + "="*40)
            print(f"【数据集采集模式】当前高度: {self.current_pose.pose.position.z:.2f}m")
            print("1:直线 | 2:正方形 | 3:S形 | 4:螺旋 | q:着陆")
            mode = input("请输入采集任务编号: ")
            
            if mode.lower() == 'q':
                self.cmd_pub.publish("AUTO.LAND")
                break
            if mode not in ['1', '2', '3', '4']: continue

            # 准备 CSV 文件
            filename = f"Task_{mode}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            file_full_path = os.path.join(self.save_path, filename)
            
            self.is_collecting = True
            rospy.loginfo(f"开始采集任务 {mode}，数据将保存至: {filename}")

            with open(file_full_path, 'w', newline='') as f:
                writer = csv.writer(f)
                # 写入表头：时间戳, 标签, 状态(x,y,z,r,p,y), 动作(vx,vy,vz,wz)
                writer.writerow(['timestamp', 'label', 'pos_x', 'pos_y', 'pos_z', 
                                 'roll', 'pitch', 'yaw', 'vel_x', 'vel_y', 'vel_z', 'ang_z'])

                start_time = rospy.get_time()
                self.prev_z_err = 0
                
                try:
                    while not rospy.is_shutdown() and (rospy.get_time() - start_time < 30):
                        t = rospy.get_time() - start_time
                        cmd = Twist()
                        
                        # --- 1. 计算特技控制逻辑 (动作生成) ---
                        curr_z = self.current_pose.pose.position.z
                        alt_err = self.target_alt - curr_z
                        cmd.linear.z = 2.0 * alt_err + 0.3 * (alt_err - self.prev_z_err)
                        self.prev_z_err = alt_err

                        if mode == '1': cmd.linear.x = 2.0
                        elif mode == '2':
                            if t % 16 < 4: cmd.linear.x = 1.5
                            elif t % 16 < 8: cmd.linear.y = 1.5
                            elif t % 16 < 12: cmd.linear.x = -1.5
                            else: cmd.linear.y = -1.5
                        elif mode == '3':
                            cmd.linear.x = 1.5; cmd.linear.y = 2.0 * math.sin(0.8 * t)
                        elif mode == '4':
                            r, om = 2.0, 0.6
                            cmd.linear.x = -r * om * math.sin(om * t)
                            cmd.linear.y = r * om * math.cos(om * t)
                            cmd.linear.z += 0.3 # 螺旋上升

                        # --- 2. 执行动作 ---
                        self.vel_pub.publish(cmd)

                        # --- 3. 获取并解析状态空间 ---
                        pos = self.current_pose.pose.position
                        ori = self.current_pose.pose.orientation
                        # 四元数转欧拉角 (弧度)
                        (r, p, y) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

                        # --- 4. 写入数据 ---
                        writer.writerow([
                            rospy.get_time(), mode, 
                            pos.x, pos.y, pos.z,           # 位置状态
                            r, p, y,                       # 姿态状态
                            cmd.linear.x, cmd.linear.y,    # 动作空间
                            cmd.linear.z, cmd.angular.z
                        ])

                        self.rate.sleep()
                except KeyboardInterrupt:
                    pass
            
            self.is_collecting = False
            rospy.loginfo(f">>> 任务 {mode} 采集完成。")

if __name__ == '__main__':
    try:
        GPSAerobaticCollectorV7().run()
    except rospy.ROSInterruptException:
        pass