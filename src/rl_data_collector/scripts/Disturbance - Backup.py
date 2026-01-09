#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import csv
import os
import time
import random
import threading
from geometry_msgs.msg import Twist, PoseStamped, Pose
from mavros_msgs.msg import State
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

class GPSAerobaticCollectorV9:
    def __init__(self):
        rospy.init_node('expert_collector_node')
        
        # 命名空间与参数
        self.uav_id = "iris_0"
        self.xtdrone_ns = "/xtdrone/" + self.uav_id
        self.mavros_ns = "/" + self.uav_id
        self.target_alt = 2.3
        self.is_collecting = False 

        # --- 闭环控制增益 (P 控制) ---
        self.Kp_pos = 1.5  # 位置纠偏强度，值越大纠偏越快
        
        # --- 扰动参数 ---
        self.perturb_duration = 0.6
        self.perturb_magnitude = 2.5  # 增强扰动，让纠偏数据更明显
        self.perturb_cooldown = 5.0
        self.last_perturb_time = 0

        # ROS 发布与订阅
        self.pos_pub = rospy.Publisher(self.xtdrone_ns + "/cmd_pose_enu", Pose, queue_size=10)
        self.vel_pub = rospy.Publisher(self.xtdrone_ns + "/cmd_vel_flu", Twist, queue_size=10)
        self.cmd_pub = rospy.Publisher(self.xtdrone_ns + "/cmd", String, queue_size=10)
        self.current_state = State()
        self.current_pose = PoseStamped()
        rospy.Subscriber(self.mavros_ns + "/mavros/state", State, self.state_cb)
        rospy.Subscriber(self.mavros_ns + "/mavros/local_position/pose", PoseStamped, self.pose_cb)
        
        self.rate = rospy.Rate(30)
        self.save_path = os.path.expanduser("~/flight_dataset")
        if not os.path.exists(self.save_path): os.makedirs(self.save_path)
        threading.Thread(target=self.maintain_hover_audit, daemon=True).start()

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg

    def maintain_hover_audit(self):
        hover_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if not self.is_collecting and self.current_state.armed:
                target = Pose()
                target.position.z = self.target_alt
                self.pos_pub.publish(target)
            hover_rate.sleep()

    def run(self):
        # 1. 解锁起飞逻辑 (与 V7 保持一致)
        while not rospy.is_shutdown() and not self.current_state.connected: self.rate.sleep()
        rospy.loginfo("等待解锁...")
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD": self.cmd_pub.publish("OFFBOARD")
            if not self.current_state.armed: self.cmd_pub.publish("ARM")
            if self.current_pose.pose.position.z > (self.target_alt - 0.3): break
            takeoff_p = Pose()
            takeoff_p.position.z = self.target_alt
            self.pos_pub.publish(takeoff_p)
            self.rate.sleep()

        # 2. 采集主循环
        while not rospy.is_shutdown():
            print("\n" + "="*40)
            print("【闭环纠偏采集 V9】1:直线 | 2:正方形 | 3:S形 | q:着陆")
            mode = input("输入编号: ")
            if mode.lower() == 'q': self.cmd_pub.publish("AUTO.LAND"); break
            
            filename = f"Task_{mode}_Recovery_{time.strftime('%H%M%S')}.csv"
            file_full_path = os.path.join(self.save_path, filename)
            self.is_collecting = True

            with open(file_full_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'pos_x', 'pos_y', 'pos_z', 'roll', 'pitch', 'yaw', 'vel_x', 'vel_y', 'vel_z', 'is_perturbed'])
                
                start_time = rospy.get_time()
                perturb_end_time = 0
                noise_vx, noise_vy = 0, 0

                while not rospy.is_shutdown() and (rospy.get_time() - start_time < 40):
                    now = rospy.get_time()
                    t = now - start_time
                    
                    # --- A. 定义理想参考轨迹 (Reference Trajectory) ---
                    ref_x, ref_y, ref_z = 0, 0, self.target_alt
                    if mode == '1': # 直线轨迹：随时间增加 X
                        ref_x = 1.5 * t; ref_y = 0
                    elif mode == '2': # 正方形轨迹
                        side = 6.0; period = 16.0
                        if t % period < 4: ref_x = 1.5 * (t%4); ref_y = 0
                        elif t % period < 8: ref_x = side; ref_y = 1.5 * (t%4)
                        elif t % period < 12: ref_x = side - 1.5 * (t%4); ref_y = side
                        else: ref_x = 0; ref_y = side - 1.5 * (t%4)
                    elif mode == '3': # S形轨迹
                        ref_x = 1.2 * t; ref_y = 2.5 * math.sin(0.6 * t)

                    # --- B. 闭环专家指令计算 (核心纠偏逻辑) ---
                    curr_pos = self.current_pose.pose.position
                    # 计算位置偏差
                    err_x = ref_x - curr_pos.x
                    err_y = ref_y - curr_pos.y
                    err_z = ref_z - curr_pos.z
                    
                    # 专家根据偏差给出纠偏速度 (P控制)
                    cmd_expert = Twist()
                    cmd_expert.linear.x = self.Kp_pos * err_x
                    cmd_expert.linear.y = self.Kp_pos * err_y
                    cmd_expert.linear.z = self.Kp_pos * err_z
                    
                    # 限速，防止纠偏动作过猛
                    def limit(val, l): return max(min(val, l), -l)
                    cmd_expert.linear.x = limit(cmd_expert.linear.x, 3.0)
                    cmd_expert.linear.y = limit(cmd_expert.linear.y, 3.0)

                    # --- C. 扰动注入 ---
                    is_perturbed = False
                    if now > perturb_end_time and (now - self.last_perturb_time) > self.perturb_cooldown:
                        if random.random() < 0.01:
                            is_perturbed = True
                            perturb_end_time = now + self.perturb_duration
                            self.last_perturb_time = perturb_end_time
                            noise_vx = random.uniform(-1, 1) * self.perturb_magnitude
                            noise_vy = random.uniform(-1, 1) * self.perturb_magnitude
                            rospy.logwarn(">>> 扰动发生！")

                    # 发布指令
                    cmd_final = Twist()
                    if now < perturb_end_time:
                        is_perturbed = True
                        cmd_final.linear.x = cmd_expert.linear.x + noise_vx
                        cmd_final.linear.y = cmd_expert.linear.y + noise_vy
                        cmd_final.linear.z = cmd_expert.linear.z
                    else:
                        is_perturbed = False
                        cmd_final = cmd_expert

                    self.vel_pub.publish(cmd_final)

                    # --- D. 记录数据 ---
                    ori = self.current_pose.pose.orientation
                    (r, p, y) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
                    writer.writerow([now, curr_pos.x, curr_pos.y, curr_pos.z, r, p, y, 
                                     cmd_expert.linear.x, cmd_expert.linear.y, cmd_expert.linear.z, 
                                     1 if is_perturbed else 0])
                    self.rate.sleep()
            
            self.is_collecting = False
            rospy.loginfo("采集结束。")

if __name__ == '__main__':
    try: GPSAerobaticCollectorV9().run()
    except rospy.ROSInterruptException: pass