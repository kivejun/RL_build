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

class GPSAerobaticCollectorV10:
    def __init__(self):
        rospy.init_node('expert_collector_node')
        
        self.uav_id = "iris_0"
        self.xtdrone_ns = "/xtdrone/" + self.uav_id
        self.mavros_ns = "/" + self.uav_id
        self.target_alt = 2.3
        self.is_collecting = False 

        # --- 强扰动参数配置 ---
        self.perturb_chance = 0.02      # 触发概率提升 (3%)
        self.perturb_duration = 0.5     # 扰动持续时间
        self.perturb_magnitude = 3.5    # 扰动最大强度 (m/s)，显著增强
        self.perturb_cooldown = 3.5     # 冷却时间缩短
        self.last_perturb_time = 0

        # --- 闭环纠偏增益 ---
        self.Kp_pos = 1.8               # 增强纠偏力度，保证能拉回来

        # 发布与订阅
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
        # 起飞逻辑 (保持 V7 成功版本)
        while not rospy.is_shutdown() and not self.current_state.connected: self.rate.sleep()
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD": self.cmd_pub.publish("OFFBOARD")
            if not self.current_state.armed: self.cmd_pub.publish("ARM")
            if self.current_pose.pose.position.z > (self.target_alt - 0.3): break
            p = Pose(); p.position.z = self.target_alt
            self.pos_pub.publish(p); self.rate.sleep()

        while not rospy.is_shutdown():
            print("\n" + "="*40)
            print("【全向扰动采集 V10】1:直线 | 2:正方形 | 3:S形 | q:着陆")
            mode = input("编号: ")
            if mode.lower() == 'q': self.cmd_pub.publish("AUTO.LAND"); break
            
            filename = f"Task_{mode}_OmniPerturb_{time.strftime('%H%M%S')}.csv"
            file_full_path = os.path.join(self.save_path, filename)
            self.is_collecting = True

            with open(file_full_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'pos_x', 'pos_y', 'pos_z', 'roll', 'pitch', 'yaw', 'vel_x', 'vel_y', 'vel_z', 'is_perturbed'])
                
                start_time = rospy.get_time()
                perturb_end_time = 0
                noise_vec = [0.0, 0.0, 0.0]

                while not rospy.is_shutdown() and (rospy.get_time() - start_time < 40):
                    now = rospy.get_time()
                    t = now - start_time
                    
                    # 1. 理想轨迹计算 (保持不变)
                    ref_x, ref_y, ref_z = 0, 0, self.target_alt
                    if mode == '1': ref_x = 1.5 * t
                    elif mode == '2':
                        s, p = 6.0, 16.0
                        if t%p<4: ref_x=1.5*(t%4); ref_y=0
                        elif t%p<8: ref_x=s; ref_y=1.5*(t%4)
                        elif t%p<12: ref_x=s-1.5*(t%4); ref_y=s
                        else: ref_x=0; ref_y=s-1.5*(t%4)
                    elif mode == '3': ref_x = 1.2 * t; ref_y = 2.5 * math.sin(0.6 * t)

                    # 2. 闭环专家指令 (P控制)
                    cp = self.current_pose.pose.position
                    cmd_expert = Twist()
                    cmd_expert.linear.x = self.Kp_pos * (ref_x - cp.x)
                    cmd_expert.linear.y = self.Kp_pos * (ref_y - cp.y)
                    cmd_expert.linear.z = self.Kp_pos * (ref_z - cp.z)

                    # 3. 全向随机扰动逻辑
                    is_p = False
                    if now > perturb_end_time and (now - self.last_perturb_time) > self.perturb_cooldown:
                        if random.random() < self.perturb_chance:
                            is_p = True
                            perturb_end_time = now + self.perturb_duration
                            self.last_perturb_time = perturb_end_time
                            # 生成三维球形空间内的随机扰动向量
                            noise_vec = [random.uniform(-1, 1) * self.perturb_magnitude for _ in range(3)]
                            rospy.logwarn(f">>> 注入全向扰动: X:{noise_vec[0]:.1f} Y:{noise_vec[1]:.1f} Z:{noise_vec[2]:.1f}")

                    # 4. 执行动作
                    cmd_final = Twist()
                    if now < perturb_end_time:
                        is_p = True
                        cmd_final.linear.x = cmd_expert.linear.x + noise_vec[0]
                        cmd_final.linear.y = cmd_expert.linear.y + noise_vec[1]
                        cmd_final.linear.z = cmd_expert.linear.z + noise_vec[2]
                    else:
                        is_p = False
                        cmd_final = cmd_expert

                    self.vel_pub.publish(cmd_final)

                    # 5. 记录数据
                    ori = self.current_pose.pose.orientation
                    (r, p, y) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
                    writer.writerow([now, cp.x, cp.y, cp.z, r, p, y, 
                                     cmd_expert.linear.x, cmd_expert.linear.y, cmd_expert.linear.z, 
                                     1 if is_p else 0])
                    self.rate.sleep()
            
            self.is_collecting = False
            rospy.loginfo("采集结束。")

if __name__ == '__main__':
    try: GPSAerobaticCollectorV10().run()
    except rospy.ROSInterruptException: pass