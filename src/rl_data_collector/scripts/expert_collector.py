#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import csv
import os
import time
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import euler_from_quaternion

class AerobaticAutoLoop:
    def __init__(self):
        rospy.init_node('aerobatic_auto_loop')
        
        self.ns = "/iris_0"
        self.target_alt = 1.0  # 目标高度 1.0 米

        # --- 1. 话题发布者 ---
        self.pos_pub = rospy.Publisher(self.ns + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher(self.ns + "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        
        # --- 2. 状态订阅 ---
        self.current_state = State()
        self.current_pose = PoseStamped()
        rospy.Subscriber(self.ns + "/mavros/state", State, self.state_cb)
        rospy.Subscriber(self.ns + "/mavros/local_position/pose", PoseStamped, self.pose_cb)
        
        self.arming_client = rospy.ServiceProxy(self.ns + '/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy(self.ns + '/mavros/set_mode', SetMode)
        
        self.rate = rospy.Rate(30)

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg

    def return_to_origin(self):
        """执行返航逻辑：飞回 (0,0,1)"""
        rospy.loginfo(">>> 正在飞回原点 (0,0,1)...")
        target = PoseStamped()
        target.pose.position.x = 0
        target.pose.position.y = 0
        target.pose.position.z = self.target_alt
        
        while not rospy.is_shutdown():
            # 计算距离原点的水平距离
            dist = math.sqrt(self.current_pose.pose.position.x**2 + self.current_pose.pose.position.y**2)
            if dist < 0.2: # 如果距离原点小于 20 厘米，认为到达
                rospy.loginfo(">>> 已到达原点附近，准备下一次任务。")
                # 停留 1 秒稳一稳
                for _ in range(30):
                    self.pos_pub.publish(target)
                    self.rate.sleep()
                break
            
            self.pos_pub.publish(target)
            self.rate.sleep()

    def run(self):
        # 初始连接
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("等待 FCU 连接...")
            self.rate.sleep()

        # --- 初始起飞阶段 ---
        rospy.loginfo("发送初始位姿并尝试起飞...")
        takeoff_pose = PoseStamped()
        takeoff_pose.pose.position.z = self.target_alt
        
        for _ in range(100): self.pos_pub.publish(takeoff_pose); self.rate.sleep()
        
        # 强制解锁并进入 OFFBOARD
        while not rospy.is_shutdown() and (self.current_state.mode != "OFFBOARD" or not self.current_state.armed):
            self.set_mode_client(custom_mode="OFFBOARD")
            self.arming_client(True)
            self.pos_pub.publish(takeoff_pose)
            self.rate.sleep()
            if self.current_pose.pose.position.z > (self.target_alt - 0.2): break

        # --- 大循环开始 ---
        while not rospy.is_shutdown():
            print("\n" + "="*40)
            print("可用特技模式: ")
            print("1: 直线 | 2: 正方形 | 3: S形 | 4: 螺旋 | q: 退出并着陆")
            mode = input("请选择要执行的动作编号: ")
            print("="*40)

            if mode.lower() == 'q':
                rospy.loginfo("执行着陆指令...")
                self.set_mode_client(custom_mode="AUTO.LAND")
                break

            if mode not in ['1', '2', '3', '4']:
                print("输入无效，请重新选择。")
                continue

            # 开始采集
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            data_dir = os.path.expanduser("~/flight_dataset")
            if not os.path.exists(data_dir): os.makedirs(data_dir)
            file_path = os.path.join(data_dir, f"Mode{mode}_{timestamp}.csv")
            
            rospy.loginfo(f"开始执行模式 {mode}，数据保存至: {os.path.basename(file_path)}")
            
            with open(file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'label', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz', 'wz'])
                
                start_time = rospy.get_time()
                try:
                    while not rospy.is_shutdown():
                        t = rospy.get_time() - start_time
                        cmd = Twist()
                        
                        # 高度维持 P 控制
                        alt_err = self.target_alt - self.current_pose.pose.position.z
                        cmd.linear.z = 0.8 * alt_err 

                        # 动作逻辑
                        if mode == '1': # 直线
                            cmd.linear.x = 2.0
                        elif mode == '2': # 正方形
                            if t % 16 < 4: cmd.linear.x = 1.5
                            elif t % 16 < 8: cmd.linear.y = 1.5
                            elif t % 16 < 12: cmd.linear.x = -1.5
                            else: cmd.linear.y = -1.5
                        elif mode == '3': # S形
                            cmd.linear.x = 1.5
                            cmd.linear.y = 2.0 * math.sin(0.8 * t)
                        elif mode == '4': # 螺旋
                            r, om = 2.5, 0.7
                            cmd.linear.x = -r * om * math.sin(om * t)
                            cmd.linear.y = r * om * math.cos(om * t)
                            cmd.linear.z += 0.2

                        self.vel_pub.publish(cmd)

                        # 记录状态
                        q = self.current_pose.pose.orientation
                        r, p, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                        writer.writerow([rospy.get_time(), mode, 
                                         self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z,
                                         r, p, yaw, cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z])
                        
                        self.rate.sleep()
                        if t > 30: break # 每个动作执行 30 秒
                except KeyboardInterrupt:
                    rospy.loginfo("手动中断当前动作。")
            
            # 动作结束，飞回原点
            self.return_to_origin()

if __name__ == '__main__':
    try:
        AerobaticAutoLoop().run()
    except rospy.ROSInterruptException:
        pass