#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import pandas as pd
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import tf.transformations as tf
import os # 导入os库

class FrenetPlanner:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('frenet_planner_node', anonymous=True)

        # --- 从参数服务器获取参数 ---
        self.path_file = rospy.get_param('~path_file', 'global_path36412.csv')
        self.pose_topic = rospy.get_param('~pose_topic', '/gnss_pose')
        self.trigger_distance = rospy.get_param('~trigger_distance', 5.0)
        # 这两个变量会从您的 frent_test.launch 文件中获取文件名
        self.output_sl_file = rospy.get_param('~output_sl_file', 'sl_data.csv')
        self.output_graph_file = rospy.get_param('~output_graph_file', 'sl_graph.png')

        # 读取参考路径
        self.ref_path = self.load_path()
        if self.ref_path is None:
            rospy.signal_shutdown("Failed to load path file.")
            return

        # 预计算路径的累积距离（s坐标）
        self.ref_s = self.calculate_s_coordinates()

        # 状态变量
        self.is_triggered = False
        self.s_data = []
        self.l_data = []

        # 订阅车辆位姿话题
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)

        # 设置ROS关闭时的统一回调函数
        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("Frenet Planner Node is running. Waiting for pose messages...")

    def load_path(self):
        """从CSV文件加载路径点"""
        try:
            df = pd.read_csv(self.path_file)
            path = df[['x', 'y']].values
            rospy.loginfo("Path loaded successfully with %d waypoints.", len(path))
            return path
        except Exception as e:
            rospy.logerr("Error loading path file: %s", e)
            return None

    def calculate_s_coordinates(self):
        """预计算参考路径上每个点的s坐标（累积距离）"""
        s_coords = [0.0]
        for i in range(1, len(self.ref_path)):
            dx = self.ref_path[i, 0] - self.ref_path[i-1, 0]
            dy = self.ref_path[i, 1] - self.ref_path[i-1, 1]
            dist = math.sqrt(dx**2 + dy**2)
            s_coords.append(s_coords[-1] + dist)
        return np.array(s_coords)

    def find_closest_waypoint(self, current_x, current_y):
        """在参考路径上找到离当前位置最近的点的索引"""
        distances = np.linalg.norm(self.ref_path - np.array([current_x, current_y]), axis=1)
        closest_index = np.argmin(distances)
        return closest_index

    def cartesian_to_frenet(self, current_x, current_y, current_yaw):
        """将笛卡尔坐标转换为Frenet坐标"""
        closest_idx = self.find_closest_waypoint(current_x, current_y)

        if closest_idx >= len(self.ref_path) - 1:
            prev_idx = len(self.ref_path) - 2
        else:
            prev_idx = closest_idx
        
        next_idx = prev_idx + 1

        ref_x1, ref_y1 = self.ref_path[prev_idx]
        ref_x2, ref_y2 = self.ref_path[next_idx]
        
        path_dx = ref_x2 - ref_x1
        path_dy = ref_y2 - ref_y1
        
        vec_x = current_x - ref_x1
        vec_y = current_y - ref_y1

        proj_val = (vec_x * path_dx + vec_y * path_dy) / (path_dx**2 + path_dy**2)
        proj_val = max(0.0, min(1.0, proj_val))

        s = self.ref_s[prev_idx] + proj_val * math.sqrt(path_dx**2 + path_dy**2)

        cross_product = path_dx * vec_y - path_dy * vec_x
        proj_point_x = ref_x1 + proj_val * path_dx
        proj_point_y = ref_y1 + proj_val * path_dy
        
        l = np.sign(cross_product) * math.sqrt((current_x - proj_point_x)**2 + (current_y - proj_point_y)**2)
                                               
        return s, l

    def pose_callback(self, msg):
        """处理接收到的位姿消息"""
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        
        if not self.is_triggered:
            first_point = self.ref_path[0]
            distance_to_start = math.sqrt((current_x - first_point[0])**2 + (current_y - first_point[1])**2)
            if distance_to_start < self.trigger_distance:
                self.is_triggered = True
                rospy.loginfo("Vehicle has approached the starting point. Starting SL data collection.")
        
        if self.is_triggered:
            orientation_q = msg.pose.orientation
            _, _, current_yaw = tf.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            s, l = self.cartesian_to_frenet(current_x, current_y, current_yaw)
            self.s_data.append(s)
            self.l_data.append(l)
            rospy.loginfo_throttle(1.0, "Current Frenet coordinates: s=%.2f, l=%.2f", s, l)

    def save_sl_to_csv(self):
        """将收集到的s和l数据保存到CSV文件"""
        rospy.loginfo("Attempting to save S-L data to CSV...")
        try:
            # 【核心修正】使用从launch文件读取的变量来检查和删除文件
            if os.path.exists(self.output_sl_file):
                os.remove(self.output_sl_file)
                rospy.loginfo("Removed existing file: %s", self.output_sl_file)
            
            sl_data_dict = {'s': self.s_data, 'l': self.l_data}
            df = pd.DataFrame(sl_data_dict)
            df.to_csv(self.output_sl_file, index=False)
            rospy.loginfo("S-L data saved successfully to %s", self.output_sl_file)
        except Exception as e:
            rospy.logerr("Error saving S-L data to CSV: %s", e)

    def generate_sl_graph(self):
        """生成并保存S-L图"""
        rospy.loginfo("Attempting to generate S-L graph...")
        try:
            # 【核心修正】使用从launch文件读取的变量来检查和删除文件
            if os.path.exists(self.output_graph_file):
                os.remove(self.output_graph_file)
                rospy.loginfo("Removed existing file: %s", self.output_graph_file)
            
            plt.figure(figsize=(10, 6))
            plt.plot(self.s_data, self.l_data, marker='.', linestyle='-', markersize=2)
            plt.xlabel('S (m)')
            plt.ylabel('L (m)')
            plt.title('S-L Graph of Vehicle Trajectory')
            plt.grid(True)
            plt.savefig(self.output_graph_file)
            rospy.loginfo("S-L graph saved to %s", self.output_graph_file)
        except Exception as e:
            rospy.logerr("Error generating S-L graph: %s", e)

    def shutdown_hook(self):
        """节点关闭时执行的操作，统一处理数据保存"""
        rospy.loginfo("Shutting down Frenet Planner Node. Saving data...")
        if not self.s_data or not self.l_data:
            rospy.loginfo("No data collected. Nothing to save.")
            return
        
        self.save_sl_to_csv()
        self.generate_sl_graph()

if __name__ == '__main__':
    try:
        FrenetPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass