#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import pandas as pd
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
# 【修改】导入 can_msgs::ecu 消息类型
from pure_pursuit.msg import ecu
# 【新增】导入 vehicle_status 消息类型
from pure_pursuit.msg import vehicle_status
import matplotlib.pyplot as plt
import tf.transformations as tf
import os
from std_srvs.srv import SetBool, SetBoolResponse

class FrenetPlannerEnhanced:
    def __init__(self):
        rospy.init_node('frenet_planner_enhanced_node', anonymous=True)

        # --- 参数 ---
        self.path_file = rospy.get_param('~path_file', 'global_path_with_velocity.csv')
        self.pose_topic = rospy.get_param('~pose_topic', '/gnss_pose')
        # 【修改】定义新的 vehicle_status 话题参数
        self.vehicle_status_topic = rospy.get_param('~vehicle_status_topic', '/vehicle_status')
        # 【修改】将 cmd_vel_topic 改为 ecu_topic
        self.ecu_topic = rospy.get_param('~ecu_topic', '/ecu')
        
        self.output_raw_csv_file = rospy.get_param('~output_raw_csv_file', 'tracking_errors_raw.csv')
        self.output_downsampled_csv_file = rospy.get_param('~output_downsampled_csv_file', 'tracking_errors_downsampled_1m.csv')
        self.downsample_interval = rospy.get_param('~downsample_interval', 1.0)
        
        self.output_graph_file = rospy.get_param('~output_graph_file', 'tracking_errors.png')

        self.ref_path = self.load_path()
        if self.ref_path is None:
            rospy.signal_shutdown("Failed to load path file.")
            return

        self.ref_s, self.ref_yaw = self.calculate_path_properties()

        # --- 状态变量 ---
        self.is_triggered = False
        self.current_pose = None

        self.actual_velocity = 0.0
        self.desired_velocity = 0.0

        self.s_data, self.l_data, self.heading_error_data, self.velocity_error_data, self.x_data, self.y_data = [], [], [], [], [], []

        # --- 订阅者 和 服务 ---
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=1)
        # 【修改】将 /odom 订阅者替换为 /vehicle_status 订阅者
        self.status_sub = rospy.Subscriber(self.vehicle_status_topic, vehicle_status, self.vehicle_status_callback, queue_size=1)
        # 【修改】将 cmd_vel 订阅者改为 ecu 订阅者
        self.ecu_sub = rospy.Subscriber(self.ecu_topic, ecu, self.ecu_callback, queue_size=1)
        self.trigger_service = rospy.Service('~toggle_logging', SetBool, self.handle_toggle_logging)
        rospy.loginfo("Service '~toggle_logging' is ready.")

        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("Frenet Planner Enhanced Node is running.")

    # 【新增】用于处理 /vehicle_status 消息的回调函数
    def vehicle_status_callback(self, msg):
        # 从 cur_speed 字段获取速度值(km/h) 
        # 将其除以 3.6 转换为 m/s，并赋给 actual_velocity
        self.actual_velocity = msg.cur_speed / 3.6
        
    # 【修改】替换 cmd_vel_callback 为 ecu_callback
    def ecu_callback(self, msg):
        # 从 ecu 消息的 motor 字段获取期望速度 (km/h)
        # 将其除以 3.6 转换为 m/s，并赋给 desired_velocity
        self.desired_velocity = msg.motor / 3.6
        
    def ensure_dir_exists(self, filepath):
        directory = os.path.dirname(filepath)
        if not os.path.exists(directory):
            try:
                os.makedirs(directory)
                rospy.loginfo("Created directory: %s", directory)
            except OSError as e:
                rospy.logerr("Failed to create directory %s: %s", directory, e)
                
    def handle_toggle_logging(self, req):
        if req.data:
            if self.is_triggered: return SetBoolResponse(success=True, message="Logging is already active.")
            if self.current_pose is None: return SetBoolResponse(success=False, message="Cannot start logging, no pose received yet.")
            self.is_triggered = True
            rospy.loginfo("Logging started by manual trigger.")
            return SetBoolResponse(success=True, message="Logging started.")
        else:
            if not self.is_triggered: return SetBoolResponse(success=True, message="Logging is already stopped.")
            self.is_triggered = False
            rospy.loginfo("Logging stopped by manual trigger.")
            return SetBoolResponse(success=True, message="Logging stopped.")

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        if self.is_triggered:
            current_x, current_y = self.current_pose.position.x, self.current_pose.position.y
            orientation_q = self.current_pose.orientation
            _, _, current_yaw = tf.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            s, l, heading_error = self.cartesian_to_frenet(current_x, current_y, current_yaw)
            # 此处的 self.actual_velocity 和 self.desired_velocity 都已经转换为 m/s
            velocity_error = self.desired_velocity - self.actual_velocity
            self.s_data.append(s); self.l_data.append(l); self.heading_error_data.append(heading_error)
            self.velocity_error_data.append(velocity_error); self.x_data.append(current_x); self.y_data.append(current_y)
            rospy.loginfo_throttle(1.0, "s: %.2f, l: %.2f, h_err: %.2f rad, v_err: %.2f m/s", s, l, heading_error, velocity_error)
    
    def find_closest_waypoint(self, current_x, current_y):
        """在整个参考路径上找到离当前位置最近的点的索引"""
        distances = np.linalg.norm(self.ref_path - np.array([current_x, current_y]), axis=1)
        closest_index = np.argmin(distances)
        return closest_index

    def cartesian_to_frenet(self, current_x, current_y, current_yaw):
        closest_idx = self.find_closest_waypoint(current_x, current_y)

        if closest_idx >= len(self.ref_path) - 1:
            prev_idx = len(self.ref_path) - 2
        else:
            prev_idx = closest_idx
        next_idx = prev_idx + 1

        ref_x1, ref_y1 = self.ref_path[prev_idx]
        ref_x2, ref_y2 = self.ref_path[next_idx]
        path_dx, path_dy = ref_x2 - ref_x1, ref_y2 - ref_y1
        vec_x, vec_y = current_x - ref_x1, current_y - ref_y1

        proj_val = (vec_x * path_dx + vec_y * path_dy) / (path_dx**2 + path_dy**2)
        proj_val = max(0.0, min(1.0, proj_val))

        s = self.ref_s[prev_idx] + proj_val * math.sqrt(path_dx**2 + path_dy**2)
        cross_product = path_dx * vec_y - path_dy * vec_x
        proj_point_x = ref_x1 + proj_val * path_dx
        proj_point_y = ref_y1 + proj_val * path_dy
        l = np.sign(cross_product) * math.sqrt((current_x - proj_point_x)**2 + (current_y - proj_point_y)**2)
        path_yaw = self.ref_yaw[closest_idx]
        heading_error = self.normalize_angle(current_yaw - path_yaw)
        return s, l, heading_error

    def save_raw_csv(self):
        rospy.loginfo("Attempting to save raw tracking error data to CSV...")
        try:
            self.ensure_dir_exists(self.output_raw_csv_file)
            if os.path.exists(self.output_raw_csv_file): os.remove(self.output_raw_csv_file)
            min_len = min(len(self.s_data), len(self.l_data), len(self.heading_error_data), 
                          len(self.velocity_error_data), len(self.x_data), len(self.y_data))
            error_data_dict = {
                's_m': self.s_data[:min_len], 'lateral_error_m': self.l_data[:min_len],
                'heading_error_rad': self.heading_error_data[:min_len], 'velocity_error_ms': self.velocity_error_data[:min_len],
                'x': self.x_data[:min_len], 'y': self.y_data[:min_len]
            }
            df = pd.DataFrame(error_data_dict)
            df.to_csv(self.output_raw_csv_file, index=False)
            rospy.loginfo("Raw tracking error data saved successfully to %s", self.output_raw_csv_file)
        except Exception as e:
            rospy.logerr("Error saving raw data to CSV: %s", e)
            
    def save_downsampled_csv(self):
        rospy.loginfo("Attempting to save downsampled tracking error data...")
        if not self.s_data: return
        try:
            self.ensure_dir_exists(self.output_downsampled_csv_file)
            if os.path.exists(self.output_downsampled_csv_file): os.remove(self.output_downsampled_csv_file)
            downsampled_data = { 's_m': [], 'lateral_error_m': [], 'heading_error_rad': [], 'velocity_error_ms': [], 'x': [], 'y': [] }
            last_s_saved = -float('inf')
            for i in range(len(self.s_data)):
                if self.s_data[i] - last_s_saved >= self.downsample_interval:
                    downsampled_data['s_m'].append(self.s_data[i])
                    downsampled_data['lateral_error_m'].append(self.l_data[i])
                    downsampled_data['heading_error_rad'].append(self.heading_error_data[i])
                    downsampled_data['velocity_error_ms'].append(self.velocity_error_data[i])
                    downsampled_data['x'].append(self.x_data[i])
                    downsampled_data['y'].append(self.y_data[i])
                    last_s_saved = self.s_data[i]
            df = pd.DataFrame(downsampled_data)
            df.to_csv(self.output_downsampled_csv_file, index=False)
            rospy.loginfo("Downsampled tracking error data saved successfully to %s", self.output_downsampled_csv_file)
        except Exception as e:
            rospy.logerr("Error saving downsampled data to CSV: %s", e)
            
    def generate_error_graphs(self):
        rospy.loginfo("Attempting to generate tracking error graphs...")
        try:
            self.ensure_dir_exists(self.output_graph_file)
            if os.path.exists(self.output_graph_file): os.remove(self.output_graph_file)
            fig, axs = plt.subplots(3, 1, figsize=(10, 18))
            fig.suptitle('Vehicle Tracking Errors', fontsize=16)
            axs[0].plot(self.s_data, self.l_data, marker='.', linestyle='-', markersize=2)
            axs[0].set_xlabel('S (m)'); axs[0].set_ylabel('Lateral Error (m)'); axs[0].set_title('Lateral Error vs. S'); axs[0].grid(True)
            axs[1].plot(self.s_data, np.degrees(self.heading_error_data), marker='.', linestyle='-', markersize=2, color='g')
            axs[1].set_xlabel('S (m)'); axs[1].set_ylabel('Heading Error (degrees)'); axs[1].set_title('Heading Error vs. S'); axs[1].grid(True)
            axs[2].plot(self.s_data, self.velocity_error_data, marker='.', linestyle='-', markersize=2, color='r')
            axs[2].set_xlabel('S (m)'); axs[2].set_ylabel('Velocity Error (m/s)'); axs[2].set_title('Velocity Error vs. S'); axs[2].grid(True)
            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            plt.savefig(self.output_graph_file)
            rospy.loginfo("Tracking error graphs saved to %s", self.output_graph_file)
        except Exception as e:
            rospy.logerr("Error generating graphs: %s", e)
            
    def shutdown_hook(self):
        rospy.loginfo("Shutting down Frenet Planner Enhanced Node. Saving data...")
        if not self.s_data:
            rospy.loginfo("No data collected. Nothing to save.")
            return
        self.save_raw_csv()
        self.save_downsampled_csv()
        self.generate_error_graphs()
        
    def load_path(self):
        try:
            df = pd.read_csv(self.path_file)
            path = df[['x', 'y']].values
            rospy.loginfo("Path loaded successfully with %d waypoints.", len(path))
            return path
        except Exception as e:
            rospy.logerr("Error loading path file: %s", e)
            return None
            
    def calculate_path_properties(self):
        s_coords = [0.0]
        yaws = []
        for i in range(len(self.ref_path) - 1):
            dx = self.ref_path[i+1, 0] - self.ref_path[i, 0]
            dy = self.ref_path[i+1, 1] - self.ref_path[i, 1]
            dist = math.sqrt(dx**2 + dy**2)
            s_coords.append(s_coords[-1] + dist)
            yaws.append(math.atan2(dy, dx))
        yaws.append(yaws[-1])
        return np.array(s_coords), np.array(yaws)
        
    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    try:
        FrenetPlannerEnhanced()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass