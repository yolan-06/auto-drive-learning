#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_euler
import sys

def main():
    """
    主函数，用于调用Gazebo服务重置模型位姿。
    """
    # 初始化ROS节点
    rospy.init_node('reset_gazebo_model_pose')

    # 从ROS参数服务器获取用户设定的参数，如果未设定则使用默认值
    model_name = rospy.get_param('~model_name', 'your_robot_name_here')
    
    # 设定期望的初始位置 (x, y, z)
    initial_x = rospy.get_param('~x', 0.0)
    initial_y = rospy.get_param('~y', 0.0)
    initial_z = rospy.get_param('~z', 0.0)
    
    # 设定期望的初始朝向 (以弧度为单位的偏航角 Yaw)
    initial_yaw = rospy.get_param('~yaw', 0.0)

    if model_name == 'your_robot_name_here':
        rospy.logerr("错误：请在launch文件中设置正确的'model_name'参数!")
        sys.exit(1)

    rospy.loginfo("正在等待 /gazebo/set_model_state 服务...")
    rospy.wait_for_service('/gazebo/set_model_state')
    
    try:
        set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        initial_pose = Pose()
        initial_pose.position.x = initial_x
        initial_pose.position.y = initial_y
        initial_pose.position.z = initial_z
        
        q = quaternion_from_euler(0, 0, initial_yaw)
        initial_pose.orientation.x = q[0]
        initial_pose.orientation.y = q[1]
        initial_pose.orientation.z = q[2]
        initial_pose.orientation.w = q[3]

        initial_twist = Twist() # Twist 默认为零

        state_msg = ModelState()
        state_msg.model_name = model_name
        state_msg.pose = initial_pose
        state_msg.twist = initial_twist
        state_msg.reference_frame = 'world'

        response = set_state_service(state_msg)
        
        if response.success:
            # 【语法修正】使用Python 2兼容的 .format() 语法
            rospy.loginfo("成功将模型 '{0}' 重置到初始位置。".format(model_name))
        else:
            # 【语法修正】使用Python 2兼容的 .format() 语法
            rospy.logerr("重置模型 '{0}' 失败: {1}".format(model_name, response.status_message))

    except rospy.ServiceException as e:
        # 【语法修正】使用Python 2兼容的 .format() 语法
        rospy.logerr("服务调用失败: {0}".format(e))
    except Exception as e:
        # 【语法修正】使用Python 2兼容的 .format() 语法
        rospy.logerr("发生未知错误: {0}".format(e))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass