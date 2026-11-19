#include <ros/ros.h>//定义ROS头文件
#include <iostream>
#include <string>

#include <pure_pursuit/LQR_core.h>

#define PI 3.1415926





int main(int argc, char** argv){

    ros::init(argc, argv, "LQR");//定义纯跟踪节点
    waypoint_follower::LQRNode LQR_ppn;//纯跟踪节点类实例化
    LQR_ppn.run();//通过实例化对象调用类的成员函数，再通过run()函数间接调用其他成员函数，从而实现纯跟踪算法的一系列功能

    return 0;
}
