#include <ros/ros.h>//定义ROS头文件
#include <iostream>
#include <string>

// User defined includes
#include <pure_pursuit/pure_pursuit_core.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pure_pursuit");//定义纯跟踪节点
  // std::cout << "------------" << std::endl;
  // system("------------");
  waypoint_follower::PurePursuitNode ppn;//纯跟踪节点类实例化
  ppn.run();//通过实例化对象调用类的成员函数，再通过run()函数间接调用其他成员函数，从而实现纯跟踪算法的一系列功能

  return 0;
}
