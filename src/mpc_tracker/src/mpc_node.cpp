#include <ros/ros.h>
#include <mpc_tracker/mpc_core.h> // 包含我们刚定义的 ROS 包装类

/**
 * @brief MPC Tracker 节点主函数
 * @note 这个文件模仿 pure_pursuit_node.cpp 的结构。
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "mpc_tracker_node");

    // 实例化 MPC ROS 包装类
    mpc_tracker::MpcTrackerNode mpc_node;

    // 运行节点的主循环
    mpc_node.run();

    return 0;
}