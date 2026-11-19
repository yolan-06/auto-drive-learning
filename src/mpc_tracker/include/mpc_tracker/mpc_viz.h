#ifndef MPC_TRACKER_MPC_VIZ_H
#define MPC_TRACKER_MPC_VIZ_H

// ROS includes
#include <ros/ros.h> // 定义ROS头文件

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

// C++ includes
#include <vector>

// Eigen includes (MPC 核心依赖)
#include <Eigen/Dense>

/**
 * @brief MPC 可视化命名空间
 * 仿照 pure_pursuit_viz.h 的结构
 */
namespace mpc_tracker_viz
{

/**
 * @brief 显示所有的全局路径点
 * (功能与 pure_pursuit_viz::displayallWaypoints 相同)
 * @param waypoints 包含所有全局路径点 Pose 的向量
 * @return visualization_msgs::Marker LINE_STRIP 标记
 */
visualization_msgs::Marker displayallWaypoints(const std::vector<geometry_msgs::Pose>& waypoints);


/**
 * @brief 显示 MPC 求解器预测的未来轨迹
 * @param predicted_states 预测的状态向量 [x0, y0, yaw0, x1, y1, yaw1, ...]
 * @param state_dim 状态维度 (在我们之前的实现中是 3)
 * @return visualization_msgs::Marker LINE_STRIP 标记 (建议用绿色)
 */
visualization_msgs::Marker displayPredictPath(const Eigen::VectorXd& predicted_states, int state_dim);


/**
 * @brief 显示 MPC 当前正在跟踪的参考轨迹 (xref)
 * @param ref_trajectory 参考轨迹矩阵 (N+1)x4，每行 [x, y, yaw, v]
 * @return visualization_msgs::Marker LINE_STRIP 标记 (建议用蓝色)
 */
visualization_msgs::Marker displayReferenceTrajectory(const Eigen::MatrixXd& ref_trajectory);


} // namespace mpc_tracker_viz

#endif // MPC_TRACKER_MPC_VIZ_H