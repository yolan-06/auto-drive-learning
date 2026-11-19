#include <mpc_tracker/mpc_viz.h> // 包含我们刚定义的头文件
#include <string>
#include <vector>
#include <geometry_msgs/Point.h>


namespace mpc_tracker_viz
{

// 定义坐标系，与 pure_pursuit_viz.cpp 一致
const std::string MAP_FRAME = "map"; 

/**
 * @brief 显示所有的全局路径点
 * (功能与 pure_pursuit_viz::displayallWaypoints 相同)
 */
visualization_msgs::Marker displayallWaypoints(const std::vector<geometry_msgs::Pose>& waypoints)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time::now(); 
  marker.ns = "all_waypoints_marker"; // 命名空间
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.points.resize(waypoints.size());
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    marker.points[i] = waypoints[i].position;
  }

  marker.scale.x = 0.2; // 路径线宽
  marker.color.a = 1.0; // 不透明
  marker.color.r = 0.0;
  marker.color.g = 0.0; // 蓝色
  marker.color.b = 0.8;
  marker.frame_locked = true; 
  return marker;
}


/**
 * @brief 显示 MPC 求解器预测的未来轨迹
 */
visualization_msgs::Marker displayPredictPath(const Eigen::VectorXd& predicted_states, int state_dim)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time::now(); 
  marker.ns = "mpc_predict_path_marker"; 
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  // 遍历预测的状态
  for (size_t i = 0; i < predicted_states.size() / state_dim; ++i)
  {
    geometry_msgs::Point p;
    p.x = predicted_states(i * state_dim + 0); // x
    p.y = predicted_states(i * state_dim + 1); // y
    p.z = 0; // 假设在 2D 平面
    marker.points.push_back(p);
  }

  marker.scale.x = 0.35; // 预测路径线宽 (比全局路径粗一点)
  marker.color.a = 1.0;  // 不透明
  marker.color.r = 0.0;
  marker.color.g = 1.0;  // 绿色
  marker.color.b = 0.0;
  marker.frame_locked = true;
  return marker;
}


/**
 * @brief 显示 MPC 当前正在跟踪的参考轨迹 (xref)
 */
visualization_msgs::Marker displayReferenceTrajectory(const Eigen::MatrixXd& ref_trajectory)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time::now(); 
  marker.ns = "mpc_reference_traj_marker"; // MPC 参考轨迹
  marker.id = 2;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  // 遍历参考轨迹 (ref_trajectory 维度是 [state_dim, N+1])
  for (size_t i = 0; i < ref_trajectory.cols(); ++i)
  {
    geometry_msgs::Point p;
    p.x = ref_trajectory(0, i); // x
    p.y = ref_trajectory(1, i); // y
    p.z = 0; // 假设在 2D 平面
    marker.points.push_back(p);
  }

  marker.scale.x = 0.3; // 参考路径线宽
  marker.color.a = 1.0;  // 不透明
  marker.color.r = 1.0;
  marker.color.g = 0.0;  // 红色
  marker.color.b = 0.0;
  marker.frame_locked = true;
  return marker;
}


} // namespace mpc_tracker_viz