#ifndef MPC_TRACKER_MPC_CORE_H
#define MPC_TRACKER_MPC_CORE_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <mpc_tracker/mpc_viz.h>

// C++ includes
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath> // 用于 hypot 和 atan2

// MPC 算法核心
#include <mpc_tracker/mpc.h> 
#include <tf2/utils.h> // 用于 getYaw

namespace mpc_tracker
{

class MpcTrackerNode
{
public:
    MpcTrackerNode(); // 构造函数
    ~MpcTrackerNode(); // 析构函数

    void run(); // 主运行函数

private:
    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // MPC 算法实例
    Mpc mpc_;

    // --------- 订阅者 -------- -
    ros::Subscriber sub_pose_; 

    // --------- 发布者 -------- -
    ros::Publisher pub_twist_;        // 控制指令 
    
    // 可视化发布者
    ros::Publisher pub_all_waypoints_; 
    ros::Publisher pub_mpc_viz_predict_;
    ros::Publisher pub_mpc_viz_ref_; 

    // ROS 参数
    std::string waypoints_file_;
    int mpc_horizon_N_;          // MPC 预测时域
    double mpc_step_dt_;         // MPC 时间步长
    int loop_rate_;              // 控制循环频率

    // 阿克曼模型参数
    double wheelbase_L_; // 轴距 

    // 状态变量
    bool is_pose_set_;
    bool is_path_set_;
    geometry_msgs::PoseStamped current_pose_; // 当前车辆位姿
    std::vector<geometry_msgs::Pose> waypoints_; // 全局路径点 (x, y, z, yaw)
    std::vector<double> waypoints_v_;      // 全局路径速度

    // MPC 维度 
    int state_dim_;
    int control_dim_;

    
    // 用于存储上一个周期的最近点索引，以实现渐进式搜索
    size_t last_near_idx_;

    
    // 用于标记是否已到达终点并停止
    bool is_goal_reached_;

   
    // 用于处理第一次启动时的索引初始化
    bool is_first_run_;


    // ----------------- 核心函数 -----------------

    /**
     * @brief 初始化 ROS 相关的发布者、订阅者和参数
     */
    void initForROS();

    /**
     * @brief 从 CSV 文件加载全局路径点
     * @param file CSV 文件路径
     * @return bool 是否加载成功
     */
    bool loadGlobalPath(const std::string &file);

    /**
     * @brief ROS 回调函数：接收 /gnss_pose
     */
    void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);

    /**
     * @brief 全局搜索：查找距离当前位置最近的路径点索引 (仅用于初始化)
     * @return size_t 最近点的索引
     */
    size_t findNearestIdx() const;

    /**
     * @brief 渐进式查找最近索引 (用于跟踪) 
     * @details 仅从上一个索引 (last_idx) 向前搜索一个固定窗口
     */
    size_t findProgressiveNearestIdx(size_t last_idx) const;


    /**
     * @brief 构造未来 N+1 步的参考状态 xref 和 N 步的参考控制 uref
     * @param begin_idx 参考轨迹的起始索引
     * @param N 预测时域
     * @param dt 时间步长
     * @param[out] xref (state_dim x N+1) 参考状态矩阵 [x; y; yaw]
     * @param[out] uref (control_dim x N) 参考控制矩阵 [v; delta]
     * @return bool 是否成功构建
     */
    bool buildReferenceTrajectory(size_t begin_idx, int N, double dt, 
                                  Eigen::MatrixXd &xref, Eigen::MatrixXd &uref);

    /**
     * @brief 发布计算出的控制指令
     */
    void publishControl(const Eigen::Vector2d &control_cmd);

    /**
     * @brief 发布 MPC 预测和参考轨迹 (用于Rviz可视化)
     */
    void publishMpcViz(const Eigen::MatrixXd &xref, const Eigen::VectorXd &pred_states);

    /**
     * @brief 归一化角度到 [-pi, pi]
     */
    double normalizeAngle(double angle);

}; // class MpcTrackerNode

} // namespace mpc_tracker

#endif // MPC_TRACKER_MPC_CORE_H