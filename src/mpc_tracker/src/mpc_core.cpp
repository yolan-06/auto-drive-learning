#include <mpc_tracker/mpc_core.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath> // 用于 tan, hypot, atan2
#include <algorithm> // 用于 std::min, std::max

namespace mpc_tracker
{

// 构造函数 
MpcTrackerNode::MpcTrackerNode()
    : private_nh_("~"), 
      is_pose_set_(false),
      is_path_set_(false),
      mpc_horizon_N_(30), 
      mpc_step_dt_(0.05), 
      loop_rate_(30),  
      wheelbase_L_(0.9), // 默认值
      state_dim_(3),     // [x, y, yaw]
      control_dim_(2),   // [v, delta]
      last_near_idx_(0),  // 初始化渐进搜索索引
      is_goal_reached_(false), // 初始化“到达”标志
      is_first_run_(true)    //初始化“首次运行”标志
{
    initForROS();
}

// 析构函数
MpcTrackerNode::~MpcTrackerNode() {}

// 初始化 ROS
void MpcTrackerNode::initForROS()
{
    // 从参数服务器获取参数
    private_nh_.param<std::string>("waypoints_file", waypoints_file_,
                                   "/path/to/your/default.csv"); 
    private_nh_.param("mpc_horizon", mpc_horizon_N_, 30);
    private_nh_.param("mpc_dt", mpc_step_dt_, 0.05);
    private_nh_.param("loop_rate", loop_rate_, 30);
    private_nh_.param("wheelbase", wheelbase_L_, 0.9);
    if (std::abs(wheelbase_L_) < 1e-3)
    {
        ROS_WARN("[MPC Core] Wheelbase is too small (%f), using default 0.9m", wheelbase_L_);
        wheelbase_L_ = 0.9;
    }
    mpc_.setWheelbase(wheelbase_L_);
    mpc_.setDimensions(state_dim_, control_dim_, mpc_horizon_N_, mpc_step_dt_);
    sub_pose_ = nh_.subscribe("/gnss_pose", 1, &MpcTrackerNode::callbackFromCurrentPose, this);
    pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_raw", 1); 
    pub_all_waypoints_ = private_nh_.advertise<visualization_msgs::Marker>("viz/all_waypoints", 1, true); 
    pub_mpc_viz_predict_ = private_nh_.advertise<visualization_msgs::Marker>("viz/predict_path", 1);
    pub_mpc_viz_ref_ = private_nh_.advertise<visualization_msgs::Marker>("viz/reference_path", 1);
}

// 加载全局路径
bool MpcTrackerNode::loadGlobalPath(const std::string &file)
{
    std::ifstream csv_file(file);
    if (!csv_file.is_open())
    {
        ROS_ERROR("[MPC Core] Cannot open global path file: %s", file.c_str());
        return false;
    }
    waypoints_.clear();
    waypoints_v_.clear();
    std::string line;
    std::getline(csv_file, line); 
    while (std::getline(csv_file, line))
    {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;
        while (std::getline(ss, cell, ','))
        {
            row.push_back(cell);
        }
        if (row.size() < 5) continue; 
        try
        {
            geometry_msgs::Pose p;
            p.position.x = std::stod(row[0]);
            p.position.y = std::stod(row[1]);
            p.position.z = 0.0; 
            double yaw = std::stod(row[3]);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            p.orientation = tf2::toMsg(q);
            waypoints_.push_back(p);
            waypoints_v_.push_back(std::stod(row[4])); 
        }
        catch (const std::invalid_argument& e)
        {
            ROS_WARN("[MPC Core] Invalid number in CSV row: %s", line.c_str());
        }
    }
    if (waypoints_.empty())
    {
        ROS_ERROR("[MPC Core] Global path is empty after loading.");
        return false;
    }
    if (!waypoints_v_.empty())
    {
        waypoints_v_.back() = 0.0;
    }
    ROS_INFO("[MPC Core] Global path loaded successfully (%zu points)", waypoints_.size());
    pub_all_waypoints_.publish(mpc_tracker_viz::displayallWaypoints(waypoints_));
    return true;
}

// GNSS 位姿回调
void MpcTrackerNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    current_pose_ = *msg;
    is_pose_set_ = true;
}

//全局最近点索引 (仅用于初始化)
size_t MpcTrackerNode::findNearestIdx() const
{
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;
    const double px = current_pose_.pose.position.x;
    const double py = current_pose_.pose.position.y;

    for (size_t i = 0; i < waypoints_.size(); ++i)
    {
        const double dx = waypoints_[i].position.x - px;
        const double dy = waypoints_[i].position.y - py;
        const double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq)
        {
            min_dist_sq = dist_sq;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}


// 渐进式查找最近索引
size_t MpcTrackerNode::findProgressiveNearestIdx(size_t last_idx) const
{
    const size_t SEARCH_WINDOW = 200;
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t best_idx = last_idx; 

    const double px = current_pose_.pose.position.x;
    const double py = current_pose_.pose.position.y;
    const size_t total_points = waypoints_.size();

    size_t end_idx = std::min(last_idx + SEARCH_WINDOW, total_points);

    for (size_t i = last_idx; i < end_idx; ++i)
    {
        const double dx = waypoints_[i].position.x - px;
        const double dy = waypoints_[i].position.y - py;
        const double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq)
        {
            min_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    return best_idx;
}


// 归一化角度
double MpcTrackerNode::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 构造参考轨迹 
bool MpcTrackerNode::buildReferenceTrajectory(size_t begin_idx, int N, double dt, 
                                            Eigen::MatrixXd &xref, Eigen::MatrixXd &uref)
{
    if (waypoints_.empty()) return false;
    const size_t total_points = waypoints_.size();
    xref.resize(state_dim_, N + 1);
    uref.resize(control_dim_, N);
    size_t current_idx = begin_idx;
    double dist_traveled = 0.0; 
    bool end_of_path_reached = false;
    xref(0, 0) = waypoints_[current_idx].position.x;
    xref(1, 0) = waypoints_[current_idx].position.y;
    xref(2, 0) = tf2::getYaw(waypoints_[current_idx].orientation);
    for (int i = 0; i < N; ++i) 
    {
        if (end_of_path_reached)
        {
            xref(0, i + 1) = xref(0, i); 
            xref(1, i + 1) = xref(1, i);
            xref(2, i + 1) = xref(2, i);
            uref(0, i) = 0.0; 
            uref(1, i) = 0.0; 
            continue; 
        }
        double v_r = waypoints_v_[current_idx]; 
        double dist_step = v_r * dt;
        double target_dist = dist_traveled + dist_step;
        size_t p1_idx = current_idx;
        size_t p2_idx = std::min(p1_idx + 1, total_points - 1);
        double search_dist = dist_traveled;
        while (p2_idx < total_points - 1) 
        {
            const auto& p1_pos = waypoints_[p1_idx].position;
            const auto& p2_pos = waypoints_[p2_idx].position;
            double segment_dist = std::hypot(p2_pos.x - p1_pos.x, p2_pos.y - p1_pos.y);
            if (search_dist + segment_dist >= target_dist)
            {
                break; 
            }
            search_dist += segment_dist; 
            p1_idx = p2_idx;
            p2_idx = std::min(p1_idx + 1, total_points - 1);
        }
        const auto& p1 = waypoints_[p1_idx].position;
        const auto& p2 = waypoints_[p2_idx].position;
        const double p1_v = waypoints_v_[p1_idx];
        const double p2_v = waypoints_v_[p2_idx]; 
        const double p1_yaw = tf2::getYaw(waypoints_[p1_idx].orientation);
        const double p2_yaw = tf2::getYaw(waypoints_[p2_idx].orientation);
        double segment_dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
        double interp_ratio = 0.0;
        if (p1_idx >= total_points - 1)
        {
             interp_ratio = 1.0; 
             end_of_path_reached = true;
        }
        else if (segment_dist > 1e-6)
        {
            interp_ratio = (target_dist - search_dist) / segment_dist;
        }
        interp_ratio = std::max(0.0, std::min(1.0, interp_ratio));
        if (interp_ratio > 0.9999 && p2_idx >= total_points - 1)
        {
            end_of_path_reached = true;
        }
        double x_interp = (1.0 - interp_ratio) * p1.x + interp_ratio * p2.x;
        double y_interp = (1.0 - interp_ratio) * p1.y + interp_ratio * p2.y;
        double v_interp = (1.0 - interp_ratio) * p1_v + interp_ratio * p2_v;
        double yaw_diff = normalizeAngle(p2_yaw - p1_yaw);
        double yaw_interp = normalizeAngle(p1_yaw + interp_ratio * yaw_diff);
        if (end_of_path_reached)
        {
            v_interp = 0.0;
            x_interp = waypoints_.back().position.x;
            y_interp = waypoints_.back().position.y;
            yaw_interp = tf2::getYaw(waypoints_.back().orientation);
        }
        xref(0, i + 1) = x_interp;
        xref(1, i + 1) = y_interp;
        xref(2, i + 1) = yaw_interp;
        uref(0, i) = v_interp; 
        double yaw_rate_r = normalizeAngle(xref(2, i + 1) - xref(2, i)) / dt;
        const double delta_max_limit = 0.47;
        const double delta_min_limit = -0.47;
        double delta_r = 0.0; 
        if (std::abs(uref(0, i)) > 1e-6) 
        {
            delta_r = std::atan( (yaw_rate_r * wheelbase_L_) / uref(0, i) );
        }
        delta_r = std::max(delta_min_limit, std::min(delta_max_limit, delta_r));
        uref(1, i) = delta_r; 
        dist_traveled = target_dist; 
        if (interp_ratio > 0.9999 && p1_idx < total_points - 1)
        {
            current_idx = p2_idx;
        }
        else
        {
            current_idx = p1_idx;
        }
    }
    return true;
}


// 发布控制指令
void MpcTrackerNode::publishControl(const Eigen::Vector2d &control_cmd)
{
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = ros::Time::now();
    twist_msg.header.frame_id = "base_link"; 
    twist_msg.twist.linear.x = control_cmd(0); 
    twist_msg.twist.angular.z = control_cmd(0) * std::tan(control_cmd(1)) / wheelbase_L_;
    pub_twist_.publish(twist_msg);
}

// 发布可视化信息
void MpcTrackerNode::publishMpcViz(const Eigen::MatrixXd &xref, const Eigen::VectorXd &pred_states)
{
    pub_mpc_viz_ref_.publish(mpc_tracker_viz::displayReferenceTrajectory(xref));
    pub_mpc_viz_predict_.publish(mpc_tracker_viz::displayPredictPath(pred_states, state_dim_));
}


// 主运行函数
void MpcTrackerNode::run()
{
    if (!loadGlobalPath(waypoints_file_))
    {
        ROS_ERROR("Failed to load global path. Shutting down.");
        return;
    }
    is_path_set_ = true;

    ros::Rate loop_rate(loop_rate_);
    
    Eigen::MatrixXd xref(state_dim_, mpc_horizon_N_ + 1);
    Eigen::MatrixXd uref(control_dim_, mpc_horizon_N_);

    while (ros::ok())
    {
        ros::spinOnce(); 

        if (is_goal_reached_)
        {
            loop_rate.sleep();
            continue;
        }

        if (!is_pose_set_ || !is_path_set_)
        {
            ROS_WARN_THROTTLE(1.0, "Waiting for pose and path to be set...");
            loop_rate.sleep();
            continue;
        }

        // --- 核心 MPC 逻辑  ---
        
        
        // 1. 根据是否为首次运行，选择不同的搜索方法
        size_t near_idx;
        if (is_first_run_)
        {
            // 第一次运行时, 使用 *全局* 搜索来找到我们的起始位置
            near_idx = findNearestIdx(); 
            is_first_run_ = false; //关闭此标志
            ROS_INFO("[MPC Core] First run: Global search found nearest index %zu", near_idx);
        }
        else
        {
            // 后续所有运行, 都使用 *渐进式* 搜索
            near_idx = findProgressiveNearestIdx(last_near_idx_);
        }
        last_near_idx_ = near_idx; //  存储当前索引，供下一周期使用
        
        
        if (!buildReferenceTrajectory(near_idx, mpc_horizon_N_, mpc_step_dt_, xref, uref))
        {
            ROS_WARN_THROTTLE(1.0, "Failed to build reference trajectory.");
            loop_rate.sleep();
            continue;
        }

        // 2. 设置初始状态 x0
        Eigen::Vector3d x0;
        x0(0) = current_pose_.pose.position.x;
        x0(1) = current_pose_.pose.position.y;
        x0(2) = tf2::getYaw(current_pose_.pose.orientation);
                
        // 3. 调用 MPC 求解
        if (!mpc_.solveMpc(x0, xref, uref))
        {
            ROS_WARN_THROTTLE(1.0, "[MPC Core] MPC solving failed!");
        }
        
        // 4. 获取控制指令
        Eigen::Vector2d control_cmd = mpc_.getControlCmd();
        
        // 5. 到达终点检查 
        const size_t total_points = waypoints_.size();
        const double FINAL_STOP_DIST = 0.25; 
        
        if (last_near_idx_ >= total_points - 1)
        {
            double dist_to_end_sq = std::pow(x0(0) - waypoints_.back().position.x, 2) + 
                                    std::pow(x0(1) - waypoints_.back().position.y, 2);
            
            if (dist_to_end_sq < (FINAL_STOP_DIST * FINAL_STOP_DIST) && 
                std::abs(control_cmd(0)) < 0.1) 
            {
                ROS_INFO("[MPC Core] Goal reached. Stopping control loop.");
                is_goal_reached_ = true;
                control_cmd.setZero(); 
            }
        }

        // 6. 发布控制指令
        publishControl(control_cmd);

        // 7. 发布可视化信息
        Eigen::VectorXd pred_states = mpc_.getPredictState();
        publishMpcViz(xref, pred_states);

        // 8. 循环等待
        loop_rate.sleep();
    }
}

} // namespace mpc_tracker