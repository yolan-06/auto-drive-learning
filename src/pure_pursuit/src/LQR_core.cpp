#include <pure_pursuit/LQR_core.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>

namespace
{
    inline double kmph2mps(double kmph)
    {
        return kmph * 1000.0 / 3600.0;
    }
}

namespace waypoint_follower
{
    // Constructor
    LQRNode::LQRNode()//定义纯跟踪节点类的构造函数的具体实现
        : private_nh_("~")//给各成员变量赋初始值
        , LQR_(100)
        , ugv()
        , LOOP_RATE_(30)
        , is_waypoint_set_(false)
        , is_pose_set_(false)
        , is_velocity_set_(false)
        , current_linear_velocity_(0)
        , command_linear_velocity_(0)
        //direction_(LaneDirection::Forward)
        , velocity_source_(-1)
        // , const_lookahead_distance_(4.0)
        , const_velocity_(5.0)
        , k(3.0)
        // , lookahead_distance_ratio_(2.0)
        // , minimum_lookahead_distance_(6.0)
    {
        initForROS();//初始化ROS节点
        // health_checker_ptr_ =
        //     std::make_shared<autoware_health_checker::HealthChecker>(nh_, private_nh_);//nh_专门用于定义发布者和订阅者变量，private_nh_专门用于定义配置参数
        // health_checker_ptr_->ENABLE();//启动智能指针
        // initialize for PurePursuit
        // stan_.setLinearInterpolationParameter(is_linear_interpolation_);//设置是否进行线性插值标志位参数
    }

    // Destructor
    LQRNode::~LQRNode()//定义纯跟踪节点类析构函数的具体实现
    {
    }
    void LQRNode::initForROS()//初始化ROS函数的具体实现
    {
        // ros parameter settings
        private_nh_.param("velocity_source", velocity_source_, 0);//对纯跟踪算法的一些配置参数进行赋值
        private_nh_.param("is_linear_interpolation", is_linear_interpolation_, true);
        private_nh_.param(
            "publishes_for_steering_robot", publishes_for_steering_robot_, false);
        private_nh_.param(
            "add_virtual_end_waypoints", add_virtual_end_waypoints_, false);
        // private_nh_.param("const_lookahead_distance", const_lookahead_distance_, 4.0);
        private_nh_.param("const_velocity", const_velocity_, 5.0);
        private_nh_.param("k_stanley", k, 3.0);
        // private_nh_.param("lookahead_ratio", lookahead_distance_ratio_, 2.0);
        // private_nh_.param(
        //     "minimum_lookahead_distance", minimum_lookahead_distance_, 6.0);
        nh_.param("wheel_base", wheel_base_, 2.7);

        // setup subscriber设置订阅者
        // sub1_ = nh_.subscribe("final_waypoints", 10,
        //     &StanleyNode::callbackFromWayPoints, this);
        sub2_ = nh_.subscribe("gnss_pose", 10,
            &LQRNode::callbackFromCurrentPose, this);
        // sub3_ = nh_.subscribe("config/waypoint_follower", 10,
        //     &StanleyNode::callbackFromConfig, this);
        // sub4_ = nh_.subscribe("gnss_velocity", 10,
        //     &StanleyNode::callbackFromCurrentVelocity, this);

        // setup publisher设置发布者
        pub1_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
        // pub2_ = nh_.advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 10);
        pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
        pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
        pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
        // debug tool
        pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0);
        pub15_ =
            nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
        // pub16_ = nh_.advertise<std_msgs::Float32>("angular_gravity", 0);
        // pub17_ = nh_.advertise<std_msgs::Float32>("deviation_of_current_position", 0);
        pub18_ =
            nh_.advertise<visualization_msgs::Marker>("expanded_waypoints_mark", 0);
        pub19_ = nh_.advertise<visualization_msgs::Marker>("all_waypoints_mark", 0);
        // // pub7_ = nh.advertise<std_msgs::Bool>("wf_stat", 0);
    }

    void LQRNode::run()
    {
        static bool is_waypoints_ready = false;
        // std::cout << "11111111111111" << std::endl;
        if (!is_waypoints_ready)
        {
            // std::cout << "11111111111111" << std::endl;
            std::ifstream file("/home/nvidia/catkin_gps/src/pure_pursuit/src/global_path_with_velocity.csv", std::ios::in);
            // std::cout << "**************" << std::endl;
            std::string line;
            getline(file, line); // 读取第一行数据名称
            std::vector<std::string> headers; // 存储数据名称
            std::stringstream ss(line);
            std::string header;
            // std::cout << "&&&&&&&&&&&&****" << std::endl;
            while (getline(ss, header, ',')) {
                headers.push_back(header);
                // std::cout << "&&&&&&&&&&&&" << std::endl;
            }
            // std::cout << "22222222222222" << std::endl;
            // 2. 逐行读取数据
            std::vector<std::vector<std::string>> data; // 存储所有数据
            while (getline(file, line)) { // 循环读取除第一行外的所有行
                std::vector<std::string> row; // 存储单行数据
                std::stringstream ss(line);
                std::string field;
                while (getline(ss, field, ',')) { // 将逐个字段读取并存储到 row 中
                row.push_back(field);
                // std::cout << row.size() << std::endl;
                }
                data.push_back(row); // 将一行数据存储到 data 中
            }
            // std::cout << data.size() << std::endl;
            // std::cout << "3333333333333" << std::endl;
            for (int i = 0; i < data.size(); i++) { // 循环遍历所有行
                geometry_msgs::Pose waypoint;
                waypoint.position.x = std::stod(data[i][0]);
                waypoint.position.y = std::stod(data[i][1]);
                waypoint.position.z = std::stod(data[i][2]);
                tf2::Quaternion quat;
                quat.setRPY(0, 0, std::stod(data[i][3]));
                geometry_msgs::Quaternion quat_msg;
                tf2::convert(quat, quat_msg);
                // cout << endl;
                waypoint.orientation = quat_msg;
                waypoints.push_back(waypoint);
                // 读取速度并存储
                double velocity = std::stod(data[i][4]);
                velocity = kmph2mps(velocity);
                waypoint_velocities_.push_back(velocity);
                // std::cout << "11111111111111" << std::endl;
            }
            // for (int i = 0; i < waypoints.size(); i++)
            // {
            //     std::cout << waypoints[i].position.x << "---" << waypoints[i].position.y << "---" << waypoints[i].position.z << std::endl;
            // }
            // std::cout << waypoints.size() << std::endl;

            LQR_.setwatpoints(waypoints);
            is_waypoints_ready = true;
        }
        ros::Rate loop_rate(LOOP_RATE_);
        // is_pose_set_ = true;
        while (ros::ok())
        {
            ros::Time start_time = ros::Time::now();
            ros::spinOnce();
            if (!is_pose_set_ || !is_waypoints_ready)//如果位置、路径点和速度三者中的任一变量没有设置则向终端发布警示信息
            {
                ROS_WARN("Necessary topics are not subscribed yet ... ");//警示信息：必要话题至今任未订阅
                // std::cout << "waypoints.size()" << std::endl;
                // for (int i = 0; i < waypoints.size(); i++)
                // {
                //     std::cout << waypoints[i].position.x << "---" << waypoints[i].position.y << "---" << waypoints[i].position.z << std::endl;
                // }
                // ROS_WARN(" ... ");//警示信息：必要话题至今任未订阅
                loop_rate.sleep();
                continue;//跳出单次循环，忽略后面程序，直接进入下一次循环
            }
            // std::cout << "waypoints.size()" << std::endl;
            // LQR_.set_k_parameter(k);//设置控制参数
            // MatrixXd Q(3,3);
            // Q<<200,0,0,
            //         0,200,0,
            //         0,0,100;
            // MatrixXd R(2,2);
            // R<<0.05,0.0,
            //         0.0,0.2;

            // LQR_.set_v_parameter(computeCommandVelocity());
            // LQR_.set_L_parameter(wheel_base_);//设置控制参数
            vector<double>one_trial = LQR_.calTargetIndex();
            double k = one_trial[1],ref_yaw = one_trial[2], s0=one_trial[3], k1=one_trial[4];
            // stan_.setMinimumLookaheadDistance(minimum_lookahead_distance_);//设置最小前瞻量
            double kappa = 0;//初始化曲率

            // --- 新增的自适应Q、R矩阵调整逻辑 ---

            // // // 1. 定义基础值和调节增益 (这些参数需要根据实际车辆进行精细调整)
            // double base_q_xy = 150.0;     // 直线行驶时对位置误差的基础惩罚
            // double gain_q_xy = 500.0;     // 随曲率增加的位置误差惩罚增益
            // double base_q_yaw = 100.0;      // 直线行驶时对航向误差的基础惩罚
            // double gain_q_yaw = 2400.0;     // 随曲率增加的航向误差惩罚增益

            // // // 1. 定义基础值和调节增益 (这些参数需要根据实际车辆进行精细调整)
            // // double base_q_xy = 200.0;     // 直线行驶时对位置误差的基础惩罚
            // // double gain_q_xy = 400.0;     // 随曲率增加的位置误差惩罚增益
            // // double base_q_yaw = 100.0;      // 直线行驶时对航向误差的基础惩罚
            // // double gain_q_yaw = 200.0;     // 随曲率增加的航向误差惩罚增益

            // double base_r_steer = 4.5;     // 直线行驶时对转向控制的基础惩罚
            // double gain_r_steer = 1000;       // 随曲率增加的转向控制惩罚削减量
            // double min_r_steer = 0.001;     // 确保转向惩罚不会过小或为负的下限值

            // // 1. 定义基础值和调节增益 (这些参数需要根据实际车辆进行精细调整)
            double base_q_xy = 150.0;     // 直线行驶时对位置误差的基础惩罚
            double gain_q_xy = 700.0;     // 随曲率增加的位置误差惩罚增益
            double base_q_yaw = 100.0;      // 直线行驶时对航向误差的基础惩罚
            double gain_q_yaw = 2400.0;     // 随曲率增加的航向误差惩罚增益

            // // 1. 定义基础值和调节增益 (这些参数需要根据实际车辆进行精细调整)
            // double base_q_xy = 200.0;     // 直线行驶时对位置误差的基础惩罚
            // double gain_q_xy = 400.0;     // 随曲率增加的位置误差惩罚增益
            // double base_q_yaw = 100.0;      // 直线行驶时对航向误差的基础惩罚
            // double gain_q_yaw = 200.0;     // 随曲率增加的航向误差惩罚增益

            double base_r_steer = 6;     // 直线行驶时对转向控制的基础惩罚
            double gain_r_steer = 1100;       // 随曲率增加的转向控制惩罚削减量
            double min_r_steer = 0.001;     // 确保转向惩罚不会过小或为负的下限值

            // // // 1. 定义基础值和调节增益 (这些参数需要根据实际车辆进行精细调整)
            // double base_q_xy = 200.0;     // 直线行驶时对位置误差的基础惩罚
            // double gain_q_xy = 500.0;     // 随曲率增加的位置误差惩罚增益
            // double base_q_yaw = 150.0;      // 直线行驶时对航向误差的基础惩罚
            // double gain_q_yaw = 2400.0;     // 随曲率增加的航向误差惩罚增益

            // // // 1. 定义基础值和调节增益 (这些参数需要根据实际车辆进行精细调整)
            // // double base_q_xy = 200.0;     // 直线行驶时对位置误差的基础惩罚
            // // double gain_q_xy = 400.0;     // 随曲率增加的位置误差惩罚增益
            // // double base_q_yaw = 100.0;      // 直线行驶时对航向误差的基础惩罚
            // // double gain_q_yaw = 200.0;     // 随曲率增加的航向误差惩罚增益

            // double base_r_steer = 4.5;     // 直线行驶时对转向控制的基础惩罚
            // double gain_r_steer = 3000;       // 随曲率增加的转向控制惩罚削减量
            // double min_r_steer = 0.001;     // 确保转向惩罚不会过小或为负的下限值

            // 2. 根据曲率的绝对值计算自适应的权重值
            // 曲率越大，对位置(x,y)和航向(yaw)的误差惩罚就越大
            double q_xy_val = base_q_xy + gain_q_xy * std::abs(k1);
            double q_yaw_val = base_q_yaw + gain_q_yaw * std::abs(k1);

            // 曲率越大，对转向控制的惩罚就越小，以允许更大胆的转向动作
            double r_steer_val = std::max(min_r_steer, base_r_steer - gain_r_steer * std::abs(k1));

            // 3. 使用计算出的值来设置Q和R矩阵
            MatrixXd Q(3,3);
            // Q矩阵对角线元素分别对应状态量X=[x_err, y_err, yaw_err]的惩罚
            Q << q_xy_val, 0, 0,
                0, q_xy_val, 0,
                0, 0, q_yaw_val;

            MatrixXd R(2,2);
            // R矩阵对角线元素分别对应控制量u=[v_correction, delta_correction]的惩罚
            R << 0.05, 0.0,          // 对速度变化的惩罚，暂时保持不变
                0.0, r_steer_val;   // 对转向角变化的自适应惩罚

            // --- 自适应逻辑结束 ---

            // geometry_msgs::Pose current_ref_pose;
            // if(s0 < waypoints.size() - 10)
            // {
            //     current_ref_pose = waypoints[s0 + 10];
            // }
            // bool can_get_curvature = LQR_.stanleyControl(&kappa);//判断是否可以正常获取曲率
            // double ref_delta = atan2(current_ref_pose.position.y - LQR_.getCurrentPose().position.y, current_ref_pose.position.x - LQR_.getCurrentPose().position.x);
            // tf::Quaternion quat_ref2;
            // tf::quaternionMsgToTF(LQR_.getCurrentPose().orientation, quat_ref2);
            // double roll_ref2, pitch_ref2, yaw_ref2;//定义存储roll,pitch and yaw的容器
            // tf::Matrix3x3(quat_ref2).getRPY(roll_ref2, pitch_ref2, yaw_ref2); //进行转换
            // ref_delta = ref_delta - yaw_ref2;
            // MODIFICADO: 根据 velocity_source_ 参数选择目标速度
            double target_velocity = 0.0;
            if (velocity_source_ == 0) // 模式 0: 使用CSV文件中的速度
            {
                if (s0 >= 0 && s0 < waypoint_velocities_.size())
                {
                    target_velocity = waypoint_velocities_[static_cast<size_t>(s0)];
                }
            }
            else // 模式 1 (或其他值): 使用恒定速度
            {
                target_velocity = kmph2mps(const_velocity_);
            }
            
            LQR_.set_v_parameter(target_velocity); 
            LQR_.set_L_parameter(wheel_base_);

            double ref_delta = atan2(wheel_base_*k,1);
            ros::Duration elapsed_time = ros::Time::now() - start_time;
            // cout << elapsed_time << endl;
            double dt_2 = elapsed_time.toSec();
            // cout << "************" << ref_delta << endl;
            // cout << "************" << ref_yaw << endl;
            vector<MatrixXd>state_space = ugv.stateSpace(ref_delta,ref_yaw,0.05,target_velocity,wheel_base_);
            geometry_msgs::Pose current_pose = LQR_.getCurrentPose();
            // cout << state_space[0] << endl;
            // cout << state_space[1] << endl;
            // cout << Q << endl;
            // cout << R << endl;
            double delta = LQR_.lqrControl(current_pose, waypoints, s0, state_space[0], state_space[1], Q, R);
            delta = delta+ref_delta;
            if(delta > M_PI * 0.5)
            {
                delta =  M_PI * 0.5;
            }
            else if (delta < -M_PI * 0.5)
            {
                delta =  -M_PI * 0.5;
            }
            publishTwistStamped(delta, s0, target_velocity);//重点：调用发布速度指令函数
            pub11_.publish(displayNextWaypoint(LQR_.getPoseOfNextWaypoint()));//调用发布下一个路径点可视化函数
            // pub13_.publish(displaySearchRadius(
            // stan_.getCurrentPose().position, stan_.getLookaheadDistance()));//调用发布搜索半径可视化函数
            // pub12_.publish(displayNextTarget(stan_.getPoseOfNextTarget()));//调用发布下一目标点可视化函数
            kappa = tan(delta) / wheel_base_;
            pub15_.publish(displayTrajectoryCircle(
                waypoint_follower::generateTrajectoryCircle(
                kappa, LQR_.getCurrentPose())));//调用发布轨迹圆可视化函数
            // if (add_virtual_end_waypoints_)//如果选择添加虚拟最终路径点
            // {
            // pub18_.publish(
            //     displayExpandWaypoints(stan_.getCurrentWaypoints(), expand_size_));//则调用发布扩展路径点可视化函数
            // }
            pub19_.publish(displayallWaypoints(waypoints));
            is_pose_set_ = false;
            is_velocity_set_ = false;
            //完成上述所有操作后将位姿、速度、路径点标志位置于false

            loop_rate.sleep();//程序挂起休眠一段时间，等待当前位姿、速度和路径点的更新
        }
    }
    // double StanleyNode::computeLookaheadDistance() const//定义计算前瞻量函数的具体实现
    // {
    //     if (velocity_source_ == enumToInteger(Mode::dialog))//若模式为手动模式，则采用固定前瞻量
    //     {
    //         return const_lookahead_distance_;
    //     }

    //     double maximum_lookahead_distance = current_linear_velocity_ * 10;
    //     double ld = current_linear_velocity_ * lookahead_distance_ratio_;//若采用路径点速度模式，则前瞻量由速度决定

    //     return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ :
    //         ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;//判断式，保证前瞻量处于最大和最小前瞻量之间
    // }

    double LQRNode::computeCommandVelocity() const//定义计算命令速度的具体实现方式
    {
        if (velocity_source_ == enumToInteger(Mode::dialog))
        {
            return kmph2mps(const_velocity_);//当模式设定为手动时，速度设定为常数
        }

        return command_linear_velocity_;//当模式为路径点模式时，则返回路径点要求跟踪速度
    }

    void LQRNode::callbackFromCurrentPose(//定义订阅整车当前位姿的回调函数具体实现方式
    const geometry_msgs::PoseStampedConstPtr& msg)
    {
        static double last_time = 0;
        // cout << ros::Time::now().toSec() - last_time << endl;
        LQR_.setCurrentPose(msg);
        is_pose_set_ = true;
        last_time = ros::Time::now().toSec();
    }

    void LQRNode::publishTwistStamped(//发布速度指令函数
    double delta, double s0, double target_velocity) const
    {
        geometry_msgs::TwistStamped ts;
        ts.header.stamp = ros::Time::now();
        // cout << delta << endl;
        ts.twist.linear.x = target_velocity;//判断能否获得曲率，若能则计算命令速度，若不能则将速度置为0
        ts.twist.angular.z = tan(delta) / wheel_base_ * ts.twist.linear.x;//判断能否获得曲率，若能则计算角速度，若不能则将角速度置为0
        // int current_index = LQR_.getIndexofNextWaypoint();
        int all_size = LQR_.getCurrentWaypoints().size() - 1;
        // cout << s0 << endl;
        // cout << all_size << endl;
        if(s0 == all_size)
        {
            ts.twist.linear.x = 0;
            ts.twist.angular.z = 0;
        }
        pub1_.publish(ts);//调用发布函数，将速度指令发布出去
    }



}
