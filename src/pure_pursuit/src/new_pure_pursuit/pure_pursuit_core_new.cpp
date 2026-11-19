#include <pure_pursuit/pure_pursuit_core.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include<visualization_msgs/Marker.h>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>


namespace waypoint_follower
{
    // Constructor
    PurePursuitNode::PurePursuitNode()//定义纯跟踪节点类的构造函数的具体实现
        : private_nh_("~")//给各成员变量赋初始值
        , pp_()
        , LOOP_RATE_(30)
        , is_waypoint_set_(false)
        , is_pose_set_(false)
        , is_velocity_set_(false)
        , current_linear_velocity_(0)
        , command_linear_velocity_(0)
        //direction_(LaneDirection::Forward)
        , velocity_source_(-1)
        , const_lookahead_distance_(4.0)
        , const_velocity_(5.0)
        , lookahead_distance_ratio_(2.0)
        , minimum_lookahead_distance_(6.0)
    {
        initForROS();//初始化ROS节点
        // health_checker_ptr_ =
        //     std::make_shared<autoware_health_checker::HealthChecker>(nh_, private_nh_);//nh_专门用于定义发布者和订阅者变量，private_nh_专门用于定义配置参数
        // health_checker_ptr_->ENABLE();//启动智能指针
        // initialize for PurePursuit
        pp_.setLinearInterpolationParameter(is_linear_interpolation_);//设置是否进行线性插值标志位参数
    }

    // Destructor
    PurePursuitNode::~PurePursuitNode()//定义纯跟踪节点类析构函数的具体实现
    {
    }
    void PurePursuitNode::initForROS()//初始化ROS函数的具体实现
    {
        // ros parameter settings
        private_nh_.param("velocity_source", velocity_source_, 0);//对纯跟踪算法的一些配置参数进行赋值
        private_nh_.param("is_linear_interpolation", is_linear_interpolation_, true);
        private_nh_.param(
            "publishes_for_steering_robot", publishes_for_steering_robot_, false);
        private_nh_.param(
            "add_virtual_end_waypoints", add_virtual_end_waypoints_, false);
        private_nh_.param("const_lookahead_distance", const_lookahead_distance_, 4.0);
        private_nh_.param("const_velocity", const_velocity_, 5.0);
        private_nh_.param("lookahead_ratio", lookahead_distance_ratio_, 2.0);
        private_nh_.param(
            "minimum_lookahead_distance", minimum_lookahead_distance_, 6.0);
        private_nh_.param("wheel_base", wheel_base_, 2.7);

        // setup subscriber设置订阅者
        sub1_ = nh_.subscribe("fixed_localpath", 10,
            &PurePursuitNode::callbackFromWayPoints, this);
        sub2_ = nh_.subscribe("gnss_pose", 10,
            &PurePursuitNode::callbackFromCurrentPose, this);
        // sub3_ = nh_.subscribe("config/waypoint_follower", 10,
        //     &PurePursuitNode::callbackFromConfig, this);
        // sub4_ = nh_.subscribe("gnss_velocity", 10,
        //     &PurePursuitNode::callbackFromCurrentVelocity, this);

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

    void PurePursuitNode::run()
    {
        // static bool is_waypoints_ready = false;
        // std::cout << "11111111111111" << std::endl;
        // if (!is_waypoints_ready)
        // {
        //     // std::cout << "11111111111111" << std::endl;
        //     std::ifstream file("/home/fields2cover/control_ws/src/pure_pursuit/src/global_path801.csv", std::ios::in);
        //     // std::cout << "**************" << std::endl;
        //     std::string line;
        //     getline(file, line); // 读取第一行数据名称
        //     std::vector<std::string> headers; // 存储数据名称
        //     std::stringstream ss(line);
        //     std::string header;
        //     // std::cout << "&&&&&&&&&&&&****" << std::endl;
        //     while (getline(ss, header, ',')) {
        //         headers.push_back(header);
        //         // std::cout << "&&&&&&&&&&&&" << std::endl;
        //     }
        //     // std::cout << "22222222222222" << std::endl;
        //     // 2. 逐行读取数据
        //     std::vector<std::vector<std::string>> data; // 存储所有数据
        //     while (getline(file, line)) { // 循环读取除第一行外的所有行
        //         std::vector<std::string> row; // 存储单行数据
        //         std::stringstream ss(line);
        //         std::string field;
        //         while (getline(ss, field, ',')) { // 将逐个字段读取并存储到 row 中
        //         row.push_back(field);
        //         // std::cout << row.size() << std::endl;
        //         }
        //         data.push_back(row); // 将一行数据存储到 data 中
        //     }
        //     // std::cout << data.size() << std::endl;
        //     // std::cout << "3333333333333" << std::endl;
        //     for (int i = 0; i < data.size(); i++) { // 循环遍历所有行
        //         geometry_msgs::Pose waypoint;
        //         waypoint.position.x = std::stod(data[i][0]);
        //         waypoint.position.y = std::stod(data[i][1]);
        //         waypoint.position.z = std::stod(data[i][2]);
        //         tf2::Quaternion quat;
        //         quat.setRPY(0, 0, std::stod(data[i][3]));
        //         geometry_msgs::Quaternion quat_msg;
        //         tf2::convert(quat, quat_msg);
        //         // cout << endl;
        //         waypoint.orientation = quat_msg;
        //         waypoints.push_back(waypoint);
        //         // std::cout << "11111111111111" << std::endl;
        //     }
            // for (int i = 0; i < waypoints.size(); i++)
            // {
            //     std::cout << waypoints[i].position.x << "---" << waypoints[i].position.y << "---" << waypoints[i].position.z << std::endl;
            // }
            // std::cout << waypoints.size() << std::endl;

        //     pp_.setwatpoints(waypoints);
        //     is_waypoints_ready = true;
        // }
        ros::Rate loop_rate(LOOP_RATE_);
        // is_pose_set_ = true;
        while (ros::ok())
        {
            ros::spinOnce();
            if (!is_pose_set_ || !is_waypoint_set_)//如果位置、路径点和速度三者中的任一变量没有设置则向终端发布警示信息
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
            pp_.setLookaheadDistance(computeLookaheadDistance());//设置前瞻量
            pp_.setMinimumLookaheadDistance(minimum_lookahead_distance_);//设置最小前瞻量

            double kappa = 0;//初始化曲率
            bool can_get_curvature = pp_.canGetCurvature(&kappa);//判断是否可以正常获取曲率
            publishTwistStamped(can_get_curvature, kappa);//重点：调用发布速度指令函数
            pub11_.publish(displayNextWaypoint(pp_.getPoseOfNextWaypoint()));//调用发布下一个路径点可视化函数
            pub13_.publish(displaySearchRadius(
            pp_.getCurrentPose().position, pp_.getLookaheadDistance()));//调用发布搜索半径可视化函数
            pub12_.publish(displayNextTarget(pp_.getPoseOfNextTarget()));//调用发布下一目标点可视化函数
            pub15_.publish(displayTrajectoryCircle(
                waypoint_follower::generateTrajectoryCircle(
                pp_.getPoseOfNextTarget(), pp_.getCurrentPose())));//调用发布轨迹圆可视化函数
            if (add_virtual_end_waypoints_)//如果选择添加虚拟最终路径点
            {
            pub18_.publish(
                displayExpandWaypoints(pp_.getCurrentWaypoints(), expand_size_));//则调用发布扩展路径点可视化函数
            }
            pub19_.publish(displayallWaypoints(waypoints));
            is_pose_set_ = false;
            is_velocity_set_ = false;
            is_waypoint_set_ = false;
            //完成上述所有操作后将位姿、速度、路径点标志位置于false

            loop_rate.sleep();//程序挂起休眠一段时间，等待当前位姿、速度和路径点的更新
        }
    }
    double PurePursuitNode::computeLookaheadDistance() const//定义计算前瞻量函数的具体实现
    {
        if (velocity_source_ == enumToInteger(Mode::dialog))//若模式为手动模式，则采用固定前瞻量
        {
            return const_lookahead_distance_;
        }

        double maximum_lookahead_distance = current_linear_velocity_ * 10;
        double ld = current_linear_velocity_ * lookahead_distance_ratio_;//若采用路径点速度模式，则前瞻量由速度决定

        return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ :
            ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;//判断式，保证前瞻量处于最大和最小前瞻量之间
    }

    double PurePursuitNode::computeCommandVelocity() const//定义计算命令速度的具体实现方式
    {
        if (velocity_source_ == enumToInteger(Mode::dialog))
        {
            return kmph2mps(const_velocity_);//当模式设定为手动时，速度设定为常数
        }

        return command_linear_velocity_;//当模式为路径点模式时，则返回路径点要求跟踪速度
    }

    void PurePursuitNode::callbackFromCurrentPose(//定义订阅整车当前位姿的回调函数具体实现方式
    const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pp_.setCurrentPose(msg);
        is_pose_set_ = true;
    }

    void PurePursuitNode::callbackFromWayPoints(const nav_msgs::Path::ConstPtr& msg)
    {
        // 清空之前的路径点
        waypoints.clear();

        // 遍历路径中的每个点
        for (const auto& pose_stamped : msg->poses)
        {
            // 将路径点存入waypoints中
            waypoints.push_back(pose_stamped.pose);
        }

        // 打印接收到的路径点数量
        // ROS_INFO("Received %lu waypoints.", waypoints.size());
        pp_.setwatpoints(waypoints);
        is_waypoint_set_ = true;
    }

    void PurePursuitNode::publishTwistStamped(//发布速度指令函数
    const bool& can_get_curvature, const double& kappa) const
    {
        geometry_msgs::TwistStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;//判断能否获得曲率，若能则计算命令速度，若不能则将速度置为0
        ts.twist.angular.z = can_get_curvature ? kappa * ts.twist.linear.x : 0;//判断能否获得曲率，若能则计算角速度，若不能则将角速度置为0
        pub1_.publish(ts);//调用发布函数，将速度指令发布出去
    }



}
