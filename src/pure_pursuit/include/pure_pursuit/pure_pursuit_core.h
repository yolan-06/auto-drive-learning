#ifndef PURE_PURSUIT_PURE_PURSUIT_CORE_H
#define PURE_PURSUIT_PURE_PURSUIT_CORE_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>

// C++ includes
#include <vector>

#include <pure_pursuit/pure_pursuit.h>
#include <pure_pursuit/pure_pursuit_viz.h>

namespace waypoint_follower
{
    enum class Mode : int32_t
    {
        waypoint,
        dialog,

        unknown = -1,
    };//定义枚举类

    template <class T>
    typename std::underlying_type<T>::type enumToInteger(T t)
    {
        return static_cast<typename std::underlying_type<T>::type>(t);
    }//创建模板函数获取枚举模式对应的值
    class PurePursuitNode
    {
    public:
        PurePursuitNode();//定义纯跟踪类的构造函数和析构函数
        ~PurePursuitNode();

        void run();
    private:
        ros::NodeHandle nh_;//定义ros句柄
        ros::NodeHandle private_nh_;
        PurePursuit pp_;//实例化纯跟踪类
        ros::Publisher pub1_, pub2_,
        pub11_, pub12_, pub13_, pub14_, pub15_, pub18_,pub19_;
        ros::Subscriber sub2_;

        const int LOOP_RATE_;  // processing frequency（定义处理频率）

        // variables
        bool is_linear_interpolation_, publishes_for_steering_robot_,//定义多个必要的成员变量
            add_virtual_end_waypoints_;
        bool is_waypoint_set_, is_pose_set_, is_velocity_set_;
        double current_linear_velocity_, command_linear_velocity_;
        double wheel_base_;
        int expand_size_;
        int32_t velocity_source_;          // 0 = waypoint, 1 = Dialog
        double const_lookahead_distance_;  // meter
        double const_velocity_;            // km/h
        std::vector<double> waypoint_velocities_; // 存储每个航点速度的向量
        double lookahead_distance_ratio_;
        // the next waypoint must be outside of this threshold.
        double minimum_lookahead_distance_;
        std::vector<geometry_msgs::Pose> waypoints;

        void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
        void callbackFromCurrentVelocity(
            const geometry_msgs::TwistStampedConstPtr& msg);

        void initForROS();//定义初始化函数

        // functions
        void publishTwistStamped(//发布速度指令函数
        const bool& can_get_curvature, const double& kappa, const double& target_velocity) const;//定义发布控制命令函数
        void publishControlCommandStamped(
            const bool& can_get_curvature, const double& kappa) const;
        //   void publishDeviationCurrentPosition(
        //     const geometry_msgs::Point& point,
        //     const std::vector<autoware_msgs::Waypoint>& waypoints) const;//定义发布当前位置偏移量函数
        //   void connectVirtualLastWaypoints(
        //     autoware_msgs::Lane* expand_lane, LaneDirection direction);//定义连接虚拟最终路点函数

        int getSgn() const;//获取标志位
        double computeLookaheadDistance() const;//计算前瞻量
        double computeCommandVelocity() const;//计算命令速度
        double computeCommandAccel() const;//计算命令加速度
        double computeAngularGravity(double velocity, double kappa) const;//计算角速度
    };
    double convertCurvatureToSteeringAngle(
    const double& wheel_base, const double& kappa);//将曲率转换为转角

    inline double kmph2mps(double velocity_kmph)//将Km/h转为m/s
    {
        return (velocity_kmph * 1000) / (60 * 60);
    }
} // namespace way


#endif  // PURE_PURSUIT_PURE_PURSUIT_CORE_H