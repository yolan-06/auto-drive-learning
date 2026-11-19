#ifndef PURE_PURSUIT_PURE_PURSUIT_H
#define PURE_PURSUIT_PURE_PURSUIT_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// C++ includes
#include <vector>

namespace waypoint_follower
{
    class PurePursuit
    {
    public:
        PurePursuit();//定义纯跟踪类的构造函数和析构函数
        ~PurePursuit();
        void setLookaheadDistance(const double& ld)//设置前瞻量
        {
            lookahead_distance_ = ld;
        }
        void setMinimumLookaheadDistance(const double& minld)//设置最小前瞻量
        {
            minimum_lookahead_distance_ = minld;
        }
        void setCurrentVelocity(const double& cur_vel)//设置当前速度
        {
            current_linear_velocity_ = cur_vel;
        }
        //   void setCurrentWaypoints(const std::vector<autoware_msgs::Waypoint>& wps)//设置当前路径点
        //   {
        //     current_waypoints_ = wps;
        //   }
        void setCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)//设置当前位姿
        {
            current_pose_ = msg->pose;
        }
        void setLinearInterpolationParameter(const bool& param)//设置是否进行线性插值参数
        {
            is_linear_interpolation_ = param;
        }

        // for debug on ROS
        //   geometry_msgs::Point getPoseOfNextWaypoint() const//获取下一路径点位姿
        //   {
        //     return current_waypoints_.at(next_waypoint_number_).pose.pose.position;
        //   }
        geometry_msgs::Point getPoseOfNextTarget() const//获取下一目标点位姿
        {
            return next_target_position_;
        }
        geometry_msgs::Pose getCurrentPose() const//获取当前整车位姿
        {
            return current_pose_;
        }
        //   std::vector<autoware_msgs::Waypoint> getCurrentWaypoints() const//获取当前路径点
        //   {
        //     return current_waypoints_;
        //   }
        double getLookaheadDistance() const//获取前瞻量
        {
            return lookahead_distance_;
        }
        double getMinimumLookaheadDistance() const//获取最小前瞻量
        {
            return minimum_lookahead_distance_;
        }
        // processing
        bool canGetCurvature(double* output_kappa);//判断是否能够有效获得曲率

        void setwatpoints(std::vector<geometry_msgs::Pose> a_waypoints)
        {
            for (int i = 0; i < a_waypoints.size(); i++)
            {
                waypoints2.push_back(a_waypoints[i]);
            }
        }

        geometry_msgs::Point getPoseOfNextWaypoint() const//获取下一路径点位姿
        {
            return waypoints2.at(next_waypoint_number_).position;
        }

        std::vector<geometry_msgs::Pose> getCurrentWaypoints() const//获取当前路径点
        {
        return waypoints2;
        }

    private:
        // constant
        const double RADIUS_MAX_;//最大半径
        const double KAPPA_MIN_;//最小曲率

        // variables
        bool is_linear_interpolation_;//是否设置线性插值标志位
        int next_waypoint_number_;//下一路径点索引
        geometry_msgs::Point next_target_position_;//下一个目标位姿
        double lookahead_distance_;//定义前瞻量变量
        double minimum_lookahead_distance_;//定义最小前瞻量变量
        geometry_msgs::Pose current_pose_;//定义当前位姿
        double current_linear_velocity_;//定义当前线速度
        std::vector<geometry_msgs::Pose> waypoints2;
        //   std::vector<autoware_msgs::Waypoint> current_waypoints_;//定义当前路径点

        // functions
        double calcCurvature(geometry_msgs::Point target) const;//声明计算曲率函数
        bool interpolateNextTarget(
            int next_waypoint, geometry_msgs::Point* next_target) const;//声明线性插值下一目标点函数
        void getNextWaypoint();//声明获取下一路径点函数
    };
} // namespace waypoint_follower


#endif  // PURE_PURSUIT_PURE_PURSUIT_H