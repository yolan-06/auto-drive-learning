//
// Created by chh3213 on 2022/11/24.
//

#ifndef PURE_PURSUIT_LQR_H
#define PURE_PURSUIT_LQR_H
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#define EPS 1.0e-4
#include <Eigen/Dense>
#include <vector>
#include <iostream>
using namespace std;
using namespace Eigen;

namespace waypoint_follower
{
    class LQR
    {

        private:
        // constant
        const double RADIUS_MAX_;//最大半径
        const double KAPPA_MIN_;//最小曲率
        int N;

        // variables
        bool is_linear_interpolation_;//是否设置线性插值标志位
        int next_waypoint_number_;//下一路径点索引
        geometry_msgs::Point next_target_position_;//下一个目标位姿
        double lookahead_distance_;//定义前瞻量变量
        double minimum_lookahead_distance_;//定义最小前瞻量变量
        geometry_msgs::Pose current_pose_;//定义当前位姿
        double current_linear_velocity_;//定义当前线速度
        std::vector<geometry_msgs::Pose> waypoints2;
        double k;
        double v_set;
        double L_set;
        //   std::vector<autoware_msgs::Waypoint> current_waypoints_;//定义当前路径点

        // functions
        double normalizeAngle(double angle);
        double calcCurvature(geometry_msgs::Point target) const;//声明计算曲率函数
        bool interpolateNextTarget(
            int next_waypoint, geometry_msgs::Point* next_target) const;//声明线性插值下一目标点函数
        void getNextWaypoint();//声明获取下一路径点函数

        public:
        LQR(int n);//定义纯跟踪类的构造函数和析构函数
        ~LQR();
        // Stanley(double k);
        // vector<double> calcTrackError(vector<double> robot_state);
        vector<double> calTargetIndex();
        // double normalizeAngle(double angle);
        bool stanleyControl(double* out_kappa_stanley);
        MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
        double lqrControl(geometry_msgs::Pose robot_state, vector<geometry_msgs::Pose> refer_path, double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
        void set_k_parameter(const double& k_1)//设置前瞻量
        {
            k = k_1;
        }
        void set_v_parameter(const double& v_1)//设置前瞻量
        {
            v_set = v_1;
        }
        void set_L_parameter(const double& L_1)//设置前瞻量
        {
            L_set = L_1;
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

        int getIndexofNextWaypoint() const
        {
            return next_waypoint_number_;
        }

        std::vector<geometry_msgs::Pose> getCurrentWaypoints() const//获取当前路径点
        {
            return waypoints2;
        }

    };
} // namespace waypoint_follower

// class Stanley {
// private:
//     double k;
// public:
//     Stanley(double k);
    // double calTargetIndex(vector<double>robot_state, vector<vector<double>>refer_path);
    // double normalizeAngle(double angle);
    // vector<double> stanleyControl( vector<double>robot_state, vector<vector<double>>refer_path);
// };


#endif //CHHROBOTICS_CPP_STANLEY_H
