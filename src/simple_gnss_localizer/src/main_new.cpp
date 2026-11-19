// #include <ros/ros.h>
// #include <geometry_msgs/PointStamped.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/Twist.h>
// #include <sensor_msgs/NavSatFix.h>
// #include <sensor_msgs/Imu.h>
// #include <std_msgs/Bool.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/tf.h>
// #include <gnss_localizer/hcinspvatzcb.h>

// #include <iostream>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

// #include "gnss_localizer/utils.h"
// #include <Eigen/Dense>
// #include <cmath>


// const double PI = M_PI;

// static ros::Publisher pose_publisher;
// static ros::Publisher stat_publisher, path_publisher, odometry_publisher;
// static std_msgs::Bool gnss_stat_msg;

// static geometry_msgs::PoseStamped _prev_pose;
// static geometry_msgs::Quaternion _quat;
// nav_msgs::Path nav_path_;

// static double yaw;
// static bool _orientation_ready = false;

// Eigen::Vector3d init_lla_;
// static bool _first_fix_gnss = false;

// bool _save_origin = false;
// bool _load_origin = false;
// double ori_lat, ori_lon, ori_alt;

// static void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
// {
//     // 从IMU消息中提取四元数并转换为偏航角
//     tf::Quaternion q(
//         msg->orientation.x,
//         msg->orientation.y,
//         msg->orientation.z,
//         msg->orientation.w);
//     tf::Matrix3x3 m(q);
//     double roll, pitch;
//     m.getRPY(roll, pitch, yaw);

//     // 确保yaw在[-π, π]范围内
//     if (yaw < -PI) {
//         yaw += 2 * PI;
//     } else if (yaw > PI) {
//         yaw -= 2 * PI;
//     }

//     _quat = tf::createQuaternionMsgFromYaw(yaw);
//     _orientation_ready = true;
// }

// static void GNSSCallback(const gnss_localizer::hcinspvatzcb::ConstPtr &msg)
// {
//     // 检查GPS状态
//     if (msg->stat[1] != 4)
//     {
//         ROS_WARN("[GpsCallBack]: Bad gps message!");
//         return;
//     }

//     if (!_first_fix_gnss)
//     {
//         init_lla_ = Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude);
//         _first_fix_gnss = true;
//         if (_save_origin)
//         {
//             ROS_WARN("ori gnss: %.8f,%.8f,%.8f", init_lla_[0], init_lla_[1], init_lla_[2]);
//         }
//         ROS_WARN("first gnss: %.8f,%.8f,%.8f", init_lla_[0], init_lla_[1], init_lla_[2]);
//     }
//     if (_load_origin)
//     {
//         init_lla_ = Eigen::Vector3d(ori_lat, ori_lon, ori_alt);
//     }

//     static tf::TransformBroadcaster pose_broadcaster;
//     tf::Transform pose_transform;
//     tf::Quaternion pose_q;

//     Eigen::Vector3d enu;
//     ImuGpsLocalization::ConvertLLAToENU(init_lla_, Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude), &enu);

//     geometry_msgs::PoseStamped pose;
//     pose.header = msg->header;
//     pose.header.frame_id = "map";
//     pose.pose.position.x = enu(0);
//     pose.pose.position.y = enu(1);
//     pose.pose.position.z = enu(2);

//     // 设置gnss_stat
//     if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
//     {
//         gnss_stat_msg.data = false;
//     }
//     else
//     {
//         gnss_stat_msg.data = true;
//     }

//     if (_orientation_ready)
//     {
//         pose.pose.orientation = _quat;

//         nav_msgs::Odometry odometry_pose;
//         odometry_pose.header = pose.header;
//         odometry_pose.pose.pose = pose.pose;
//         odometry_pose.pose.covariance[0] = 0.01;
//         odometry_pose.pose.covariance[7] = 0.01;
//         odometry_pose.pose.covariance[14] = 0.01;

//         pose_publisher.publish(pose);
//         odometry_publisher.publish(odometry_pose);
//         stat_publisher.publish(gnss_stat_msg);

//         // 发布路径
//         nav_path_.header = pose.header;
//         nav_path_.poses.push_back(pose);
//         path_publisher.publish(nav_path_);

//         // 发布TF变换
//         static tf::TransformBroadcaster br;
//         tf::Transform transform;
//         tf::Quaternion q;
//         transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
//         q.setRPY(0, 0, yaw);
//         transform.setRotation(q);
//         br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "gps"));
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "gnss_localizer");
//     ros::NodeHandle nh;
//     ros::NodeHandle pnh("~");

//     // 获取参数
//     pnh.getParam("save_origin", _save_origin);
//     pnh.getParam("load_origin", _load_origin);
//     pnh.getParam("ori_lat", ori_lat);
//     pnh.getParam("ori_lon", ori_lon);
//     pnh.getParam("ori_alt", ori_alt);

//     // 从参数服务器获取话题名称
//     std::string gnss_topic, imu_topic;
//     pnh.param<std::string>("gnss_topic", gnss_topic, "/fix");
//     pnh.param<std::string>("imu_topic", imu_topic, "/imu");

//     ROS_WARN("load ori: %.8f,%.8f,%.8f", ori_lat, ori_lon, ori_alt);

//     pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
//     stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
//     path_publisher = nh.advertise<nav_msgs::Path>("nav_path", 10);
//     odometry_publisher = nh.advertise<nav_msgs::Odometry>("gnss_odometry", 1000);

//     ros::Subscriber gnss_pose_subscriber = nh.subscribe(gnss_topic, 100, GNSSCallback);
//     ros::Subscriber imu_subscriber = nh.subscribe(imu_topic, 100, IMUCallback);

//     ros::spin();
//     return 0;
// }


// #include <ros/ros.h>
// #include <geometry_msgs/PointStamped.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/Twist.h>
// #include <sensor_msgs/NavSatFix.h>
// #include <sensor_msgs/Imu.h>
// #include <std_msgs/Bool.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/tf.h>
// #include <gnss_localizer/hcinspvatzcb.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>

// #include <iostream>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <Eigen/Dense>
// #include <cmath>

// #include "gnss_localizer/utils.h"

// const double PI = M_PI;

// static ros::Publisher pose_publisher;
// static ros::Publisher stat_publisher, path_publisher, odometry_publisher;
// static std_msgs::Bool gnss_stat_msg;

// static geometry_msgs::PoseStamped _prev_pose;
// static geometry_msgs::Quaternion _quat;
// nav_msgs::Path nav_path_;

// static double yaw;
// static bool _orientation_ready = false;

// Eigen::Vector3d init_lla_;
// static bool _first_fix_gnss = false;

// bool _save_origin = false;
// bool _load_origin = false;
// double ori_lat, ori_lon, ori_alt;

// static void syncCallback(const sensor_msgs::NavSatFixConstPtr &gnss_msg, 
//                         const sensor_msgs::Imu::ConstPtr& imu_msg)
// {
//     // 调试日志：确认接收到数据
//     ROS_INFO("SyncCallback: GNSS [lat=%.8f, lon=%.8f, alt=%.8f, stat[1]=%d], IMU [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
//              gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude, gnss_msg->stat[1],
//              imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);

//     // 处理 IMU 数据
//     tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, 
//                      imu_msg->orientation.z, imu_msg->orientation.w);
//     tf::Matrix3x3 m(q);
//     double roll, pitch;
//     m.getRPY(roll, pitch, yaw);
//     if (yaw < -PI) yaw += 2 * PI;
//     else if (yaw > PI) yaw -= 2 * PI;
//     _quat = tf::createQuaternionMsgFromYaw(yaw);
//     _orientation_ready = true;
//     ROS_INFO("IMU yaw: %.3f, orientation_ready: %d", yaw, _orientation_ready);

//     // // 处理 GNSS 数据
//     // if (gnss_msg->stat[1] != 4)
//     // {
//     //     ROS_WARN("[GpsCallBack]: Bad gps message, stat[1]=%d", gnss_msg->stat[1]);
//     //     return;
//     // }

//     // Check the gps_status.
//     if (msg->status.status == 2 || msg->status.status == 0)
//     {
//     }
//     else
//     {
//       ROS_WARN("[GpsCallBack]: Bad gps message!");
//       // return;
//     }

//     if (!_first_fix_gnss)
//     {
//         init_lla_ = Eigen::Vector3d(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude);
//         _first_fix_gnss = true;
//         if (_save_origin)
//         {
//             ROS_WARN("ori gnss: %.8f,%.8f,%.8f", init_lla_[0], init_lla_[1], init_lla_[2]);
//         }
//         ROS_INFO("first gnss: %.8f,%.8f,%.8f", init_lla_[0], init_lla_[1], init_lla_[2]);
//     }
//     if (_load_origin)
//     {
//         init_lla_ = Eigen::Vector3d(ori_lat, ori_lon, ori_alt);
//         ROS_INFO("Loaded origin: %.8f,%.8f,%.8f", ori_lat, ori_lon, ori_alt);
//     }

//     Eigen::Vector3d enu;
//     ImuGpsLocalization::ConvertLLAToENU(init_lla_, 
//         Eigen::Vector3d(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude), &enu);
//     ROS_INFO("ENU: x=%.3f, y=%.3f, z=%.3f", enu(0), enu(1), enu(2));

//     geometry_msgs::PoseStamped pose;
//     pose.header = gnss_msg->header;
//     pose.header.frame_id = "map";
//     pose.pose.position.x = enu(0);
//     pose.pose.position.y = enu(1);
//     pose.pose.position.z = enu(2);

//     if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
//     {
//         gnss_stat_msg.data = false;
//         ROS_WARN("Invalid ENU coordinates, setting gnss_stat to false");
//     }
//     else
//     {
//         gnss_stat_msg.data = true;
//     }

//     if (_orientation_ready)
//     {
//         pose.pose.orientation = _quat;

//         nav_msgs::Odometry odometry_pose;
//         odometry_pose.header = pose.header;
//         odometry_pose.pose.pose = pose.pose;
//         odometry_pose.pose.covariance[0] = 0.01;
//         odometry_pose.pose.covariance[7] = 0.01;
//         odometry_pose.pose.covariance[14] = 0.01;

//         ROS_INFO("Publishing pose: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f", 
//                  pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw);
//         pose_publisher.publish(pose);
//         odometry_publisher.publish(odometry_pose);
//         stat_publisher.publish(gnss_stat_msg);

//         nav_path_.header = pose.header;
//         nav_path_.poses.push_back(pose);
//         path_publisher.publish(nav_path_);

//         static tf::TransformBroadcaster br;
//         tf::Transform transform;
//         tf::Quaternion q;
//         transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
//         q.setRPY(0, 0, yaw);
//         transform.setRotation(q);
//         br.sendTransform(tf::StampedTransform(transform, gnss_msg->header.stamp, "map", "gps"));
//     }
//     else
//     {
//         ROS_WARN("Orientation not ready, skipping publish");
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "gnss_localizer");
//     ros::NodeHandle nh;
//     ros::NodeHandle pnh("~");

//     pnh.getParam("save_origin", _save_origin);
//     pnh.getParam("load_origin", _load_origin);
//     pnh.getParam("ori_lat", ori_lat);
//     pnh.getParam("ori_lon", ori_lon);
//     pnh.getParam("ori_alt", ori_alt);

//     std::string gnss_topic, imu_topic;
//     pnh.param<std::string>("gnss_topic", gnss_topic, "/fix");
//     pnh.param<std::string>("imu_topic", imu_topic, "/imu");

//     ROS_INFO("Parameters: save_origin=%d, load_origin=%d, ori_lat=%.8f, ori_lon=%.8f, ori_alt=%.8f",
//              _save_origin, _load_origin, ori_lat, ori_lon, ori_alt);
//     ROS_INFO("Subscribing to GNSS topic: %s, IMU topic: %s", gnss_topic.c_str(), imu_topic.c_str());

//     pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
//     stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
//     path_publisher = nh.advertise<nav_msgs::Path>("nav_path", 10);
//     odometry_publisher = nh.advertise<nav_msgs::Odometry>("gnss_odometry", 1000);

//     message_filters::Subscriber<gnss_localizer::hcinspvatzcb> gnss_sub(nh, gnss_topic, 100);
//     message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, imu_topic, 100);
//     message_filters::TimeSynchronizer<gnss_localizer::hcinspvatzcb, sensor_msgs::Imu> sync(gnss_sub, imu_sub, 100);
//     sync.registerCallback(boost::bind(&syncCallback, _1, _2));

//     ros::spin();
//     return 0;
// }


#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>

#include "gnss_localizer/utils.h"

const double PI = M_PI;

static ros::Publisher pose_publisher;
static ros::Publisher stat_publisher, path_publisher, odometry_publisher;
static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
nav_msgs::Path nav_path_;

static double yaw;
static bool _orientation_ready = false;

Eigen::Vector3d init_lla_;
static bool _first_fix_gnss = false;

bool _save_origin = false;
bool _load_origin = false;
double ori_lat, ori_lon, ori_alt;

static void syncCallback(const sensor_msgs::NavSatFixConstPtr& gnss_msg, 
                        const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    // 调试日志：确认接收到数据和时间戳
    double time_diff = fabs(gnss_msg->header.stamp.toSec() - imu_msg->header.stamp.toSec());
    ROS_INFO("SyncCallback: GNSS [lat=%.8f, lon=%.8f, alt=%.8f, status=%d, stamp=%f], "
             "IMU [x=%.3f, y=%.3f, z=%.3f, w=%.3f, stamp=%f], time_diff=%f",
             gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude, gnss_msg->status.status, gnss_msg->header.stamp.toSec(),
             imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w, imu_msg->header.stamp.toSec(),
             time_diff);

    // 检查 IMU 四元数有效性（放宽容差）
    double norm = sqrt(imu_msg->orientation.x * imu_msg->orientation.x +
                       imu_msg->orientation.y * imu_msg->orientation.y +
                       imu_msg->orientation.z * imu_msg->orientation.z +
                       imu_msg->orientation.w * imu_msg->orientation.w);
    if (fabs(norm - 1.0) > 0.1)
    {
        ROS_WARN("Invalid IMU quaternion, norm=%f, skipping", norm);
        return;
    }

    // 处理 IMU 数据
    tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, 
                     imu_msg->orientation.z, imu_msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);
    if (yaw < -PI) yaw += 2 * PI;
    else if (yaw > PI) yaw -= 2 * PI;
    _quat = tf::createQuaternionMsgFromYaw(yaw);
    _orientation_ready = true;
    ROS_INFO("IMU yaw: %.3f, orientation_ready: %d", yaw, _orientation_ready);

    // 检查 GNSS 数据状态
    if (gnss_msg->status.status != 0 && gnss_msg->status.status != 2)
    {
        ROS_WARN("[GpsCallBack]: Bad gps message, status=%d", gnss_msg->status.status);
    }

    if (!_first_fix_gnss)
    {
        init_lla_ = Eigen::Vector3d(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude);
        _first_fix_gnss = true;
        if (_save_origin)
        {
            ROS_WARN("ori gnss: %.8f,%.8f,%.8f", init_lla_[0], init_lla_[1], init_lla_[2]);
        }
        ROS_INFO("first gnss: %.8f,%.8f,%.8f", init_lla_[0], init_lla_[1], init_lla_[2]);
    }
    if (_load_origin)
    {
        init_lla_ = Eigen::Vector3d(ori_lat, ori_lon, ori_alt);
        ROS_INFO("Loaded origin: %.8f,%.8f,%.8f", ori_lat, ori_lon, ori_alt);
    }

    Eigen::Vector3d enu;
    ImuGpsLocalization::ConvertLLAToENU(init_lla_, 
        Eigen::Vector3d(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude), &enu);
    ROS_INFO("ENU: x=%.3f, y=%.3f, z=%.3f", enu(0), enu(1), enu(2));

    geometry_msgs::PoseStamped pose;
    pose.header = gnss_msg->header;
    pose.header.frame_id = "map";
    pose.pose.position.x = enu(0);
    pose.pose.position.y = enu(1);
    pose.pose.position.z = enu(2);

    // 放宽 ENU 坐标检查，仅检查 x 和 y
    if (pose.pose.position.x == 0.0 && pose.pose.position.y == 0.0)
    {
        gnss_stat_msg.data = false;
        ROS_WARN("Invalid ENU coordinates, setting gnss_stat to false");
    }
    else
    {
        gnss_stat_msg.data = true;
    }

    // 计算与前一位置的距离
    double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                          pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
    ROS_INFO("Distance to previous pose: %.3f", distance);

    if (_orientation_ready)
    {
        pose.pose.orientation = _quat;

        nav_msgs::Odometry odometry_pose;
        odometry_pose.header = pose.header;
        odometry_pose.pose.pose = pose.pose;
        odometry_pose.pose.covariance[0] = 0.01;
        odometry_pose.pose.covariance[7] = 0.01;
        odometry_pose.pose.covariance[14] = 0.01;

        ROS_INFO("Publishing pose: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f", 
                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw);
        pose_publisher.publish(pose);
        odometry_publisher.publish(odometry_pose);
        stat_publisher.publish(gnss_stat_msg);

        nav_path_.header = pose.header;
        nav_path_.poses.push_back(pose);
        path_publisher.publish(nav_path_);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
        q.setRPY(0, 0, yaw);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, gnss_msg->header.stamp, "map", "gps"));

        _prev_pose = pose;
    }
    else
    {
        ROS_WARN("Orientation not ready, skipping publish");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss_localizer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("save_origin", _save_origin);
    pnh.getParam("load_origin", _load_origin);
    pnh.getParam("ori_lat", ori_lat);
    pnh.getParam("ori_lon", ori_lon);
    pnh.getParam("ori_alt", ori_alt);

    std::string gnss_topic, imu_topic;
    pnh.param<std::string>("gnss_topic", gnss_topic, "/gps/fix");
    pnh.param<std::string>("imu_topic", imu_topic, "/imu");

    ROS_INFO("Parameters: save_origin=%d, load_origin=%d, ori_lat=%.8f, ori_lon=%.8f, ori_alt=%.8f",
             _save_origin, _load_origin, ori_lat, ori_lon, ori_alt);
    ROS_INFO("Subscribing to GNSS topic: %s, IMU topic: %s", gnss_topic.c_str(), imu_topic.c_str());

    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
    stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
    path_publisher = nh.advertise<nav_msgs::Path>("nav_path", 10);
    odometry_publisher = nh.advertise<nav_msgs::Odometry>("gnss_odometry", 1000);

    message_filters::Subscriber<sensor_msgs::NavSatFix> gnss_sub(nh, gnss_topic, 1000);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, imu_topic, 1000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), gnss_sub, imu_sub);
    sync.setMaxIntervalDuration(ros::Duration(0.02)); // 20ms 容差
    sync.registerCallback(boost::bind(&syncCallback, _1, _2));

    ros::spin();
    return 0;
}