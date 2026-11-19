
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <gnss_localizer/hcinspvatzcb.h>

#include <iostream>
#include <Eigen/Core>

#include "gnss_localizer/utils.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>

const double PI = M_PI;

static ros::Publisher pose_publisher;

static ros::Publisher stat_publisher, path_publisher, odometry_publisher;
static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
nav_msgs::Path nav_path_;

static double yaw;
// true if position history is long enough to compute orientation
static bool _orientation_ready = false;

Eigen::Vector3d init_lla_; // The initial reference gps point.
static bool _first_fix_gnss = false;


bool _save_origin = false;
bool _load_origin = false;
double ori_lat, ori_lon, ori_alt;

static void GNSSCallback(const gnss_localizer::hcinspvatzcb::ConstPtr &msg)
{
  // Check the gps_status.
  if (msg->stat[1] == 4)
  {
  }
  else
  {
    ROS_WARN("[GpsCallBack]: Bad gps message!");
    // return;
  }

  if (!_first_fix_gnss)
  {
    init_lla_ = Eigen::Vector3d(msg->latitude,
                                msg->longitude,
                                msg->altitude);
    _first_fix_gnss = true;
    if (_save_origin)
    {
      //  TODO: save origin
      ROS_WARN("ori gnss: %.8f,%.8f,%.8f", init_lla_[0], init_lla_[1], init_lla_[2]);
    }
    ROS_WARN("first gnss: %.8f,%.8f,%.8f", init_lla_[0], init_lla_[1], init_lla_[2]);
  }
  if (_load_origin)
  {
    init_lla_ = Eigen::Vector3d(ori_lat, ori_lon, ori_alt);
  }

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;

  Eigen::Vector3d enu;
  ImuGpsLocalization::ConvertLLAToENU(init_lla_, Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude), &enu);

  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  // pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  // pose.pose.position.x = enu(0) + 46.5;
  // pose.pose.position.y = enu(1) + 3.5;  //卫星建图定位
  pose.pose.position.x = enu(0);
  pose.pose.position.y = enu(1);
  pose.pose.position.z = enu(2);

  // set gnss_stat
  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
  }
  else
  {
    gnss_stat_msg.data = true;
  }

  // double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
  //                        pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
  // std::cout << "distance : " << distance << std::endl;

  // if (distance > 0.02)
  // {
  //   yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
  //   _quat = tf::createQuaternionMsgFromYaw(yaw);
  //   _prev_pose = pose;
  //   _orientation_ready = true;
  // }
  // yaw = -(msg->heading2 / 180 * M_PI - M_PI / 2);
  yaw = - msg->heading2 / 180 * M_PI + M_PI / 2;
  if (yaw < - M_PI)
  {
    yaw = yaw + 2 * M_PI;
  }
  else if (yaw > M_PI)
  {
    yaw = yaw - 2 * M_PI;
  }
  
  _quat = tf::createQuaternionMsgFromYaw(yaw);
  _orientation_ready = true;

  if (_orientation_ready)
  {
    //  pub pose
    pose.pose.orientation = _quat;

    // // 定义静态变量
    // static Eigen::Matrix4d transform_matrix(
    //     0.61608979, -0.78763759, -0.00777113, -3.48256211,
    //     0.78762641,  0.6161317,  -0.00513516, -1.76254284,
    //     0.00883269, -0.00295703, 0.99995662, -0.05768201,
    //     0,           0,           0,           1
    // );
    // // 将Pose消息转换为TF Transform
    // tf::Pose tf_pose;
    // tf::poseMsgToTF(pose.pose, tf_pose);

    // // 应用变换矩阵
    // tf::Pose transformed_pose = tf_pose * tf::Pose(tf::Matrix3x3(transform_matrix.topLeftCorner<3, 3>()), tf::Vector3(transform_matrix.topRightCorner<3, 1>()));

    // static Eigen::Matrix4d transform_matrix(
    //     0.61608979, -0.78763759, -0.00777113, -3.48256211,
    //     0.78762641,  0.6161317,  -0.00513516, -1.76254284,
    //     0.00883269, -0.00295703, 0.99995662, -0.05768201,
    //     0.000,       0.000,      0.000,       1.000);

    // Eigen::Affine3d transform1;
    // transform1.linear() = Eigen::Matrix3d::Identity(); // 设置为单位矩阵
    // // transform1.translation() << 5.37596529, -5.46160298, 0.05011102;
    // // transform1.translation() << -7.66318131, 0.002697826, 0.08825404;
    // transform1.translation() << -0.15, 0.6575389, 1.8;

    // // 如果你需要设置旋转矩阵
    // Eigen::Matrix3d rotation_matrix;
    // // rotation_matrix << 0.75062337, -0.85255263, 0.00261115,
    // //                   0.68922307, 0.80357035, 0.58538066,
    // //                   0.28530417, -0.58169091, 0.81070418;
    // rotation_matrix << 0.706289369, -0.707923223, 0.000194125,
    //                   0.707923250, 0.706289350, -0.000163287,
    //                   -0.000021514, 0.000252753589, 0.999999968;
    // transform1.linear() = rotation_matrix;

    // // 转换为4x4矩阵
    // Eigen::Matrix4d transform_matrix = transform1.matrix();

    // // 从Eigen矩阵中提取平移向量
    // double tx = transform_matrix(0, 3);
    // double ty = transform_matrix(1, 3);
    // double tz = transform_matrix(2, 3);

    // // 创建tf::Vector3对象
    // tf::Vector3 tf_translation(tx, ty, tz);

    // // 从Eigen矩阵中提取旋转矩阵
    // tf::Matrix3x3 tf_rotation_matrix(
    //     transform_matrix(0, 0), transform_matrix(0, 1), transform_matrix(0, 2),
    //     transform_matrix(1, 0), transform_matrix(1, 1), transform_matrix(1, 2),
    //     transform_matrix(2, 0), transform_matrix(2, 1), transform_matrix(2, 2)
    // );

    // // 创建tf::Quaternion对象
    // tf::Quaternion tf_rotation;
    // tf_rotation_matrix.getRotation(tf_rotation);

    // // 创建tf::Transform对象
    // tf::Transform tf_transform2;
    // tf_transform2.setOrigin(tf_translation);
    // tf_transform2.setRotation(tf_rotation);

    // // 将Pose消息转换为TF Transform
    // tf::Pose tf_pose;
    // tf::poseMsgToTF(pose.pose, tf_pose);

    // // 应用变换矩阵
    // tf::Pose transformed_pose;
    // transformed_pose = tf_transform2 * tf_pose;

    // // 将变换后的TF Transform转换回Pose消息
    // geometry_msgs::PoseStamped transformed_pose_msg;
    // tf::poseTFToMsg(transformed_pose, transformed_pose_msg.pose);

    // // 保留原始消息的头部信息
    // transformed_pose_msg.header = pose.header;

    nav_msgs::Odometry odometry_pose;
    odometry_pose.header = pose.header;
    odometry_pose.pose.pose = pose.pose;
    odometry_pose.pose.covariance[0] = 0.01;
    odometry_pose.pose.covariance[7] = 0.01;
    odometry_pose.pose.covariance[14] = 0.01;

    pose_publisher.publish(pose);
    // pose_publisher.publish(transformed_pose_msg);
    odometry_publisher.publish(odometry_pose);
    stat_publisher.publish(gnss_stat_msg);

    // publish the path
    nav_path_.header = pose.header;
    nav_path_.poses.push_back(pose);
    // nav_path_.poses.push_back(transformed_pose_msg);
    path_publisher.publish(nav_path_);

    //  pub tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    q.setRPY(0, 0, yaw);
    // transform3.setOrigin(tf::Vector3(transformed_pose_msg.pose.position.x, transformed_pose_msg.pose.position.y, transformed_pose_msg.pose.position.z));
    // tf::Quaternion q_1(transformed_pose_msg.pose.orientation.x, transformed_pose_msg.pose.orientation.y, transformed_pose_msg.pose.orientation.z, transformed_pose_msg.pose.orientation.w);
    // tf::Matrix3x3 m(q_1);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    // q.setRPY(roll, pitch, yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "gps"));

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

  ROS_WARN("load ori: %.8f,%.8f,%.8f", ori_lat, ori_lon, ori_alt);

  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
  stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
  path_publisher = nh.advertise<nav_msgs::Path>("nav_path", 10);
  odometry_publisher = nh.advertise<nav_msgs::Odometry> ("gnss_odometry", 1000);

  ros::Subscriber gnss_pose_subscriber = nh.subscribe("/fix", 100, GNSSCallback);

  ros::spin();
  return 0;
}