#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "autoware_msgs/VehicleCmd.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include "std_msgs/Float64.h"
#include "autoware_msgs/ControlCommand.h"
#include <cstdlib>
#include <math.h>
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
using namespace std;

void callTwist(const geometry_msgs::TwistStamped::ConstPtr &v);
ros::Publisher pub1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pp_test");
    ros::NodeHandle nh;
    pub1 = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub2 = nh.subscribe<geometry_msgs::TwistStamped>("/twist_cmd", 1000, callTwist);
    ros::spin();
    return 0;
}

void callTwist(const geometry_msgs::TwistStamped::ConstPtr &v)
{
    geometry_msgs::Twist a;
    a.linear.x = v->twist.linear.x;
    a.angular.z = v->twist.angular.z;
    pub1.publish(a);
    // cout << v->linear.x << "      " << v->angular.z << endl;
}