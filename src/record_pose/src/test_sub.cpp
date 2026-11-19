#include "ros/ros.h"
//#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "autoware_msgs/VehicleCmd.h"

void domsgto(const autoware_msgs::VehicleCmd::ConstPtr &p)
{
    ROS_INFO("%.2f,%.2f",p->twist_cmd.twist.linear.x,p->twist_cmd.twist.angular.z);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<autoware_msgs::VehicleCmd>("/vehicle_cmd",1000,domsgto);
    ros::spin();
    return 0;
}