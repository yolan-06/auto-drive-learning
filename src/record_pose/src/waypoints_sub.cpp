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
#include "autoware_msgs/LaneArray.h"
#include "std_msgs/Header.h"
#include "autoware_msgs/Waypoint.h"
#include "/home/autowareapp/record_pose_ws/src/record_pose/src/autoware_msgs/Lane.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"

using namespace std;

void callwaypoints(const autoware_msgs::Lane::ConstPtr &p);
vector<double> p_x;
vector<double> p_y;
vector<double> p_z;
vector<double> p_yaw;
vector<int> p_index;

void outfile()
{
    // bezier();
    string name = "global_path"+ to_string(p_index[0]) + ".csv";
    // ofstream outfile(name);
    std::ofstream ofs(name.c_str(), std::ios::app);
    // outfile << "x"<< '\t' << "y" << '\t' << "z" << '\t' << 'yaw' << '\t' << 'velocity' << '\t' << 'change_flag' << '\n';
    ofs << "x,y,z,yaw,velocity,change_flag" << std::endl;
    for (int i = 0; i < p_index[0]; i++)
    {
        // outfile << p_x[i] << '\t' << p_y[i] << '\t' << p_z[i] << '\t' << p_yaw[i] << '\t' << 1.5 << '\t' << 0 <<'\n';
        ofs << std::fixed << std::setprecision(4) << p_x[i] << "," << p_y[i] << ","
        << p_z[i] << "," << p_yaw[i] << ",0,0" << std::endl;
        // ofs << std::fixed << std::setprecision(4) << p_x2[i] << "," << p_y2[i] << ","
        // << p_z2[i] << "," << p_yaw2[i] << ",0,0" << std::endl;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoints_sub");
    ros::NodeHandle nh;
    vector<double> p_x;
    vector<double> p_y;
    vector<double> p_z;
    vector<double> p_yaw;
    vector<int> p_index;
    ros::Subscriber sub = nh.subscribe<autoware_msgs::Lane>("/safety_waypoints", 1000, callwaypoints);
    ros::spin();
    outfile();
    return 0;
}

void callwaypoints(const autoware_msgs::Lane::ConstPtr &p)
{
    for (int i = 0; i < p->waypoints.size(); i++)
    {
        p_x.push_back(p->waypoints[i].pose.pose.position.x);
        p_y.push_back(p->waypoints[i].pose.pose.position.y);
        p_z.push_back(p->waypoints[i].pose.pose.position.z);
        tf::Quaternion quat;
        tf::quaternionMsgToTF(p->waypoints[i].pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        p_yaw.push_back(yaw);
        p_index.push_back(p->waypoints.size());
    }
    
}