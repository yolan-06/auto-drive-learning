#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
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
#include <cstdlib>
#include <math.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include "sensor_msgs/Imu.h"

// *** 修改 ***
// 包含在 record_pose 包内定义的 ecu.h 头文件
#include "record_pose/ecu.h"

using namespace std;

class sub_pub
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Publisher pub1;
    vector<float> a;
    vector<float> b;
    vector<float> a1;
    int i=0;
    int k2=0;
    vector<float> vl;
    vector<float> vr;

public:
    sub_pub()
    {
        sub1 = nh.subscribe<geometry_msgs::TwistStamped>("/twist_raw",1000,&sub_pub::Callback,this);
        sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/gnss_pose",1000,&sub_pub::Callback2,this);
        
        // *** 修改 ***
        // 发布的消息类型为 record_pose::ecu
        pub1 = nh.advertise<record_pose::ecu>("/ecu",1000);
    }

    void Callback(const geometry_msgs::TwistStamped::ConstPtr &p1);
    void Callback2(const geometry_msgs::PoseStamped::ConstPtr &p2);
    void outfile();
    void outfile5();
};

void sub_pub::Callback(const geometry_msgs::TwistStamped::ConstPtr &p1)
{
    // *** 修改 ***
    // 创建的消息实例类型为 record_pose::ecu
    record_pose::ecu ecu_msg;

    ecu_msg.header.stamp = ros::Time::now();
    ecu_msg.header.frame_id = "base_link";

    ecu_msg.motor = p1->twist.linear.x * 3.6;

    if (p1->twist.angular.z != 0)
    {
        double radius = p1->twist.linear.x / p1->twist.angular.z;
        if (radius != 0)
        {
            ecu_msg.steer = - atan(0.9 / radius) / 3.1415926 * 180;
        }
        else
        {
            ecu_msg.steer = 0.0;
        }
    }
    else
    {
        ecu_msg.steer = 0.0;
    }

    if (p1->twist.linear.x > 0)
    {
        ecu_msg.brake = false;
        ecu_msg.shift = 1;
    }
    else
    {
        ecu_msg.brake = true;
        ecu_msg.shift = 2;
    }
    
    pub1.publish(ecu_msg);
}

void sub_pub::Callback2(const geometry_msgs::PoseStamped::ConstPtr &p2)
{
    geometry_msgs::Pose pose;
    pose.position.x = p2->pose.position.x;
    pose.position.y = p2->pose.position.y;
    a.push_back(pose.position.x);
    b.push_back(pose.position.y);
    geometry_msgs::Quaternion quat;
    quat = p2->pose.orientation;
    tf::Quaternion quat2;
    tf::quaternionMsgToTF(quat, quat2);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat2).getRPY(roll, pitch, yaw);
    a1.push_back(yaw);
    i++;
}

void sub_pub::outfile()
{
    string name = "testfile.xls";
    ofstream outfile(name);
    for (int j = 0; j < i; j++)
    {
        outfile<<a[j]<<'\t'<<b[j]<<'\t'<<a1[j]<<'\n';
    }
}

void sub_pub::outfile5()
{
    string name = "testfile5.xls";
    ofstream outfile(name);
    for (int j4 = 0; j4 < k2; j4++)
    {
        outfile << vl[j4] << '\t' << vr[j4] << '\n' ;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"sub_pub");
    sub_pub sp;
    ros::spin();
    sp.outfile();
    sp.outfile5();
    return 0;
}