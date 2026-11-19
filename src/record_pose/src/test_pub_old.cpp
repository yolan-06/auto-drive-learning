#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// #include "autoware_msgs/VehicleCmd.h"
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
// #include "autoware_msgs/ControlCommand.h"
#include <cstdlib>
#include <math.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include "sensor_msgs/Imu.h"
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
using namespace std;

class sub_pub
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;
    ros::Subscriber sub5;
    ros::Publisher pub1;
    ros::Publisher pub2;
    //float a[];
    vector<float> a;
    vector<float> b;
    vector<float> a1;
    vector<float> b1;
    int i=0;
    long long xl, xr;
    char cl,cr;
    string sl,sr;
    string control;
    string control1;
    vector<string> ctrlArray;
    vector<string> ctrlArray1;
    int k=0;
    int k2=0;
    vector<int> pwml;
    vector<int> pwmr;
    vector<float> vl;
    vector<float> vr;
    vector<float> delta;
    vector<float> velocity_x;
    vector<float> velocity_y;
    vector<float> angular_z;
    // vector<float> real_time;
    // int mrh=0;
    // double dt = 0.0;

public:
    sub_pub()
    {
        sub1 = nh.subscribe<geometry_msgs::TwistStamped>("/twist_raw",1000,&sub_pub::Callback,this);
        sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/gnss_pose",1000,&sub_pub::Callback2,this);
        // sub3 = nh.subscribe<geometry_msgs::TwistStamped>("/current_velocity",1000,&sub_pub::Callback3,this);
        // sub4 = nh.subscribe<sensor_msgs::PointCloud2>("/points_raw",1000,&sub_pub::Callback4,this);
        // sub5 = nh.subscribe<geometry_msgs::PoseStamped>("/gnss_pose",1000,&sub_pub::Callback5,this);
        pub1 = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    }

    void Callback(const geometry_msgs::TwistStamped::ConstPtr &p1);
    void Callback2(const geometry_msgs::PoseStamped::ConstPtr &p2);
    // void Callback3(const geometry_msgs::TwistStamped::ConstPtr &p3);
    // void Callback4(const sensor_msgs::PointCloud2::ConstPtr &P4);
    // void Callback5(const geometry_msgs::PoseStamped::ConstPtr &p5);
    void outfile();
    // void hex(int l,int r);
    // void outfile2();
    // void outfile3();
    // void outfile4();
    void outfile5();
    // void outfile6();
};

// void sub_pub::hex(int l,int r)
// {
//     pwml.push_back(l);
//     pwmr.push_back(r);
//     k2++;
//     sl = "";
//     sr = "";
//     while (l!=0)
//     {
//         xl = l % 16;
//         if(xl<10)
//         {
//             cl = xl + '0';
//         }
//         else
//         {
//             cl = xl + 'A' - 10;
//         }
//         sl = cl + sl;
//         l = l / 16;
//     }
//     if(sl == "")
//     {
//         cout << 0 << endl;
//     }
//     else
//     {
//         cout << sl << endl;
//     }
//     while (r!=0)
//     {
//         xr = r % 16;
//         if(xr<10)
//         {
//             cr = xr + '0';
//         }
//         else
//         {
//             cr = xr + 'A' - 10;
//         }
//         sr = cr + sr;
//         r = r / 16;
//     }
//     if(sr == "")
//     {
//         cout << 0 << endl;
//     }
//     else
//     {
//         cout << sr << endl;
//     }
//     char templ1, templ2;
//     templ1 = sl[3];
//     sl[3] = sl[1];
//     sl[1] = templ1;
//     templ2 = sl[2];
//     sl[2] = sl[0];
//     sl[0] = templ2;

//     char tempr1, tempr2;
//     tempr1 = sr[3];
//     sr[3] = sr[1];
//     sr[1] = tempr1;
//     tempr2 = sr[2];
//     sr[2] = sr[0];
//     sr[0] = tempr2;

//     // //printf("sl:%s\t,sr:%s\n", sl.c_str(), sr.c_str());

//     cout<<"After change:"<<"sl :"<< sl
//         <<"   sr :"<<sr <<endl;


//     control = "cansend can0 18FF33F0#" + sl + "0000" + sr + "0000";
//     control1 = sl + "0000" + sr + "0000";
//     cout << control << endl;
//     cout << control1 << endl;
//     ros::Time t1 = ros::Time::now();
//     cout << t1 << endl;
//     // real_time.push_back(t1.toSec());
//     // dt = t1.toNSec();
//     // cout << dt << endl;
//     system(control.c_str());
//     system("pwd");
//     ctrlArray.push_back(control);
//     ctrlArray1.push_back(control1);
//     k++;

// }

// void sub_pub::outfile2()
// {
//     string name = "testfile2.xls";
//     ofstream outfile(name);
//     for (int j1 = 0; j1 < k; j1++)
//     {
//         outfile<<ctrlArray[j1]<<'\n';

//     }
// }

// void sub_pub::outfile3()
// {
//     string name = "testfile3.xls";
//     ofstream outfile(name);
//     for (int j2 = 0; j2 < k2; j2++)
//     {
//         outfile << pwml[j2] << '\t' << pwmr[j2] << '\n' ;

//     }
// }

// void sub_pub::outfile4()
// {
//     string name = "testfile4.xls";
//     ofstream outfile(name);
//     for (int j3 = 0; j3 < k; j3++)
//     {
//         outfile<<ctrlArray1[j3]<<'\n';

//     }
// }

void sub_pub::outfile5()
{
    string name = "testfile5.xls";
    ofstream outfile(name);
    for (int j4 = 0; j4 < k2; j4++)
    {
        outfile << vl[j4] << '\t' << vr[j4] << '\n' ;

    }
}

void sub_pub::Callback(const geometry_msgs::TwistStamped::ConstPtr &p1)
{
    geometry_msgs::Twist twist;
    twist.linear.x = p1->twist.linear.x;
    twist.angular.z = p1->twist.angular.z;
    // autoware_msgs::ControlCommand ctrl_excavator;
    // ctrl_excavator.linear_velocity = p1->ctrl_cmd.linear_velocity;
    // cout << ctrl_excavator.linear_velocity << endl;
    // ctrl_excavator.steering_angle = p1->ctrl_cmd.steering_angle;
    // cout << ctrl_excavator.steering_angle << endl;
    // double wheel_base = 2.7;
    // double R = abs(wheel_base/tan(ctrl_excavator.steering_angle));
    // cout << R << endl;
    // double ctrl_excavator_angular = ctrl_excavator.linear_velocity/R;
    // cout << ctrl_excavator_angular << endl;
    // double pedrail_width = 0.4;
    double lefttoright = 2.3;
    double ctrl_excavator_left;
    double ctrl_excavator_right;
    // int can_excavator_left;
    // int can_excavator_right;
    if (p1->twist.angular.z != 0)
    {
        ctrl_excavator_left = p1->twist.linear.x - 2.3 / 2 * p1->twist.angular.z;
        ctrl_excavator_right = p1->twist.linear.x + 2.3 / 2 * p1->twist.angular.z;
    }
    // else if(p1->twist.angular.z < 0)
    // {
    //     ctrl_excavator_left = ctrl_excavator_angular*(R+lefttoright/2);
    //     ctrl_excavator_right = ctrl_excavator_angular*(R-lefttoright/2);
    // }
    else
    {
        ctrl_excavator_left = p1->twist.linear.x;
        ctrl_excavator_right = p1->twist.linear.x;
    }
    vl.push_back(ctrl_excavator_left);
    vr.push_back(ctrl_excavator_right);
    // delta.push_back(ctrl_excavator.steering_angle);
    // cout << ctrl_excavator_left << endl;
    // cout << ctrl_excavator_right << endl;
    // can_excavator_left = int(round((ctrl_excavator_left/1.028)*35000)+5000);
    // can_excavator_right = int(round((ctrl_excavator_right/1.028)*35000)+5000);
    // cout << can_excavator_left << endl;
    // cout << can_excavator_right << endl;
    // // pwml.push_back(can_excavator_left);
    // // pwmr.push_back(can_excavator_right);
    k2++;
    // hex(can_excavator_left,can_excavator_right);
    // pwml.push_back(can_excavator_left);
    // pwmr.push_back(can_excavator_right);
    // k2++;
    pub1.publish(twist);
}

void sub_pub::Callback2(const geometry_msgs::PoseStamped::ConstPtr &p2)
{
    // geometry_msgs::Twist twist;
    // twist.linear.x = p1->twist_cmd.twist.linear.x;
    // twist.angular.z = p1->twist_cmd.twist.angular.z;
    // pub1.publish(twist);
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
    // cout<<a[i]<<b[i]<<endl;
    i++;

}

// void sub_pub::Callback3(const geometry_msgs::TwistStamped::ConstPtr &p3)
// {
//     geometry_msgs::Twist twist;
//     twist.linear.x = p3->twist.linear.x;
//     twist.linear.y = p3->twist.linear.y;
//     twist.angular.z = p3->twist.angular.z;
//     velocity_x.push_back(twist.linear.x);
//     velocity_y.push_back(twist.linear.y);
//     angular_z.push_back(twist.angular.z);
// }

// void sub_pub::Callback5(const geometry_msgs::PoseStamped::ConstPtr &p5)
// {
//     // geometry_msgs::Twist twist;
//     // twist.linear.x = p1->twist_cmd.twist.linear.x;
//     // twist.angular.z = p1->twist_cmd.twist.angular.z;
//     // pub1.publish(twist);
//     geometry_msgs::Pose pose;
//     pose.position.x = p5->pose.position.x;
//     pose.position.y = p5->pose.position.y;
//     a1.push_back(pose.position.x);
//     b1.push_back(pose.position.y);
//     cout<<a1[i]<<b1[i]<<endl;
//     i++;

// }

void sub_pub::outfile()
{
    string name = "testfile.xls";
    ofstream outfile(name);
    for (int j = 0; j < i; j++)
    {
        outfile<<a[j]<<'\t'<<b[j]<<'\t'<<a1[j]<<'\n';

    }
    // outfile.close();

}

// void sub_pub::outfile6()
// {
//     string name = "testfile6.xls";
//     ofstream outfile(name);
//     for (int j = 0; j < i; j++)
//     {
//         outfile<<a1[j]<<'\t'<<b1[j] <<'\n';

//     }
//     //outfile.close();


// }


int main(int argc, char **argv)
{
    ros::init(argc, argv,"sub_pub");
    sub_pub sp;
    ros::spin();
    sp.outfile();
    // sp.outfile2();
    // sp.outfile3();
    // sp.outfile4();
    sp.outfile5();
    // sp.outfile6();
    return 0;
}



























// geometry_msgs::Twist twist;

// void domsg(const autoware_msgs::VehicleCmd::ConstPtr& p)
// {
//     twist.linear.x = p->twist_cmd.twist.linear.x;
//     twist.linear.y = p->twist_cmd.twist.linear.y;
//     twist.linear.z = p->twist_cmd.twist.linear.z;
//     twist.angular.x = p->twist_cmd.twist.angular.x;
//     twist.angular.y = p->twist_cmd.twist.angular.y;
//     twist.angular.z = p->twist_cmd.twist.angular.z;
//     ros::Publisher pub =

// }

// int main(int argc, char *argv[])
// {
//     ros::init(argc,argv,"mycontrol");
//     ros::NodeHandle nh;
//     //ros::NodeHandle nh;
//     //ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);

//     ros::Rate rate(10);


//     //ros::Subscriber sub2 = nh.advertise<geometry_msgs::TwistStamped>("/vehicle_cmd",10,domsg);
//     ros::Subscriber sub = nh.subscribe<autoware_msgs::VehicleCmd>("/vehicle_cmd",1000,domsg);
//     // twist.linear.x=2.0;
//     // twist.linear.y=0.0;
//     // twist.linear.z=0.0;
//     // twist.angular.x=0.0;
//     // twist.angular.y=0.0;
//     // twist.angular.z=2.0;

//     // while (ros::ok())
//     // {
//     //     pub.publish(twist);

//     //     ROS_INFO("%.2f,%.2f",twist.linear.x,twist.angular.z);

//     //     rate.sleep();
//     //     ros::spin();

//     // }




//     return 0;
// }
