//
// Created by chh3213 on 2022/11/24.
//
#include "pure_pursuit/LQR.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <geometry_msgs/Pose.h>

namespace waypoint_follower
{

    LQR::LQR(int n)//纯跟踪类构造函数定义
        : RADIUS_MAX_(9e10)
        , KAPPA_MIN_(1 / RADIUS_MAX_)
        // , is_linear_interpolation_(false)
        , next_waypoint_number_(-1)
        // , lookahead_distance_(0)
        // , minimum_lookahead_distance_(6)
        , current_linear_velocity_(0)
        , k(1)
        , v_set(0.56)
        , N(n)
    {
    }

    // Destructor
    LQR::~LQR()
    {
    }

    vector<double> LQR::calTargetIndex() {
        vector<double>dists;
        for (int i = 0; i < waypoints2.size(); i++) {
            double dist = sqrt(pow(waypoints2[i].position.x-current_pose_.position.x,2)+pow(waypoints2[i].position.y-current_pose_.position.y,2));
            dists.push_back(dist);
        }
        double min_index =  min_element(dists.begin(),dists.end())-dists.begin(); //返回vector最小元素的下标
        tf::Quaternion quat_ref;
        tf::quaternionMsgToTF(waypoints2[min_index].orientation, quat_ref);
        double roll_ref, pitch_ref, yaw_ref;//定义存储roll,pitch and yaw的容器
        tf::Matrix3x3(quat_ref).getRPY(roll_ref, pitch_ref, yaw_ref); //进行转换
        // double psi = yaw_vehicle;
        double dx,dy,ddx,ddy;
        int suc;
        suc = 5;
        if (min_index == 0){
             dx = waypoints2[min_index + 1].position.x - waypoints2[min_index].position.x;
             dy = waypoints2[min_index + 1].position.y - waypoints2[min_index].position.y;
             ddx = waypoints2[2].position.x + waypoints2[0].position.x - 2*waypoints2[1].position.x;
             ddy = waypoints2[2].position.y + waypoints2[0].position.y - 2*waypoints2[1].position.y;
        }
        else if(min_index == waypoints2.size()-1){
             dx = waypoints2[min_index].position.x - waypoints2[min_index - 1].position.x;
             dy = waypoints2[min_index].position.y - waypoints2[min_index - 1].position.y;
             ddx = waypoints2[min_index].position.x + waypoints2[min_index - 2].position.x - 2*waypoints2[min_index - 1].position.x;
             ddy = waypoints2[min_index].position.y + waypoints2[min_index - 2].position.y - 2*waypoints2[min_index - 1].position.y;
        }
        // else if (min_index <= waypoints2.size()-7)
        // {
        //      dx = waypoints2[min_index + 6].position.x - waypoints2[min_index + 5].position.x;
        //      dy = waypoints2[min_index + 6].position.y - waypoints2[min_index + 5].position.y;
        //      ddx = waypoints2[min_index + 6].position.x + waypoints2[min_index + 4].position.x - 2*waypoints2[min_index + 5].position.x;
        //      ddy = waypoints2[min_index + 6].position.y + waypoints2[min_index + 4].position.y - 2*waypoints2[min_index + 5].position.y;
        // }
        else if (min_index <= waypoints2.size()-suc-2)
        {
             dx = waypoints2[min_index + 1 + suc].position.x - waypoints2[min_index + suc].position.x;
             dy = waypoints2[min_index + 1 + suc].position.y - waypoints2[min_index + suc].position.y;
             ddx = waypoints2[min_index + 1 + suc].position.x + waypoints2[min_index + suc-1].position.x - 2*waypoints2[min_index + suc].position.x;
             ddy = waypoints2[min_index + 1 + suc].position.y + waypoints2[min_index + suc-1].position.y - 2*waypoints2[min_index + suc].position.y;
        }
        else{
             dx = waypoints2[min_index + 1].position.x - waypoints2[min_index].position.x;
             dy = waypoints2[min_index + 1].position.y - waypoints2[min_index].position.y;
             ddx = waypoints2[min_index + 1].position.x + waypoints2[min_index - 1].position.x - 2*waypoints2[min_index].position.x;
             ddy = waypoints2[min_index + 1].position.y + waypoints2[min_index - 1].position.y - 2*waypoints2[min_index].position.y;
        }
        double k = (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3 / 2);

        // double dx1,dy1,ddx1,ddy1;
        // if (min_index == 0){
        //      dx1 = waypoints2[min_index + 1].position.x - waypoints2[min_index].position.x;
        //      dy1 = waypoints2[min_index + 1].position.y - waypoints2[min_index].position.y;
        //      ddx1 = waypoints2[2].position.x + waypoints2[0].position.x - 2*waypoints2[1].position.x;
        //      ddy1 = waypoints2[2].position.y + waypoints2[0].position.y - 2*waypoints2[1].position.y;
        // }else if(min_index == waypoints2.size()-1){
        //      dx1 = waypoints2[min_index].position.x - waypoints2[min_index - 1].position.x;
        //      dy1 = waypoints2[min_index].position.y - waypoints2[min_index - 1].position.y;
        //      ddx1 = waypoints2[min_index].position.x + waypoints2[min_index - 2].position.x - 2*waypoints2[min_index - 1].position.x;
        //      ddy1 = waypoints2[min_index].position.y + waypoints2[min_index - 2].position.y - 2*waypoints2[min_index - 1].position.y;
        // }else{
        //      dx1 = waypoints2[min_index + 1].position.x - waypoints2[min_index].position.x;
        //      dy1 = waypoints2[min_index + 1].position.y - waypoints2[min_index].position.y;
        //      ddx1 = waypoints2[min_index + 1].position.x + waypoints2[min_index - 1].position.x - 2*waypoints2[min_index].position.x;
        //      ddy1 = waypoints2[min_index + 1].position.y + waypoints2[min_index - 1].position.y - 2*waypoints2[min_index].position.y;
        // }
        int suc1;
        suc1 = 0;
        double dx1,dy1,ddx1,ddy1;
        if (min_index == 0){
             dx1 = waypoints2[min_index + 1].position.x - waypoints2[min_index].position.x;
             dy1 = waypoints2[min_index + 1].position.y - waypoints2[min_index].position.y;
             ddx1 = waypoints2[2].position.x + waypoints2[0].position.x - 2*waypoints2[1].position.x;
             ddy1 = waypoints2[2].position.y + waypoints2[0].position.y - 2*waypoints2[1].position.y;
        }else if(min_index == waypoints2.size()-1){
             dx1 = waypoints2[min_index].position.x - waypoints2[min_index - 1].position.x;
             dy1 = waypoints2[min_index].position.y - waypoints2[min_index - 1].position.y;
             ddx1 = waypoints2[min_index].position.x + waypoints2[min_index - 2].position.x - 2*waypoints2[min_index - 1].position.x;
             ddy1 = waypoints2[min_index].position.y + waypoints2[min_index - 2].position.y - 2*waypoints2[min_index - 1].position.y;
        }else if (min_index <= waypoints2.size()-suc1-2)
        {
             dx1 = waypoints2[min_index + 1 + suc1].position.x - waypoints2[min_index + suc1].position.x;
             dy1 = waypoints2[min_index + 1 + suc1].position.y - waypoints2[min_index + suc1].position.y;
             ddx1 = waypoints2[min_index + 1 + suc1].position.x + waypoints2[min_index + suc1-1].position.x - 2*waypoints2[min_index + suc1].position.x;
             ddy1 = waypoints2[min_index + 1 + suc1].position.y + waypoints2[min_index + suc1-1].position.y - 2*waypoints2[min_index + suc1].position.y;
        }
        else{
             dx1 = waypoints2[min_index + 1].position.x - waypoints2[min_index].position.x;
             dy1 = waypoints2[min_index + 1].position.y - waypoints2[min_index].position.y;
             ddx1 = waypoints2[min_index + 1].position.x + waypoints2[min_index - 1].position.x - 2*waypoints2[min_index].position.x;
             ddy1 = waypoints2[min_index + 1].position.y + waypoints2[min_index - 1].position.y - 2*waypoints2[min_index].position.y;
        }

        double k1 = (ddy1 * dx1 - ddx1 * dy1) / pow((dx1 * dx1 + dy1 * dy1), 3 / 2);

        geometry_msgs::Pose current_ref_pose;
        current_ref_pose = waypoints2[min_index];

        tf::Quaternion quat_vehicle;
        tf::quaternionMsgToTF(current_pose_.orientation, quat_vehicle);
        double roll_vehicle, pitch_vehicle, yaw_vehicle;//定义存储roll,pitch and yaw的容器
        tf::Matrix3x3(quat_vehicle).getRPY(roll_vehicle, pitch_vehicle, yaw_vehicle); //进行转换
        double psi = yaw_vehicle;

        double e_y;
        if(-(current_ref_pose.position.x - current_pose_.position.x) * sin(psi) + (current_ref_pose.position.y - current_pose_.position.y) * cos(psi) > 0){
        //if((robot_state[1]-current_ref_point[1])*cos(psi_t)-(robot_state[0]-current_ref_point[0])*sin(psi_t)<=0){
            // e_y = sqrt(pow(current_pose_.position.x - current_ref_pose.position.x,2)+pow(current_pose_.position.y - current_ref_pose.position.y,2));
            e_y = abs(-(current_ref_pose.position.x - current_pose_.position.x) * sin(psi) + (current_ref_pose.position.y - current_pose_.position.y) * cos(psi));
        }
        else{
            // e_y = -sqrt(pow(current_pose_.position.x - current_ref_pose.position.x,2)+pow(current_pose_.position.y - current_ref_pose.position.y,2));
            e_y = - abs(-(current_ref_pose.position.x - current_pose_.position.x) * sin(psi) + (current_ref_pose.position.y - current_pose_.position.y) * cos(psi));
        }

        next_waypoint_number_ = min_index;

        return {e_y,k,yaw_ref,min_index,k1};
    }

    double LQR::normalizeAngle(double angle) {
        while(angle> 2.0*M_PI){
            angle-=2.0*M_PI;
        }
        while(angle<-2.0*M_PI){
            angle+=2.0*M_PI;
        }
        return angle;
    }

    MatrixXd LQR::calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R) {
        MatrixXd Qf= Q;
        MatrixXd P = Qf;
        MatrixXd P_;
        // cout << N << endl;
        for(int i=0;i<N;i++){
            P_ = Q+A.transpose()*P*A-A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
            //小于预设精度时返回
            if((P_-P).maxCoeff()<EPS&&(P-P_).maxCoeff()<EPS)break;
            //if((P_-P).cwiseAbs().maxCoeff()<EPS)break;

            P = P_;

        }
        // cout << "++++++++++++" << endl;
        // cout <<  P_ << endl;
        // cout << "++++++++++++" << endl;
        return P_;
    }

    double LQR::lqrControl(geometry_msgs::Pose robot_state, vector<geometry_msgs::Pose> refer_path, double s0, MatrixXd A, MatrixXd B,
                       MatrixXd Q, MatrixXd R) {
        MatrixXd X(3,1);

        if (s0 <= refer_path.size() - 1)
        {
            tf::Quaternion quat_ref1;
            tf::quaternionMsgToTF(refer_path[s0].orientation, quat_ref1);
            double roll_ref1, pitch_ref1, yaw_ref1;//定义存储roll,pitch and yaw的容器
            tf::Matrix3x3(quat_ref1).getRPY(roll_ref1, pitch_ref1, yaw_ref1); //进行转换

            tf::Quaternion quat_vehicle1;
            tf::quaternionMsgToTF(robot_state.orientation, quat_vehicle1);
            double roll_vehicle1, pitch_vehicle1, yaw_vehicle1;//定义存储roll,pitch and yaw的容器
            tf::Matrix3x3(quat_vehicle1).getRPY(roll_vehicle1, pitch_vehicle1, yaw_vehicle1); //进行转换
            double psi = yaw_vehicle1;

            double diff_psi = normalizeAngle(psi-yaw_ref1);

            X<<robot_state.position.x-refer_path[s0].position.x,robot_state.position.y-refer_path[s0].position.y,diff_psi;
            MatrixXd P = calRicatti(A,B,Q,R);
            MatrixXd K = -(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
            MatrixXd u = K*X; //[v-ref_v,delta-ref_delta]
            // std::cout << "dv=" << u(0,0) << std::endl;
            return u(1,0);
        }
        else
        {
            tf::Quaternion quat_ref1;
            tf::quaternionMsgToTF(refer_path[s0].orientation, quat_ref1);
            double roll_ref1, pitch_ref1, yaw_ref1;//定义存储roll,pitch and yaw的容器
            tf::Matrix3x3(quat_ref1).getRPY(roll_ref1, pitch_ref1, yaw_ref1); //进行转换

            tf::Quaternion quat_vehicle1;
            tf::quaternionMsgToTF(robot_state.orientation, quat_vehicle1);
            double roll_vehicle1, pitch_vehicle1, yaw_vehicle1;//定义存储roll,pitch and yaw的容器
            tf::Matrix3x3(quat_vehicle1).getRPY(roll_vehicle1, pitch_vehicle1, yaw_vehicle1); //进行转换
            double psi = yaw_vehicle1;

            double diff_psi = normalizeAngle(psi-yaw_ref1);

            X<<robot_state.position.x-refer_path[s0].position.x,robot_state.position.y-refer_path[s0].position.y,diff_psi;
            MatrixXd P = calRicatti(A,B,Q,R);
            MatrixXd K = -(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
            MatrixXd u = K*X; //[v-ref_v,delta-ref_delta]
            // std::cout << "dv=" << u(0,0) << std::endl;
            return u(1,0);
        }
        

        // cout << "++++++++++++" << endl;
        // cout << u << endl;
        // cout << "++++++++++++" << endl;
        // cout << u(0,0) << endl;
        // cout << u(1,0) << endl;

    }

    // bool LQR::stanleyControl(double* out_kappa_stanley) {
    //     int current_target_index = calTargetIndex();
    //     next_waypoint_number_ = current_target_index;
    //     //cout<<current_target_index<<endl;
    //     geometry_msgs::Pose current_ref_pose;

    //     static int k_final = 0;
    //     // 当计算出来的目标临近点索引大于等于参考轨迹上的最后一个点索引时
    //     if(current_target_index >= waypoints2.size() - 1){
    //         if(k_final == 0){
    //             current_target_index = waypoints2.size()-1;
    //             current_ref_pose = waypoints2[waypoints2.size()-1];
    //             k_final++;
    //         }
    //         else{
    //             cout << "++++" << endl;
    //             return 0;
    //         }
    //     }
    //     else{
    //         current_ref_pose = waypoints2[current_target_index];
    //     }

    //     tf::Quaternion quat_vehicle;
    //     tf::quaternionMsgToTF(current_pose_.orientation, quat_vehicle);
    //     double roll_vehicle, pitch_vehicle, yaw_vehicle;//定义存储roll,pitch and yaw的容器
    //     tf::Matrix3x3(quat_vehicle).getRPY(roll_vehicle, pitch_vehicle, yaw_vehicle); //进行转换
    //     double psi = yaw_vehicle;

    //     double e_y;
    //     if(-(current_ref_pose.position.x - current_pose_.position.x) * sin(psi) + (current_ref_pose.position.y - current_pose_.position.y) * cos(psi) > 0){
    //     //if((robot_state[1]-current_ref_point[1])*cos(psi_t)-(robot_state[0]-current_ref_point[0])*sin(psi_t)<=0){
    //         e_y = sqrt(pow(current_pose_.position.x - current_ref_pose.position.x,2)+pow(current_pose_.position.y - current_ref_pose.position.y,2));
    //     }
    //     else{
    //         e_y = -sqrt(pow(current_pose_.position.x - current_ref_pose.position.x,2)+pow(current_pose_.position.y - current_ref_pose.position.y,2));
    //     }

    //     // if(abs(e_y) > 0.3)
    //     // {
    //     //     e_y = 0;
    //     // }

    //     if(current_target_index < waypoints2.size() - 10)
    //     {
    //         current_ref_pose = waypoints2[current_target_index + 10];
    //     }

    //     // tf::Quaternion quat_ref;
    //     // tf::quaternionMsgToTF(current_ref_pose.orientation, quat_ref);
    //     // double roll_ref, pitch_ref, yaw_ref;//定义存储roll,pitch and yaw的容器
    //     // tf::Matrix3x3(quat_ref).getRPY(roll_ref, pitch_ref, yaw_ref); //进行转换


    //     // double psi_t = yaw_ref;
    //     double psi_t = atan2(current_ref_pose.position.y - current_pose_.position.y, current_ref_pose.position.x - current_pose_.position.x);

    //     // tf::Quaternion quat_vehicle;
    //     // tf::quaternionMsgToTF(current_pose_.orientation, quat_vehicle);
    //     // double roll_vehicle, pitch_vehicle, yaw_vehicle;//定义存储roll,pitch and yaw的容器
    //     // tf::Matrix3x3(quat_vehicle).getRPY(roll_vehicle, pitch_vehicle, yaw_vehicle); //进行转换
    //     // double psi = yaw_vehicle;

    //     // 计算横向误差e_y
    //     // 参考自https://blog.csdn.net/renyushuai900/article/details/98460758

    //     // if(current_pose_.position.x - current_ref_pose.position.x > 0)
    //     // {
    //     //     if((current_pose_.position.x - current_ref_pose.position.x)*tan(psi_t)-(current_pose_.position.y - current_ref_pose.position.y)>0){
    //     // //if((robot_state[1]-current_ref_point[1])*cos(psi_t)-(robot_state[0]-current_ref_point[0])*sin(psi_t)<=0){
    //     //         e_y = sqrt(pow(current_pose_.position.x - current_ref_pose.position.x,2)+pow(current_pose_.position.y - current_ref_pose.position.y,2));
    //     //     }
    //     //     else{
    //     //         e_y = -sqrt(sqrt(pow(current_pose_.position.x - current_ref_pose.position.x,2)+pow(current_pose_.position.y - current_ref_pose.position.y,2)));
    //     //     }
    //     // }
    //     // else
    //     // {
    //     //     if((current_pose_.position.x - current_ref_pose.position.x)*tan(psi_t)-(current_pose_.position.y - current_ref_pose.position.y)>0){
    //     // //if((robot_state[1]-current_ref_point[1])*cos(psi_t)-(robot_state[0]-current_ref_point[0])*sin(psi_t)<=0){
    //     //         e_y = -sqrt(pow(current_pose_.position.x - current_ref_pose.position.x,2)+pow(current_pose_.position.y - current_ref_pose.position.y,2));
    //     //     }
    //     //     else{
    //     //         e_y = sqrt(sqrt(pow(current_pose_.position.x - current_ref_pose.position.x,2)+pow(current_pose_.position.y - current_ref_pose.position.y,2)));
    //     //     }
    //     // }

    //     //# 通过公式(5)计算转角,符号保持一致
    //     // double v = robot_state[3];
    //     double theta_e = psi_t - psi;
    //     double delta_e = atan2(k*e_y, v_set);
    //     // if(abs(e_y) < 0.01)
    //     // {
    //     //     delta_e = 0;
    //     // }
    //     cout << "------------航向误差转向角：" << theta_e << "--------------" << endl;
    //     cout << "------------横向误差距离值：" << e_y << "--------------" << endl;
    //     cout << "------------stanley控制增益:" << k << "--------------" << endl;
    //     cout << "------------整车速度值：" << v_set << "--------------" << endl;
    //     cout << "++++++++++++横向误差转向角：" << delta_e << "++++++++++++++" << endl;
    //     double delta = normalizeAngle(delta_e+theta_e);
    //     cout << "************航向角总调整量：" << delta << "*************" << endl;
    //     if(delta > M_PI * 0.46)
    //     {
    //         delta =  M_PI * 0.46;
    //     }
    //     else if (delta < -M_PI * 0.46)
    //     {
    //         delta =  -M_PI * 0.46;
    //     }
    //     cout << delta << endl;

    //     *out_kappa_stanley = tan(delta) / L_set;
    //     return true;
    // }

    tf::Vector3 point2vector1(geometry_msgs::Point point)
    {
        tf::Vector3 vector(point.x, point.y, point.z);
        return vector;
    }

    double getPlaneDistance1(geometry_msgs::Point target1, geometry_msgs::Point target2)
    {
        tf::Vector3 v1 = point2vector1(target1);
        v1.setZ(0);
        tf::Vector3 v2 = point2vector1(target2);
        v2.setZ(0);
        return tf::tfDistance(v1, v2);
    }

    geometry_msgs::Point calcRelativeCoordinate1(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
    {
        tf::Transform inverse;
        tf::poseMsgToTF(current_pose, inverse);
        tf::Transform transform = inverse.inverse();

        tf::Point p;
        pointMsgToTF(point_msg, p);
        tf::Point tf_p = transform * p;
        geometry_msgs::Point tf_point_msg;
        pointTFToMsg(tf_p, tf_point_msg);

        return tf_point_msg;
    }


    inline double deg2rad(double _angle)
    {
        return _angle / 180.0 * M_PI;
    }


}