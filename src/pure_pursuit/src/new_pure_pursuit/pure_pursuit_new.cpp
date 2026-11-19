#include <pure_pursuit/pure_pursuit.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

namespace waypoint_follower
{

    PurePursuit::PurePursuit()//纯跟踪类构造函数定义
        : RADIUS_MAX_(9e10)
        , KAPPA_MIN_(1 / RADIUS_MAX_)
        , is_linear_interpolation_(false)
        , next_waypoint_number_(-1)
        , lookahead_distance_(0)
        , minimum_lookahead_distance_(6)
        , current_linear_velocity_(0)
    {
    }

    // Destructor
    PurePursuit::~PurePursuit()
    {
    }

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

    bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c)
    {
        // (x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
        double sub_x = std::fabs(start.x - end.x);
        double sub_y = std::fabs(start.y - end.y);
        double error = std::pow(10, -5);  // 0.00001

        if (sub_x < error && sub_y < error)
        {
            // ROS_INFO("two points are the same point!!");
            return false;
        }

        *a = end.y - start.y;
        *b = (-1) * (end.x - start.x);
        *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

        return true;
    }

    double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c)
    {
        double d = std::fabs(a * point.x + b * point.y + c) / std::sqrt(std::pow(a, 2) + std::pow(b, 2));

        return d;
    }

    inline double deg2rad(double _angle)
    {
        return _angle / 180.0 * M_PI;
    }

    tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree)
    {
        tf::Vector3 w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(),
                        sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
        tf::Vector3 unit_w1 = w1.normalize();

        return unit_w1;
    }

    double PurePursuit::calcCurvature(geometry_msgs::Point target) const//计算曲率
    {
        double kappa;
        double denominator = pow(getPlaneDistance1(target, current_pose_.position), 2);
        double numerator = 2 * calcRelativeCoordinate1(target, current_pose_).y;

    if (denominator != 0)
    {
        kappa = numerator / denominator;
    }
    else
    {
        if (numerator > 0)
        {
            kappa = KAPPA_MIN_;
        }
        else
        {
            kappa = -KAPPA_MIN_;
        }
    }
        ROS_INFO("kappa : %lf", kappa);
        return kappa;
    }

    bool PurePursuit::interpolateNextTarget(//通过线性插值获得下一目标跟踪点
    int next_waypoint, geometry_msgs::Point* next_target) const
    {
        double ERROR = pow(10, -5);  // 0.00001

        int path_size = static_cast<int>(waypoints2.size());
        if (next_waypoint == path_size - 1)
        {
            *next_target = waypoints2.at(next_waypoint).position;
            return true;
        }
        double search_radius = lookahead_distance_;



        geometry_msgs::Point zero_p;
        geometry_msgs::Point end =
            waypoints2.at(next_waypoint).position;
        geometry_msgs::Point start =
            waypoints2.at(next_waypoint - 1).position;

        // let the linear equation be "ax + by + c = 0"
        // if there are two points (x1,y1) , (x2,y2),
        // a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
        double a = 0;
        double b = 0;
        double c = 0;
        double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
        if (!get_linear_flag)
            // std::cout << "start and end is the same" << std::endl;
            // ROS_INFO_STREAM("start and end is the same");
            return false;

        // let the center of circle be "(x0,y0)", in my code ,
        // the center of circle is vehicle position
        // the distance  "d" between the foot of
        // a perpendicular line and the center of circle is ...
        //    | a * x0 + b * y0 + c |
        // d = -------------------------------
        //          √( a~2 + b~2)
        double d = getDistanceBetweenLineAndPoint(current_pose_.position, a, b, c);

        // ROS_INFO("a : %lf ", a);
        // ROS_INFO("b : %lf ", b);
        // ROS_INFO("c : %lf ", c);
        // ROS_INFO("distance : %lf ", d);

        if (d > search_radius)
            // std::cout << "d > search_radius" << std::endl;
            // ROS_INFO_STREAM("d > search_radius");
            return false;

        // unit vector of point 'start' to point 'end'
        tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
        tf::Vector3 unit_v = v.normalize();

        // normal unit vectors of v
        // rotate to counter clockwise 90 degree
        tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);
        // rotate to counter clockwise 90 degree
        tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);

        // the foot of a perpendicular line
        geometry_msgs::Point h1;
        h1.x = current_pose_.position.x + d * unit_w1.getX();
        h1.y = current_pose_.position.y + d * unit_w1.getY();
        h1.z = current_pose_.position.z;

        geometry_msgs::Point h2;
        h2.x = current_pose_.position.x + d * unit_w2.getX();
        h2.y = current_pose_.position.y + d * unit_w2.getY();
        h2.z = current_pose_.position.z;

        // ROS_INFO("error : %lf", error);
        // ROS_INFO("whether h1 on line : %lf", h1.y - (slope * h1.x + intercept));
        // ROS_INFO("whether h2 on line : %lf", h2.y - (slope * h2.x + intercept));

        // check which of two foot of a perpendicular line is on the line equation
        geometry_msgs::Point h;
        if (fabs(a * h1.x + b * h1.y + c) < ERROR)
        {
            h = h1;
            //   ROS_INFO("use h1");
        }
        else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
        {
            //   ROS_INFO("use h2");
            h = h2;
        }
        else
        {
            // std::cout << "h_point error" << std::endl;
            // ROS_INFO_STREAM("h_point error");
            return false;
        }

        // get intersection[s]
        // if there is a intersection
        if (d == search_radius)
        {
            *next_target = h;
            return true;
        }
        else
        {
            // if there are two intersection
            // get intersection in front of vehicle
            double s = sqrt(pow(search_radius, 2) - pow(d, 2));
            geometry_msgs::Point target1;
            target1.x = h.x + s * unit_v.getX();
            target1.y = h.y + s * unit_v.getY();
            target1.z = current_pose_.position.z;

            geometry_msgs::Point target2;
            target2.x = h.x - s * unit_v.getX();
            target2.y = h.y - s * unit_v.getY();
            target2.z = current_pose_.position.z;

            // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
            // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
            // displayLinePoint(a, b, c, target1, target2, h);  // debug tool

            // check intersection is between end and start
            double interval = getPlaneDistance1(end, start);
            if (getPlaneDistance1(target1, end) < interval)
            {
                // ROS_INFO("result : target1");
                *next_target = target1;
                return true;
            }
            else if (getPlaneDistance1(target2, end) < interval)
            {
                // ROS_INFO("result : target2");
                *next_target = target2;
                return true;
            }
            else
            {
                // ROS_INFO("result : false ");
                // std::cout << "can find target" << std::endl;
                ROS_INFO_STREAM("can find target");
                return false;
            }
        }
    }

    void PurePursuit::getNextWaypoint()//获取下一路径点
    {
        int path_size = static_cast<int>(waypoints2.size());//获得当前路径点个数
        static int j = 0;
        static int k = 0;

        // if waypoints are not given, do nothing.
        if (path_size == 0)//判断当前有无路径点，若无则将下一路径点索引置为-1
        {
            next_waypoint_number_ = -1;
            return;
        }

        // look for the next waypoint.
        for (int i = j; i < path_size; i++)//遍历当前各路径点，寻找下一路径点
        {
            // if search waypoint is the last
            if (i == (path_size - 1) && k == 0)//当查询到最终路径点时，将下一路径点索引置为最终路径点索引
            {
                ROS_INFO("search waypoint is the last");
                next_waypoint_number_ = i;
                k++;
                return;
            }

            // if there exists an effective waypoint
            if (getPlaneDistance1(
            waypoints2.at(i).position, current_pose_.position)//计算当前整车位姿与路径点距离是否大于前瞻量，若大于则把该路径点索引置为下一路径点索引
            > lookahead_distance_)
            {
                next_waypoint_number_ = i;
                j = i;
                // std::cout << "++++++++++" << i << "++++++++++" << std::endl;
                // std::cout << "----------" << j << "----------" << std::endl;
                return;
            }
            // if (getPlaneDistance1(
            // waypoints2.at(i).position, current_pose_.position)//计算当前整车位姿与路径点距离是否大于前瞻量，若大于则把该路径点索引置为下一路径点索引
            // > lookahead_distance_ && i > )
            // {
            //     next_waypoint_number_ = path_size - 1;
            //     return;
            // }
            // ROS_INFO(i);

        }

        // if this program reaches here , it means we lost the waypoint!
        next_waypoint_number_ = -1;//当下一路径点索引置为-1时则代表未找到
        return;
    }

    bool PurePursuit::canGetCurvature(double* output_kappa)//判断是否能正常求解曲率
    {
        // search next waypoint
        getNextWaypoint();//获取下一路径点，若下一路径点索引为-1则代表未找到下一路径点
        // std::cout << "*************" << next_waypoint_number_ << "*****************" << std::endl;
        if (next_waypoint_number_ == -1)
        {
            ROS_INFO("lost next waypoint");
            return false;
        }
        // check whether curvature is valid or not
        bool is_valid_curve = false;//将有效曲率标志位置为false
        for (const auto& el : waypoints2)//遍历当前路径点，判断当前路径点中是否有与当前整车位姿相距距离超过最小前瞻量的路径点
        {
            if (getPlaneDistance1(el.position, current_pose_.position)
            > minimum_lookahead_distance_)
            {
            is_valid_curve = true;
            break;
            }
        }
        if (!is_valid_curve)
        {
            return false;
        }
        // if is_linear_interpolation_ is false or next waypoint is first or last
        if (!is_linear_interpolation_ || next_waypoint_number_ == 0 ||
            next_waypoint_number_ == (static_cast<int>(waypoints2.size() - 1)))//如果不进行线性插值或下一路径点为第一路径点或最终路径点
        {
            next_target_position_ =
            waypoints2.at(next_waypoint_number_).position;//将目标点直接设置为下一路径点
            *output_kappa = calcCurvature(next_target_position_);//计算相关曲率
            return true;
        }

        // linear interpolation and calculate angular velocity
        bool interpolation =
            interpolateNextTarget(next_waypoint_number_, &next_target_position_);//进行线性插值，并判断线性插值是否成功，若失败则向终端发送失去目标点信息

        if (!interpolation)
        {
            // std::cout << "++++++++++++++++++++++++++++++++" << std::endl;
            // next_target_position_ = waypoints2.at(next_waypoint_number_).position;
            ROS_INFO_STREAM("lost target! ");
            return false;//返回不能正确计算曲率的信息
        }

        // ROS_INFO("next_target : ( %lf , %lf , %lf)",
        //  next_target.x, next_target.y,next_target.z);

        *output_kappa = calcCurvature(next_target_position_);//利用线性插值获得下一目标位姿，计算曲率，并返回能够正确计算曲率的信息
        return true;
    }
}