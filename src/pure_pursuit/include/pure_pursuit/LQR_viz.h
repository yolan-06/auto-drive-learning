/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PURE_PURSUIT_LQR_VIZ_H
#define PURE_PURSUIT_LQR_VIZ_H

// ROS includes
#include <ros/ros.h>//定义ROS头文件

#include <geometry_msgs/PoseStamped.h>//定义消息类型
#include <geometry_msgs/TwistStamped.h>

#include <visualization_msgs/Marker.h>

// C++ includes
#include <memory>
#include <vector>

// User defined includes
// #include <libwaypoint_follower/libwaypoint_follower.h>

namespace waypoint_follower
{
// display the next waypoint by markers.
visualization_msgs::Marker displayNextWaypoint(geometry_msgs::Point position);//显示下一个路径点

visualization_msgs::Marker displayallWaypoints(const std::vector<geometry_msgs::Pose>& waypoints);

// display the next target by markers.
visualization_msgs::Marker displayNextTarget(geometry_msgs::Point target);//显示下一个目标点
visualization_msgs::Marker displayExpandWaypoints(
  const std::vector<geometry_msgs::Pose>& waypoints, int size);//显示扩展路径点

double calcRadius(
  geometry_msgs::Point target, geometry_msgs::Pose current_pose);//计算路径半径

// generate the locus of pure pursuit
std::vector<geometry_msgs::Point> generateTrajectoryCircle(
  double r_kappa, geometry_msgs::Pose current_pose);//生成轨迹圆
// display the locus of pure pursuit by markers.
visualization_msgs::Marker displayTrajectoryCircle(
  std::vector<geometry_msgs::Point> traj_circle_array);//显示轨迹圆

// display the search radius by markers.
visualization_msgs::Marker displaySearchRadius(
  geometry_msgs::Point current_pose, double search_radius);//显示搜索半径
}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_PURE_PURSUIT_VIZ_H
