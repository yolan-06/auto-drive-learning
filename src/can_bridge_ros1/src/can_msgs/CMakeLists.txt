cmake_minimum_required(VERSION 2.8.3)
project(can_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             std_msgs
             message_generation
)

add_message_files(
    FILES
    ecu.msg
    vehicle_status.msg
    battery.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs message_generation
)

include_directories(
)


