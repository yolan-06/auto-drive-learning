cmake_minimum_required(VERSION 3.0.2)
project(gnss_localizer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Eigen3  REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  message_generation
  message_filters
)

# Add GeographicLib.
add_subdirectory(third_party/GeographicLib)

add_message_files(
  FILES
  hcinspvatzcb.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gnss_localizer
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf
#  DEPENDS system_lib
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs turtlesim message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  third_party/GeographicLib/include
)

add_executable(gnss_localizer
	src/main_fix_none.cpp
)

target_link_libraries(gnss_localizer
  ${catkin_LIBRARIES}
	${EIGEN3_LIBS}
  libGeographiccc
)

add_executable(gnss_localizer_trans
	src/main_dev_02.cpp
)

target_link_libraries(gnss_localizer_trans
  ${catkin_LIBRARIES}
	${EIGEN3_LIBS}
  libGeographiccc
)

set(CMAKE_CXX_STANDARD 11)

#############
## Install ##
#############

