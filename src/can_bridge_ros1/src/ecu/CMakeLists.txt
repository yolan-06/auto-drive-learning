cmake_minimum_required(VERSION 3.0.2)
project(ecu)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  can_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_generation 
)

###########
## Build ##
###########

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  link_directories("${PROJECT_SOURCE_DIR}/lib/x86/linux_64")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_32")
  link_directories("${PROJECT_SOURCE_DIR}/lib/x86/linux_32")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  link_directories("${PROJECT_SOURCE_DIR}/lib/arm/64bit")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch32")
  link_directories("${PROJECT_SOURCE_DIR}/lib/arm/32bit")
endif()


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ecuTest.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
