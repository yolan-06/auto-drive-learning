cmake_minimum_required(VERSION 3.0.2)
project(mpc_tracker)

# C++14
add_compile_options(-std=c++14)

# -------------------------- 手动指定路径 --------------------------
set(OSQP_INCLUDE_DIR "/usr/local/include/osqp")
set(OSQP_LIBRARY "/usr/local/lib/libosqp.so")

set(OSQPEIGEN_INCLUDE_DIR "/usr/local/include/OsqpEigen")
set(OSQPEIGEN_LIBRARY "/usr/local/lib/libOsqpEigen.so")

# 验证路径存在
if(NOT EXISTS ${OSQP_LIBRARY})
  message(FATAL_ERROR "osqp library not found at ${OSQP_LIBRARY}")
endif()
if(NOT EXISTS ${OSQPEIGEN_LIBRARY})
  message(FATAL_ERROR "OsqpEigen library not found at ${OSQPEIGEN_LIBRARY}")
endif()


# -------------------------- catkin --------------------------
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

# -------------------------- catkin 导出 --------------------------
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf2 tf2_geometry_msgs visualization_msgs
)

# -------------------------- 构建 --------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include 
  ${OSQP_INCLUDE_DIR}
  ${OSQPEIGEN_INCLUDE_DIR}
)

link_directories(/usr/local/lib)

add_executable(${PROJECT_NAME}_node
  src/mpc_node.cpp
  src/mpc_core.cpp
  src/mpc.cpp
  src/mpc_viz.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OSQP_LIBRARY}
  ${OSQPEIGEN_LIBRARY}
)

# -------------------------- 安装 --------------------------
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})