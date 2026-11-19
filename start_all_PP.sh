#!/bin/bash

# 脚本功能: 
# 1. 启动 roscore
# 2. 启动所有指定的 roslaunch 文件
# 3. 自动调用 rosservice 开启日志记录
# 4. 启动 RViz 并加载指定配置

# 当用户按下 Ctrl+C 时，执行 cleanup 函数
trap 'cleanup' SIGINT

# 清理函数，用于关闭所有后台进程
cleanup() {
    echo -e "\n\n正在关闭所有ROS节点..."
    # 查找并杀死所有由这个脚本启动的 roslaunch, rviz, 和 rosmaster 进程
    pkill -P $$ roslaunch
    pkill -P $$ rviz
    pkill -P $$ rosmaster
    echo "所有节点已关闭。"
    exit 0
}

# Source ROS环境变量和您的工作空间
# 请确保下面的路径是正确的
source /opt/ros/noetic/setup.bash
source ~/catkin_gps/devel/setup.bash

# 启动 roscore
echo "正在启动 roscore..."
roscore &
sleep 5  # 等待 roscore 完全启动

# 在后台启动各个 roslaunch 进程
echo "正在启动 chcnav..."
roslaunch chcnav demo_9.launch &
sleep 3

echo "正在启动 gnss_localizer..."
roslaunch gnss_localizer run.launch &
sleep 3

echo "正在启动 pure_pursuit..."
roslaunch pure_pursuit pure_pursuit.launch &
sleep 3

# --- 新增指令 ---
echo "正在启动 frenet_test..."
roslaunch pure_pursuit frenet_test.launch &
sleep 3

echo "正在启动 can_bridge..."
roslaunch can_bridge can_bridge.launch &
# 等待几秒钟，确保frenet_planner_enhanced_node的服务已经成功启动
echo "等待5秒以确保服务已就绪..."
sleep 5 

echo "正在调用服务开启日志记录..."
# 注意：rosservice call 是一个执行后立即返回的命令，所以不需要在后台运行 (即不需要 &)
rosservice call /frenet_planner_enhanced_node/toggle_logging "data: true"
# --- 新增指令结束 ---

# 启动 RViz
echo "正在启动 RViz..."
rviz -d /home/nvidia/catkin_gps/1.rviz &

# 等待所有后台进程结束
# 这将使脚本保持运行状态，直到用户按下 Ctrl+C
echo -e "\n所有节点已启动。按 Ctrl+C 关闭所有进程。"
wait
