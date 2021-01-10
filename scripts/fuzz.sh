#!/bin/sh

export ROS_DIST=/opt/ros/eloquent

# ABSOLUTE path to the ROS_WS
export ROS_WS=/opt/ros_ws
# Name of the package we are interested in

echo core > /proc/sys/kernel/core_pattern
cd /sys/devices/system/cpu
echo performance | tee cpu*/cpufreq/scaling_governor
cd ${ROS_WS}

export CC=afl-clang 
export CXX=afl-clang++ 
export CXXFLAGS="-fprofile-arcs -ftest-coverage" 
export CMAKE_EXE_LINKER_FLAGS="-fprofile-arcs -ftest-coverage -g -O0"
export CMAKE_BUILD_TYPE="Coverage"

VERBOSE=1 colcon build

echo source ${ROS_DIST}/setup.sh
. ${ROS_DIST}/setup.sh
echo source ${ROS_WS}/install/setup.sh
. ${ROS_WS}/install/setup.sh

# m - none supposedly also works
afl-fuzz -m 1000 -i ${ROS_WS}/src/rclcpp_fuzz/fuzz/in \
    -o ${ROS_WS}/src/rclcpp_fuzz/fuzz/out/ -- \
    install/rclcpp_fuzz/lib/rclcpp_fuzz/client
