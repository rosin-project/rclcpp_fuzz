#!/bin/sh

export ROS_DIST=/opt/ros/eloquent

# ABSOLUTE path to the ROS_WS
export ROS_WS=/opt/ros_ws
# Name of the package we are interested in

echo 1. Configuration requirements of afl-fuzz
echo core > /proc/sys/kernel/core_pattern
cd /sys/devices/system/cpu
echo performance | tee cpu*/cpufreq/scaling_governor
cd ${ROS_WS}

# If llvm-cov is named gcov then it acts as if it received the gcov verb
echo
echo 2. Disguise llvm-cov as gcov for lcov
ln `which llvm-cov-4.0` ${ROS_WS}/gcov -sfv
GCOVTOOL="--gcov-tool ${ROS_WS}/gcov"


# CONFIGURATION OF THE SCRIPT
LCOVDIR=build/
# Path to where we place coverage reports (best visible on the host)
# If visible on the host you can just view in browser
REPORT=src/rclcpp_fuzz/fuzz/Coverage_Report
# How long time we allow afl-fuzz to run
DURATION=100s
echo Fuzzing duration is set to ${DURATION}

export CC=afl-clang 
export CXX=afl-clang++ 
export CXXFLAGS="-fprofile-arcs -ftest-coverage -g -O0" 
export CMAKE_EXE_LINKER_FLAGS="-fprofile-arcs -ftest-coverage"
export CMAKE_BUILD_TYPE="Coverage"
export AFL_USE_ASAN=1
export VERBOSE=1

echo source ${ROS_DIST}/setup.sh
. ${ROS_DIST}/setup.sh

# We assume that rclcpp_fuzz is in the workspace, but let's get its
# dependencies in, and let's compile. This is a convenience step if the
# docker container is just started from an empty rclcpp build.
# Perhaps these steps will go to the dockerfile eventually.
echo
echo 3. Installing dependencies for the client/server project
rosdep install --from-paths src -y

echo
echo 4. Compiling the client server project
colcon build --event-handlers console_direct+

# This makes the build client and server available to ros2 run
echo
echo 5. source ${ROS_WS}/install/setup.sh
. ${ROS_WS}/install/setup.sh

echo
echo "6. Removing stale fuzzing results (might not be desirable)"
rm ${ROS_WS}/src/rclcpp_fuzz/fuzz/out/* -Rfv

echo
echo "7. Removing stale coverage results (might not be desirable)"
find . -name "*.gcda" -print0 | xargs -0 rm

echo
echo "8. Clean up lcov state"
lcov ${GCOVTOOL} -directory ${LCOVDIR} --zerocounters

echo
echo "9. Create baseline to make sure untouched files show up in the report"
lcov ${GCOVTOOL} -c -i -d ${LCOVDIR} -o ${REPORT}.base

echo
echo "10. Starting the server"
ros2 run rclcpp_fuzz server > /dev/null &

echo
echo "11. Starting afl-fuzz for ${DURATION} time"
# m - none (you can replace none with the memory size in megabytes)
timeout ${DURATION} afl-fuzz -m none -t 2000 -i ${ROS_WS}/src/rclcpp_fuzz/fuzz/in/ \
    -o ${ROS_WS}/src/rclcpp_fuzz/fuzz/out/ -- \
    install/rclcpp_fuzz/lib/rclcpp_fuzz/client

echo
echo "12. Killing the server (the client and afl-fuzz should already be dead)"
pkill -e server

echo
echo 13.Capturing lcov counters and generating report
lcov ${GCOVTOOL} --directory ${LCOVDIR} \
     --capture --output-file ${REPORT}.info

echo
echo 14. Add baseline counters
lcov ${GCOVTOOL} -a ${REPORT}.base -a ${REPORT}.info \
     --output-file ${REPORT}.total

echo
echo 15. Consolidate and clean
lcov ${GCOVTOOL} --remove ${REPORT}.total \
     --output-file ${REPORT}.info.cleaned

echo
echo 16. Generate HTML report
genhtml -o ${REPORT} ${REPORT}.info.cleaned


