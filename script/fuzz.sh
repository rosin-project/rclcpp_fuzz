#!/bin/sh

export ROS_DIST=/opt/ros/eloquent

# ABSOLUTE path to the ROS_WS
export ROS_WS=/opt/ros_ws
# Name of the package we are interested in

# colors for more readability, ad
CYAN='\033[1;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color
STEP=1

msg () {
  printf "\n${CYAN}[${STEP}]${WHITE} $1 ${NC}\n\n"
  let "STEP += 1"
}

#### CONFIGURATION OF THE SCRIPT

LCOVDIR=build/
msg "Will track coverage of all files in ${LCOVDIR}"

# How long time we allow afl-fuzz to run
# On AW's PC 30s is a minimum useful time to run, so some data emerges
# Paco: we need a bit more to get some meaningful results
DURATION=600s
msg "Fuzzing duration is set to ${DURATION}"

# Path to where we place coverage reports (best visible on the host)
# If visible on the host you can just view in browser
REPORT=src/rclcpp_fuzz/fuzz/lcov/Coverage_Report
msg "Coverage report will be placed in ${REPORT}"


msg "Setting configuration requirements of afl-fuzz"
echo core > /proc/sys/kernel/core_pattern
cd /sys/devices/system/cpu
echo performance | tee cpu*/cpufreq/scaling_governor
cd ${ROS_WS}

# If llvm-cov is named gcov then it acts as if it received the gcov verb

msg "Disguise llvm-cov as gcov for lcov"
ln `which llvm-cov-4.0` ${ROS_WS}/gcov -sfv
GCOVTOOL="--gcov-tool ${ROS_WS}/gcov"




export CC=afl-clang 
export CXX=afl-clang++ 
export CXXFLAGS="-fprofile-arcs -ftest-coverage -g -O0" 
export CMAKE_EXE_LINKER_FLAGS="-fprofile-arcs -ftest-coverage"
export CMAKE_BUILD_TYPE="Coverage"
export AFL_USE_ASAN=1
# Change to 1 to see the compilation messages
export VERBOSE=0

EVENT_HANDLERS="--event-handlers console_direct+"
if [ ${VERBOSE} -eq "0" ]; then
  EVENT_HANDLERS="";
fi

msg "source ${ROS_DIST}/setup.sh"
. ${ROS_DIST}/setup.sh


# We assume that rclcpp_fuzz is in the workspace, but let's get its
# dependencies in, and let's compile. This is a convenience step if the
# docker container is just started from an empty rclcpp build.
# Perhaps these steps will go to the dockerfile eventually.

msg "Installing dependencies for the client/server project"
rosdep install --from-paths src -y


msg "Compiling the client server project"
colcon build ${EVENT_HANDLERS} 


# This makes the build client and server available to ros2 run
msg "source ${ROS_WS}/install/setup.sh"
. ${ROS_WS}/install/setup.sh


msg "Removing stale fuzzing results (might not be desirable)"
rm ${ROS_WS}/src/rclcpp_fuzz/fuzz/out/* -Rfv


msg "Removing stale coverage results (might not be desirable)"
find . -name "*.gcda" -print0 | xargs -0 rm


msg "Clean up lcov state"
lcov ${GCOVTOOL} -directory ${LCOVDIR} --zerocounters


msg "Create baseline to make sure untouched files show up in the report"
lcov ${GCOVTOOL} -c -i -d ${LCOVDIR} -o ${REPORT}.base


msg "Starting the server"
echo ros2 run rclcpp_fuzz server \> /dev/null \&
ros2 run rclcpp_fuzz server > /dev/null &


msg "Starting afl-fuzz for the duration of ${DURATION}"
# m - none (you can replace none with the memory size in megabytes)
timeout ${DURATION} afl-fuzz -m none -t 2000 -i ${ROS_WS}/src/rclcpp_fuzz/fuzz/in/ \
    -o ${ROS_WS}/src/rclcpp_fuzz/fuzz/out/ -- \
    install/rclcpp_fuzz/lib/rclcpp_fuzz/client


msg "Killing the server (the client and afl-fuzz should already be dead)"
pkill -e server


msg "Capturing lcov counters and generating report"
lcov ${GCOVTOOL} --directory ${LCOVDIR} \
     --capture --output-file ${REPORT}.info


msg "Add baseline counters"
lcov ${GCOVTOOL} -a ${REPORT}.base -a ${REPORT}.info \
     --output-file ${REPORT}.total


msg "Consolidate and clean"
lcov ${GCOVTOOL} --remove ${REPORT}.total \
     --output-file ${REPORT}.info.cleaned


msg "Generate HTML report"
genhtml -o ${REPORT} ${REPORT}.info.cleaned


msg "Generate plots to rclcpp_fuzz/fuzz/graphs (crashes if afl had not enough time)"
mkdir -pv src/rclcpp_fuzz/fuzz/graphs
afl-plot src/rclcpp_fuzz/fuzz/out/  src/rclcpp_fuzz/fuzz/graphs/
