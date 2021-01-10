#!/bin/sh

# ABSOLUTE path to the ROS_WS
export ROS_DIST=/opt/ros/eloquent
export ROS_WS=/opt/ros_ws
# Name of the package we are interested in
export PKG=rclcpp
export LCOVDIR=build
export REPNAME=src/rclcpp_fuzz/Coverage_$PKG

mkdir -pv src/rclcpp_fuzz

ln `which llvm-cov-4.0` ${ROS_WS}/gcov -sfv

GCOVTOOL="--gcov-tool ${ROS_WS}/gcov"
echo source ${ROS_DIST}/setup.sh
. ${ROS_DIST}/setup.sh
echo source ${ROS_WS}/install/setup.sh
. ${ROS_WS}/install/setup.sh

export CC=afl-clang 
export CXX=afl-clang++ 
export CXXFLAGS="-fprofile-arcs -ftest-coverage" 
export CMAKE_EXE_LINKER_FLAGS="-fprofile-arcs -ftest-coverage"
export CMAKE_BUILD_TYPE="Coverage"
cd $ROS_WS

echo
echo 1. Clean up lcov state
lcov ${GCOVTOOL} -directory ${LCOVDIR} --zerocounters

echo
echo 2. Create baseline to make sure untouched files show up in the report
lcov ${GCOVTOOL} -c -i -d ${LCOVDIR} -o ${REPNAME}.base

echo
echo 3. Run tests
colcon test --event-handlers console_direct+ # --packages-select ${PKG}
ros2 run rclcpp_fuzz server &
echo 42 45 | ros2 run rclcpp_fuzz client
pkill server

echo
echo 4. Capturing lcov counters and generating report
lcov ${GCOVTOOL} --directory ${LCOVDIR} \
     --capture --output-file ${REPNAME}.info

echo
echo 5. Add baseline counters
lcov ${GCOVTOOL} -a ${REPNAME}.base -a ${REPNAME}.info \
     --output-file ${REPNAME}.total

echo
echo 6. Consolidate and clean
lcov ${GCOVTOOL} --remove ${REPNAME}.total \
     --output-file ${REPNAME}.info.cleaned

echo
echo 7. Generate HTML report
genhtml -o ${REPNAME} ${REPNAME}.info.cleaned

echo
echo 8. Remove the intermediate files
rm -fv ${REPNAME}{.info.cleaned,.total,.base,.info}
