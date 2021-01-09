#!/bin/sh

# ABSOLUTE path to the ROS_WS
export ROS_DIST=/opt/ros/eloquent
export ROS_WS=/opt/ros_ws
# Name of the package we are interested in
export FPACKAGE=rclcpp

echo source ${ROS_DIST}/setup.sh
. ${ROS_DIST}/setup.sh
echo source ${ROS_WS}/install/setup.sh
. ${ROS_WS}/install/setup.sh

export CC=afl-clang 
export CXX=afl-clang++ 
export CXXFLAGS="-fprofile-arcs -ftest-coverage" 
export CMAKE_EXE_LINKER_FLAGS="-fprofile-arcs -ftest-coverage"

cd $ROS_WS

echo
echo 1. Clean up lcov state
lcov --gcov-tool gcov -directory ./build/${FPACKAGE} --zerocounters

echo
echo 2. Create baseline to make sure untouched files show up in the report
lcov --gcov-tool gcov -c -i -d ./build/${FPACKAGE} -o Coverage_${FPACKAGE}.base

echo
echo 3. Run tests
colcon test

echo
echo 4. Capturing lcov counters and generating report
lcov --gcov-tool gcov --directory ./build/${FPACKAGE} \
     --capture --output-file Coverage_${FPACKAGE}.info


echo
echo 5. Add baseline counters
lcov --gcov-tool gcov -a Coverage_${FPACKAGE}.base \
     -a Coverage_${FPACKAGE}.info \
     --output-file Coverage_${FPACKAGE}.total

echo
echo 6. Consolidate and clean
lcov --gcov-tool gcov --remove Coverage_${FPACKAGE}.total \
     --output-file install/${FPACKAGE}/Coverage_${FPACKAGE}.info.cleaned

echo
echo 7. Generate HTML report
genhtml -o Coverage_${FPACKAGE} \
     install/${FPACKAGE}/Coverage_${FPACKAGE}.info.cleaned


# COMMAND ${CMAKE_COMMAND} -E remove ${Coverage_NAME}.base ${Coverage_NAME}.total ${PROJECT_BINARY_DIR}/${Coverage_NAME}.info.cleaned
