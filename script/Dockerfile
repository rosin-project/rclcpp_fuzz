ARG ROSDIST=eloquent
ARG VARIANT=""
FROM ros:$ROSDIST$VARIANT
ENV ROSDIST eloquent

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && apt-get install -y afl clang lcov afl-cov gnuplot

# TODO remove
RUN apt-get install -y mc file tree neovim less silversearcher-ag 

ENV ROS_WS /opt/ros_ws
WORKDIR $ROS_WS
RUN mkdir -pv $ROS_WS/src
RUN git clone -b $ROSDIST --depth 1 \
    https://github.com/ros2/rclcpp $ROS_WS/src/rclcpp 
RUN /bin/bash -c "source /opt/ros/${ROSDIST}/setup.bash && rosdep install -y --from-paths src"

ENV CC afl-clang 
ENV CXX afl-clang++ 
ENV CXXFLAGS "-fprofile-arcs -ftest-coverage -g -O3" 
ENV CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage"
ENV AFL_USE_ASAN 1 
ENV VERBOSE 1
RUN /bin/bash -c "source /opt/ros/${ROSDIST}/setup.bash \ 
                    && colcon build --event-handlers console_direct+"
