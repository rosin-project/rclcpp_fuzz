docker run \
  --env="DISPLAY" \
  --net=host \
  --group-add video \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --volume="$XAUTHORITY:/root/.Xauthority:rw"   \
  --device=/dev/dri:/dev/dri \
  --privileged \
  --mount type=bind,source=${PWD},target=/opt/ros_ws/src/rclcpp_fuzz \
  -it fuzz_rclcpp 
