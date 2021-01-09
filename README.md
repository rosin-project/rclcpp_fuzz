# RCLCPP fuzzing scratchpad

Infrastructure that Andrzej is collecting that he uses in fuzzing
experiments with ROS2

`Dockerfile` - the image in which we are running this experiments. Build the container using `docker build -t fuzz_rclcpp .`

`fuzz.sh` - A script that we can use to run the experiment. Run it inside the container.

`start.sh` - A script I use to start the docker image by `source start.sh` it has several overkill options but it is useful to talk to ROS gui tools, if need be
