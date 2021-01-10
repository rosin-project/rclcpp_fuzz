# RCLCPP fuzzing scratchpad

Infrastructure that Andrzej is collecting that he uses in fuzzing
experiments with ROS2.

The idea is that you:

1. clone this repo on the host system.
2. Build the docker using the provided dockerfile (below).
3. Start the docker container using `docker run` (see `script.sh`)
   below. It is important that the cloned source tree of this project
   is mounted in the work space as src/rclcpp_fuzz
4. Use the fuzz.sh or the lcov.sh script (below) to fuzz or test
   rclcpp

## Key Files

The scripts are annotated, and it makes sense to read them.  In fact,
they can be seen more as instructions than automation.

`script/Dockerfile` - the image in which we are running this experiments.
  Build the container using `docker build -t fuzz_rclcpp .` when the
  current directory is scripts.  This is done outside the container,
  on the host system.

`scripts/fuzz.sh` - A script that we can use to run the fuzzing of the
  simple service client, with the rclcpp instrumented. Run it inside the
  container by invoking `source src/rclcpp_fuzz/scripts/fuzz.sh`. In
  the container we assume you are in the root of the work space,
  unless stated otherwise.  The script builds the instrumented client,
  initialzes the coverage data, runs the fuzzer, and produces the
  coverage report in the directory of this projec tree under fuzz/ (so
  you can access it conveniently from the host system)

  The results of afl-plot are found in `fuzz/graphs/` after a
  succesful run.  The results of lcov are found in
  'fuzz/Coverage_Report/' after a succesful run.

  To give `afl-fuzz` more time change the value of `DURATION` in the
  `fuzz.sh` script.

`scripts/lcov.sh` - A script that we can use to produce the coverage
  report for the built-in rclcpp tests. Run it inside the container by
  invoking `source src/rclcpp_fuzz/scripts/lcov.sh`

`scripts/start.sh` - A script I use to start the docker image by
  `source start.sh` it has several overkill options but it is useful to
  talk to ROS gui tools, if need be.  It is used on Ubuntu 20.04, so you
  may need to make your own for mac, etc.  The important thing is to
  mount this repository in the workspace as src/rclcpp_fuzz.  The rest
  is ignorable most likely

## Known problems

- Coverage files are created by the user root in the container. They
  are not readable for your usual user on the host system.  Use `sudo
  chown -Rv youruser:youruser .` to regain access.

- `Afl-plot` (the very last step of `fuzz.sh`) fails if `afl-fuzz` has
  not produced any results.  This happens if `DURATION` in `fuzz.sh`
  is set too short.
