#!/bin/bash

# run from /StateSaver/

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Build with testing enabled
colcon build --cmake-args -DBUILD_TESTING=ON
if [ $? -ne 0 ]; then
    echo "ERROR: Main project build failed"
    exit 1
fi

# Source the local setup
source install/setup.bash

# Run the test executable directly (only if builds succeeded)
echo "Build succeeded, running tests..."
./build/state_saver/state_saver_test

# Source the local setup
source install/setup.bash

# Run the test executable directly (only if builds succeeded)
echo "Build succeeded, running tests..."
./build/thrust_control/thrust_control_tests
                                                                                                                                                                                                                                           
                                                                                                                                                                                                                                           
                                                                                                                                                                                                                                           
                                                                                                                                                                                                                                           
                                                                                                                                                                                                                                           
                                                                                                                                                                                                                                           
                 