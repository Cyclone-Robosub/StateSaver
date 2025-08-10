#!/bin/bash

# run from /StateSaver/

# Source ROS environment
cd ../
source /opt/ros/jazzy/setup.bash

# Source the local setup

# Run the test executable directly (only if builds succeeded)
echo "Open either the state.csv or the latest gtest result text file.\n"
echo "Press Control - C twice to stop the test"
cd build/state_saver
./state_saver_test # > ../../Latest_gtest_results.txt                                                                               
                                                                                                                                                                                                                                           
                                                                                                                                                                                                                                           
                                                                                                                                                                                                                                           
                                                                                                                                                                                                                                           
                 