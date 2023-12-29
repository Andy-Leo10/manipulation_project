#! /bin/bash

# PLACE THIS FILE IN /src !!!
# This script fixes the "initial_positions.yaml file not found"
# error that occurs during the ros2 launch of move_group and 
# moveit_rviz nodes.
# This script needs to be executed only once, if the issue comes up.
# Usage: cd ~/ros2_ws/src && chmod 777 ././fix_ros2_launch_issue.bash && ./fix_ros2_launch_issue.bash

dir_path="/home/simulations/ros2_sims_ws/install/ur_description/share/ur_description"
src_path=$dir_path"/config/initial_positions.yaml"
dst_path=$dir_path"/urdf/initial_positions.yaml"
cp -f $src_path $dst_path

dir_path="/home/user/ros2_ws/install/ur_description/share/ur_description"
src_path=$dir_path"/config/initial_positions.yaml"
dst_path=$dir_path"/urdf/initial_positions.yaml"
cp -f $src_path $dst_path

cd /home/user/ros2_ws

# End of Code