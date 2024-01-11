#!/bin/bash

# Define the commands to run in each terminal
command1="roscd; cd ..; source devel/setup.bash; ros2 launch my_moveit_config move_group.launch.py use_sim_time:=True"
command2="roscd; cd ..; source devel/setup.bash; ros2 launch my_moveit_config moveit_rviz.launch.py"
command3="roscd; cd ..; source devel/setup.bash; ros2 launch moveit2_scripts pick_and_place_perception.launch.py"

# Open a new terminal and run the first command
xterm -e "bash -c '$command1; exec bash'" &
# Wait for 1 second
sleep 1

# Open a new terminal and run the second command
xterm -e "bash -c '$command2; exec bash'" &
# Wait for 1 second
sleep 4

# Open a new terminal and run the third command
xterm -e "bash -c '$command3; exec bash'" &