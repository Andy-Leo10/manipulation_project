# manipulation_project
manipulation project with robot UR3e

## Run the simulation
```
source ~/ros2_ws/install/setup.bash
ros2 launch the_construct_office_gazebo warehouse_ur3e.launch.xml
```
Check if everything is correct
```
ros2 control list_controllers
ros2 topic echo /joint_states
```

## Launch most importants nodes
+ Move Group
```
ros2 launch my_moveit_config move_group.launch.py
```
+ Robot Interface 
```
ros2 launch my_moveit_config moveit_rviz.launch.py
```
+ Custom Task: Pick & Place
```
ros2 launch moveit2_scripts pick_and_place.launch.py use_sim_time:=True
```

## Others
GUI for arms
```
sudo apt-get update
sudo apt-get install ros-humble-rqt-joint-trajectory-controller
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```
URDF Tree 
```
ros2 run tf2_tools view_frames
```
Capture positions
```
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo base_link tool0
```
Generate new cube in simulation
```
ros2 run gazebo_ros spawn_entity.py -file ~/ros2_ws/src/grasp_box.urdf -x 5.28 -y -3.84 -z 1.0 -entity grasp_box_x
```