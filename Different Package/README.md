Notes while following the following github stuff..

#download the following

1. Download the moveit package: https://moveit.ros.org/install/
```
sudo apt install ros-noetic-moveit
```
2.  install the following git hub package
```
cd ~/catkin_ws/src
git clone https://github.com/ClangWU/ur5_package.git
```
```
cd .. && catkin_make
```
3. blah?
```
source devel/setup.bash
```
```
roslaunch ur5_robotiq85_moveit_config demo_gazebo.launch
```
- tracing that launch file..
-         ~/catkin_ws/src/ur5_package/ur5_robotiq85_moveit_config/launch/demo_gazebo.launch
-         contains: ~/catkin_ws/src/ur5_package/ur5_robotiq85_moveit_config/launch/demo.launch
-         contains: ~/catkin_ws/src/ur5_package/ur5_robotiq85_moveit_config/launch/gazebo.launch
        
#trying toi make custom world
added this line into bashrc file...
```
pico ~/.bashrc

```
```
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/Desktop 
```
-Theoretically this command should work now since i created a "custom world"
**custom world is not working........**

```
roslaunch ur5_robotiq85_moveit_config demo_gazebo.launch world_name:=~/Desktop/testing_tables.world

```
did some really jank shit.. did a new save called demo2_gazebo.launch
on line 33 added:  <arg name="world_name" value="$(dirname)/testing_tables.world"/>
I then saved a test world into that directory..
-run this bad boy from the home desktop
```
roslaunch ur5_robotiq85_moveit_config demo2_gazebo.launch
```
