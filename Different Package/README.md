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
-  ~/catkin_ws/src/ur5_package/ur5_robotiq85_moveit_config/launch/demo_gazebo.launch
- contains: ~/catkin_ws/src/ur5_package/ur5_robotiq85_moveit_config/launch/demo.launch
- contains: ~/catkin_ws/src/ur5_package/ur5_robotiq85_moveit_config/launch/gazebo.launch
        

