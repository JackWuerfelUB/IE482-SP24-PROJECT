# IE482-SP24-PROJECT
## Robot Bartender, AKA just get the UR5 to do something
- Students: *Jack Wuerfel, Thomas Allan*
- Professor: Dr. Chase Murray
- University at Buffalo

---

## Motivation / Overview of your project.
  
- `Overview`  We initially planned on using the UR5 robot arm as an automated bartender. Our goal was to have the robot take in voice commands, translate that to a pre-assigned location of the specified drink, and have the arm carefully grab/transport that drink to the user at the bar.  But such was not the case; after the first few weeks of research, the project underwent ... what is essentially the reverse of "scope creep".  The deeper we dove into the individual steps of the project, the more we discovered the complexity of the tasks before us.  We started by removing one small aspect of the project until eventually we realized that "we just need to get this damn thing to move!"

- `Motivation` Our motivation was initially out of genuine curiosity: can we get a robot to serve us drinks? pass the butter? do other things?  As stated previously, we were quickly humbled by the sheer scope of our project.  At that point, we'd be happy if we could control a UR5 arm (in Gazebo or the in-class arm) let alone use voice commands to move it and use the end effector.  The motivation became twofold: A- perform enough tasks and functions with the UR5 for a passing grade, and 2- create a usable repository that provides detailed documentation on the implementation of the UR5 in Gazebo.

- `Scope Shift` We started off with many ideas of grandiose proportions but soon came to the realization that senoiritis was real, this project was quite monumental, and there were not a lot of resources out there with the majority of the "work" already done (*repositories we could clone that were simply plug-and-play, or had adequate documentation for someone to easily implement themselves*). We aim to describe the manner in which a pre-existing repository was helpful in forming the basis of our robot bartender.
  
## Installation Instructions
- (delete this) Assume that your reader has already installed ROS Noetic on a machine running Ubuntu 20.04.
- (delete this) Provide **detailed** installation instructions so your audience can re-create your project.
    - (delete this) Include command line instructions in markdown code blocks so it's easy for your audience to copy-paste the commands.
 
1. Set up Linux environment by running the following script line by line:
   ```
   https://github.com/IE-482-582/spring2024/blob/main/vm/autoinstall_482-582_2004.sh
   ```
2. Download the moveit package:
   ```
   sudo apt install ros-noetic-moveit
   ```
3. Install this repository into your catkin workspace and perform a catkin make:
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/JackWuerfelUB/IE482-SP24-PROJECT.git
   ```
   ```
   cd .. && catkin_make
   ```
4. Add Gazebo resource pathing:
   ```
   pico ~/.bashrc
   ```
   ```
   export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/IE482-SP24-PROJECT/ur5_robotiq85_moveit_config/worlds
   ```  

## Demonstration (1 of 2) - Showing Basic UR5 package Capabilties
- (delete this) Use a combination of screen shots, video, and paragraph explanation to show off your project.
- Run the line of code listed in the following section "How to Run the Code (1 of 2) - For Basic UR5 package Capabilties"
- 

## How to Run the Code (1 of 2) - For Basic UR5 package Capabilties
- (delete this) Now that your audience has installed the necessary software, how do they run it?
Run the following code:
>RVIZ and Gazebo should open.
   ```
   roslaunch ur5_robotiq85_moveit_config demo_gazebo.launch
   ```


## Demonstration (2 of 2) - Showing Basic UR5 package Capabilties
- (delete this) Use a combination of screen shots, video, and paragraph explanation to show off your project.


## How to Run the Code (2 of 2) - For Basic UR5 Capabilties
- (delete this) Now that your audience has installed the necessary software, how do they run it?

## References
- (delete this) Include links to websites you found helpful.
- (delete this) Also mention websites you tried but were not as helpful
> This amazing respository contained many of the building blocks on which we were able to expand our knowledge of the UR5.
```
https://github.com/IE-482-582/spring2024/tree/main/notes
```
> This repository was a helpful starting point from which to add basic functionality to the UR5
```
https://github.com/ClangWU/ur5_package
```
> MoveIt was beneficial in __________________
```
https://moveit.ros.org/install/
```
> This Tutorial was not helpful in that it runs on ROS Melodic.
```
https://www.youtube.com/watch?v=ayp87SjrwPc
```
## Future Work
- If this became a hobby project for us (no timeline, no pressure, no grade), we'd publish a detailed repository that new UR5 users would use to bring the model into Gazebo and get it to move (using ROS Noetic and Ubuntu 20.04).  Ideally, there would be several types of end effectors available in the model space to use and control.  The user could control the UR5 via keypad shortcuts, similar to using Teleop to move the Husky as we did in class.
- Jack list bugs here _____________.  Are there some bugs you need to fix?  Please document where these are, what you've tried to do to fix them, and suggestions you have for how these could be fixed by someone else.
- Jack list new features we'd add _____________. Are there new features you'd add?  Please provide as many details as possible.

---
# (delete this) Structure of your Repo
(delete this) Your repository should have (at a minimum):
- (delete this) A `README.md` file (details described above)
- (delete this) An `images` or `media` folder where you stash all of the pictures, small videos, etc, that are referenced in your `README.md` file
- (delete this) One or more directories where you place your code (i.e., Python scripts and other supporting files).

--- 


