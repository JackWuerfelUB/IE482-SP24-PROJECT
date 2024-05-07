# IE482-SP24-PROJECT
## Robotic Bartender, AKA just get the arm to do something
- Students: *Jack Wuerfel, Thomas Allan*
- Professor: Dr. Chase Murray
- University at Buffalo

---

## Motivation / Overview of your project.
  
- `Application` What do you plan to do?  We want to use the UR5 robot arm as an automated bartender. Our goal is to have the robot take in voice commands, translate that to a pre-assigned location of the specified drink, and have the arm carefully grab/transport that drink to the user at the bar.

- `Motivation` Why should anyone care?  Our motivation is getting robo-crunk. Just kidding. But in all seriousness, we've seen a progression towards automation at restaurants and other service industries within the past five years. For example, an Asian buffet in Williamsville called "Wind" uses a robot (called a BellaBot) to deliver small plates of food to your table. A robotic arm that delivers drinks could potentially free up staff to perform other duties that the robot can't do yet (saying YET because the robot may eventually be able to take payments for drinks, detect and clean spills, and recognize regular patrons. Imagine if Sam Malone from "Cheers!" was a robot.)

- `Scope Shift` We started off with many ideas of grandiose proportions but soon came to the realization that senoiritis was real, this project was quite monumental, and there were not alot of resources out there with the majority of the "work" already done (*meaning ready made repo's to clone that just worked or had adequate documentation for someone to easily use*). The goal shifted and we want to explain how a repo we found online works and try to get it to behave in the manner of a robotic bartender.
  
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
- (delete this) If you had more time, what would you do with this project?
    - (delete this) Are there some bugs you need to fix?  Please document where these are, what you've tried to do to fix them, and suggestions you have for how these could be fixed by someone else.
    - (delete this) Are there new features you'd add?  Please provide as many details as possible.

---
# (delete this) Structure of your Repo
(delete this) Your repository should have (at a minimum):
- (delete this) A `README.md` file (details described above)
- (delete this) An `images` or `media` folder where you stash all of the pictures, small videos, etc, that are referenced in your `README.md` file
- (delete this) One or more directories where you place your code (i.e., Python scripts and other supporting files).

--- 


