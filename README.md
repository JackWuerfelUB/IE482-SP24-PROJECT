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
- Run the line of code listed in the following section "How to Run the Code (1 of 2) - For Basic UR5 package Capabilties"

- UR5 spawned into expty Gazebo World
![UR5 spawned into expty Gazebo World](https://github.com/JackWuerfelUB/IE482-SP24-PROJECT/blob/main/Media/DEMO1%20-%20base%20UR5%20Package%20stuff/1-Demo1%20(arm%20in%20empty%20world).png)

- RVIZ window with "planning" window shown, ensure planning group is "ur5_arm".
![Model in Gazebo](https://github.com/JackWuerfelUB/IE482-SP24-PROJECT/blob/main/Media/DEMO1%20-%20base%20UR5%20Package%20stuff/2-RVIZ%20Demo%20planning%20UR5%20Arm.png)

- RVIZ window with "Joints" window shown, manipulate each joint to move robot arm.
![Model in Gazebo](https://github.com/JackWuerfelUB/IE482-SP24-PROJECT/blob/main/Media/DEMO1%20-%20base%20UR5%20Package%20stuff/3-RVIZ%20Motion%20Planning%20Joints.png)

- RVIZ window back on "Planning" window, press "Plan & Execute" to initiate movement.
![Model in Gazebo](https://github.com/JackWuerfelUB/IE482-SP24-PROJECT/blob/main/Media/DEMO1%20-%20base%20UR5%20Package%20stuff/4-Motion%20Planning%20Close%20Up.png)

- RVIZ model will continue to show the movement that was executed, like a forever looping gif.
![Model in Gazebo](https://github.com/JackWuerfelUB/IE482-SP24-PROJECT/blob/main/Media/DEMO1%20-%20base%20UR5%20Package%20stuff/5-RVIZ%20ARM%20movement.png)

- Example of the UR5 model moving in the Gazebo simulated environment after movement was planned and executed in RVIZ.
![Model in Gazebo](https://github.com/JackWuerfelUB/IE482-SP24-PROJECT/blob/main/Media/DEMO1%20-%20base%20UR5%20Package%20stuff/6-UR5%20arm%20after%20movement.png)


## How to Run the Code (1 of 2) - For Basic UR5 package Capabilties

Run the following code:
>RVIZ and Gazebo should open.
   ```
   roslaunch ur5_robotiq85_moveit_config demo_gazebo.launch
   ```

## Demonstration (2 of 2) - For Robotic Bartender

- The first image below shows the custom gazebo simulated world we created for this demonstration. The robot is positioned on a table and their is a beer can in front of it.
![Model in Gazebo](https://github.com/JackWuerfelUB/IE482-SP24-PROJECT/blob/main/Media/DEMO2%20-%20Bartender%20bot/1%20-%20Bot%20spawned%20in%20empty%20world%20with%20rvix%20in%20background.png)

- The second image shows a the gazebo environment and temrinal window once the user runs the movement script. As you can see in the below image the patron has many options of drinks to choose from.
![Model in Gazebo](https://github.com/JackWuerfelUB/IE482-SP24-PROJECT/blob/main/Media/DEMO2%20-%20Bartender%20bot/2-%20%20robotender%20awaiting%20order.png)

- The third image shows the robot failing to grab a beer. The robot then has a moment of existential dread and realizes it is not cut out to be a bartender.
![Model in Gazebo](https://github.com/JackWuerfelUB/IE482-SP24-PROJECT/blob/main/Media/DEMO2%20-%20Bartender%20bot/3-%20robotender%20failing%20to%20grab%20drink%20and%20self%20shaming.png)

## How to Run the Code (2 of 2) - For Robotic Bartender
- (delete this) Now that your audience has installed the necessary software, how do they run it?
1. In terminal 1, run the following code:
   ```
   roslaunch ur5_robotiq85_moveit_config demo2_gazebo.launch
   ```

2. In terminal 2, run the following script:
   ```
   cd ~/catkin_ws/src/IE482-SP24-PROJECT/ur5_motion_planning/scripts
   python3 ur5_moveit_gazebo_env-JRW-TA-SAVE.py 
   ```
  
![Model in Gazebo](https://github.com/JackWuerfelUB/IE482-SP24-PROJECT/blob/main/Media/meme.png)

## Some things we achieved (don't fail us please):
- Creation of custom world
- Modification of launch files
- Stellar attendence in class

## Final Status 
To Do / Progress Table
| Task | Progress (%) |
| -------------------------------------------- | ----------- |
| Get package funtioning and understand it | 81.2% |
| Create custom world and get it to open with launch file | 100% |
| write a script to take the users input of which drink they want and then does something with that | 6.5% |
| Create documentation and structure package so someone can just download this repo | 95% |
| Pass IE482 | Low % |


## References

> Repository used for Spring24 semester of IE482, "somewhat helpful" [just kidding].
```
https://github.com/IE-482-582/spring2024/tree/main/notes
```
> This repository was a helpful starting point from which to add basic functionality to the UR5.
```
https://github.com/ClangWU/ur5_package
```
> MoveIt is required for the UR5 package.
```
https://moveit.ros.org/install/
```
> This Tutorial was not helpful in that it runs on ROS Melodic.
```
https://www.youtube.com/watch?v=ayp87SjrwPc
```
## Future Work
- If this became a hobby project for us (no timeline, no pressure, no grade), we'd publish a detailed repository that new UR5 users would use to bring the model into Gazebo and get it to move (using ROS Noetic and Ubuntu 20.04).  Ideally, there would be several types of end effectors available in the model space to use and control.  The user could control the UR5 via keypad shortcuts, similar to using Teleop to move the Husky as we did in class.
- Figure out a way to communicate with the UR5 that is in Bell 427 and use this Repo as the "simulated" version of that environment.
- Modify the python script mentioned in demonstration number 2.


--- 


