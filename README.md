# IE482-SP24-PROJECT

Jack Wuerfel, Thomas Allan

3/8/2024

Idea 1 - DrinkBot

_What do you plan to do? (application)_  We want to use the UR5 robot arm as an automated bartender.  Our goal is to have the robot to take in voice commands, translate that to a pre-assigned location of the specified drink, and have the arm carefully grab/transport that drink to the user at the bar.

_Why should anyone care? (motivation)_  Our motivation is getting robo-crunk.  Just kidding.  But in all seriousness, we've seen a progression towards automation at restaurants and other service industries within the past five years.  For example, an Asian buffet in Williamsville called "Wind" uses a robot to deliver small plates of food to your table.  A robotic arm that delivers drinks could potentially free up staff to perform other duties that the robot can't do yet (saying YET because the robot may eventually be able to take payments for drinks, detect and clean spills, and recognize regular patrons.  Imagine if Sam Malone from "Cheers!" was a robot.)

_How do you plan to accomplish this? (milestones)_  The first thing to do is set up a virtual world in Gazebo.  This world would have the UR5, a bar, and a shelf that contains bottles.  Next, we should focus on getting the UR5 arm to move using ROS.  After we get the arm to move manually, we will then program a set of commands that tell the robot to grab a specific drink or bottle.  After this, we will implement voice commands that tell the robot to grab/transport a specific drink.

The following programs/tools will be implemented during this project: (in no particular order)
- Whisper AI or LlamaIndex, if applicable
- UR5
- Gazebo
- SolidWorks
- Big Ditch "Hayburner" IPA
- Margarita(s)

Limitations/Assumptions:
- One-semester project, time limitation
- Learning new software and programming, knowledge limitation
- Potential difficulty connecting to robotic arm, could possibly use voice commands through the laptop to operate in Gazebo?
---
Idea 2 - Random Inspection Bot

_What do you plan to do? (application)_  We want to use the UR5 robot arm as tool to transport parts from a manufacturing floor into a Coordinate Measuring Machine (CMM).  We plan on creating a "completed parts bin" at a machine shop from which the robot randomly selects.  The arm will then transport this part to the CMM, wait for pass/fail criteria from the CMM, then move the part to the appropriate pass/fail bin.

_Why should anyone care? (motivation)_  In a machine shop or manufacturing facility, it's vital that the parts being manufactured be inspected before they get distributed.  Depending on the level of quality desired, a variety of inspection methods may be utilized.  In this case, we are simulating a medium volume shop that produces high precision parts.  Robot arms can be used in place of an inspector in order to automate this process.  The company can better utilize their staff if less of them are required to perform inspections.  

_How do you plan to accomplish this? (milestones)_  The first thing to do is set up a virtual world in Gazebo.  This world would have the UR5, an inspection table reserved for the CMM, and a parts bin that contains randomly sized parts.  Next, we should focus on getting the UR5 arm to move using ROS.  After we get the arm to move manually, we will then program a set of commands that tell the robot to grab a random part.  After this, we will implement the code to have the robot grab a random part and transport it to the CMM table for inspection.  Finally, we will implement code to generate a given probability of pass/fail conditions, and the robot will be able to grab the part from the CMM and move it to the right bin.

The following programs/tools will be implemented during this project: (in no particular order)
- UR5
- SolidWorks
- Gazebo

Limitations/Assumptions:
- One-semester project, time limitation
- CMM operated independently, assuming it correctly identifies pass/fail
- Potential difficulty connecting to robotic arm, could just simulate in Gazebo
---
Links:

https://wiki.ros.org/ur_gazebo

https://openai.com/research/whisper

https://github.com/openai/whisper

https://www.youtube.com/watch?v=jOHb4M-jj4E

https://www.youtube.com/watch?v=ayp87SjrwPc

https://www.youtube.com/watch?v=0EKkzCQ7cCE

https://docs.google.com/presentation/d/1kP1_A7p5AVnGYRBCUNZKsF0EOGWE4AL7kdgR3cCcOcU/edit#slide=id.p

https://www.llamaindex.ai/open-source

https://www.llamaindex.ai/blog/ai-voice-assistant-enhancing-accessibility-in-ai-with-llamaindex-and-gpt3-5-f5509d296f4a

https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation
