# Bug2PathPlanning_ROS-Python

## Overview

The task is to create a package that, when launched, will navigate the turtlebot to three different goal positions one after another that are published on a topic called /goals. Avoid obstacles on the way and reach all the three goals
![The Goal Points](https://github.com/kishanmandal94/Path-planning-of-Two-wheel-robot-in-ROS/blob/master/images/final_project_AMR_goals.png)

## Approach
Bug2 is a simple algorithm in which the robot follows the straight imaginary line to the target using the co-ordinates of the destination. If it encounters an object, it goes around the object until it re-encounters the straight line and then starts following it again.

There are mainly two states namely: go_to_point and wall_follow. 

In the "go_to_point" state, the robot is just following the straight imaginary line.

But when encounters an object eg. wall, it tries to circumscribe the object while the algorithm keeps checking whether it hits the imaginary straight line or not and if it does then it returns back to the "imaginary line" state. A special checker function is often called to see how far it is from the imaginary line.

## Launching the file

```bash
roslaunch km_183437_miniprj start.launch 
```
### Refrences

[1] [Link to Youtube tutorials](https://www.youtube.com/playlist?list=PLK0b4e05LnzY2I4sXWTOA4_82cMh6tL-5)

[2] [Link to the reference code used](https://bitbucket.org/theconstructcore/two-wheeled-robot-motion-planning/src/master/scripts/)
