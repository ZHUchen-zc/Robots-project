<p align="center">  
   <img src = "source/ub.png" width = 500>
</p >

# <p align="center">Robotics Project</p >

<p align="center">Professor: Ralph SEULIN</p >  
<p align="center">Students: ZHU Chen,GU Heng</p >  

## Contents
1. [Introduction](#introduction)
2. [Move Robot](#Move-Robot)
3. [Mapping and Localization](#Mapping-and-Localization)
4. [Navigation](#Navigation)
5. [Follow waypoints](#Follow-waypoints)
6. [Conclusion](#Conclusion)

## Introduction
## Move robot
## Mapping and Localization
## Navigation
In order to achieve autonomous navigation of the robot, we need to use a navigation stack. The navigation stack is a set of ROS nodes and algorithms that are used to automatically move the robot from one point to another, thereby avoiding all obstacles that the robot may encounter.  

<p align="center">  
   <img src = "source/navigation package.png" width = 800>
</p >  

In order to realize the path planning of the robot, we need to use the 2D Nav target tool of the mobile robot, costmap, and calculate the trajectory. In order to achieve these goals, we need to use the move_base package. The move_base package contains the move_base node.  The move_base node is one of the major elements in the ROS Navigation Stack, since it links all of the elements that take place in the Navigation process. As shown below:  

<p align="center">  
   <img src = "source/move_base.png" width = 400>
</p >  

The move_base node implements SimpleActionServer and receives target information with geometry_msgs/PoseStamped messages. The following table describes the description of some topics subscribed and published by the move_base node.  

<p align="center">  
   <img src = "source/movebase topic.png" width = 900>
</p >  











## Follow waypoints
