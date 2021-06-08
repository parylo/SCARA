# SCARA ROS project

## Table of contents
* [General info](#general-info)
* [Interfaces](#interfaces)
* [RViz representation](#rviz-representation)
* [GUI](#gui)
* [Vision System](#vision-system)
* [Object Recognition](#object-recognition)
* [Folders description](#folders-description)

## General info

HW information: [Scara ROS Project](https://pparylo.wixsite.com/projects/scara-ros-project)

Basic assumptions:
- Inverse kinematic and collision detection handle by MoveIt
- HW (actuators, gripper pump and valve, camera LEDs) controlled by SW on Arduino
- PyQt based GUI for manual links and end-effector movement, path planning, etc
- OpenCV based vision system for object detection 
- Deep learning based object recognition
## Interfaces
Graphic representation of information flow:

<img src="/Interfaces.png" height="700"/>

## RViz representation
<img src="/RViz.png" height="600"/>

In RViz current position of the robot is represented by colored links. The orange structure represents the target position of manipulator links. It can be changed by dragging the end-effector to the desired position. After this path can be planned and executed. Home and random valid positions can be also selected via a drop-down list.

**[Demo Video (YouTube)](https://youtu.be/MqDa4g5WqWI)**

## GUI
GUI is created with PyQt library (including Qt Designer).

Initial basic version:

<img src="/GUI.png" height="600"/>

1. Current XYZ position of end-effector (gripper).
2. Current joints position. Users can grab a slider to set join to a certain angle or distance. After slider release, the joint will move to a set position.
3. User can input end-effector XYZ coordinates. After clicking MOVE button, the manipulator will move end-effector to the desired position.
4. User can move end-effector to specific direction with distance specified by radio buttons.
5. HOME button set manipulator to predefined home position. GRIPPER button activates and deactivate the gripper tool. 

**[Demo Video (YouTube)](https://youtu.be/WDLDMQaBE6g)**

## Vision System
*TO DO
## Object Recognition
*TO DO
## Folders description
### scara_moveit
It is the main package of this project. It contains scripts to convert joint coordinates, GUI files, and Arduino microcontroller files.
### scara_description_pkg
Contains URDF and XARCO files that described the kinematic structure of the robot. In addition, it contains mesh files of all manipulator links for 3D visualization and collision detection.
### scara_moveit_config
It is autogenerated by MovIt Setup Assistant folder.
### scara_scara_arm_ikfast_plugin
Contains ikfast inverse kinematic solver. Default MoveIt solver does not support 3 and 4 DOF manipulator very well. Therefore, a new kinematic solver had to be generated for this specific robot structure.
