# An-Online-dynamic-point-removal-SLAM-Framework-for-Dynamic-Environments
Here is the corresponding code and robot video for our manuscript. The manuscript is currently under review. At present, due to a large number of experiments, the code is relatively messy. After the manuscript is accepted, it will be continuously updated in the future.

![OurRobot](https://github.com/zhuhongwei123/An-Online-dynamic-point-removal-SLAM-Framework-for-Dynamic-Environments/blob/main/Robot_hardware.png)

## Our parking lot point cloud data, and pose estimated by Fast-lio2
https://drive.google.com/drive/folders/1d7xF9cENaoifsUSjsupPBWco56bnRkK1?usp=drive_link 

## This is a demonstration video of our robot, including robot self-exploration and self-service 3D mapping, which has been realized in a simple environment. In complex environments, key technologies are still being explored.
https://youtu.be/1xsb6KZ6HNI

## Requirements
Based on C++17
ROS (and Eigen, PCL, OpenMP): the all examples in this readme are tested under Ubuntu 18.04 and ROS Melodic.

## How to use
$ mkdir -p ~/catkin/workspace_ws/src \\
$ cd ~/catkin/workspace_ws/src \\
$ git clone https://github.com/zhuhongwei123/An-Online-dynamic-point-removal-SLAM-Framework-for-Dynamic-Environments.git

$ cd ..   

$ catkin_make

$ source devel/setup.bash

