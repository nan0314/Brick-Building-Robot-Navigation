# Brick Building Robot Perception + Navigation

This is a project I worked on while earning my Master's degree at Northwestern University. Our goal was to mount an HDT Adroit Manipulator Arm on a Clearpath Jackal to create a robot capable of picking, transporting, and placing bricks such that it may be used to complete tasks such as wall building.

## Description

This repository contains packages that enable the perception and navigation portion of the project via the Jackal equipped with a Velodyne Lidar and Realsense Depth Camera D435i. The packages enable the Jackal to locate bricks in its environment, navigate to them, and return to its starting location so that bricks may be picked and placed as desired.

## Getting Started

### Dependencies

* ROS Noetic
* Realsense2_camera
* PCL

### Installing

* Follow @dinvincible98's instructions for setting up the Jackal with ROS Noetic
* Clone this repository into your workspace on the remote pc as well as the Jackal
* Move setup_laptop.bash from Jackal_ROS_Noetic_Bringup to the devel folder of your workspace on the remote pc and do the same with setup_jackal.bash on the Jackal.
* Adjust line 4 of the bash files to source your workspace. **Remember to source these bash files every time you open a new terminal on either the remote pc or Jackal.**

### Executing program

* On the Jackal, source setup_jackal.bash and run the following command:
```
roslaunch brick_building startup_jackal.launch
```
* On the remote pc, source setup_laptop.bash and run the following command:
```
roslaunch brick_building build.launch
```
* The robot will start detecting bricks. The Jackal will move to the nearest detected brick when the /target_ready topic recieves an empty message. The jackal will move back to the home position when the /home_ready topic recieves an empty message. This can be done using the following commands:
```
rostopic pub /target_ready std_msgs/Empty "{}"      // Jackal will move to brick
rostopic pub /home_ready std_msgs/Empty "{}"        // Jackal will move to home
```

## TODO

* Implement Exploration algorithm for when no bricks are detected
* Implement secondary brick memory to store locations of other detected bricks and search for them after current brick has been picked and placed.

## Authors

Nathaniel Nyberg

## Acknowledgments

Inspiration, code snippets, etc.
* [@dinvincible98](https://github.com/dinvincible98/Jackal_ROS_Noetic_Bringup.git)
