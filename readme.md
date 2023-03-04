![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)  
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

# Polhemus Patriot Package for ROS  

## Package Description  
This a ROS package made to implement a Patriot Tracker.  
It publishes a Pose type message for each station at 60Hz.  

## Requirements  
usb-1.0  
fxload  
This has been tested on Ubuntu 18.04 and ROS Melodic.

## Set up  
In the setup directory, use:  
$ ./install.bash
This copies some key files over and changes the permission for the USB.  
The file is only set up for the Patriot tracker and needs to be changed to set up other devices.  
Use install_all.bash for all the trackers.  

## What is a Polhemus Tracker?  
A Polhemus Tracker is a device that uses magnetic signals to locate a tracking device in space relative to a source.  
It gives the pose (position and orientation) of the tracker. https://polhemus.com/  
Types potentially supported by this code:  
* Patriot
* Liberty
* Fastrak

## How to use with other Trackers  
This code is specifically written for the Patriot HS Tracker system. To use with other trackers changes need to be made.  
Known changes:  
* Change the setup file for the correct device.  
* Change the numbers for product id and other values for the correct device.  
* Make sure all the commands (i.e. strings sent to device to change configuration) work.  Change as needed.  
* Need to set up for the correct number of stations (Patriot only uses 2 and for some functions that is hard coded in.)  

## License  
This work is licensed under GPL v3.  
