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
This has been tested on Ubuntu 18.04 and ROS Melodic.

## Set up  
In the setup directory, use:  
$ sudo ./setup.sh
This copies some key files over and changes the permission for the USB
The file is only set up for the Patriot tracker and needs to be changed to set up other devices.

## What is a Polhemus Tracker?  

## How to use with other Trackers  
* Change the set up file for the correct device
* Change the numbers...
* Make sure all the commands work.  Change as needed.
* Need to set up for the correct number of stations (Patriot only uses 2 and this code is not always ready for more.)  
