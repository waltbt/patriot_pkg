// patriot_pkg_exec.cpp

/*
patriot_pkg version 0.1 -- ROS Package for the Patriot Polhemus Trackers.
Copyright  ©  2023  Benjamin Thomas Walt

This file is part of the ROS patriot_pkg.

patriot_pkg is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

patriot_pkg is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with patriot_pkg.  If not, see <http://www.gnu.org/licenses/>.

*************************************************************************
*/

#include <ros/ros.h>
#include "patriot_pkg/TrackerControl.hpp"
#include "patriot_pkg/tracker_pose.h"



int main (int argc, char **argv)
{
  ros::init(argc, argv, "patriot_pkg_node");
  ros::NodeHandle nh;
  ros::Rate rate(60); //Patriot works at maximum 60Hz
  ros::Publisher arm_pose_pub = nh.advertise<patriot_pkg::tracker_pose>("patriot/arm_pose", 10);
  TrackerControl* tracker = new TrackerControl;
  int num_stations = tracker->GetStationNumber();
  if (num_stations <= 0 || num_stations > 2){ //Patriot can only have 1 or 2 stations
    std::cout << "Error setting up Patriot" << std::endl;
  }
  std::cout<<"Num Stations: " << num_stations<<std::endl;
  _Quaternion Quaternion[2];
  patriot_pkg::tracker_pose msg;
  int ret = -1;
  tracker->GetPose(Quaternion); //Read just to get rid of any garbage readings
  // First read often contains junk and will throw a station number error
  while(ros::ok()){
    ret = tracker->GetPose(Quaternion);
    if(ret == 0){
      for(int itr=0;itr<num_stations;itr++){
        // Uncomment for troubleshooting
        // std::cout << std::endl;
        // std::cout << "Station: " << Quaternion[itr].station_number << std::endl;
        // std::cout << "X: " << Quaternion[itr].x_pos_cm << std::endl;
        // std::cout << "Y: " << Quaternion[itr].y_pos_cm << std::endl;
        // std::cout << "Z: " << Quaternion[itr].z_pos_cm << std::endl;

        if(Quaternion[itr].station_number == 0){
          msg.tracker_1_pose.position.x = Quaternion[itr].x_pos_cm;
          msg.tracker_1_pose.position.y = Quaternion[itr].y_pos_cm;
          msg.tracker_1_pose.position.z = Quaternion[itr].z_pos_cm;
          msg.tracker_1_pose.orientation.x = Quaternion[itr].q_x;
          msg.tracker_1_pose.orientation.y = Quaternion[itr].q_y;
          msg.tracker_1_pose.orientation.z = Quaternion[itr].q_z;
          msg.tracker_1_pose.orientation.w = Quaternion[itr].q_w;
        }else if(Quaternion[itr].station_number == 1){
          msg.tracker_2_pose.position.x = Quaternion[itr].x_pos_cm;
          msg.tracker_2_pose.position.y = Quaternion[itr].y_pos_cm;
          msg.tracker_2_pose.position.z = Quaternion[itr].z_pos_cm;
          msg.tracker_2_pose.orientation.x = Quaternion[itr].q_x;
          msg.tracker_2_pose.orientation.y = Quaternion[itr].q_y;
          msg.tracker_2_pose.orientation.z = Quaternion[itr].q_z;
          msg.tracker_2_pose.orientation.w = Quaternion[itr].q_w;
        }else{
          std::cout << "Error - Unknown station number: " << Quaternion[itr].station_number  << std::endl;
        }

      }// end of for loop
      arm_pose_pub.publish(msg);
    }else{
      std::cout << "Failed Read " << std::endl;
    }
    rate.sleep();
  }// End of while loop

}
