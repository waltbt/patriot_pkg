// patriot_pkg_exec.cpp

/*
patriot_pkg version 0.2 -- ROS Package for the Patriot Polhemus Trackers.
Copyright  ©  2023-4  Benjamin Thomas Walt

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
#include "patriot_pkg/set_hemisphere.h"
#include "patriot_pkg/get_hemisphere.h"

// TrackerControl* tracker = new TrackerControl;

// void set_hemisphere_callback(const patriot_pkg::set_hemisphere::ConstPtr& msg);

// /*Callback to set the hemisphere in use.  The values sent, must have a specific format, but this is not checked.*/
// void set_hemisphere_callback(const patriot_pkg::set_hemisphere::ConstPtr& msg){
//   tracker->SetHemisphere(msg->hemisphere[0],msg->hemisphere[1],msg->hemisphere[2]);
//   std::cout << "Setting Hemisphere X: " << msg->hemisphere[0] << " Y: " << msg->hemisphere[1] << " Z: " << msg->hemisphere[2] << std::endl;
// }

// /*Callback to check the hemisphere in use.  The values is what is stored in the state, not the device.*/
// bool get_hemisphere_callback(patriot_pkg::get_hemisphere::Request& req, patriot_pkg::get_hemisphere::Response& resp){
//   int* ret_value = tracker->GetHemisphere();
//   resp.hemisphere = {ret_value[0],ret_value[1],ret_value[2]}; // Is there a cleaner way to do this?
//   std::cout << "Setting Hemisphere X: " << ret_value[0] << " Y: " << ret_value[1] << " Z: " << ret_value[2] << std::endl;
//   return true; //This seems to be required, but serves no function in this code
// }

// int main (int argc, char **argv)
// {
//   ros::init(argc, argv, "patriot_pkg_node");
//   ros::NodeHandle nh;
//   ros::Rate rate(60); //Patriot works at maximum 60Hz
//   ros::Publisher arm_pose_pub = nh.advertise<patriot_pkg::tracker_pose>("patriot/arm_pose", 10);
//   ros::Subscriber set_hemisphere_sub = nh.subscribe("patriot/set_hemisphere", 10, set_hemisphere_callback);
//   ros::ServiceServer service = nh.advertiseService("patriot/get_hemisphere", get_hemisphere_callback);
//   // tracker = new TrackerControl;
//   int num_stations = tracker->GetStationNumber();
//   if (num_stations <= 0 || num_stations > 2){ //Patriot can only have 1 or 2 stations
//     std::cout << "Error setting up Patriot" << std::endl;
//   }
//   std::cout<<"Num Stations: " << num_stations<<std::endl;
//   _Quaternion Quaternion[2];
//   patriot_pkg::tracker_pose msg;
//   int ret = -1;
//   tracker->GetPose(Quaternion); //Read just to get rid of any garbage readings
//   // First read often contains junk and will throw a station number error
//   while(ros::ok()){
//     ret = tracker->GetPose(Quaternion);
//     if(ret == 0){
//       for(int itr=0;itr<num_stations;itr++){
//         // Uncomment for troubleshooting
//         // std::cout << std::endl;
//         // std::cout << "Station: " << Quaternion[itr].station_number << std::endl;
//         // std::cout << "X: " << Quaternion[itr].x_pos_cm << std::endl;
//         // std::cout << "Y: " << Quaternion[itr].y_pos_cm << std::endl;
//         // std::cout << "Z: " << Quaternion[itr].z_pos_cm << std::endl;

//         if(Quaternion[itr].station_number == 0){
//           msg.tracker_1_pose.position.x = Quaternion[itr].x_pos_cm;
//           msg.tracker_1_pose.position.y = Quaternion[itr].y_pos_cm;
//           msg.tracker_1_pose.position.z = Quaternion[itr].z_pos_cm;
//           msg.tracker_1_pose.orientation.x = Quaternion[itr].q_x;
//           msg.tracker_1_pose.orientation.y = Quaternion[itr].q_y;
//           msg.tracker_1_pose.orientation.z = Quaternion[itr].q_z;
//           msg.tracker_1_pose.orientation.w = Quaternion[itr].q_w;
//         }else if(Quaternion[itr].station_number == 1){
//           msg.tracker_2_pose.position.x = Quaternion[itr].x_pos_cm;
//           msg.tracker_2_pose.position.y = Quaternion[itr].y_pos_cm;
//           msg.tracker_2_pose.position.z = Quaternion[itr].z_pos_cm;
//           msg.tracker_2_pose.orientation.x = Quaternion[itr].q_x;
//           msg.tracker_2_pose.orientation.y = Quaternion[itr].q_y;
//           msg.tracker_2_pose.orientation.z = Quaternion[itr].q_z;
//           msg.tracker_2_pose.orientation.w = Quaternion[itr].q_w;
//         }else{
//           std::cout << "Error - Unknown station number: " << Quaternion[itr].station_number  << std::endl;
//         }

//       }// end of for loop
//       arm_pose_pub.publish(msg);
//     }else{
//       std::cout << "Failed Read " << std::endl;
//     }
//     ros::spinOnce(); //This allows the callback to function
//     rate.sleep();
//   }// End of while loop

// }






class PatriotMain {
    private:
    // int counter;
    // ros::Publisher pub;
    // ros::Subscriber number_subscriber;
    // ros::ServiceServer reset_service;
    ros::Publisher arm_pose_pub;
    ros::Subscriber set_hemisphere_sub;
    ros::ServiceServer service;
    TrackerControl::TrackerControl* tracker;
    int num_stations;
    
    public:
    PatriotMain(ros::NodeHandle *nh) {
        // counter = 0;
        // pub = nh->advertise<std_msgs::Int64>("/number_count", 10);    
        // number_subscriber = nh->subscribe("/number", 1000, 
        //     &NumberCounter::callback_number, this);
        // reset_service = nh->advertiseService("/reset_counter", 
        //     &NumberCounter::callback_reset_counter, this);


        ros::Rate rate(60); //Patriot works at maximum 60Hz
        arm_pose_pub = nh.advertise<patriot_pkg::tracker_pose>("patriot/arm_pose", 10);
        set_hemisphere_sub = nh->subscribe("patriot/set_hemisphere", 10, &PatriotMain::set_hemisphere_callback, this);
        service = nh->advertiseService("patriot/get_hemisphere", &PatriotMain::get_hemisphere_callback, this);
        
        tracker = new TrackerControl;
        
        num_stations = tracker->GetStationNumber();
        if (num_stations <= 0 || num_stations > 2){ //Patriot can only have 1 or 2 stations
          std::cout << "Error setting up Patriot" << std::endl;
        }
        std::cout << "Num Stations: " << num_stations << std::endl;
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
          ros::spinOnce(); //This allows the callback to function
          rate.sleep();
        }// End of while loop
    }


    /*Callback to set the hemisphere in use.  The values sent, must have a specific format, but this is not checked.*/
    void set_hemisphere_callback(const patriot_pkg::set_hemisphere::ConstPtr& msg){
      tracker->SetHemisphere(msg->hemisphere[0],msg->hemisphere[1],msg->hemisphere[2]);
      std::cout << "Setting Hemisphere X: " << msg->hemisphere[0] << " Y: " << msg->hemisphere[1] << " Z: " << msg->hemisphere[2] << std::endl;
    }

    /*Callback to check the hemisphere in use.  The values is what is stored in the state, not the device (the device cannot tell you what the current config is).*/
    bool get_hemisphere_callback(patriot_pkg::get_hemisphere::Request& req, patriot_pkg::get_hemisphere::Response& resp){
      int* ret_value = tracker->GetHemisphere();
      resp.hemisphere = {ret_value[0],ret_value[1],ret_value[2]}; // Is there a cleaner way to do this?
      std::cout << "Hemisphere set to X: " << ret_value[0] << " Y: " << ret_value[1] << " Z: " << ret_value[2] << std::endl;
      return true; //This seems to be required, but serves no function in this code
    }
    };

int main (int argc, char **argv)
{
    ros::init(argc, argv, "patriot_pkg_node");
    ros::NodeHandle nh;
    PatriotMain pm = PatriotMain(&nh);
    ros::spin();
}
