#include <ros/ros.h>
#include "patriot_pkg/TrackerControl.hpp"
#include "patriot_pkg/tracker_pose.h"


void get_pose_loop(TrackerControl* tracker, _Quaternion Quaternion){

}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "patriot_pkg_node");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  ros::Publisher arm_pose_pub = nh.advertise<patriot_pkg::tracker_pose>("patriot/arm_pose", 10);
  TrackerControl* tracker = new TrackerControl;
  // int* test = tracker->GetHemisphere();
  // std::cout << test[0] << std::endl;
  // std::cout << test[1] << std::endl;
  // std::cout << test[2] << std::endl;
  _Quaternion Quaternion[2];
  patriot_pkg::tracker_pose msg;
  int ret = -1;
  while(ros::ok()){
    ret = tracker->GetPose(Quaternion);
    std::cout << std::endl;
    // std::cout << "Return: " << ret << std::endl;
    if(ret == 0){
      // std::cout << "Do this " << ret << std::endl;
      for(int itr=0;itr<2;itr++){
        // std::cout << "Next " << std::endl;
        std::cout << std::endl;
        std::cout << "Station: " << Quaternion[itr].station_number << std::endl;
        std::cout << "X: " << Quaternion[itr].x_pos_cm << std::endl;
        std::cout << "Y: " << Quaternion[itr].y_pos_cm << std::endl;
        std::cout << "Z: " << Quaternion[itr].z_pos_cm << std::endl;

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
          std::cout << "Error: " << Quaternion[itr].station_number  << std::endl;
        }

      }// end of for loop
      // std::cout << "Pub " << std::endl;
      arm_pose_pub.publish(msg);
      // ros::spin();
      // std::cout << "Pub Done " << std::endl;
    }else{
      std::cout << "Failed Read " << std::endl;
    }
    rate.sleep();
  }

}
