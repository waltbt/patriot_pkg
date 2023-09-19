/*
This file is for testing the tracker setup without ROS.  It does require that all the setup files be installed.
Created: 9/18/2023 
*/

#include "TrackerControl.hpp"


int main (int argc, char **argv)
{
TrackerControl* tracker = new TrackerControl;
  int num_stations = tracker->GetStationNumber();
  if (num_stations <= 0 || num_stations > 2){ //Patriot can only have 1 or 2 stations
    std::cout << "Error setting up Patriot" << std::endl;
  }
  std::cout<<"Num Stations: " << num_stations<<std::endl;
  _Quaternion Quaternion[2];
 
  int ret = -1;
  tracker->GetPose(Quaternion);
  
  
    while(1){
    ret = tracker->GetPose(Quaternion);
    if(ret == 0){
      for(int itr=0;itr<num_stations;itr++){
        // Uncomment for troubleshooting
         std::cout << std::endl;
         std::cout << "Station: " << Quaternion[itr].station_number << std::endl;
         std::cout << "X: " << Quaternion[itr].x_pos_cm << std::endl;
         std::cout << "Y: " << Quaternion[itr].y_pos_cm << std::endl;
         std::cout << "Z: " << Quaternion[itr].z_pos_cm << std::endl;

        
     

      }// end of for loop
    }else{
      std::cout << "Failed Read " << std::endl;
    }
  }  
  }
    
