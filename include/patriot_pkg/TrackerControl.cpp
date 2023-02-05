/*
patriot_pkg version 0.1 -- ROS Package for the Patriot Polhemus Trackers.
Copyright  ©  2023  Benjamin Thomas walt

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

/*
TrackerControl.cpp
Function: This program interfaces with PiTracker.cpp which is provided by Polhemus
to use the Patriot HS tracker.  It is a simple interface that only implements some
of the features of the tracker and only on the Patriot HS.  It should be simple
though to modify it to work with other trackers and other features.
Author: Benjamin Walt
Date: 1/19/2023
Version: 0.1
*/

#include "TrackerControl.hpp"

TrackerControl::TrackerControl(){
  /*Connect to device*/
  pTrak = new PiTracker;
  if (!pTrak){
    printf("Memory Allocation Error creating tracker communications module\n");
  }


  int ret = pTrak->UsbConnect(0x0f44,0xef20,0x04,0x88);
  if(ret != 0){
    std::cout << "Tracker USB Connection Error. Return Value: " << ret << std::endl;
  }else{
    std::cout << "Tracker USB Connection Made" << std::endl;
  }

  /*Flush out junk in buffers*/
  BYTE buf[BUFFER_SIZE];
  do {
    pTrak->WriteTrkData((void*)"\r",1);  // send just a cr, should return an short "Invalid Command" response
    usleep(100000);
    len=pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response
  } while (!len);

  /*Set to Binary*/
  pTrak->WriteTrkData((void*)"F1\r",3); //Binary
  usleep(100000);

  /*Get number of stations*/
  NumStations = StationStatus();
  if(NumStations<0){
    // Set up failed, so try again.  This often fixes it.
    NumStations = StationStatus();
  }

  std::cout << "Number of Stations Found: " << NumStations <<std::endl;

  // std::cout << ActiveStationMap << std::endl;
  if(ActiveStationMap != 0){
    for(int itr=0;itr<16;itr++){
      if((ActiveStationMap>>itr) & 1){
        std::cout << "Station " << itr << " is active." << std::endl;
      }
    }
  }else{
    std::cout << "Error: No Actve Stations Found!" << std::endl;
  }



  /*Set output format*/
  // len=0;
  // BYTE buf[BUFFER_SIZE];
  /*
  * - Applies to all stations
  2 - X, Y, Z Cartesian coordinates of position
  7 - Orientation Quaternion
  */
  pTrak->WriteTrkData((void*)"O*,2,7\r",7);
  // pTrak->WriteTrkData((void*)"O*,3,7\r",7);
  usleep(100000);
  // len=pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response
  // printf("Message: \n");
  // printf("\nData Length: %d \n", len);
  // for(int itr=0;itr<len;itr++){
  //   printf(" %c,", buf[itr] );
  // }

  /*Set Units*/
  SetUnits(CM); //centimeters

  /*Set hemisphere*/
  SetHemisphere(0,0,-1); //(0,0,-1) is -Z Direction
}


TrackerControl::~TrackerControl(){
  delete pTrak;
}


/* Summary: Gets the number of stations in use
* Parameters: None
* Returns: int - number of stations
* Notes: This command returns more details, but is only being used for the number of stations.
* Also updates the active station map, but maybe this is not super useful.
*/
int TrackerControl::StationStatus(){
  len=0;
  BYTE buf[BUFFER_SIZE];
  char temp = 'u' & 0x1f;
  static char cmd[] = { temp, '0', '\r', '\0' };
  pTrak->WriteTrkData((void*)cmd,3);
  usleep(100000);
  len=pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response

  if(len<1){
    return -2; // Failed read
  }
  if(buf[4] != 32){
    return -1; // Error
  }
  // for(int itr=0;itr<len;itr++){
  //   printf(" %d,", buf[itr] );
  // }
  //Count the number of stations in bitmap
  int station_sum = 0;
  for(int itr=0;itr<8;itr++){
    // std::cout << "Iter: " << itr << " Total: " << station_sum << std::endl;
    // std::cout << ((buf[8]>>itr)&1) << std::endl;
    // std::cout << "Total: " << station_sum << std::endl;
    station_sum = station_sum + ((buf[7]>>itr)&1);
    station_sum = station_sum + ((buf[8]>>itr)&1);
  }
  ActiveStationMap = (buf[7]<<8) + buf[8];
  // std::cout << ActiveStationMap << std::endl;
  return station_sum;
}

/* Summary: Sets the hemisphere of operation for the tracker
* Parameters: int x, int y, int z - see notes for explaination
* Must be only one value that is -1 or 1 and the rest 0
* Returns: Nothing
* Notes:
* sets the zenith of the hemisphere in direction of vector (x, y, z)
* Forward Hemisphere (+X) 1,0,0
* Back Hemisphere (-X) -1,0,0
* Right Hemisphere (+Y) 0,1,0
* Left Hemisphere (-Y) 0,-1,0
* Lower Hemisphere (+Z) 0,0,1
* Upper Hemisphere (-Z) 0,0,-1 - Default
*/
void TrackerControl::SetHemisphere(int x, int y, int z){
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "h*,%d,%d,%d\r", x, y, z);
  // Length of command varies with hemisphere so find its length
  int length_counter;
  for(length_counter=0;length_counter<32;length_counter++){
    if(cmd[length_counter]==13){ //13 is the ascii for "\r"
      length_counter++; //Add one more for length in bytes
      break;
    }
  }
  pTrak->WriteTrkData((void*)cmd,length_counter);
  usleep(100000);
  Hemisphere[0] = x;
  Hemisphere[1] = y;
  Hemisphere[2] = z;

}

/* Summary: Sets the output units between inches and cm
* Parameters: int value - either 0 (inches) or 1 (cm)
* Returns: Nothing
* Notes: inches are default
*/
void TrackerControl::SetUnits(int value){
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "U%d\r", value);
  pTrak->WriteTrkData((void*)cmd,3); // cm
  usleep(100000);
}

/* Summary: Retrieves the pose information in Quaternion format.
* Parameters: An array of quaternions
* Returns: 0 for success, -1 for failure
* Notes:
*/
int TrackerControl::GetPose(_Quaternion* Q){
  len=0;
  BYTE buf[BUFFER_SIZE];
  pTrak->WriteTrkData((void*)"p",1);

  len=pTrak->ReadTrkData(buf,BUFFER_SIZE);  // read tracker data
  // printf("Data Length: %d\n", len);
  // printf("Message: ");
  // for(int itr=0;itr<len;itr++){
  //   printf(" %d,", buf[itr] );
  // }

  // printf("\n");
  // int length = len(data_set);
  // char buffer[15];
  // int n = 0;
  // printf("Device: %c %c\n", buf[0], buf[1]);
  // printf("Station: %d\n", buf[2]);
  // printf("Command: %c\n", buf[3]);
  // printf("Error: %d\n", buf[4]);
  // printf("Size: %d", buf[7]<<8|buf[6]);
  // printf("\n");
  if(buf[4] != 32){
    std::cout << "Error Reading Pose.  No update." << std::endl;
    return -1;
  }else{
    for(int itr=0;itr<NumStations;itr++){
      int offset = itr*36; //For this set up, 36 is the offset between stations.
      int cur_station = buf[2 + offset] - 1;

      converter C; //Union converts between output binary and the desired float
      Q[cur_station].station_number = cur_station;
      C.bin_val = (buf[11 + offset]<<24) | (buf[10 + offset]<<16)| (buf[9 + offset]<<8) | buf[8 + offset];
      Q[cur_station].x_pos_cm = C.float_val;
      C.bin_val = (buf[15 + offset]<<24) | (buf[14 + offset]<<16)| (buf[13 + offset]<<8) | buf[12 + offset];
      Q[cur_station].y_pos_cm = C.float_val;
      C.bin_val = (buf[19 + offset]<<24) | (buf[18 + offset]<<16)| (buf[17 + offset]<<8) | buf[16 + offset];
      Q[cur_station].z_pos_cm = C.float_val;
      C.bin_val = (buf[23 + offset]<<24) | (buf[22 + offset]<<16)| (buf[21 + offset]<<8) | buf[20 + offset];
      Q[cur_station].q_w = C.float_val;
      C.bin_val = (buf[27 + offset]<<24) | (buf[26 + offset]<<16)| (buf[25 + offset]<<8) | buf[24 + offset];
      Q[cur_station].q_x = C.float_val;
      C.bin_val = (buf[31 + offset]<<24) | (buf[30 + offset]<<16)| (buf[29 + offset]<<8) | buf[28 + offset];
      Q[cur_station].q_y = C.float_val;
      C.bin_val = (buf[35 + offset]<<24) | (buf[34 + offset]<<16)| (buf[33 + offset]<<8) | buf[32 + offset];
      Q[cur_station].q_z = C.float_val;
    }
    return 0;
  }
}

/* Summary: Gets the number of stations in use
* Parameters: None
* Returns: Number of a stations
* Notes:
*/
int TrackerControl::GetStationNumber(){
  return NumStations;
}

/* Summary: Gets the current hemisphere in use
* Parameters: None
* Returns: hemisphere value as a pointer to an array
* Notes:
*/
int* TrackerControl::GetHemisphere(){

  return Hemisphere;
}
