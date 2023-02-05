#ifndef TRACKERCONTROL_HPP
#define TRACKERCONTROL_HPP

#include <stdio.h>
#include "PiTracker.h"
#include <unistd.h>
#include <iostream>


struct _Quaternion{
  float x_pos_cm;
  float y_pos_cm;
  float z_pos_cm;
  float q_w;
  float q_x;
  float q_y;
  float q_z;
  int station_number;
};


class TrackerControl {
public:
  TrackerControl(void);
  ~TrackerControl(void);
  int GetPose(_Quaternion* Q);
  void SetHemisphere(int x, int y, int z);
  void SetUnits(int value);
  int GetStationNumber();
  int* GetHemisphere();
  // std::vector<int> get_hemisphere();
  // int nstations;


private:
  PiTracker* pTrak;
  const int BUFFER_SIZE = 1000;
  const int INCHES = 0;
  const int CM = 1;
  //#int nstations;
  // int x_hs, y_hs, z_hs;
  int len; //Maybe don't need...
  int StationStatus(void);
  int NumStations = 0;
  int ActiveStationMap = 0; //A bitmap of which stations are in use
  int Hemisphere[3];
};

#endif
