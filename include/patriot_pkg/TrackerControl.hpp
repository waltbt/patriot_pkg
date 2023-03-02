// TrackerControl.hpp
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

// For converting raw data to float values
union converter{
  unsigned int bin_val;
  float float_val;
};


class TrackerControl {
public:
  TrackerControl(void);
  ~TrackerControl(void);
  int GetPose(_Quaternion* Q);
  void SetHemisphere(int x, int y, int z);
  void SetUnits(int value);
  int GetStationNumber(); // Returns value in NumStations
  int* GetHemisphere();



private:
  PiTracker* pTrak;
  const int BUFFER_SIZE = 1000; // Communication buffer size
  const int INCHES = 0; // For setting units
  const int CM = 1; // For setting units
  int GetStationStatus(void); // Gets number of stations from device
  int NumStations = 0; // Number of stations connected
  int ActiveStationMap = 0; // A bitmap of which stations are in use
  int Hemisphere[3]; // Hemisphere where tracker is working
};

#endif
