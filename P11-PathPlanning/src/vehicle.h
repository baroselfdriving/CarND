#ifndef VEHICLE_H
#define VEHICLE_H

#include "waypoint.h"
#include <vector>

struct Vehicle
{
  // Information common to all simulated vehicles
  int id;
  CartesianCoord position; //!< position in map frame
  CartesianCoord velocity; //!< velocity in map frame
  FrenetCoord frenet;

  // Information useful for the our car
  double yawAngle;
  double speed;
};

using VehicleList = std::vector<Vehicle>;

#endif // VEHICLE_H
