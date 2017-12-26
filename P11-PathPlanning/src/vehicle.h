#ifndef VEHICLE_H
#define VEHICLE_H

#include "waypoint.h"
#include <vector>

namespace sdcnd_t3p1
{

struct Vehicle
{
  int id;
  CartesianPose position; //!< position in map frame
  CartesianPose velocity; //!< velocity in map frame
  FrenetPoint frenet;
  double speed;
};

using VehicleList = std::vector<Vehicle>;

}

#endif // VEHICLE_H
