#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "waypoint.h"
#include "vehicle.h"
#include "behaviour.h"

#include <cmath>

class TrajectoryPlanner
{
public:
  static constexpr double LANE_WIDTH = 4;
  static constexpr double MAX_SPEED = 22.35; //!< 50 miles/hr in meters/sec
  static constexpr double MAX_ACCELERATION = 10;
  static constexpr double MAX_JERK = 10;

public:
  TrajectoryPlanner() = default;
  ~TrajectoryPlanner() = default;
  static int getLaneNumber(double d);
  CartesianCoordList getPlan(const Vehicle& car, const CartesianCoordList& previousPath, const WaypointList& wps);
};


#endif // TRAJECTORY_PLANNER_H
