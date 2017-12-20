#include "trajectory_planner.h"
#include "p11_helper.h"

//---------------------------------------------------------------------------------------------------------------------
int TrajectoryPlanner::getLaneNumber(double d)
//---------------------------------------------------------------------------------------------------------------------
{
  return static_cast<int>(std::floor(d/LANE_WIDTH));
}

//---------------------------------------------------------------------------------------------------------------------
CartesianCoordList TrajectoryPlanner::getPlan(const Vehicle& car, const CartesianCoordList& previousPath, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  CartesianCoordList path = previousPath;
  return path;
}
