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
  int currentLane = TrajectoryPlanner::getLaneNumber(car.frenet.d);

  CartesianCoordList path;

  // if previous path is almost empty, use the car as the starting reference
  size_t previousPathSz = previousPath.size();
  if(previousPathSz < 2)
  {
    // use two points that make the path tangent to car
    CartesianCoord prevPosition;
    prevPosition.x = car.position.x - cos(car.yawAngle);
    prevPosition.y = car.position.y - sin(car.yawAngle);

    path.push_back(prevPosition);
    path.push_back(car.position);
  }

  // use previous path's end point as starting reference
  else
  {
    CartesianCoord ref = previousPath[previousPathSz-1];
    CartesianCoord prevPath = previousPath[previousPathSz-2];
    double yawAngle = atan2(ref.y - prevPath.y, ref.x - prevPath.x);

    path.push_back(prevPath);
    path.push_back(ref);
  }

  FrenetCoord fp;
  fp.d = TrajectoryPlanner::LANE_WIDTH * (0.5 + currentLane);

  fp.s = car.frenet.s + .41;
  CartesianCoord wp0 = getXY(fp, wps);

  fp.s = car.frenet.s + .82;
  CartesianCoord wp1 = getXY(fp, wps);

  fp.s = car.frenet.s + 1.4;
  CartesianCoord wp2 = getXY(fp, wps);

  path.push_back(wp0);
  path.push_back(wp1);
  path.push_back(wp2);

  return path;
}
