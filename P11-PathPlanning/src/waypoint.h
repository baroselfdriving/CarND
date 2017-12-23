#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <vector>

/// 2-D cartesian coordinate
struct CartesianPose
{
  double x;
  double y;
  double yawAngle;
};

using CartesianPoseList = std::vector<CartesianPose>;

/// Frenet coordinates
struct FrenetPose
{
  double s;  //!< Frenet 's' value
  double d;  //!< Frenet 'd' value
  double dx; //!< Frenet 'd' as unit normal vector split into x & y components
  double dy;
};

/// Waypoint description
struct Waypoint
{
  CartesianPose pose; //!< cartesian coordinates in global (map) frame
  FrenetPose frenet; //!< frenet coordinates for point p on the track
};

using WaypointList = std::vector<Waypoint>;

#endif // WAYPOINT_H
