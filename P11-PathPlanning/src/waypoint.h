#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <vector>

namespace sdcnd_t3p1
{

/// 2-D cartesian coordinate
struct CartesianPose
{
  double x;
  double y;
  double heading;
};

using CartesianPoseList = std::vector<CartesianPose>;

/// Frenet coordinates
struct FrenetPoint
{
  double s;  //!< Frenet 's' value
  double d;  //!< Frenet 'd' value
  double dx; //!< Frenet 'd' as unit normal vector split into x & y components
  double dy;
};

/// Waypoint description
struct Waypoint
{
  CartesianPose point; //!< cartesian coordinates in global (map) frame
  FrenetPoint frenet; //!< frenet coordinates for point p on the track
};

using WaypointList = std::vector<Waypoint>;

}

#endif // WAYPOINT_H
