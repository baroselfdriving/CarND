#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <vector>

/// 2-D cartesian coordinate
struct CartesianCoord
{
  double x;
  double y;
};

using CartesianCoordList = std::vector<CartesianCoord>;

/// Frenet coordinates
struct FrenetCoord
{
  double s;  //!< Frenet 's' value
  double d;  //!< Frenet 'd' value
  double dx; //!< Frenet 'd' as unit normal vector split into x & y components
  double dy;
};

/// Waypoint description
struct Waypoint
{
  CartesianCoord point; //!< cartesian coordinates in global (map) frame
  FrenetCoord frenet; //!< frenet coordinates for point p on the track
};

using WaypointList = std::vector<Waypoint>;

#endif // WAYPOINT_H
