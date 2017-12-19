#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <vector>

struct Waypoint
{
  double x; //!< [x,y]: cartesian coordinates in global (map) frame
  double y;
  double s; //!< Frenet 's' value
  double dx; //!< Frenet 'd' as unit normal vector split into x & y components
  double dy;
};

using WaypointList = std::vector<Waypoint>;

#endif // WAYPOINT_H
