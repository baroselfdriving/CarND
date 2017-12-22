#ifndef P11_HELPER_H
#define P11_HELPER_H

#include "waypoint.h"

#include <string>
#include <cmath>
#include <vector>

inline double milesPerHr2metersPerSec(double mph) { return mph * 1609.34/3600.; }

inline double deg2rad(double x) { return x * M_PI / 180.; }

inline double rad2deg(double x) { return x * 180. / M_PI; }

inline double distance(double x1, double y1, double x2, double y2) { return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)); }

inline double distance(const CartesianCoord& p1, const CartesianCoord& p2) { return distance(p1.x, p1.y, p2.x, p2.y); }

/// Checks if the SocketIO event has JSON data.
/// If there is data the JSON object in string format will be returned,
/// else the empty string "" will be returned.
std::string hasJsonData(std::string s);

WaypointList::const_iterator ClosestWaypoint(const CartesianCoord& p, const WaypointList& wps);

WaypointList::const_iterator NextWaypoint(const CartesianCoord& p, double theta, const WaypointList& wps);

/// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
FrenetCoord getFrenet(const CartesianCoord& p, double theta, const WaypointList& wps);

/// Transform from Frenet s,d coordinates to Cartesian x,y
CartesianCoord getXY(const FrenetCoord& fp, const WaypointList& wps);

#endif // P11_HELPER_H
