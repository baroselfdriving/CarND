#ifndef HELPERS_H
#define HELPERS_H

#include "constants.h"
#include "vehicle.h"

#include <string>
#include <cmath>
#include <vector>

namespace sdcnd_t3p1
{

inline double milesPerHr2metersPerSec(double mph) { return mph * 1609.34/3600.; }

inline double deg2rad(double x) { return x * M_PI / 180.; }

inline double rad2deg(double x) { return x * 180. / M_PI; }

inline double distance(double x1, double y1, double x2, double y2) { return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)); }

inline double distance(const CartesianPose& p1, const CartesianPose& p2) { return distance(p1.x, p1.y, p2.x, p2.y); }

/// Find lane number
inline int getLaneNumberFromFrenetD(double d) { return static_cast<int>(std::floor(d/LANE_WIDTH)); }

/// Get Frenet d coordinate given lane number
inline double getFrenetDFromLaneNumber(int lane) { return lane * LANE_WIDTH + LANE_WIDTH/2.; }

/// Transform 'point' in global frame to coordinates of 'originFrame'
CartesianPose transformToLocal(const CartesianPose& point, const CartesianPose& originFrame);

/// Transform 'point' in 'originFrame' to coorinates of global frame
CartesianPose transformToGlobal(const CartesianPose& point, const CartesianPose& originFrame);

/// Checks if the SocketIO event has JSON data.
/// If there is data the JSON object in string format will be returned,
/// else the empty string "" will be returned.
std::string hasJsonData(std::string s);

WaypointList::const_iterator ClosestWaypoint(const CartesianPose& p, const WaypointList& wps);

WaypointList::const_iterator NextWaypoint(const CartesianPose& p, const WaypointList& wps);

/// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
FrenetPoint getFrenet(const CartesianPose& p, const WaypointList& wps);

/// Transform from Frenet s,d coordinates to Cartesian x,y
CartesianPose getCartesianFromFrenet(double s, double d, const WaypointList& wps);

/// Generate a new waypoint within a path of original waypoints
Waypoint generateWaypoint(double s, const WaypointList& wps);

/// Create new 'n' waypoints by spline fitting the set of input waypoints. Only good for points
/// that generally go in the same direction.
WaypointList generateLocalWaypoints(const WaypointList& newPoints, unsigned int n);

/// Populate with 'n' intermediate points between each pair of input waypoints using cubic spline fit.
WaypointList generateIntermediateWaypoins(const WaypointList& input, unsigned int n);

inline bool isFrenetInRange(double s, const WaypointList& wps) { return ((s > wps.front().frenet.s) && (s < wps.back().frenet.s)); }

struct NearestVehicles
{
  VehicleList::const_iterator ahead;
  VehicleList::const_iterator behind;
};

/// Get distance between two points given frenet path length
double getDistanceAlongTrack(double me, double other);

/// Find nearest vehicle ahead of me in the specified lane. If no vehicle, then return vehicles.end()
NearestVehicles findClosestVehiclesInLane(int lane, const Vehicle& me, const VehicleList& vehicles);

}

#endif // HELPERS_H
