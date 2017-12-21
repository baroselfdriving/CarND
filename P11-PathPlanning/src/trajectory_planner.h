#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "waypoint.h"
#include "vehicle.h"
#include "behaviour.h"

#include <cmath>
#include <array>
#include <deque>

class TrajectoryPlanner
{
public:
  static constexpr double LANE_WIDTH = 4;
  static constexpr double MAX_SPEED = 22.35; //!< 50 miles/hr in meters/sec
  static constexpr double MAX_ACCELERATION = 10;
  static constexpr double MAX_JERK = 10;
  static constexpr double SAFE_FOLLOW_DISTANCE = 1; //!< how close can we get to a vehicle in front
  static constexpr double SAFE_MANOEUVRE_DISTANCE = 100; //!< how much space do we want to consider a maneouvre

  static constexpr double SIM_DELTA_TIME = 0.02; //!< sim time between each waypoint
  static constexpr unsigned int SIM_NUM_WAYPOINTS =
      static_cast<unsigned int>(1./SIM_DELTA_TIME); //!< number of waypoints to pass into sim each cycle

public:
  TrajectoryPlanner();
  ~TrajectoryPlanner() = default;

  /// Find lane number
  static int getLaneNumber(double d);

  /// Find nearest vehicle ahead of me in the specified lane. If no vehicle, then return vehicles.end()
  static VehicleList::const_iterator findLeadVehicle(int lane, const Vehicle& me,
                                                     const VehicleList& vehicles, const WaypointList& wps);

  /// Call every cycle to generate a list of waypoints for the car to follow
  CartesianCoordList getPlan(const Vehicle& me, const VehicleList& others,
                             const CartesianCoordList& myPrevPath, const WaypointList& wps);

//private:
  struct PolyState
  {
    double q;
    double qDot;
    double qDotDot;
    double t;
  };

  struct FrenetState
  {
    double s;
    double sv;
    double sa;
    double sj;
    double d;
    double dv;
    double da;
    double dj;
    double t;
  };

  /// Solve for 5th order polynomial coefficients
  static std::array<double, 6> computePolynomialCoefficients(const PolyState& initial, const PolyState& final);

private:
  std::deque<FrenetState> history_;
};


#endif // TRAJECTORY_PLANNER_H
