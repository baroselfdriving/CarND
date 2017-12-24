#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "waypoint.h"
#include "vehicle.h"
#include "behaviour.h"

#include <cmath>
#include <array>
#include <vector>

class TrajectoryPlanner
{
public:
  static constexpr double LANE_WIDTH = 4;
  static constexpr double MAX_SPEED = 20; //!< 50 miles/hr = 22.35 meters/sec
  static constexpr double MAX_ACCELERATION = 10;
  static constexpr double MAX_JERK = 10;
  static constexpr double SAFE_FOLLOW_DISTANCE = 1; //!< how close can we get to a vehicle in front
  static constexpr double SAFE_MANOEUVRE_DISTANCE = 30; //!< how much space do we want to consider a maneouvre

  static constexpr double SIM_DELTA_TIME = 0.02; //!< sim time between each waypoint
  static constexpr unsigned int SIM_NUM_WAYPOINTS =
      static_cast<unsigned int>(1./SIM_DELTA_TIME); //!< number of waypoints to pass into sim each cycle

  struct State
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
    CartesianPose pose;
  };

  using StateList = std::vector<State>;

public:
  TrajectoryPlanner(const WaypointList& wps);
  ~TrajectoryPlanner() = default;

  /// Find lane number
  static int getLaneNumberFromFrenetD(double d);

  /// Get Frenet d coordinate given lane number
  static double getFrenetDFromLaneNumber(int lane);

  /// Find nearest vehicle ahead of me in the specified lane. If no vehicle, then return vehicles.end()
  static VehicleList::const_iterator findLeadVehicle(int lane, const Vehicle& me, const VehicleList& vehicles);

  /// Call every cycle to generate a list of waypoints for the car to follow
  CartesianPoseList getPlan(const Vehicle& me, const VehicleList& others,
                            const CartesianPoseList& myPrevPath, const FrenetPoint& prevPathEnd);

//private:
  struct PolynomialConstraint
  {
    double q;
    double qDot;
    double qDotDot;
    double t;
  };

  /// Solve for 5th order polynomial coefficients
  static std::array<double, 6> computePolynomialCoefficients(const PolynomialConstraint& initial,
                                                             const PolynomialConstraint& final);

  /// Generate first nPoints set of waypoints in frenet coordinates given final desired
  /// longitudinal speed, lateral position, and a time duration to achieve the final state in.
  void updateTrajectory(double longSpeed, double latPos, double timeDelta, size_t nPointsToAdd);


  /// Given frenet trajectory, update cartesian trajectory
  void smoothenTrajectory(StateList& history, CartesianPoseList& coords);

private:
  const WaypointList& trackWaypoints_;
  StateList history_;
};


#endif // TRAJECTORY_PLANNER_H
