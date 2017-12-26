#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "constants.h"
#include "waypoint.h"
#include "vehicle_model.h"
#include "behaviour.h"

#include <cmath>
#include <array>
#include <vector>

namespace sdcnd_t3p1
{

class TrajectoryPlanner
{
public:
  static constexpr double MAX_SPEED = .9 * SPEED_LIMIT;
  static constexpr double ACCELERATION_LIMIT = 10;
  static constexpr double JERK_LIMIT = 10;
  static constexpr double SAFE_MANOEUVRE_DISTANCE = 30; //!< how much space do we want to consider a maneouvre

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
    double time;
    CartesianPose pose;
  };

  using StateList = std::vector<State>;

public:
  TrajectoryPlanner(const WaypointList& wps);
  ~TrajectoryPlanner() = default;

  void reset(const CartesianPose& pose) { model_.reset(pose); history_.clear(); }

  /// Find nearest vehicle ahead of me in the specified lane. If no vehicle, then return vehicles.end()
  static VehicleList::const_iterator findLeadVehicle(int lane, const CartesianPose& me, const VehicleList& vehicles);

  /// Call every cycle to generate a list of waypoints for the car to follow
  CartesianPoseList getPlan(const Vehicle& me, const VehicleList& others,
                            const CartesianPoseList& myPrevPath, const FrenetPoint& prevPathEnd);

private:
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
  VehicleModel model_;
};

}

#endif // TRAJECTORY_PLANNER_H
