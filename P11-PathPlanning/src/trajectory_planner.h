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
  static constexpr double MIN_RESPONSE_TIME = 3; //!< min time to respond to trajectory change

  struct State
  {
    double s;   //!< on-track distance
    double sv;  //!< on-track speed
    double sa;  //!< on-track acceleration
    double sj;  //!< on-track jerk
    double d;   //!< x-track distance
    double dv;  //!< x-track speed
    double da;  //!< x-track acceleration
    double dj;  //!< x-track jerk
    double time;  //!< timestamp
    CartesianPose pose; //!< world coordinates
  };

  using StateList = std::vector<State>;

public:
  TrajectoryPlanner(const WaypointList& wps);

  ~TrajectoryPlanner() = default;

  void reset(const CartesianPose& pose) { model_.reset(pose); history_.clear(); }

  /// Call every cycle to generate a list of waypoints for the car to follow
  CartesianPoseList computePlan(int targetLane, const Vehicle& me, const VehicleList& others,
                                const CartesianPoseList& myPrevPath);

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

  /// Given forward speed, lateral position along the path, and a time slice, generate nPointsToAdd
  /// set of waypoints
  void updateTrajectory(double longSpeed, double latPos, double timeDelta, size_t nPointsToAdd,
                        CartesianPoseList& coords);

private:
  const WaypointList& trackWaypoints_;
  StateList history_;
  VehicleModel model_;
};

}

#endif // TRAJECTORY_PLANNER_H
