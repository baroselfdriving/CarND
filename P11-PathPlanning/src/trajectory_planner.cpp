#include "trajectory_planner.h"
#include "p11_helper.h"

#include <limits>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

//---------------------------------------------------------------------------------------------------------------------
TrajectoryPlanner::TrajectoryPlanner()
  : egoVelocity_({0,0}),
    egoAcceleration_({0,0}),
    prevCarPosition_({0,0})
//---------------------------------------------------------------------------------------------------------------------
{}

//---------------------------------------------------------------------------------------------------------------------
int TrajectoryPlanner::getLaneNumber(double d)
//---------------------------------------------------------------------------------------------------------------------
{
  return static_cast<int>(std::floor(d/LANE_WIDTH));
}

//---------------------------------------------------------------------------------------------------------------------
VehicleList::const_iterator TrajectoryPlanner::findLeadVehicle(int lane, const Vehicle& me,
                                                               const VehicleList& vehicles, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  // Lead vehicle is the nearest one ahead of me in the specified lane

  double nearestDist = std::numeric_limits<double>::max();
  VehicleList::const_iterator nearestVehicle = vehicles.end();
  for(VehicleList::const_iterator other = vehicles.begin(); other != vehicles.end(); ++other)
  {
    const int otherLane = getLaneNumber(other->frenet.d);
    if(lane != otherLane)
    {
      continue;
    }

    // To find if the other vehicle is behind, compute X coordinate of the other vehicle in the
    // car frame and check if the X value is negative
    const double ca = cos(me.yawAngle);
    const double sa = sin(me.yawAngle);
    const double x = ca * (other->position.x - me.position.x) + sa * (other->position.y - me.position.y);
    if( x < 0 )
    {
      continue;
    }

    double dist = distance(me.position, other->position);
    if( dist < nearestDist )
    {
      nearestDist = dist;
      nearestVehicle = other;
    }
  }
  return nearestVehicle;
}

//---------------------------------------------------------------------------------------------------------------------
TrajectoryPlanner::PolyCoeffs TrajectoryPlanner::computePolynomialCoefficients(const FrenetState& initial, const FrenetState& final)
//---------------------------------------------------------------------------------------------------------------------
{
  // Solve for ceofficients of jerk minimising polynomial as described in term3 - trajectory generation lessons

  PolyCoeffs coeffs;
  coeffs.sa = {initial.s, initial.sd, initial.sdd/2.0, 0, 0, 0};
  coeffs.da = {initial.d, initial.dd, initial.ddd/2.0, 0, 0, 0};

  const double dt = final.t - initial.t;
  const double dt2 = dt*dt;
  const double dt3 = dt2*dt;
  const double dt4 = dt3*dt;
  const double dt5 = dt4*dt;

  Eigen::Matrix3d A;
  A << dt3, dt4, dt5,
       3*dt2, 4*dt3, 5*dt4,
       6*dt, 12*dt2, 20*dt3;

  Eigen::FullPivLU<Eigen::Matrix3d> lu(A);
  if( lu.isInvertible() )
  {
    Eigen::Matrix3d Ai = A.inverse();

    Eigen::Vector3d Bs;
    Bs << final.s - (initial.s + initial.sd * dt + 0.5 * initial.sdd * dt2),
          final.sd - (initial.sd + initial.sdd * dt),
          final.sdd - initial.sdd;

    Eigen::Vector3d Bd;
    Bd << final.d - (initial.d + initial.dd * dt + 0.5 * initial.ddd * dt2),
          final.dd - (initial.dd + initial.ddd * dt),
          final.ddd - initial.ddd;

    Eigen::Vector3d X = Ai * Bs;
    coeffs.sa[3] = X[0];
    coeffs.sa[4] = X[1];
    coeffs.sa[5] = X[2];

    X = Ai * Bd;
    coeffs.da[3] = X[0];
    coeffs.da[4] = X[1];
    coeffs.da[5] = X[2];
  }
  else
  {
    std::cerr << "WARNING: Trajectory not solvable for given constraints" << std::endl;
  }
  return coeffs;
}

//---------------------------------------------------------------------------------------------------------------------
void TrajectoryPlanner::computeMotionHistory(const Vehicle& car)
//---------------------------------------------------------------------------------------------------------------------
{
  // Find average X component of velocity and acceleration in car frame
  const double ca = cos(car.yawAngle);
  const double sa = sin(car.yawAngle);
  //lateralSpeed_ = ca * (car.->position.x - me.position.x) + sa * (other->position.y - me.position.y);

  /// \todo
}

//---------------------------------------------------------------------------------------------------------------------
CartesianCoordList TrajectoryPlanner::getPlan(const Vehicle& me, const VehicleList& others,
                                              const CartesianCoordList& myPrevPath, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  // keep a history of how the car is moving
  computeMotionHistory(me);

  size_t myPrevPathSz = myPrevPath.size();
  CartesianCoordList path = myPrevPath;

  // set initial boundary conditions
  FrenetState initial;
  initial.s = me.frenet.s;
  initial.sd = me.speed;
  initial.sdd = egoAcceleration_.x;
  initial.d = me.frenet.d;
  initial.dd = egoVelocity_.y;
  initial.ddd = egoAcceleration_.y;
  initial.t = 0;

  // find nearest vehicle and set final boundary conditions
  int targetLane = getLaneNumber(me.frenet.d);
  double targetDist = SAFE_MANOEUVRE_DISTANCE;
  double targetSpeed = MAX_SPEED;
  const double minTime = MAX_SPEED/MAX_ACCELERATION;
  double targetTime = minTime;
  auto vehicleIt = findLeadVehicle(targetLane, me, others, wps);
  if( vehicleIt != others.end() )
  {
    targetDist = vehicleIt->frenet.s - SAFE_FOLLOW_DISTANCE;
    if(targetDist < SAFE_MANOEUVRE_DISTANCE)
    {
      targetSpeed = vehicleIt->speed; // match speed of vehicle in front
      targetTime = std::max(minTime, fabs(targetSpeed - me.speed)/MAX_ACCELERATION);
    }
  }
  FrenetState final;
  final.s = targetDist;
  final.sd = targetSpeed;
  final.sdd = 0;
  final.d = targetLane * LANE_WIDTH + LANE_WIDTH/2.;
  final.dd = 0;
  final.ddd = 0;
  final.t = targetTime;


  // solve for coefficients and generate waypoints
  PolyCoeffs coeffs = computePolynomialCoefficients(initial, final);

  // add waypoints to path
  const int nPoints = SIM_NUM_WAYPOINTS - myPrevPathSz;
  double t = 0;
  for(int i = 0; i < nPoints; ++i)
  {
    const double t2 = t*t;
    const double t3 = t2*t;
    const double t4 = t3*t;
    const double t5 = t4*t;
    FrenetCoord fp;
    fp.s = coeffs.sa[0] + coeffs.sa[1]*t + coeffs.sa[2]*t2 + coeffs.sa[3]*t3 + coeffs.sa[4]*t4 + coeffs.sa[5]*t5;
    fp.d = coeffs.da[0] + coeffs.da[1]*t + coeffs.da[2]*t2 + coeffs.da[3]*t3 + coeffs.da[4]*t4 + coeffs.da[5]*t5;
    t += SIM_DELTA_TIME;

    CartesianCoord wp = getXY(fp, wps);
    path.push_back(wp);
  }

  return path;
}

