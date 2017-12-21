#include "trajectory_planner.h"
#include "p11_helper.h"

#include <limits>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

//---------------------------------------------------------------------------------------------------------------------
TrajectoryPlanner::TrajectoryPlanner()
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
std::array<double, 6> TrajectoryPlanner::computePolynomialCoefficients(const PolyState& initial, const PolyState& final)
//---------------------------------------------------------------------------------------------------------------------
{
  // Solve for ceofficients of jerk minimising polynomial as described in term3 - trajectory generation lessons

  std::array<double, 6> coeffs = {initial.q, initial.qDot, initial.qDotDot/2.0, 0, 0, 0};

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
    Bs << final.q - (initial.q + initial.qDot * dt + 0.5 * initial.qDotDot * dt2),
          final.qDot - (initial.qDot + initial.qDotDot * dt),
          final.qDotDot - initial.qDotDot;

    Eigen::Vector3d X = Ai * Bs;
    coeffs[3] = X[0];
    coeffs[4] = X[1];
    coeffs[5] = X[2];
  }
  else
  {
    std::cerr << "WARNING: Trajectory not solvable for given constraints" << std::endl;
  }
  return coeffs;
}

//---------------------------------------------------------------------------------------------------------------------
CartesianCoordList TrajectoryPlanner::getPlan(const Vehicle& me, const VehicleList& others,
                                              const CartesianCoordList& myPrevPath, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  const size_t myPrevPathSz = myPrevPath.size();
  const size_t nPointsToAdd = SIM_NUM_WAYPOINTS - myPrevPathSz;
  CartesianCoordList path;//= myPrevPath;

  // set initial boundary conditions
  FrenetState initial;
  if(myPrevPathSz == 0)
  {
    // set to car coordinates at start
    history_.clear();
    initial.s = me.frenet.s;
    initial.sv = me.speed;
    initial.sa = 0;
    initial.sj = 0;
    initial.d = me.frenet.d;
    initial.dv = 0;
    initial.da = 0;
    initial.dj = 0;
    initial.t = 0;
  }
  else
  {
    // initial condition is the end of the previous path
    history_.erase(history_.begin(), history_.begin()+nPointsToAdd);
    initial = history_.back();
  }

  int targetLane = getLaneNumber(initial.d); /// \todo replace with correct lane number

  // find nearest vehicle and set final boundary conditions
  double targetDist = SAFE_MANOEUVRE_DISTANCE;
  double targetSpeed = MAX_SPEED;
  double targetTime = 2*targetDist/targetSpeed;/*
  auto vehicleIt = findLeadVehicle(targetLane, me, others, wps); ///\todo use 'me'??
  if( vehicleIt != others.end() )
  {
    targetDist = vehicleIt->frenet.s - SAFE_FOLLOW_DISTANCE;
    if(targetDist < SAFE_MANOEUVRE_DISTANCE)
    {
      targetSpeed = vehicleIt->speed; // match speed of vehicle in front
      targetTime = std::max(minTime, fabs(targetSpeed - initial.sd)/MAX_ACCELERATION);
    }
  }*/
  FrenetState final;
  final.sv = targetSpeed;
  final.sa = 0;
  final.sj = 0;
  final.d = targetLane * LANE_WIDTH + LANE_WIDTH/2.;
  final.dv = 0;
  final.da = 0;
  final.dj = 0;
  final.t = initial.t + targetTime;

/*
  // solve for coefficients and generate waypoints
  std::array<double, 6> coeffs = computePolynomialCoefficients(initial, final);
  double dt = 0;
  for(int i = 0; i < nPointsToAdd; ++i)
  {
    const double dt2 = dt*dt;
    const double dt3 = dt2*dt;
    const double dt4 = dt3*dt;
    const double dt5 = dt4*dt;

    State fs;
    fs.qDot = coeffs.sa[0] + coeffs.sa[1]*dt + coeffs.sa[2]*dt2 + coeffs.sa[3]*dt3 + coeffs.sa[4]*dt4 + coeffs.sa[5]*dt5;
    fs.qDotDot = coeffs.sa[1] + 2*coeffs.sa[2]*dt + 3*coeffs.sa[3]*dt2 + 4*coeffs.sa[4]*dt3 + 5*coeffs.sa[5]*dt4;
    fs.lonJerk = 2*coeffs.sa[2] + 6*coeffs.sa[3]*dt + 12*coeffs.sa[4]*dt2 + 20*coeffs.sa[5]*dt3;
    fs.latVel = coeffs.da[0] + coeffs.da[1]*dt + coeffs.da[2]*dt2 + coeffs.da[3]*dt3 + coeffs.da[4]*dt4 + coeffs.da[5]*dt5;
    fs.latAcc = coeffs.da[1] + 2*coeffs.da[2]*dt + 3*coeffs.da[3]*dt2 + 4*coeffs.da[4]*dt3 + 5*coeffs.da[5]*dt4;
    fs.latJerk = 2*coeffs.da[2] + 6*coeffs.da[3]*dt + 12*coeffs.da[4]*dt2 + 20*coeffs.da[5]*dt3;

    dt += SIM_DELTA_TIME;

    history_.push_back(fs);
  }

  // add waypoints to path
  for(const auto& h : history_)
  {
    FrenetCoord fp;
    fp.s = h.qDot;
    fp.d = h.latVel;
    CartesianCoord wp = getXY(fp, wps);
    path.push_back(wp);
  }

  std::cout << me.frenet.s << " " << nPointsToAdd << " " << history_.back().qDot <<  std::endl;*/
  return path;
}

