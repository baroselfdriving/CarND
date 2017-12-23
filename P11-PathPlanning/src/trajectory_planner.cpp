#include "trajectory_planner.h"
#include "p11_helper.h"
#include "integrator.h"

#include <limits>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

//---------------------------------------------------------------------------------------------------------------------
TrajectoryPlanner::TrajectoryPlanner(const WaypointList& wps)
  : trackWaypoints_(wps)
//---------------------------------------------------------------------------------------------------------------------
{
}

//---------------------------------------------------------------------------------------------------------------------
int TrajectoryPlanner::getLaneNumberFromFrenetD(double d)
//---------------------------------------------------------------------------------------------------------------------
{
  return static_cast<int>(std::floor(d/LANE_WIDTH));
}

//---------------------------------------------------------------------------------------------------------------------
double TrajectoryPlanner::getFrenetDFromLaneNumber(int lane)
//---------------------------------------------------------------------------------------------------------------------
{
  return lane * LANE_WIDTH + LANE_WIDTH/2.;
}

//---------------------------------------------------------------------------------------------------------------------
VehicleList::const_iterator TrajectoryPlanner::findLeadVehicle(int lane, const Vehicle& me, const VehicleList& vehicles)
//---------------------------------------------------------------------------------------------------------------------
{
  // Lead vehicle is the nearest one ahead of me in the specified lane

  double nearestDist = std::numeric_limits<double>::max();
  VehicleList::const_iterator nearestVehicle = vehicles.end();
  for(VehicleList::const_iterator other = vehicles.begin(); other != vehicles.end(); ++other)
  {
    const int otherLane = getLaneNumberFromFrenetD(other->frenet.d);
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
std::array<double, 6> TrajectoryPlanner::computePolynomialCoefficients(const PolynomialConstraint& initial, const PolynomialConstraint& final)
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
void TrajectoryPlanner::computeTrajectory(double longSpeed, double latPos, double timeDelta, size_t nPointsToAdd,
                                          StateList& fwps)
//---------------------------------------------------------------------------------------------------------------------
{
  Integrator integrator(SIM_DELTA_TIME);

  double s0 = 0;
  double t0 = 0;
  PolynomialConstraint sInitial = {0};
  PolynomialConstraint dInitial = {0};
  PolynomialConstraint sFinal;
  PolynomialConstraint dFinal;

  if(fwps.size() > 0)
  {
    const auto& it = fwps.back();
    s0 = it.s;
    t0 = it.t;
    sInitial.q = it.sv;
    sInitial.qDot = it.sa;
    sInitial.qDotDot = it.sj;
    sInitial.t = it.t;

    dInitial.q = it.d;
    dInitial.qDot = it.dv;
    dInitial.qDotDot = it.da;
    dInitial.t = it.t;
  }

  sFinal.q = longSpeed;
  sFinal.qDot = 0;
  sFinal.qDotDot = 0;
  sFinal.t = sInitial.t + timeDelta;

  dFinal.q = latPos;
  dFinal.qDot = 0;
  dFinal.qDotDot = 0;
  dFinal.t = dInitial.t + timeDelta;

  const std::array<double, 6> sCoeffs = computePolynomialCoefficients(sInitial, sFinal);
  const std::array<double, 6> dCoeffs = computePolynomialCoefficients(dInitial, dFinal);


  double dt = 0;
  for(size_t i = 0; i < nPointsToAdd; ++i)
  {
    const double dt2 = dt*dt;
    const double dt3 = dt2*dt;
    const double dt4 = dt3*dt;
    const double dt5 = dt4*dt;

    State fs;
    fs.sv = sCoeffs[0] + sCoeffs[1]*dt + sCoeffs[2]*dt2 + sCoeffs[3]*dt3 + sCoeffs[4]*dt4 + sCoeffs[5]*dt5;
    fs.sa = sCoeffs[1] + 2*sCoeffs[2]*dt + 3*sCoeffs[3]*dt2 + 4*sCoeffs[4]*dt3 + 5*sCoeffs[5]*dt4;
    fs.sj = 2*sCoeffs[2] + 6*sCoeffs[3]*dt + 12*sCoeffs[4]*dt2 + 20*sCoeffs[5]*dt3;

    fs.d = dCoeffs[0] + dCoeffs[1]*dt + dCoeffs[2]*dt2 + dCoeffs[3]*dt3 + dCoeffs[4]*dt4 + dCoeffs[5]*dt5;
    fs.dv = dCoeffs[1] + 2*dCoeffs[2]*dt + 3*dCoeffs[3]*dt2 + 4*dCoeffs[4]*dt3 + 5*dCoeffs[5]*dt4;
    fs.da = 2*dCoeffs[2] + 6*dCoeffs[3]*dt + 12*dCoeffs[4]*dt2 + 20*dCoeffs[5]*dt3;
    fs.dj = 6*dCoeffs[3] + 24*dCoeffs[4]*dt + 60*dCoeffs[5]*dt2;

    fs.t = t0 + dt;
    dt += SIM_DELTA_TIME;

    if(i == 0)
    {
      fs.s = s0;
      integrator.reset(fs.s, fs.sv);
    }
    else
    {
      fs.s = integrator.integrate(fs.sv);
    }

    FrenetPose fp;
    fp.s = fs.s;
    fp.d = fs.d;
    fs.pose = getXY(fp, trackWaypoints_);

    fwps.push_back(fs);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void TrajectoryPlanner::smoothenTrajectory(size_t index, StateList& history, CartesianPoseList& coords)
//---------------------------------------------------------------------------------------------------------------------
{
  /// \todo
  /// - Add nearest waypoints to make up the path
  /// - Smooth the path with spline
  // get cartesian coordinates of waypoints and add to path
  for(StateList::iterator it = (history.begin()+index); it < history.end(); ++it)
  {
    FrenetPose fp;
    fp.s = it->s;
    fp.d = it->d;
    coords.push_back(it->pose);
/*
    std::cout << h.t << ", " << h.s << ", " << h.sv << ", " << h.sa << ", " << h.sj
              << ", " << h.d << ", " << h.dv << ", " << h.da << ", " << h.dj
              << ", " << wp.x << ", " << wp.y << std::endl;*/
  }
  //std::cout << history_.begin()->t << " " << history_.begin()->s << " " << (history_.end()-1)->t << " " << (history_.end()-1)->s << std::endl;

}

//---------------------------------------------------------------------------------------------------------------------
CartesianPoseList TrajectoryPlanner::getPlan(const Vehicle& me, const VehicleList& others,
                                             const CartesianPoseList& myPrevPath, const FrenetPose& prevPathEnd)
//---------------------------------------------------------------------------------------------------------------------
{
  const size_t myPrevPathSz = myPrevPath.size();
  size_t nPointsToAdd = SIM_NUM_WAYPOINTS - myPrevPathSz;
  CartesianPoseList path = myPrevPath;

  // set initial boundary conditions
  if(myPrevPathSz == 0)
  {
    history_.clear();

    // set to car coordinates at start
    State initial;
    initial.s = me.frenet.s;
    initial.sv = me.speed;
    initial.sa = 0;
    initial.sj = 0;
    initial.d = me.frenet.d;
    initial.dv = 0;
    initial.da = 0;
    initial.dj = 0;
    initial.t = 0;
    initial.pose.x = me.position.x;
    initial.pose.y = me.position.y;
    initial.pose.yawAngle = me.yawAngle;

    history_.push_back(initial);
    nPointsToAdd -= 1;
  }
  else
  {
    // clear history of points already processed by sim
    history_.erase(history_.begin(), history_.begin() + nPointsToAdd);
  }

  // find nearest vehicle and set final boundary conditions
  int targetLane = 1;                                             /// \todo replace with correct lane number
  double targetD = getFrenetDFromLaneNumber(targetLane) ;
  double targetDist = SAFE_MANOEUVRE_DISTANCE;
  double targetSpeed = MAX_SPEED;
  double targetTime = 10;//2*targetDist/targetSpeed;
/*
  auto vehicleIt = findLeadVehicle(targetLane, me, others, wps);  ///\todo use 'me'??
  if( vehicleIt != others.end() )
  {
    targetDist = std::min(targetDist, distance(vehicleIt->position, me.position) - SAFE_FOLLOW_DISTANCE); ///\todo use 'me'?
    if(targetDist < SAFE_MANOEUVRE_DISTANCE)
    {
      targetSpeed = vehicleIt->speed; // match speed of vehicle in front
      targetTime = fabs(targetSpeed - me.speed)/MAX_ACCELERATION;
    }
    std::cout << targetDist << " " << vehicleIt->speed << " " << targetSpeed << " " << targetTime << std::endl;
  }
*/
  // Generate waypoints in frenet coordinates
  computeTrajectory(targetSpeed, targetD, targetTime, nPointsToAdd, history_);

  // Convert to cartesian path
  smoothenTrajectory(myPrevPathSz+1, history_, path);

  return path;
}

