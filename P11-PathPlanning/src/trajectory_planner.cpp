#include "trajectory_planner.h"
#include "p11_helper.h"
#include "integrator.h"
#include "spline.h"

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
void TrajectoryPlanner::updateTrajectory(double longSpeed, double latPos, double timeDelta, size_t nPointsToAdd)
//---------------------------------------------------------------------------------------------------------------------
{
  Integrator integrator(SIM_DELTA_TIME);

  double s0 = 0;
  double t0 = 0;
  PolynomialConstraint sInitial = {0};
  PolynomialConstraint dInitial = {0};
  PolynomialConstraint sFinal;
  PolynomialConstraint dFinal;

  // set initial conditions if available
  if(history_.size() > 0)
  {
    const auto& it = history_.back();
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

  // set final conditions
  sFinal.q = longSpeed;
  sFinal.qDot = 0;
  sFinal.qDotDot = 0;
  sFinal.t = sInitial.t + timeDelta;

  dFinal.q = latPos;
  dFinal.qDot = 0;
  dFinal.qDotDot = 0;
  dFinal.t = dInitial.t + timeDelta;

  // Fit polynomial between end points
  const std::array<double, 6> sCoeffs = computePolynomialCoefficients(sInitial, sFinal);
  const std::array<double, 6> dCoeffs = computePolynomialCoefficients(dInitial, dFinal);

  // Generate intermediate waypoints
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

    fs.pose = getCartesianFromFrenet(fs.s, fs.d, trackWaypoints_);

    history_.push_back(fs);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void TrajectoryPlanner::smoothenTrajectory(size_t index, StateList& history, CartesianPoseList& coords)
//---------------------------------------------------------------------------------------------------------------------
{
  /*
  assert(index > 1);
  assert(index < history.size());

  // smoothen the cartesian coordinates

  /// - Find frame at the origin of the segment
  /// - transform all points to coordinates of segment frame
  /// - fit spline. get cartesian coordinates in segment frame
  /// - transform cartesian coords into global frame

  tk::spline splinator;
  CartesianPoseList segment;
  const CartesianPose originPose = (history.begin() + index - 1)->pose;
  const double cosa = cos(originPose.heading);
  const double sina = sin(originPose.heading);
  for(StateList::iterator it = (history.begin()+index -2); it < history.end(); ++it)
  {
    // transform to segment frame and set
    CartesianPose p;
    p.x = cosa * (it->pose.x - originPose.x) + sina * (it->pose.y - originPose.y);
    p.y = -sina * (it->pose.x - originPose.x) + cosa * (it->pose.y - originPose.y);
    p.heading = it->pose.heading - originPose.heading;

    segment.push_back(p);
  }
  splinator.set_points(segment);
  CartesianPoseList::iterator segmentIt = segment.begin();
  */
  for(StateList::iterator it = (history.begin()+index-1); it < history.end(); ++it)
  {
    // Get smoothed coordinate
    //CartesianPose p;
    //p.x = segmentIt->x;
    //p.y = splinator(p.x);

    // transform to global frame
    //it->pose.x = originPose.x + cosa * p.x - sina * p.y;
    //it->pose.y = originPose.y + sina * p.x + cosa * p.y;
    //it->pose.heading = segmentIt->heading + originPose.heading;
    //segmentIt++;
    coords.push_back(it->pose);
  }
  //std::cout << history_.begin()->t << " " << history_.begin()->s << " " << (history_.end()-1)->t << " " << (history_.end()-1)->s << std::endl;

}

//---------------------------------------------------------------------------------------------------------------------
CartesianPoseList TrajectoryPlanner::getPlan(const Vehicle& me, const VehicleList& others,
                                             const CartesianPoseList& myPrevPath, const FrenetPoint& prevPathEnd)
//---------------------------------------------------------------------------------------------------------------------
{
  const size_t myPrevPathSz = myPrevPath.size();
  size_t nPointsToAdd = SIM_NUM_WAYPOINTS - myPrevPathSz;
  CartesianPoseList path = myPrevPath;

  // set initial boundary conditions
  if(myPrevPathSz < 1)
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
    initial.pose.heading = me.yawAngle;

    history_.push_back(initial);
    nPointsToAdd -= 1;

/*
    // set another point just in front of the car to set starting direction of travel
    FrenetPoint nextFp = getFrenet(initial.pose, trackWaypoints_);

    initial.s = nextFp.s;
    initial.sv = me.speed;
    initial.sa = 0;
    initial.sj = 0;
    initial.d = nextFp.d;
    initial.dv = 0;
    initial.da = 0;
    initial.dj = 0;
    initial.t = 0;
    initial.pose = getCartesianFromFrenet(nextFp.s, nextFp.d, trackWaypoints_);
    initial.pose.heading = me.yawAngle;

    history_.push_back(initial);
    nPointsToAdd -= 1;*/
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
  updateTrajectory(targetSpeed, targetD, targetTime, nPointsToAdd);

  // Smooth the cartesian path so generated
  smoothenTrajectory(myPrevPathSz+1, history_, path);

  return path;
}

