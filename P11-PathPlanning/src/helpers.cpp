#include "helpers.h"
#include "spline.h"

#include <limits>
#include <iostream>
#include <iomanip>

namespace sdcnd_t3p1
{

//---------------------------------------------------------------------------------------------------------------------
std::string hasJsonData(std::string s)
//---------------------------------------------------------------------------------------------------------------------
{
  auto foundNull = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (foundNull != std::string::npos)
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//---------------------------------------------------------------------------------------------------------------------
WaypointList::const_iterator ClosestWaypoint(const CartesianPose& p, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  double nearest = std::numeric_limits<double>::max();
  WaypointList::const_iterator itNearest;
  for(WaypointList::const_iterator it = wps.begin(); it != wps.end(); ++it)
  {
    double dist = distance(p, it->point);
    if(dist < nearest)
    {
      nearest = dist;
      itNearest = it;
    }
  }
  return itNearest;
}

//---------------------------------------------------------------------------------------------------------------------
WaypointList::const_iterator NextWaypoint(const CartesianPose& p, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  auto closestWaypoint = ClosestWaypoint(p, wps);

  double mapX = closestWaypoint->point.x;
  double mapY = closestWaypoint->point.y;

  double track = atan2((mapY - p.y),(mapX - p.x));

  double angle = fabs(p.heading - track);
  angle = std::min(2*M_PI - angle, angle);
  if(angle > M_PI/4) // not within 45 degrees of heading
  {
    closestWaypoint++;
    if (closestWaypoint == wps.end())
    {
      closestWaypoint = wps.begin();
    }
  }

  return closestWaypoint;
}

//---------------------------------------------------------------------------------------------------------------------
FrenetPoint getFrenet(const CartesianPose& p, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  auto nextWp = NextWaypoint(p, wps);
  auto prevWp = ( (nextWp == wps.begin()) ? (wps.end()-1) : (nextWp-1) );

  double n_x = nextWp->point.x - prevWp->point.x;
  double n_y = nextWp->point.y - prevWp->point.y;
  double x_x = p.x - prevWp->point.x;
  double x_y = p.y - prevWp->point.y;

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  FrenetPoint fp;
  fp.d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - prevWp->point.x;
  double center_y = 2000 - prevWp->point.y;
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    fp.d *= -1;
  }

  // calculate s value
  fp.s = 0;
  for(WaypointList::const_iterator it = wps.begin(); it != (prevWp-1); ++it)
  {
    WaypointList::const_iterator itNext = it+1;
    fp.s += distance(it->point, itNext->point);
  }
  fp.s += distance(0, 0, proj_x, proj_y);

  fp.dx = fp.dy = 0; // don't care

  return fp;

}

//---------------------------------------------------------------------------------------------------------------------
CartesianPose getCartesianFromFrenet(double s, double d, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  WaypointList::const_iterator prev_wp = wps.begin();
  if(s < prev_wp->frenet.s)
  {
    std::cout << "WARNING [getCartesianFromFrenet] s = " << s << " not within waypoints range ["
              << wps.front().frenet.s << ", " << wps.back().frenet.s << "]" << std::endl;
  }
  while(s > (prev_wp+1)->frenet.s && (prev_wp != wps.end()-1) )
  {
    prev_wp++;
  }

  WaypointList::const_iterator wp2 = (prev_wp+1);
  if(wp2 == wps.end())
  {
    wp2 = wps.begin();
    std::cout << "WARNING [getCartesianFromFrenet] Reached end of track - wrapped around" << std::endl;
  }

  const double heading = atan2((wp2->point.y - prev_wp->point.y),(wp2->point.x - prev_wp->point.x));

  // the x,y,s along the segment
  //double seg_s = (s - prev_wp->frenet.s);
  double seg_s = getDistanceAlongTrack(s, prev_wp->frenet.s);
  double seg_x = prev_wp->point.x + seg_s*cos(heading);
  double seg_y = prev_wp->point.y + seg_s*sin(heading);

  double perp_heading = heading-M_PI/2;

  CartesianPose p;
  p.x = seg_x + d * cos(perp_heading);
  p.y = seg_y + d * sin(perp_heading);
  p.heading = heading;

  return p;
}

//---------------------------------------------------------------------------------------------------------------------
CartesianPose transformToLocal(const CartesianPose& point, const CartesianPose& originFrame)
//---------------------------------------------------------------------------------------------------------------------
{
  const double cosa = cos(originFrame.heading);
  const double sina = sin(originFrame.heading);

  CartesianPose out;
  out.x = cosa * (point.x - originFrame.x) + sina * (point.y - originFrame.y);
  out.y = -sina * (point.x - originFrame.x) + cosa * (point.y - originFrame.y);
  out.heading = point.heading - originFrame.heading;

  return out;
}

//---------------------------------------------------------------------------------------------------------------------
CartesianPose transformToGlobal(const CartesianPose& point, const CartesianPose& originFrame)
//---------------------------------------------------------------------------------------------------------------------
{
  const double cosa = cos(originFrame.heading);
  const double sina = sin(originFrame.heading);

  // transform to global frame
  CartesianPose out;
  out.x = originFrame.x + cosa * point.x - sina * point.y;
  out.y = originFrame.y + sina * point.x + cosa * point.y;
  out.heading = point.heading + originFrame.heading;

  return out;
}

//---------------------------------------------------------------------------------------------------------------------
Waypoint generateWaypoint(double s, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  Waypoint wp;
  wp.point = getCartesianFromFrenet(s,0,wps);
  wp.frenet.s = s;
  while(wp.frenet.s > MAX_TRACK_LENGTH)
  {
    wp.frenet.s -= MAX_TRACK_LENGTH;
  }
  wp.frenet.d = 0;
  wp.frenet.dx = cos(wp.point.heading-M_PI/2.);
  wp.frenet.dy = sin(wp.point.heading-M_PI/2.);
  return wp;
}

//---------------------------------------------------------------------------------------------------------------------
WaypointList generateLocalWaypoints(const WaypointList& points, unsigned int n)
//---------------------------------------------------------------------------------------------------------------------
{
  if(n < 1)
  {
    return points;
  }

  const CartesianPose segmentOrigin = points[0].point;
  const size_t nAnchors = points.size();
  CartesianPoseList splinePoints(nAnchors);

  // Convert to local coordinates
  for(size_t i = 0; i < nAnchors; ++i)
  {
    splinePoints[i] = transformToLocal(points[i].point, segmentOrigin);
  }

  // fit spline
  tk::spline splinator;
  //splinator.set_boundary(tk::spline::first_deriv, 0, tk::spline::first_deriv, 0, true);
  splinator.set_points(splinePoints);

  // Generate intermediate points and convert to global coords
  const double dx = (splinePoints[nAnchors-1].x - splinePoints[0].x)/n;
  WaypointList finePoints(n);
  for(unsigned int i = 0; i < n; ++i)
  {
    if(i == 0)
    {
      finePoints[0] = points[0];
    }
    else
    {
      const auto& prevWp = finePoints.at(i-1);
      Waypoint wp;
      wp.point.x = dx*i;
      wp.point.y = splinator(wp.point.x);
      wp.point.heading = 0;
      wp.point = transformToGlobal(wp.point, segmentOrigin);
      wp.point.heading = atan2((wp.point.y - prevWp.point.y),(wp.point.x - prevWp.point.x));
      wp.frenet.s = prevWp.frenet.s + distance(prevWp.point, wp.point); /// \todo deal with track boundary
      wp.frenet.d = 0;
      wp.frenet.dx = cos(wp.point.heading-M_PI/2.);
      wp.frenet.dy = sin(wp.point.heading-M_PI/2.);
      finePoints[i] = wp;
    }
  }
/*
  std::cout << std::setprecision(8);
  for(const auto& wp : finePoints)
  {
    std::cout << wp.point.x << ", " << wp.point.y << ", " << wp.frenet.s << ", "
              << wp.frenet.dx << ", " << wp.frenet.dy << std::endl;
  }
*/
  return finePoints;
}

//---------------------------------------------------------------------------------------------------------------------
WaypointList generateIntermediateWaypoins(const WaypointList& input, unsigned int n)
//---------------------------------------------------------------------------------------------------------------------
{
  if(n < 1)
  {
    return input;
  }
  WaypointList finePoints;

  /// procedure
  /// - (*) read next waypoint P0
  /// - Get next two waypoints and convert them into F0 coordinates called P1 and P2
  /// - Add P0, P1 and P2 to the spline
  /// - Generate n points P01...P0n between P0 and P1
  /// - Convert these points to world coordinates. Also compute s,d,dx,dy
  /// - add P0....P1 to list
  /// - goto * (do this until the second last point. At this point, we will have completed the loop)

  size_t numInputs = input.size();
  for(size_t iOriginWp = 0; iOriginWp < numInputs - 1; ++iOriginWp)
  {
    const CartesianPose segmentOrigin = input[iOriginWp].point;
    const unsigned int nSplineAnchors = 3;
    CartesianPoseList splinePoints(nSplineAnchors);
    for(size_t iNextWp = 0; iNextWp < nSplineAnchors; ++iNextWp)
    {
      splinePoints[iNextWp] = transformToLocal(input[(iOriginWp+iNextWp)%numInputs].point, segmentOrigin);
    }

    tk::spline splinator;
    //splinator.set_boundary(tk::spline::first_deriv, 0, tk::spline::first_deriv, 0, true);
    splinator.set_points(splinePoints);

    const double dx = (splinePoints[1].x - splinePoints[0].x)/n;
    for(unsigned int i = 1; i <= n; ++i)
    {
      if(iOriginWp == 0 && i == 1)
      {
        finePoints.push_back(input[0]);
      }
      else
      {
        WaypointList::const_iterator prevWpIt = finePoints.end()-1;
        Waypoint wp;
        wp.point.x = dx*i;
        wp.point.y = splinator(wp.point.x);
        wp.point.heading = 0;
        wp.point = transformToGlobal(wp.point, segmentOrigin);
        wp.point.heading = atan2((wp.point.y - prevWpIt->point.y),(wp.point.x - prevWpIt->point.x));
        wp.frenet.s = prevWpIt->frenet.s + distance(prevWpIt->point, wp.point);
        wp.frenet.d = 0;
        wp.frenet.dx = cos(wp.point.heading-M_PI/2.);
        wp.frenet.dy = sin(wp.point.heading-M_PI/2.);
        finePoints.push_back(wp);
      }
    }
  }
/*
  for(const auto& wp : finePoints)
  {
    std::cout << wp.point.x << ", " << wp.point.y << ", " << wp.frenet.s << ", "
              << wp.frenet.dx << ", " << wp.frenet.dy << std::endl;
  }
*/
  return finePoints;
}

//---------------------------------------------------------------------------------------------------------------------
double getDistanceAlongTrack(double me, double other)
//---------------------------------------------------------------------------------------------------------------------
{
  double d = me - other;
  const double halfTrackLength = MAX_TRACK_LENGTH/2.;
  if( d < -halfTrackLength )
  {
    d += MAX_TRACK_LENGTH;
  }

  if(d > halfTrackLength)
  {
    d -= MAX_TRACK_LENGTH;
  }
  return d;
}


//---------------------------------------------------------------------------------------------------------------------
NearestVehicles findClosestVehiclesInLane(int lane, const Vehicle& me, const VehicleList& vehicles)
//---------------------------------------------------------------------------------------------------------------------
{
  NearestVehicles nearestVehicles;
  nearestVehicles.ahead = vehicles.end();
  nearestVehicles.behind = vehicles.end();

  double nearestAhead = -(MAX_TRACK_LENGTH + 1);
  double nearestBehind = MAX_TRACK_LENGTH + 1;

  for(VehicleList::const_iterator other = vehicles.begin(); other != vehicles.end(); ++other)
  {
    const int otherLane = getLaneNumberFromFrenetD(other->frenet.d);
    if(lane != otherLane)
    {
      continue;
    }

    const double dist = getDistanceAlongTrack(me.frenet.s, other->frenet.s);

    // vehicle is behind
    if(dist > 0)
    {
      if(dist < nearestBehind)
      {
        nearestBehind = dist;
        nearestVehicles.behind = other;
      }
    }

    // vehicle is ahead
    else
    {
      if( dist > nearestAhead )
      {
        nearestAhead = dist;
        nearestVehicles.ahead = other;
      }

    }
  }
  return nearestVehicles;
}

}
