#include "helpers.h"
#include "spline.h"

#include <limits>

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
  const double maxS = wps.back().frenet.s;
  while(s > maxS)
  {
    s -= maxS;
  }

  WaypointList::const_iterator prev_wp = wps.begin();
  while(s > (prev_wp+1)->frenet.s && (prev_wp != wps.end()-1) )
  {
    prev_wp++;
  }

  WaypointList::const_iterator wp2 = (prev_wp+1);
  if(wp2 == wps.end())
  {
    wp2 = wps.begin();
  }

  const double heading = atan2((wp2->point.y - prev_wp->point.y),(wp2->point.x - prev_wp->point.x));

  // the x,y,s along the segment
  double seg_s = (s - prev_wp->frenet.s);
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
WaypointList generateFinerWaypoints(const WaypointList& input, unsigned int n)
//---------------------------------------------------------------------------------------------------------------------
{
  WaypointList finePoints;
  return finePoints;
}

}
