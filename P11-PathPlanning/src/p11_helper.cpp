#include "p11_helper.h"

#include <limits>

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
    double dist = distance(p, it->pose);
    if(dist < nearest)
    {
      nearest = dist;
      itNearest = it;
    }
  }
  return itNearest;
}

//---------------------------------------------------------------------------------------------------------------------
WaypointList::const_iterator NextWaypoint(const CartesianPose& p, double theta, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  auto closestWaypoint = ClosestWaypoint(p, wps);

  double mapX = closestWaypoint->pose.x;
  double mapY = closestWaypoint->pose.y;

  double heading = atan2((mapY - p.y),(mapX - p.x));

  double angle = fabs(theta-heading);
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
FrenetPose getFrenet(const CartesianPose& p, double theta, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  auto nextWp = NextWaypoint(p, theta, wps);
  auto prevWp = ( (nextWp == wps.begin()) ? (wps.end()-1) : (nextWp-1) );

  double n_x = nextWp->pose.x - prevWp->pose.x;
  double n_y = nextWp->pose.y - prevWp->pose.y;
  double x_x = p.x - prevWp->pose.x;
  double x_y = p.y - prevWp->pose.y;

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  FrenetPose fp;
  fp.d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - prevWp->pose.x;
  double center_y = 2000 - prevWp->pose.y;
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
    fp.s += distance(it->pose, itNext->pose);
  }
  fp.s += distance(0, 0, proj_x, proj_y);

  fp.dx = fp.dy = 0; // don't care

  return fp;

}

//---------------------------------------------------------------------------------------------------------------------
CartesianPose getXY(const FrenetPose& fp, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  WaypointList::const_iterator prev_wp = wps.begin();
  while(fp.s > (prev_wp+1)->frenet.s && (prev_wp != wps.end()-1) )
  {
    prev_wp++;
  }

  WaypointList::const_iterator wp2 = (prev_wp+1);
  if(wp2 == wps.end())
  {
    wp2 = wps.begin();
  }

  const double heading = atan2((wp2->pose.y - prev_wp->pose.y),(wp2->pose.x - prev_wp->pose.x));

  // the x,y,s along the segment
  double seg_s = (fp.s - prev_wp->frenet.s);
  double seg_x = prev_wp->pose.x + seg_s*cos(heading);
  double seg_y = prev_wp->pose.y + seg_s*sin(heading);

  double perp_heading = heading-M_PI/2;

  CartesianPose p;
  p.x = seg_x + fp.d * cos(perp_heading);
  p.y = seg_y + fp.d * sin(perp_heading);
  p.yawAngle = heading;

  return p;

}
