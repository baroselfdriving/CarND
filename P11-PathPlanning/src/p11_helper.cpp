#include "p11_helper.h"

#include <limits>

//---------------------------------------------------------------------------------------------------------------------
std::string hasJsonData(std::string s)
//---------------------------------------------------------------------------------------------------------------------
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos)
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
WaypointList::const_iterator ClosestWaypoint(const CartesianCoord& p, const WaypointList& wps)
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
WaypointList::const_iterator NextWaypoint(const CartesianCoord& p, double theta, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  auto closestWaypoint = ClosestWaypoint(p, wps);

  double mapX = closestWaypoint->point.x;
  double mapY = closestWaypoint->point.y;

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
FrenetCoord getFrenet(const CartesianCoord& p, double theta, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  auto next_wp = NextWaypoint(p, theta, wps);
  auto prev_wp = ( (next_wp == wps.begin()) ? (wps.end()-1) : (next_wp-1) );

  double n_x = next_wp->point.x - prev_wp->point.x;
  double n_y = next_wp->point.y - prev_wp->point.y;
  double x_x = p.x - prev_wp->point.x;
  double x_y = p.y - prev_wp->point.y;

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  FrenetCoord fp;
  fp.d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - prev_wp->point.x;
  double center_y = 2000 - prev_wp->point.y;
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    fp.d *= -1;
  }

  // calculate s value
  fp.s = 0;
  for(WaypointList::const_iterator it = wps.begin(); it != prev_wp; ++it)
  {
    WaypointList::const_iterator itNext = it++;
    fp.s += distance(it->point.x, it->point.y, itNext->point.x, itNext->point.y);
  }
  fp.s += distance(0, 0, proj_x, proj_y);

  return fp;

}

//---------------------------------------------------------------------------------------------------------------------
CartesianCoord getXY(const FrenetCoord& fp, const WaypointList& wps)
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

  double heading = atan2((wp2->point.y - prev_wp->point.y),(wp2->point.x - prev_wp->point.x));

  // the x,y,s along the segment
  double seg_s = (fp.s - prev_wp->frenet.s);
  double seg_x = prev_wp->point.x + seg_s*cos(heading);
  double seg_y = prev_wp->point.y + seg_s*sin(heading);

  double perp_heading = heading-M_PI/2;

  CartesianCoord p;
  p.x = seg_x + fp.d * cos(perp_heading);
  p.y = seg_y + fp.d * sin(perp_heading);

  return p;

}
