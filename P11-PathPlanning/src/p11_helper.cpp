#include "p11_helper.h"

#include <limits>

//---------------------------------------------------------------------------------------------------------------------
std::string hasData(std::string s)
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
WaypointList::const_iterator ClosestWaypoint(double x, double y, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  double nearest = std::numeric_limits<double>::max();
  WaypointList::const_iterator itNearest;
  for(WaypointList::const_iterator it = wps.begin(); it != wps.end(); ++it)
  {
    double dist = distance(x, y, it->x, it->y);
    if(dist < nearest)
    {
      nearest = dist;
      itNearest = it;
    }
  }
  return itNearest;
}

//---------------------------------------------------------------------------------------------------------------------
WaypointList::const_iterator NextWaypoint(double x, double y, double theta, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  auto closestWaypoint = ClosestWaypoint(x, y, wps);

  double mapX = closestWaypoint->x;
  double mapY = closestWaypoint->y;

  double heading = atan2((mapY-y),(mapX-x));

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
std::vector<double> getFrenet(double x, double y, double theta, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  auto next_wp = NextWaypoint(x,y, theta, wps);
  auto prev_wp = ( (next_wp == wps.begin()) ? (wps.end()-1) : (next_wp-1) );

  double n_x = next_wp->x - prev_wp->x;
  double n_y = next_wp->y - prev_wp->y;
  double x_x = x - prev_wp->x;
  double x_y = y - prev_wp->y;

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - prev_wp->x;
  double center_y = 2000 - prev_wp->y;
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(WaypointList::const_iterator it = wps.begin(); it != prev_wp; ++it)
  {
    WaypointList::const_iterator itNext = it++;
    frenet_s += distance(it->x, it->y, itNext->x, itNext->y);
  }
  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s,frenet_d};

}

//---------------------------------------------------------------------------------------------------------------------
std::vector<double> getXY(double s, double d, const WaypointList& wps)
//---------------------------------------------------------------------------------------------------------------------
{
  WaypointList::const_iterator prev_wp = wps.begin();

  while(s > prev_wp->s && (prev_wp != wps.end()) )
  {
    prev_wp++;
  }

  WaypointList::const_iterator wp2 = ++prev_wp;
  if(wp2 == wps.end())
  {
    wp2 = wps.begin();
  }

  double heading = atan2((wp2->y - prev_wp->y),(wp2->x - prev_wp->x));

  // the x,y,s along the segment
  double seg_s = (s - prev_wp->s);
  double seg_x = prev_wp->x + seg_s*cos(heading);
  double seg_y = prev_wp->y + seg_s*sin(heading);

  double perp_heading = heading-M_PI/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}
