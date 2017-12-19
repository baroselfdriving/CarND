#ifndef P11_HELPER_H
#define P11_HELPER_H

#include <string>
#include <cmath>
#include <vector>

inline double deg2rad(double x) { return x * M_PI / 180.; }

inline double rad2deg(double x) { return x * 180. / M_PI; }

inline double distance(double x1, double y1, double x2, double y2) { return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)); }


/// Checks if the SocketIO event has JSON data.
/// If there is data the JSON object in string format will be returned,
/// else the empty string "" will be returned.
std::string hasData(std::string s);

unsigned int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

unsigned int NextWaypoint(double x, double y, double theta,
                          const std::vector<double> &maps_x, const std::vector<double> &maps_y);

/// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y);

/// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d,
                          const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y);

#endif // P11_HELPER_H
