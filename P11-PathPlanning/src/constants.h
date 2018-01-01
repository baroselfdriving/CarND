#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace sdcnd_t3p1
{

static constexpr double LANE_WIDTH = 3.8;
static constexpr double MAX_TRACK_LENGTH = 6945.554;
static constexpr double SPEED_LIMIT = 22.35; //!< 50 miles/hr = 22.35 meters/sec

static constexpr double SIM_DELTA_TIME = 0.02; //!< sim time between each waypoint
static constexpr unsigned int SIM_NUM_WAYPOINTS =
    static_cast<unsigned int>(1./SIM_DELTA_TIME); //!< number of waypoints to pass into sim each cycle
}

#endif // CONSTANTS_H

