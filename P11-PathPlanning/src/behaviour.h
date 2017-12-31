#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include "constants.h"

#include <vector>
#include <map>

namespace sdcnd_t3p1
{

enum BehaviourType
{
  LANE_FOLLOW,
  LANE_CHANGE_LEFT,
  LANE_CHANGE_RIGHT
};

using BehaviourTypesList = std::vector<BehaviourType>;

static const std::map<BehaviourType, std::string> BEHAVIOURTYPE_STRING_MAP = {
  { BehaviourType::LANE_FOLLOW, "LANE_FOLLOW" },
  { BehaviourType::LANE_CHANGE_LEFT, "LANE_CHANGE_LEFT" },
  { BehaviourType::LANE_CHANGE_RIGHT, "LANE_CHANGE_RIGHT"} };

struct Behaviour
{
  static constexpr double MIN_RESPONSE_TIME = 3; //!< min time needed to respond to change in driving conditions
  static constexpr double LANE_CHANGE_DURATION = 5; //!< time required to change lane
  static constexpr double INITIAL_SPEED_UP_TIME = 15; //!< ramp up at t=0
  static constexpr double MAX_SPEED = .92 * SPEED_LIMIT;

  BehaviourType type;
  double targetSpeed;
  int targetLane;
  double duration; //!< number of time steps in this behaviour
};

using BehavioursList = std::vector<Behaviour>;

}
#endif // BEHAVIOUR_H

