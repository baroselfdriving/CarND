#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <vector>

namespace sdcnd_t3p1
{

enum BehaviourType
{
  LANE_FOLLOW,
  LANE_CHANGE_LEFT,
  LANE_CHANGE_RIGHT
};

struct Behaviour
{
  BehaviourType type;
  double targetSpeed;
  int targetLane;
  double cost;
};

using BehavioursList = std::vector<Behaviour>;

}
#endif // BEHAVIOUR_H

