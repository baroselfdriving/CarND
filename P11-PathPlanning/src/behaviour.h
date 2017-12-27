#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

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
  double cost;
};

}
#endif // BEHAVIOUR_H
