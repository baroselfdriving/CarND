#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

enum BehaviourType
{
  LANE_FOLLOW,
  LANE_CHANGE_LEFT,
  LANE_CHANGE_RIGHT
};

struct Behaviour
{
  BehaviourType type;
  double timeHorizon;
};

#endif // BEHAVIOUR_H

