#ifndef BEHAVIOUR_PLANNER_H
#define BEHAVIOUR_PLANNER_H

#include "vehicle.h"
#include "behaviour.h"

class BehaviourPlanner
{
public:
  BehaviourPlanner() = default;
  ~BehaviourPlanner() = default;

  Behaviour compute(const VehicleList& otherVehicles, const Vehicle& me);
};

#endif // BEHAVIOUR_PLANNER_H

