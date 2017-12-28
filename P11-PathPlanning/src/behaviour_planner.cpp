#include "behaviour_planner.h"

namespace sdcnd_t3p1
{

//---------------------------------------------------------------------------------------------------------------------
Behaviour BehaviourPlanner::compute(const VehicleList& otherVehicles, const Vehicle& me)
//---------------------------------------------------------------------------------------------------------------------
{
  Behaviour b;
  b.type = LANE_FOLLOW;
  b.cost = 0;
  return b;
}

//---------------------------------------------------------------------------------------------------------------------
BehavioursList BehaviourPlanner::getSuccessorStates(Behaviour currentBehaviour)
//---------------------------------------------------------------------------------------------------------------------
{
  BehavioursList options;

  switch(currentBehaviour.type)
  {
  case BehaviourType::LANE_FOLLOW:
    break;
  case BehaviourType::LANE_CHANGE_LEFT:
    break;
  case BehaviourType::LANE_CHANGE_RIGHT:
    break;
  };
  return options;
}

}
