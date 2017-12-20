#include "behaviour_planner.h"

//---------------------------------------------------------------------------------------------------------------------
Behaviour BehaviourPlanner::compute(const VehicleList& otherVehicles, const Vehicle& me)
//---------------------------------------------------------------------------------------------------------------------
{
  Behaviour b;
  b.type = LANE_FOLLOW;
  b.timeHorizon = 0;
  return b;
}
