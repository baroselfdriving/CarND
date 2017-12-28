#ifndef BEHAVIOUR_PLANNER_H
#define BEHAVIOUR_PLANNER_H

#include "vehicle.h"
#include "behaviour.h"
#include "behaviour_predictor.h"

namespace sdcnd_t3p1
{

class BehaviourPlanner
{
public:
  BehaviourPlanner() = default;
  ~BehaviourPlanner() = default;

  Behaviour compute(const BehaviourPredictor::PredictionMap& predictions, const VehicleList& otherVehicles, const Vehicle& me);

  BehavioursList getSuccessorStates(Behaviour currentBehaviour);


private:

};

}
#endif // BEHAVIOUR_PLANNER_H

