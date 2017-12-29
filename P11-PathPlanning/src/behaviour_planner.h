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
  static constexpr double MIN_LANE_KEEP_TIMESTEPS = 50;

public:

  BehaviourPlanner();

  ~BehaviourPlanner() = default;

  void reset (const Vehicle& me);

  Behaviour compute(const BehaviourPredictor::LanePredictionMap& predictions, const Vehicle& me);

  /// Cost function for potential collisions.
  double collisionCost(const BehaviourPredictor::Prediction& pred);

  /// Cost function for separation distance. Larger distance between vehicles = lower costs
  double separationCost(const BehaviourPredictor::Prediction& pred);

  /// Cost function of deviation from max speed. Higher cost for higher deviation.
  double speedDeviationCost(const BehaviourPredictor::Prediction& pred);

  /// Cost function for available lane change options
  double manouvrebilityCost(const BehaviourPredictor::Prediction& pred);

  /// Cost for changing lanes too frequently
  double frequentLaneChangeCost(const BehaviourPredictor::Prediction& pred);

private:

  /// Get valid successor state
  BehaviourTypesList getSuccessorStates(const BehaviourPredictor::LanePredictionMap& predictions, const Vehicle& me);

  /// Compute total cost for a change into a lane
  double computeCost(const BehaviourPredictor::Prediction& prediction);


private:
  Behaviour current_;
};

}
#endif // BEHAVIOUR_PLANNER_H

