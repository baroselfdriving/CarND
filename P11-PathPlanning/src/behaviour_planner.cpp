#include "behaviour_planner.h"
#include "trajectory_planner.h"
#include "helpers.h"

#include <algorithm>

namespace sdcnd_t3p1
{

//---------------------------------------------------------------------------------------------------------------------
BehaviourPlanner::BehaviourPlanner()
//---------------------------------------------------------------------------------------------------------------------
{
  current_.duration = 0;
  current_.targetSpeed = 0;
  current_.targetLane = 0;
  current_.type = BehaviourType::LANE_FOLLOW;
}

//---------------------------------------------------------------------------------------------------------------------
void BehaviourPlanner::reset(const Vehicle& me)
//---------------------------------------------------------------------------------------------------------------------
{
  current_.duration = 0;
  current_.targetSpeed = me.speed;
  current_.targetLane = getLaneNumberFromFrenetD(me.frenet.d);
  current_.type = BehaviourType::LANE_FOLLOW;
}

//---------------------------------------------------------------------------------------------------------------------
BehaviourTypesList BehaviourPlanner::getSuccessorStates(const Vehicle& me)
//---------------------------------------------------------------------------------------------------------------------
{
  BehaviourTypesList list;
  const int myLane = getLaneNumberFromFrenetD(me.frenet.d);

  switch(current_.type)
  {
  case BehaviourType::LANE_FOLLOW:
  {
    list.push_back(BehaviourType::LANE_FOLLOW);
    list.push_back(BehaviourType::LANE_CHANGE_LEFT);
    list.push_back(BehaviourType::LANE_CHANGE_RIGHT);
    break;
  }
  case BehaviourType::LANE_CHANGE_LEFT:
  {
    list.push_back( (current_.targetLane == myLane) ? BehaviourType::LANE_FOLLOW : BehaviourType::LANE_CHANGE_LEFT );
    break;
  }
  case BehaviourType::LANE_CHANGE_RIGHT:
  {
    list.push_back( (current_.targetLane == myLane) ? BehaviourType::LANE_FOLLOW : BehaviourType::LANE_CHANGE_RIGHT);
    break;
  }
  };
  return list;
}

//---------------------------------------------------------------------------------------------------------------------
Behaviour BehaviourPlanner::compute(const BehaviourPredictor::LanePredictionMap& predictions, const Vehicle& me)
//---------------------------------------------------------------------------------------------------------------------
{
  std::map<BehaviourType, double> costMap;
  std::map<BehaviourType, BehaviourPredictor::Prediction> predictionForBehaviour;

  current_.duration += 1;
  const int myLane = getLaneNumberFromFrenetD(me.frenet.d);
  const auto successorStates = getSuccessorStates(me);
  for(const auto& availableState : successorStates)
  {
    switch(availableState)
    {
    case LANE_CHANGE_LEFT:
    {
      if(myLane == 0)
      {
        continue;
      }
      const auto& p = predictions.at(myLane-1);
      costMap[LANE_CHANGE_LEFT] = computeCost(p);
      predictionForBehaviour[LANE_CHANGE_LEFT] = p;
      break;
    }
    case LANE_CHANGE_RIGHT:
    {
      if(myLane == 2)
      {
        continue;
      }
      const auto& p = predictions.at(myLane+1);
      costMap[LANE_CHANGE_RIGHT] = computeCost(p);
      predictionForBehaviour[LANE_CHANGE_RIGHT] = p;
      break;
    }
    case LANE_FOLLOW:
    {
      const auto& p = predictions.at(myLane);
      costMap[LANE_FOLLOW] = computeCost(p);
      predictionForBehaviour[LANE_FOLLOW] = p;
    }
    };
  }

  double minCost = std::numeric_limits<double>::max();
  BehaviourType bestBehaviourType = current_.type;
  for(const auto& option : costMap)
  {
    if (option.second < minCost)
    {
      minCost = option.second;
      bestBehaviourType = option.first;
    }
  }

  Behaviour behaviour;
  behaviour.type = bestBehaviourType;
  behaviour.duration = ((bestBehaviourType == current_.type) ? current_.duration : 0);
  behaviour.targetLane = predictionForBehaviour[bestBehaviourType].laneNumber;
  behaviour.targetSpeed = predictionForBehaviour[bestBehaviourType].laneSpeed;

  current_ = behaviour;

  std::cout << "B: " << current_.type << " " << current_.duration << " " << current_.targetLane << std::endl;

  return current_;
}


//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::computeCost(const BehaviourPredictor::Prediction& prediction)
//---------------------------------------------------------------------------------------------------------------------
{
  return
      100 * collisionCost(prediction) +
      90 * speedDeviationCost(prediction) +
      80 * manouvrebilityCost(prediction) +
      70 * frequentLaneChangeCost(prediction) +
      60 * separationCost(prediction);
}


//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::collisionCost(const BehaviourPredictor::Prediction& pred)
//---------------------------------------------------------------------------------------------------------------------
{
  double cost = 1.;

  if(pred.freeDistance > TrajectoryPlanner::SAFE_MANOEUVRE_DISTANCE)
  {
    cost = -1.;
  }

  return cost;
}

//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::separationCost(const BehaviourPredictor::Prediction& pred)
//---------------------------------------------------------------------------------------------------------------------
{
  const double goodSeparation = 3*TrajectoryPlanner::SAFE_MANOEUVRE_DISTANCE;
  if(pred.freeDistance < TrajectoryPlanner::SAFE_MANOEUVRE_DISTANCE)
  {
    return 1.;
  }
  else if(pred.freeDistance > goodSeparation)
  {
    return -1.;
  }
  else
  {
    return -1 * (pred.freeDistance/goodSeparation);
  }
}

//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::speedDeviationCost(const BehaviourPredictor::Prediction& pred)
//---------------------------------------------------------------------------------------------------------------------
{
  const double deviation = fabs(SPEED_LIMIT - pred.laneSpeed);
  return -0.5 + (deviation/SPEED_LIMIT);
}

//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::manouvrebilityCost(const BehaviourPredictor::Prediction& pred)
//---------------------------------------------------------------------------------------------------------------------
{
  if(pred.laneNumber == 1)
  {
    return -1;
  }
  else
  {
    return 0.5;
  }
}

//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::frequentLaneChangeCost(const BehaviourPredictor::Prediction& pred)
//---------------------------------------------------------------------------------------------------------------------
{
  const double minDuration = 100;
  if( current_.duration < minDuration)
  {
    return 1;
  }
  else if( current_.duration > 2*minDuration)
  {
    return -1;
  }
  else
  {
    return 1 - current_.duration/minDuration;
  }
}

}
