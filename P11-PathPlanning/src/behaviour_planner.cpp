#include "behaviour_planner.h"
#include "helpers.h"

#include <algorithm>
#include <iostream>

namespace sdcnd_t3p1
{

constexpr double BehaviourPlanner::MIN_LANE_KEEP_TIMESTEPS;

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
BehaviourTypesList BehaviourPlanner::getSuccessorStates(const BehaviourPredictor::LanePredictionMap& predictions, const Vehicle& me)
//---------------------------------------------------------------------------------------------------------------------
{
  BehaviourTypesList list;
  const int myLane = getLaneNumberFromFrenetD(me.frenet.d);

  switch(current_.type)
  {
  case BehaviourType::LANE_FOLLOW:
  {
    list.push_back(BehaviourType::LANE_FOLLOW);
    if( current_.duration > MIN_LANE_KEEP_TIMESTEPS)
    {
      if( predictions.at(std::max(0,myLane-1)).freeDistance > 0)
      {
        list.push_back(BehaviourType::LANE_CHANGE_LEFT);
      }
      if( predictions.at(std::min(2,myLane+1)).freeDistance > 0)
      {
        list.push_back(BehaviourType::LANE_CHANGE_RIGHT);
      }
    }
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
Behaviour BehaviourPlanner::plan(const BehaviourPredictor::LanePredictionMap& predictions, const Vehicle& me)
//---------------------------------------------------------------------------------------------------------------------
{
  std::map<BehaviourType, double> costMap;
  std::map<BehaviourType, BehaviourPredictor::Prediction> predictionForBehaviour;

  current_.duration += 1;
  const int myLane = getLaneNumberFromFrenetD(me.frenet.d);
  if((myLane > 2) || (myLane < 0))
  {
    current_.targetLane = 1;
    return current_; // something terrible happened, such as car off the road
  }
  const auto successorStates = getSuccessorStates(predictions, me);
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
/*
  if(current_.type != behaviour.type)
  {
    std::cout << BEHAVIOURTYPE_STRING_MAP.at(behaviour.type) << " : " << behaviour.targetLane << std::endl;
  }
  */
  current_ = behaviour;

  return current_;
}


//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::computeCost(const BehaviourPredictor::Prediction& prediction)
//---------------------------------------------------------------------------------------------------------------------
{
  return
      100 * collisionCost(prediction) +
      90 * speedDeviationCost(prediction) +
      50 * manouvrebilityCost(prediction) +
      80 * frequentLaneChangeCost(prediction) +
      200 * separationCost(prediction);
}


//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::collisionCost(const BehaviourPredictor::Prediction& pred)
//---------------------------------------------------------------------------------------------------------------------
{
  double cost = 1.;

  if(pred.freeDistance > Behaviour::MIN_RESPONSE_TIME * pred.laneSpeed)
  {
    cost = -1.;
  }

  //std::cout << "[C(col) : " << cost << "] ";
  return cost;
}

//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::separationCost(const BehaviourPredictor::Prediction& pred)
//---------------------------------------------------------------------------------------------------------------------
{
  const double goodSeparation = 2*Behaviour::MIN_RESPONSE_TIME;
  const double separation = pred.freeDistance/pred.laneSpeed;
  double cost = 1;
  if(separation < Behaviour::MIN_RESPONSE_TIME)
  {
    cost = 1.;
  }
  else if(separation > goodSeparation)
  {
    cost = -1.;
  }
  else
  {
    cost = 3 - 2*separation/Behaviour::MIN_RESPONSE_TIME;
  }
  //std::cout << "[C(sep) : " << cost << "]" << std::endl;
  return cost;
}

//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::speedDeviationCost(const BehaviourPredictor::Prediction& pred)
//---------------------------------------------------------------------------------------------------------------------
{
  const double deviation = fabs(Behaviour::MAX_SPEED - pred.laneSpeed);
  const double cost = -0.5 + (deviation/Behaviour::MAX_SPEED);
  //std::cout << "[C(speed) : " << cost << "] ";
  return cost;
}

//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::manouvrebilityCost(const BehaviourPredictor::Prediction& pred)
//---------------------------------------------------------------------------------------------------------------------
{
  double cost = 1;
  if(pred.laneNumber == 1)
  {
    cost = -1;
  }
  else
  {
    cost = 0.5;
  }
  //std::cout << "[C(man) : " << cost << "] ";
  return cost;
}

//---------------------------------------------------------------------------------------------------------------------
double BehaviourPlanner::frequentLaneChangeCost(const BehaviourPredictor::Prediction& pred)
//---------------------------------------------------------------------------------------------------------------------
{
  double cost = 0;
  if( current_.duration < MIN_LANE_KEEP_TIMESTEPS)
  {
    cost = 1;
  }
  else if( current_.duration > 2*MIN_LANE_KEEP_TIMESTEPS)
  {
    cost = 0;
  }
  else
  {
    cost = 2 - current_.duration/MIN_LANE_KEEP_TIMESTEPS;
  }
  //std::cout << "[C(changes) : " << cost << "] ";

  return cost;
}

}
