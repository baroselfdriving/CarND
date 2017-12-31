#include "behaviour_predictor.h"
#include "constants.h"
#include "helpers.h"

#include <iostream>
#include <iomanip>
#include <cassert>

namespace sdcnd_t3p1
{

//---------------------------------------------------------------------------------------------------------------------
BehaviourPredictor::LanePredictionMap BehaviourPredictor::predict(const Vehicle& me, const VehicleList& others)
//---------------------------------------------------------------------------------------------------------------------
{
  LanePredictionMap pmap;
  Prediction prediction;
  pmap[0] = prediction;
  pmap[1] = prediction;
  pmap[2] = prediction;

  const int myLane = getLaneNumberFromFrenetD(me.frenet.d);
  for(int lane = 0; lane < 3; ++lane)
  {
    auto& prediction = pmap.at(lane);
    prediction.laneNumber = lane;
    prediction.laneSpeed = ((lane == myLane) ? me.speed : 0);
    prediction.freeDistance = 0;

    NearestVehicles nearest = findClosestVehiclesInLane(lane, me, others);
    double freeDistRear = MAX_TRACK_LENGTH;
    double laneSpeedRear = Behaviour::MAX_SPEED;
    if(nearest.behind != others.end())
    {
      // Vehicle behind us in the same lane is responsible for it's own speed. ignore.
      // For vehicles on other lanes, consider them only if they are too close to be
      // a collision hazard if we change into their lane.
      const int theirLane = getLaneNumberFromFrenetD(nearest.behind->frenet.d);
      if(theirLane != myLane)
      {
        // assumed worst case: we suddenly lose speed but vehicle behind speeds up
        const double dist = getDistanceAlongTrack(me.frenet.s, nearest.behind->frenet.s);
        const double theirScale = 1.2;
        const double meScale = .8;
        laneSpeedRear = nearest.behind->speed;
        freeDistRear = dist - (theirScale * laneSpeedRear - meScale * me.speed) * Behaviour::LANE_CHANGE_DURATION;
      }
    }

    double freeDistAhead = MAX_TRACK_LENGTH;
    double laneSpeedFront = Behaviour::MAX_SPEED;
    if(nearest.ahead != others.end())
    {
      // assumed worst case: we speed up but vehicle in front reduces speed
      const double theirScale = 0.5;
      const double meScale = 1.2;
      const double dist = getDistanceAlongTrack(nearest.ahead->frenet.s, me.frenet.s);
      laneSpeedFront = nearest.ahead->speed;
      freeDistAhead = dist - (meScale * me.speed - theirScale * nearest.ahead->speed) * Behaviour::LANE_CHANGE_DURATION;
    }

    if((freeDistRear > 0) && // vehicle behind does not overtake us
       (freeDistAhead > 0) ) // we don't overtake vehicle ahead
    {
      prediction.freeDistance = freeDistAhead; // how much free space we've got. Only consider space up ahead.
      prediction.laneSpeed = std::max(laneSpeedFront, laneSpeedRear);
    }

    //std::cout << std::fixed << std::setprecision(1) << std::setw(6) << "l: [" << prediction.laneNumber << "] d: [" << prediction.freeDistance
    //          << "] v: " << prediction.laneSpeed << std::endl;

  }
  //std::cout << "=====" << std::endl;
  return pmap;
}


}
